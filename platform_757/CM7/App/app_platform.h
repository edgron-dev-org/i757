/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_platform.h — the platform service API for YOUR application.
 *
 * Include this from your user files (app_user.c and any app_*.c you add).
 * It gathers the services the platform offers your code: front-panel Modbus,
 * backplane expansion-module I/O, time, temperature, buzzer, and power-fail
 * retained storage. All functions here are safe to call from your own FreeRTOS
 * tasks unless noted otherwise.
 */
#ifndef APP_PLATFORM_H
#define APP_PLATFORM_H
#include <stdint.h>

/* ---- Modbus ports ----
 * The front-panel RS-485 connectors and the internal backplane expansion bus
 * are all plain Modbus RTU masters, reached through the same API. Pass one of
 * these as the `port` argument; `addr` is the Modbus slave address on that port.
 */
#define APP_PORT_485A       0   /* front-panel RS-485 (silkscreen 485A) */
#define APP_PORT_485B       1
#define APP_PORT_485C       2
#define APP_PORT_485D       3
#define APP_PORT_485E       4
#define APP_PORT_485F       5
#define APP_PORT_BACKPLANE  6   /* internal backplane bus: expansion modules, slave addr 1..16 (DIP), always master */

/* ---- Modbus port role ---- */
#define APP_MB_OFF     0
#define APP_MB_SLAVE   1
#define APP_MB_MASTER  2

/* Configure a front 485 port at runtime. Persists to flash and pushes to CM4.
 *   role   : APP_MB_OFF / APP_MB_SLAVE / APP_MB_MASTER
 *   parity : 'N' | 'E' | 'O'      stop: 1 | 2
 *   slave_addr: this board's address when role = APP_MB_SLAVE (ignored for master)
 * Returns 0 on success, <0 on error. Call this from app_user_init() (setup),
 * not concurrently from several running tasks (it writes littlefs). */
int app_mbport_configure(uint8_t port, uint8_t role, uint32_t baud,
                         char parity, uint8_t stop, uint8_t slave_addr);

/* Raw Modbus master transaction on a front port (port must be APP_MB_MASTER).
 * pdu = function code + data (no address, no CRC — the platform adds those).
 * Returns response PDU length (>=0) copied into rsp, or <0 on error. Thread-safe. */
int app_mbport_master(uint8_t port, uint8_t slave_addr,
                      const uint8_t *pdu, uint16_t pdu_len,
                      uint8_t *rsp, uint16_t rsp_cap);

/* Convenience Modbus master helpers. `port` = any APP_PORT_* (front 485 or backplane).
 * Return 0 on success; <0 on error (a Modbus exception code E is returned as -(100+E)).
 * Thread-safe — call from any of your tasks. */
int app_mb_read_holding(uint8_t port, uint8_t addr, uint16_t reg, uint16_t count, uint16_t *out);   /* FC03 */
int app_mb_read_input  (uint8_t port, uint8_t addr, uint16_t reg, uint16_t count, uint16_t *out);   /* FC04 */
int app_mb_write_single(uint8_t port, uint8_t addr, uint16_t reg, uint16_t value);                  /* FC06 */
int app_mb_write_multi (uint8_t port, uint8_t addr, uint16_t reg, uint16_t count, const uint16_t *vals); /* FC16 */

/* Coil (digital output / relay) helpers. `bits`/`out` are packed, LSB = first coil.
 * A 16-channel relay module maps its outputs to coils 0..15. Same return convention. */
int app_mb_read_coils (uint8_t port, uint8_t addr, uint16_t coil, uint16_t count, uint8_t *out);        /* FC01 */
int app_mb_write_coil (uint8_t port, uint8_t addr, uint16_t coil, uint8_t on);                          /* FC05 */
int app_mb_write_coils(uint8_t port, uint8_t addr, uint16_t coil, uint16_t count, const uint8_t *bits); /* FC15 */

/* ---- process-image I/O scanner (optional; for cyclic field I/O) ----
 * The calls above are on-demand: each blocks on one bus round-trip. For fast
 * cyclic I/O, register a table of points once in app_user_init() with
 * app_io_setup(); the CM4 scanner then polls them in the background and keeps
 * a process image you read/write as memory — non-blocking, decoupled from bus
 * timing. Reads never block; writes are RMW-safe across your tasks.
 * Contract: docs/Process_Image_and_IO_Mapping.md. */
#define APP_IO_IN_COILS     0   /* read coils            (FC01) -> app_io_di  */
#define APP_IO_IN_DISCRETE  1   /* read discrete inputs  (FC02) -> app_io_di  */
#define APP_IO_IN_INPUT_REG 2   /* read input registers  (FC04) -> app_io_ai  */
#define APP_IO_IN_HOLDING   3   /* read holding registers(FC03) -> app_io_ai  */
#define APP_IO_OUT_COILS    4   /* write coils           (FC15) -> app_io_do_set */
#define APP_IO_OUT_HOLDING  5   /* write holding registers(FC16)-> app_io_hr_set */
#define APP_IO_BYTESWAP     0x01U   /* flags: word device is byte-swapped */

typedef struct {
  uint8_t  port;    /* APP_PORT_485A..F or APP_PORT_BACKPLANE */
  uint8_t  addr;    /* Modbus slave address */
  uint8_t  access;  /* one of APP_IO_* above */
  uint8_t  flags;   /* 0 or APP_IO_BYTESWAP */
  uint16_t start;   /* first coil / register address */
  uint16_t count;   /* qty (bits for coils/discrete, words for registers) */
  uint16_t period;  /* poll interval in base ticks; 0 = entry disabled */
} app_io_point_t;

/* Register the scan table once (from app_user_init). base_tick_ms 0 = default 1 ms.
 * The point index (0..n-1) is the handle you pass to the accessors below.
 * Returns 0 on success, <0 on error (capacity / image overflow). */
int      app_io_setup (const app_io_point_t *points, uint16_t n, uint16_t base_tick_ms);
uint8_t  app_io_di    (uint16_t pt, uint16_t ch);          /* input bit (coil/discrete input) */
uint16_t app_io_ai    (uint16_t pt, uint16_t ch);          /* input word (input/holding register) */
void     app_io_do_set(uint16_t pt, uint16_t ch, uint8_t on);  /* set an output coil */
uint8_t  app_io_do_get(uint16_t pt, uint16_t ch);          /* read back a commanded output coil */
void     app_io_hr_set(uint16_t pt, uint16_t ch, uint16_t v);  /* set an output holding register */
int      app_io_ok    (uint16_t pt);                       /* 1 = last scan ok, 0 = failing, <0 = bad point */

/* ---- onboard high-speed digital inputs (HSDI, 8 channels on CN4) ----
 * Each channel can be a plain input, an edge counter, or part of a quadrature encoder.
 * Results land in the process image, so your code and an external Modbus master see the
 * same data. NOT every combination is possible — one timer is either an encoder or ONE
 * hardware counter, and only some channels can clock a timer at all:
 *
 *   encoder 0 = HSDI0(A) + HSDI1(B), index Z = HSDI2      (TIM3)
 *   encoder 1 = HSDI4(A) + HSDI5(B), index Z = HSDI3      (TIM1)
 *   hardware counters possible on : HSDI0, HSDI1, HSDI4, HSDI5, HSDI6
 *   software counters possible on : any channel (<=10 kHz; HSDI2/HSDI7 and HSDI3/HSDI4
 *                                   share an interrupt line — only one of each pair)
 * So you get at most 2 encoders + 1 hardware counter, OR 3 hardware counters, etc.
 * Contract (full combination table): docs/HSDI_Configuration_and_Counting.md. */
#define APP_HSDI_OFF        0
#define APP_HSDI_DI         1   /* level only */
#define APP_HSDI_COUNT_SW   2   /* software edge counter (EXTI), <=10 kHz; min_us qualifier available */
#define APP_HSDI_COUNT_HW   3   /* hardware counter, >=1 MHz */
#define APP_HSDI_COUNT_POLL 7   /* polled edge counter (1 ms sampling, no interrupt), <=200 Hz, any channel */
#define APP_HSDI_COUNT_CAP  8   /* timer input-capture counter: hardware filter + edge-timestamp interrupt + min_us
                                 * pulse qualifier; any channel, no EXTI line used. The mode for field pulse sensors */
#define APP_HSDI_ENC_A      4   /* encoder A phase */
#define APP_HSDI_ENC_B      5   /* encoder B phase */
#define APP_HSDI_ENC_Z      6   /* encoder index */
#define APP_HSDI_EDGE_RISING  0
#define APP_HSDI_EDGE_FALLING 1
#define APP_HSDI_EDGE_BOTH    2
#define APP_HSDI_Z_NONE     0
#define APP_HSDI_Z_ZERO     1   /* index edge zeroes the position */
#define APP_HSDI_Z_LATCH    2   /* index edge latches the position and counts a revolution */

typedef struct {
  uint8_t mode;        /* APP_HSDI_* */
  uint8_t filter;      /* timer hardware input filter 0..15 (counting / encoder modes) */
  uint8_t edge;        /* APP_HSDI_EDGE_* for counting modes */
  uint8_t z_action;    /* APP_HSDI_Z_* for the index channel */
  uint8_t debounce_ms; /* software debounce for plain-input and software-counter modes; 0 = off.
                        * Mechanical contacts want 5..20 ms; leave 0 for electronic signals. */
  uint16_t min_us;     /* COUNT_CAP / COUNT_SW pulse qualifier: a level must hold this many microseconds
                        * to count as a phase; shorter dips/spikes are ignored. 0 = every edge counts.
                        * A hall flow meter with 15 ms pulses wants ~1000. */
} app_hsdi_ch_t;

/* Configure all 8 channels at once (call from app_user_init, before/after app_io_setup).
 * Returns 0 on success, <0 if the combination is not realisable (falls back to plain DI). */
int      app_hsdi_setup   (const app_hsdi_ch_t cfg[8]);
uint8_t  app_hsdi_level   (uint8_t ch);        /* live input level, any mode */
uint32_t app_hsdi_count   (uint8_t ch);        /* edge count (hardware or software counter) */
int32_t  app_hsdi_position(uint8_t enc);       /* encoder position, enc = 0 or 1 (signed) */
uint32_t app_hsdi_revs    (uint8_t enc);       /* revolutions seen on the index input */
uint32_t app_hsdi_zlatch  (uint8_t enc);       /* position latched at the last index edge */
void     app_hsdi_reset   (uint8_t ch);        /* zero that channel's counter / position */

/* ---- large-buffer RAM: 32 MB external SDRAM as a linker region ----
 * Usage:  APP_SDRAM static uint16_t history[1000000];
 * Rules (both because the FMC controller starts late, in the first main-loop pass):
 *   - contents are NOT loaded and NOT zeroed at startup — initialise it yourself;
 *   - do not touch it from code that can run before the platform's storage init
 *     (tasks created in app_user_init are fine: they start after).
 * Cached write-back (MPU Region4): do not hand SDRAM buffers to DMA without cache maintenance. */
#define APP_SDRAM __attribute__((section(".sdram")))

/* ---- misc platform services ---- */
#include "app_time.h"      /* uint32_t app_time_now(void)  — Unix seconds (RTC/SNTP) */
int  app_temp_read(void);  /* chip temperature in °C (-99 on failure) */
void app_beep_set(uint8_t on);   /* buzzer on/off */

/* ---- power-fail retained storage ----
 * Mark any variable PF_RETAIN and it survives a power cut (battery-backed SRAM).
 * See app_pwrfail.h for the full contract (hooks, black-box, validity check). */
#include "app_pwrfail.h"

/* ---- event journal ----
 * Write a line into the on-board event journal (SD card / flash, daily CSV files —
 * see docs/Data_Logging_and_Event_Journal.md). Events show up as flags on the history
 * charts, so record every action your logic takes on the process (dosing, heating, ...).
 *   type: "ACTION" for things your code did, "NOTE" for free-form remarks.
 * printf-style; text is truncated to ~118 bytes. Safe from any task (no file I/O here;
 * a small RAM queue is drained by the platform). */
void app_log_event(const char *type, const char *fmt, ...);

#endif /* APP_PLATFORM_H */
