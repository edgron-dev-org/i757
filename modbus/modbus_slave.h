/* SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Edgron. Licensed under the Apache License, Version 2.0 (see LICENSE in this directory). */
/* modbus_slave.h — Modbus slave PDU dispatcher (contract=Backplane_Bus_Protocol.md v0.2 §3 data model)
 * Pure C99, no HAL: takes "address + request PDU", emits "response PDU"—transport layer (UART/loopback/unit test) provided by caller.
 * This file is the eventual production slave core of the H503 expansion module firmware (EX_16DO data model), not a test stub. */
#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H
#include <stdint.h>

/* Slave data model (contract §3; upper layer reads/writes fields directly, dispatcher handles protocol mapping) */
typedef struct {
  /* Input registers 0x0000~0x0007 (FC04 read-only) */
  uint16_t map_ver;        /* 0x0000 data model version=1 */
  uint16_t module_type;    /* 0x0001 1=EX_16DO */
  uint16_t fw_ver;         /* 0x0002 software version 0xMMmm (registry=Board_Type_and_Version_Registry.md) */
  uint16_t caps;           /* 0x0003 bit0=DO bit1=DI bit2=AI */
  uint16_t hw_ver;         /* 0x0008 hardware version 0xMMmm (v0.24) */
  uint32_t uptime_s;       /* 0x0004(high)/0x0005(low) */
  uint16_t crc_errs;       /* 0x0006 transport layer writes the bad-frame count here */
  uint8_t  safe_state;     /* FLAGS.bit0 */
  uint8_t  time_valid;     /* 0=time never received (FLAGS.bit2 set) */
  /* Input registers 0x0100~ (FC04, AI raw values ch1~; contract §3.3) */
  const uint16_t *ai;      /* AI array provided by the module (NULL=no AI); main loop writes / response reads, single-threaded no race */
  uint16_t ai_cnt;
  /* Input registers 0x0300~ (FC04, diagnostic block; contract v0.27): header (ntask/cpu%/stkmin)
   * + 8-reg task slots. Filled by the platform (slave_core), NULL = no diagnostics. */
  const uint16_t *diag;
  uint16_t diag_cnt;
  /* Coils 0~15 (FC01/05/15) */
  uint16_t coils;
  void (*on_coils_write)(uint16_t coils);   /* nullable: push coil writes to hardware (EX_16DO=PhotoMOS) */
  /* Discrete inputs (FC02; contract v0.25 DI extension, first user=EX_16DI):
   * module stores the debounced level bitmap (bit0=DI1, ON=1); u16 store is atomic on CM33,
   * so the sampler task may write it directly. di_cnt=0 -> FC02 answers ILLEGAL_FC. */
  uint16_t di;
  uint16_t di_cnt;         /* DI channel count (FC02 addresses 0..di_cnt-1); 0 = no DI */
  /* Input registers 0x0200~ (FC04, DI counter block; contract v0.25): u32 per channel as
   * 2 registers hi-word-first. Module-maintained shadow; the WRITER must update each hi/lo
   * pair atomically vs this dispatcher (slave_core: critical section). NULL = no counters. */
  const uint16_t *cnt;
  uint16_t cnt_regs;       /* register count = 2 * channels */
  /* Holding 0x0020~0x002F (contract v0.25): per-channel debounce 0~255 ms (>255 = exception 03).
   * Volatile by design — master re-sends after enrollment. Dispatcher validates and stores;
   * the module sampler reads (per-register u16 access, atomic). Present only when di_cnt!=0. */
  uint16_t debounce_ms[16];
  /* Holding 0x0030 (contract v0.25): CNT_CLR bitmap, W1C self-clearing, always reads 0.
   * Dispatcher ORs written bits here (bus task); the module sampler consumes-and-clears
   * (critical section on its read-modify-write). Present only when cnt != NULL. */
  volatile uint16_t cnt_clr;
  /* Holding registers */
  uint16_t time_regs[3];   /* 0x0000~0x0002 unix seconds high/low/milliseconds (master broadcast FC16) */
  uint16_t events;         /* 0x0010 event bitmap, write-1-to-clear (W1C) */
  uint16_t baud_sel;       /* 0x0011 baud rate code (contract v0.22, table extended v0.30):
                              0=1M default/1=250K/2=500K/3=2M/4=3M/5=5M/6=4800/7=9600/
                              8=19200/9=38400/10=57600/11=115200 */
  uint8_t  baud_pending;   /* set to 1 after a valid write; cleared once transport executes the switch 20ms after ACK is sent */
  /* User function-code extension hook (contract v0.23 slave OTA=0x41~0x44 go here): returns response PDU length;
   * 0=no reply (broadcast semantics), 0xFFFF=unknown -> dispatcher returns exception 01. Nullable=no extension. */
  uint16_t (*on_user_fc)(uint8_t fc, const uint8_t *pdu, uint16_t n, uint8_t *rsp);
  /* Configuration/calibration holding area 0x0200~0x02FF (contract v0.28): probe types,
   * calibration, module parameters. FC03 reads and FC06 single writes are forwarded
   * register-by-register; the hook returns 0=ok or a Modbus exception code (2=no such
   * register, 3=value rejected). NULL hooks = whole area answers exception 2. */
  uint8_t (*on_cfg_read)(uint16_t addr, uint16_t *val);
  uint8_t (*on_cfg_write)(uint16_t addr, uint16_t val);
} mb_slave_t;

#define MB_BAUD_CODE_MAX  11U  /* legal codes 0~11 (code table=contract 0x0011; v0.30 adds
                                  6=4800..11=115200 standard low rates, old codes frozen) */

void mb_slave_defaults(mb_slave_t *s);   /* bench defaults (map_ver=1/type=1/caps=DO/safe state); real modules overwrite ident from g_slave_module */

/* Handle one frame: frame_addr=received ADU address byte.
 * Returns response PDU length (caller wraps in ADU to send back); 0=no reply (broadcast frame executes without answering / not addressed to this device). */
uint16_t mb_slave_handle(mb_slave_t *s, uint8_t self_addr, uint8_t frame_addr,
                         const uint8_t *pdu, uint16_t n, uint8_t *rsp);

#endif
