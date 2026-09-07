/* SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Edgron. Licensed under the Apache License, Version 2.0 (see LICENSE in this directory). */
/* uart_drv.h — generic UART low-level driver (multi-instance/DMA/optional framing)
 * Design contract = software/docs/Universal_UART_Driver_Design.md.
 * Depends on: STM32 HAL (UART+DMA). H7 dual-core and H5 (H503 slave) are same-family and directly reusable.
 *
 * Concurrency contract (iron rule):
 *   - Each instance = single reader (application) + single writer (this driver's IRQ callback), SPSC lock-free; cross-task dispatch belongs to the upper layer.
 *   - Callbacks run in IRQ context, touch only this instance's fields; application-side API is all non-blocking (flush excepted, with timeout).
 *   - After an error the driver only raises a flag; the application main loop must periodically call uart_heal() (never restart the peripheral inside an interrupt).
 *
 * Pin discipline: this driver does NOT touch GPIO! Pin AF is attached by the board layer uniformly **after all relevant ports have been uart_open'd**
 *   (attaching AF before the peripheral is enabled drives TX low = a line break into the peer). Same goes for the 485 hardware DE pin.
 *
 * Interrupt wiring: the library does not monopolize the vector table. The application writes one line each in its own vector functions:
 *   void DMA1_Stream0_IRQHandler(void){ uart_drv_dma_rx_isr(&drv); }
 *   void USART2_IRQHandler(void)     { uart_drv_uart_isr(&drv); }
 *   NVIC enable is handled by uart_open (priority taken from cfg.irq_prio). */
#ifndef UART_DRV_H
#define UART_DRV_H
#include <stdint.h>

/* ---- Chip-family switch ----
 * H5 differs in only three places: HAL header, DMA instance type (GPDMA channel vs DMA stream), RX ring = linked-list mode. */
#if defined(STM32H503xx) || defined(STM32H523xx) || defined(STM32H533xx) || \
    defined(STM32H562xx) || defined(STM32H563xx) || defined(STM32H573xx)
#define UARTDRV_H5 1
#include "stm32h5xx_hal.h"
typedef DMA_Channel_TypeDef uartdrv_dma_t;   /* GPDMA1/2 channel */
#else
#include "stm32h7xx_hal.h"
typedef DMA_Stream_TypeDef  uartdrv_dma_t;   /* DMA1/2 stream */
#endif

#define UARTDRV_STREAM      0U   /* byte stream: application consumes via uart_read, no frame concept */
#define UARTDRV_FRAME_IDLE  1U   /* silent framing: IDLE event marks frame boundary, uart_read_frame delivers whole frame */

#define UARTDRV_DE_NONE     0U
#define UARTDRV_DE_HW       1U   /* H7 hardware DE (CR3.DEM, hardware timing backstop, recommended) */
#define UARTDRV_DE_GPIO     2U   /* GPIO manual: set before send, clear in TC callback (timing guaranteed by driver) */

typedef struct {
  USART_TypeDef      *instance;      /* USART1..UART8 */
  uint32_t            baud;
  uint32_t            parity;        /* UART_PARITY_NONE/ODD/EVEN (HAL macros) */
  uint32_t            stopbits;      /* UART_STOPBITS_1/2 */
  uint8_t             swap;          /* 1=swap TX/RX inside the chip (cabling rescue) */
  uint8_t             de_mode;       /* UARTDRV_DE_* */
  GPIO_TypeDef       *de_port;       /* used in DE_GPIO mode */
  uint16_t            de_pin;
  uartdrv_dma_t      *dma_rx;        /* NULL=RX degrades to byte-by-byte interrupt (saves a channel on low-speed ports) */
  uint32_t            req_rx;        /* H7=DMA_REQUEST_xxx / H5=GPDMA1_REQUEST_xxx */
  uartdrv_dma_t      *dma_tx;        /* NULL=TX degrades to HAL blocking transmit */
  uint32_t            req_tx;
  uint8_t            *rxbuf;         /* ring receive buffer (caller-provided, DMA writes directly, read=zero-copy consume) */
  uint16_t            rxlen;
  uint8_t            *txbuf;         /* transmit buffer (caller-provided; resident during DMA, copied in on write) */
  uint16_t            txlen;
  uint8_t             framing;       /* UARTDRV_STREAM / UARTDRV_FRAME_IDLE */
  uint8_t             max_frames;    /* frame descriptor ring depth (FRAME_IDLE, suggest 8; ≤16) */
  uint8_t             irq_prio;      /* NVIC priority (default 6) */
} uart_cfg_t;

typedef struct {
  uint32_t rx_bytes, tx_bytes;       /* cumulative bytes received/sent */
  uint32_t frames;                   /* complete frames delivered */
  uint32_t overrun;                  /* ring overflow / frame ring full -> count of honest full-ring resets */
  uint32_t hw_errs;                  /* ORE/FE/NE/PE hardware errors */
  uint32_t tx_reject;                /* count of transmits rejected while busy */
  uint32_t rearm_fail;               /* uart_heal re-arm attempts the HAL refused (each = one near-permanent RX death before 2026-08-22 fix) */
} uart_stats_t;

typedef struct { uint16_t off, len; } uart_frame_desc_t;

typedef struct {
  UART_HandleTypeDef  huart;         /* must be first member: callback container_of reverse lookup */
  DMA_HandleTypeDef   hdma_rx, hdma_tx;
#ifdef UARTDRV_H5
  DMA_NodeTypeDef     ll_node;       /* H5 GPDMA has no circular bit: RX ring = single-node self-looping linked list */
  DMA_QListTypeDef    ll_queue;
#endif
  uart_cfg_t          cfg;
  /* Ring accounting: monotonic counters (no wrap-around aliasing) — bytes in ring = wr_total - rd_total */
  volatile uint32_t   wr_total;      /* hardware write shadow (advanced by RxEvent) */
  volatile uint32_t   rd_total;      /* application read pointer (advanced by read/read_frame) */
  uint32_t            frame_start;   /* start of the frame currently being assembled (wr_total snapshot) */
  uart_frame_desc_t   fring[16];     /* frame descriptor ring */
  volatile uint8_t    f_wr, f_rd;
  volatile uint8_t    tx_busy;
  volatile uint8_t    rx_dead;       /* receive aborted after error, awaiting uart_heal */
  uint8_t             heal_tries;    /* consecutive failed heals: drives the escalation ladder (re-arm -> abort -> re-init) */
  uart_stats_t        stats;
} uart_drv_t;

int      uart_open (uart_drv_t *u, const uart_cfg_t *cfg);   /* 0=OK; ⚠️ only call on a not-yet-opened instance
                                        (the leading memset would trample a live DMA/ISR handle); change baud rate via uart_set_baud */
int      uart_set_baud(uart_drv_t *u, uint32_t baud);        /* hot-change baud rate: flush->UE off->change BRR->UE on;
                                        DMA/ring-accounting/framing all untouched */
/* hot-change all line params (baud+parity+stop bits); word length linked to parity */
int      uart_set_line(uart_drv_t *u, uint32_t baud, uint32_t parity, uint32_t stopbits);
uint16_t uart_write(uart_drv_t *u, const void *d, uint16_t n);/* non-blocking; returns 0 if busy and counts tx_reject */
int      uart_flush(uart_drv_t *u, uint32_t timeout_ms);      /* wait for TC (frame truly off the wire); 0=OK */
int      uart_tx_busy(uart_drv_t *u);
uint16_t uart_rx_count(uart_drv_t *u);                        /* STREAM: readable byte count */
uint16_t uart_read (uart_drv_t *u, void *d, uint16_t n);      /* STREAM: consuming advances the read pointer */
uint16_t uart_read_frame(uart_drv_t *u, void *d, uint16_t cap);/* FRAME_IDLE: 0=no complete frame */
void     uart_heal (uart_drv_t *u);                           /* call periodically from application main loop.
                                        Persistent (2026-08-22 fix): keeps retrying until reception is truly re-armed,
                                        escalating re-arm -> HAL abort -> full peripheral re-init. Earlier one-shot
                                        version cleared the flag before an unchecked re-arm — one HAL refusal = RX
                                        dead forever with TX still alive (greenhouse backplane field case). */
void     uart_rx_kick(uart_drv_t *u);                         /* upper-layer silent-bus watchdog hook: force the next
                                        uart_heal straight to the full re-init rung. For wedges that raised no error
                                        flag at all — from the RX owner's thread context only, never an ISR. */
void     uart_get_stats(uart_drv_t *u, uart_stats_t *out);

/* Interrupt wiring API (one line each in the application's vector functions) */
void uart_drv_uart_isr  (uart_drv_t *u);
void uart_drv_dma_rx_isr(uart_drv_t *u);
void uart_drv_dma_tx_isr(uart_drv_t *u);

#endif
