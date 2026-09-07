/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_rpc_cm4.c — M4-side RPMsg remote: the "mbus" endpoint op dispatcher.
 * Runs on the dedicated rpc thread (app_bus_cm4.c) — CM4's ONLY OpenAMP caller. Bus-touching
 * ops serialise against the per-bus threads via the bus mutexes taken inside the leaf
 * transaction functions. Contract: docs/Inter_Core_RPMsg_Protocol.md. */
#include <string.h>
#include "stm32h7xx_hal.h"
#include "openamp.h"
#include "app_pimage.h"   /* process-image control block (op0C doorbell) */

static struct rpmsg_endpoint s_ept;

void HSEM2_IRQHandler(void)   /* CM4-side HSEM interrupt */
{
  HAL_HSEM_IRQHandler();
}

/* mbus protocol (construction ②): op=1 Modbus transaction [1][addr][expect][pdu...] -> [status][rsp_pdu...];
 * all other payloads keep echo (pong:) compatibility so the CLI 'rpc' link test still works */
extern void     mbport_cm4_init(void);
extern void     mbport_cm4_slave_service(void);
extern uint16_t mbport_cm4_transact(uint8_t addr, const uint8_t *pdu, uint16_t pn,
                                    uint8_t *rsp_pdu, uint8_t expect_rsp);
extern void     mbport_cm4_raw_send(const uint8_t *d, uint16_t n);   /* op=2 bad-frame injection */
extern void     mbport_cm4_time_push(uint32_t unix_s);               /* op=3 time -> broadcast */
extern uint16_t mbport_cm4_status(uint8_t *out);                     /* op=4 scheduler status */
extern uint16_t mbport_cm4_uart_stats(uint8_t *out);                 /* op=5 two-port UART stats */
extern void     app_cm4_led_sync(uint8_t green_on);                  /* op=6 yellow LED locks, green inverted (app_cm4.c) */

/* Front-panel configurable Modbus ports (app_mbfront_cm4.c, contract v1.3 op8~0B) */
extern void     mbfront_board_feed(uint32_t unix_s, int16_t temp, uint16_t flags);
extern int      mbfront_cfg_set(uint8_t port, uint8_t role, uint32_t baud,
                                uint8_t parity, uint8_t stop, uint8_t addr);
extern uint16_t mbfront_slave_pdu(const uint8_t *pdu, uint16_t n, uint8_t *rsp);
extern uint16_t mbfront_transact(uint8_t port, uint8_t slave, uint8_t expect,
                                 uint16_t timeout_ms, const uint8_t *pdu, uint16_t pn,
                                 uint8_t *rsp_pdu, uint8_t *status);
extern uint16_t mbfront_stats(uint8_t port, uint8_t *out);

static int rpc_recv_cb(struct rpmsg_endpoint *ept, void *data, size_t len,
                       uint32_t src, void *priv)
{
  (void)src; (void)priv;
  uint8_t *d = (uint8_t *)data;
  if ((len >= 5U) && (d[0] == 0x03U))            /* time feed -> broadcast; v1.3 may carry [temp i16][flags u16] to feed the board-level map */
  {
    uint8_t ok = 0;
    uint32_t t = (uint32_t)d[1] | ((uint32_t)d[2] << 8) | ((uint32_t)d[3] << 16) | ((uint32_t)d[4] << 24);
    mbport_cm4_time_push(t);
    if (len >= 9U)
    {
      mbfront_board_feed(t, (int16_t)((uint16_t)d[5] | ((uint16_t)d[6] << 8)),
                         (uint16_t)d[7] | ((uint16_t)d[8] << 8));
    }
    else { mbfront_board_feed(t, 0, 0); }
    OPENAMP_send(ept, &ok, 1);
    return 0;
  }
  if ((len == 10U) && (d[0] == 0x08U))           /* front-panel port configuration (contract v1.3) */
  {
    uint32_t baud = (uint32_t)d[3] | ((uint32_t)d[4] << 8) | ((uint32_t)d[5] << 16) | ((uint32_t)d[6] << 24);
    uint8_t st = (uint8_t)mbfront_cfg_set(d[1], d[2], baud, d[7], d[8], d[9]);
    OPENAMP_send(ept, &st, 1);
    return 0;
  }
  if ((len >= 2U) && (d[0] == 0x09U))            /* this board's slave-map adjudication (TCP shell) */
  {
    static uint8_t rsp9[300];
    uint16_t rl = mbfront_slave_pdu(&d[1], (uint16_t)(len - 1U), &rsp9[1]);
    rsp9[0] = 0;
    OPENAMP_send(ept, rsp9, (size_t)(1U + rl));
    return 0;
  }
  if ((len >= 7U) && (d[0] == 0x0AU))            /* front-panel master-port transaction */
  {
    static uint8_t rspA[300];
    uint8_t st = 0;
    uint16_t to = (uint16_t)d[4] | ((uint16_t)d[5] << 8);
    uint16_t rl = mbfront_transact(d[1], d[2], d[3], to, &d[6], (uint16_t)(len - 6U), &rspA[1], &st);
    rspA[0] = st;
    OPENAMP_send(ept, rspA, (size_t)(1U + rl));
    return 0;
  }
  if ((len == 2U) && (d[0] == 0x0BU))            /* front-panel port statistics */
  {
    uint8_t st[40];                              /* reply is 10 x u32 — sizing this at 20 smashed
                                                  * the rpc thread's frame on every 'mbstat' */
    uint16_t n2 = mbfront_stats(d[1], st);
    OPENAMP_send(ept, st, (size_t)n2);
    return 0;
  }
  if ((len == 1U) && (d[0] == 0x0EU))            /* FreeRTOS own run-time stats table (text) */
  {
    extern uint16_t diag_cm4_runstats(uint8_t *out, uint16_t cap);
    static uint8_t rspE[460];
    uint16_t n2 = diag_cm4_runstats(rspE, sizeof(rspE));
    OPENAMP_send(ept, rspE, (size_t)n2);
    return 0;
  }
  if ((len == 1U) && (d[0] == 0x0DU))            /* CM4 self-report: tasks / stacks / heap (v1.5) */
  {
    extern uint16_t diag_cm4_tasks(uint8_t *out);
    static uint8_t rspD[400];
    uint16_t n2 = diag_cm4_tasks(rspD);
    OPENAMP_send(ept, rspD, (size_t)n2);
    return 0;
  }
  if ((len == 2U) && (d[0] == 0x11U))            /* HSDI channel signal statistics [ch] -> 11 x u32 (2026-09-03, contract v1.8) */
  {
    extern uint16_t hsdi_cm4_capstat(uint8_t ch, uint8_t *out);
    uint8_t r[48];
    uint16_t n2 = hsdi_cm4_capstat(d[1], r);
    OPENAMP_send(ept, r, (size_t)n2);
    return 0;
  }
  if ((len >= 2U) && (d[0] == 0x12U))            /* HSDI edge recorder (2026-09-04, contract v1.8): sub 0 arm [mask][lim u16],
                                                  * sub 1 status, sub 2 read [idx u16][n u8] -> n x 8 B */
  {
    extern void     hsdi_cm4_rec_arm(uint8_t mask, uint16_t lim);
    extern uint16_t hsdi_cm4_rec_status(uint8_t *out);
    extern uint16_t hsdi_cm4_rec_read(uint16_t idx, uint8_t n, uint8_t *out);
    static uint8_t r12[496];
    uint16_t n2 = 0;
    if ((d[1] == 0U) && (len == 5U)) { hsdi_cm4_rec_arm(d[2], (uint16_t)(d[3] | ((uint16_t)d[4] << 8))); n2 = hsdi_cm4_rec_status(r12); }
    else if (d[1] == 1U) { n2 = hsdi_cm4_rec_status(r12); }
    else if ((d[1] == 2U) && (len == 5U)) { n2 = hsdi_cm4_rec_read((uint16_t)(d[2] | ((uint16_t)d[3] << 8)), d[4], r12); }
    else { r12[0] = 1U; n2 = 1U; }
    OPENAMP_send(ept, r12, (size_t)n2);
    return 0;
  }
  if ((len == 4U) && (d[0] == 0x13U))            /* bench: software EXTI trigger [line, period_ms lo, hi]; period 0 = off (2026-09-03;
                                                  * op 0x0E->0x13 on 2026-09-04: 0x0E is runstats, a length-overloaded op is what the contract forbids) */
  {
    extern void hsdi_cm4_swtest(uint8_t line, uint16_t period_ms);
    uint8_t st = 0;
    hsdi_cm4_swtest(d[1], (uint16_t)(d[2] | ((uint16_t)d[3] << 8)));
    OPENAMP_send(ept, &st, 1);
    return 0;
  }
  if ((len == 2U) && (d[0] == 0x0CU))            /* process-image scanner control doorbell (contract v1.4) */
  {
    extern void pscan_cm4_reload(void);
    uint8_t st = 0;
    switch (d[1])
    {
      case 0: pscan_cm4_reload(); break;         /* reload: re-snapshot cfg[] at this safe point */
      case 1: PIMG->ctrl.running = 1U; break;    /* start scanner */
      case 2: PIMG->ctrl.running = 0U; break;    /* stop scanner */
      default: st = 1U; break;
    }
    OPENAMP_send(ept, &st, 1);
    return 0;
  }
  if ((len == 2U) && (d[0] == 0x06U))            /* LED phase sync (fire-and-forget, no reply) */
  {
    app_cm4_led_sync(d[1]);
    return 0;
  }
  if ((len == 1U) && (d[0] == 0x05U))            /* UART port statistics query */
  {
    uint8_t st[48];
    uint16_t n2 = mbport_cm4_uart_stats(st);
    OPENAMP_send(ept, st, (size_t)n2);
    return 0;
  }
  if ((len == 1U) && (d[0] == 0x04U))            /* scheduler status query */
  {
    uint8_t st[16];
    uint16_t n2 = mbport_cm4_status(st);
    OPENAMP_send(ept, st, (size_t)n2);
    return 0;
  }
  if ((len == 2U) && (d[0] == 0x07U))            /* master-port baud-rate switch (contract v1.2; code table = backplane 0x0011) */
  {
    extern int mbport_cm4_set_baud(uint8_t code);
    uint8_t st = (mbport_cm4_set_baud(d[1]) == 0) ? 0U : 1U;
    OPENAMP_send(ept, &st, 1);
    return 0;
  }
  if ((len >= 2U) && (d[0] == 0x02U))            /* raw send (test: bad-CRC frame injection) */
  {
    uint8_t ok = 0;
    mbport_cm4_raw_send(&d[1], (uint16_t)(len - 1U));
    OPENAMP_send(ept, &ok, 1);
    return 0;
  }
  if ((len >= 4U) && (d[0] == 0x01U))            /* Modbus transaction */
  {
    static uint8_t rsp[300];
    uint16_t rl = mbport_cm4_transact(d[1], &d[3], (uint16_t)(len - 3U), &rsp[1], d[2]);
    rsp[0] = (rl > 0U) ? 0U : ((d[2] == 0U) ? 0U : 1U);   /* broadcast with no reply = normal */
    OPENAMP_send(ept, rsp, (size_t)(1U + rl));
    return 0;
  }
  {
    uint8_t rsp[128];
    size_t n = (len > (sizeof(rsp) - 5U)) ? (sizeof(rsp) - 5U) : len;
    memcpy(rsp, "pong:", 5);
    memcpy(&rsp[5], data, n);
    OPENAMP_send(ept, rsp, n + 5U);
  }
  return 0;
}

int app_rpc_cm4_init(void)   /* called at the start of app_cm4_task */
{
  HAL_NVIC_SetPriority(HSEM2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(HSEM2_IRQn);
  if (MX_OPENAMP_Init(RPMSG_REMOTE, NULL) != 0) { return -1; }
  return OPENAMP_create_endpoint(&s_ept, "mbus", RPMSG_ADDR_ANY, rpc_recv_cb, NULL);
}

void app_rpc_cm4_poll(void) { OPENAMP_check_for_message(); }   /* rpc thread, 1 ms cadence */
