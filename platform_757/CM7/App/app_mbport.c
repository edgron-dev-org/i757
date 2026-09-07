/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_mbport.c — M7-side Modbus client (757 P2②: UART lives on M4, this file never touches UART)
 * Transactions are handed to CM4 over RPMsg: [0x01][addr][expect][pdu...] -> [status][rsp_pdu...]
 * The nucleo wires/wscan jumper-continuity tools are retired (on 757 those pins belong to other functions, can't grab them). */
#include "app_mbport.h"
#include <stdio.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#include "modbus_core.h"
#include "app_time.h"
#include "app_rpc.h"
#include "app_platform.h"   /* APP_PORT_BACKPLANE + generic coil helpers (used by the dashboard relay demo below) */

#define SLAVE_ADDR  2U   /* bench test module address, used by the mbx/mbspeed CLI self-tests */

/* ---- master transaction: handed to CM4 over RPMsg (protocol in file header) ---- */
static uint16_t master_transact(uint8_t addr, const uint8_t *pdu, uint16_t pn,
                                uint8_t *rsp_pdu, uint8_t expect_rsp)
{
  static uint8_t req[300], rsp[300];
  req[0] = 0x01U; req[1] = addr; req[2] = expect_rsp;
  memcpy(&req[3], pdu, pn);
  uint16_t r = app_rpc_transact(req, (uint16_t)(3U + pn), rsp, sizeof(rsp), 150);
  if ((r < 1U) || (rsp[0] != 0U) || (expect_rsp == 0U)) { return 0; }
  memcpy(rsp_pdu, &rsp[1], (size_t)(r - 1U));
  return (uint16_t)(r - 1U);
}

/* ---- CLI 'mbr <addr> <reg> [n]' / 'mbw <addr> <reg> <val>' — generic backplane register
 * access (FC03 read / FC06 write). Bench tool for the module config/cal area 0x0200~
 * (contract v0.28): probe type selection, one-touch calibration, 0xA55A save. Numbers
 * accept 0x prefixes; reads print hex and signed decimal side by side. ---- */
#include <stdlib.h>
/* core: result rendered into `out` so the CLI (console) and the MQTT downlink command
 * share one implementation — remote calibration uses the exact registers a customer would */
void app_mb_reg_str(char rw, const char *args, char *out, uint16_t cap)
{
  char *end = 0;
  unsigned long addr = strtoul(args, &end, 0);
  unsigned long reg  = strtoul(end, &end, 0);
  unsigned long p3   = strtoul(end, &end, 0);   /* count (mbr, 0->1) or value (mbw) */
  uint8_t pdu[8], rsp[64];
  uint16_t n = 0;
  out[0] = 0;
  if ((addr < 1UL) || (addr > 16UL) || (reg > 0xFFFFUL))
  {
    snprintf(out, cap, "usage: mbr <addr> <reg> [n] | mbw <addr> <reg> <val>");
    return;
  }
  if (rw == 'w')
  {
    uint16_t rl;
    if (p3 > 0xFFFFUL) { snprintf(out, cap, "value must be u16"); return; }
    pdu[0] = 0x06U;                            /* FC06 write single holding register */
    pdu[1] = (uint8_t)(reg >> 8); pdu[2] = (uint8_t)reg;
    pdu[3] = (uint8_t)(p3 >> 8);  pdu[4] = (uint8_t)p3;
    rl = app_mb_transact((uint8_t)addr, pdu, 5U, rsp, 1U);
    if (rl == 0U)            { snprintf(out, cap, "mbw: no reply"); }
    else if (rsp[0] & 0x80U) { snprintf(out, cap, "mbw: exception %u", (unsigned)rsp[1]); }
    else                     { snprintf(out, cap, "mbw ok [0x%04lX]=%lu", reg, p3); }
    return;
  }
  {
    uint16_t qty = (p3 == 0UL) ? 1U : (p3 > 16UL) ? 16U : (uint16_t)p3;
    uint16_t regs[16];
    int pl = mb_req_read(pdu, MB_FC_READ_HOLD, (uint16_t)reg, qty);
    uint16_t rl = app_mb_transact((uint8_t)addr, pdu, (uint16_t)pl, rsp, 1U);
    if (rl == 0U)       { snprintf(out, cap, "mbr: no reply"); return; }
    if (rsp[0] & 0x80U) { snprintf(out, cap, "mbr: exception %u", (unsigned)rsp[1]); return; }
    if (mb_rsp_regs(rsp, rl, MB_FC_READ_HOLD, regs, qty) != (int)qty) { snprintf(out, cap, "mbr: bad reply"); return; }
    for (uint16_t i = 0; (i < qty) && (n < cap); i++)
    {
      n += (uint16_t)snprintf(&out[n], (size_t)(cap - n), "%s[0x%04lX]=0x%04X/%d",
                              (i > 0U) ? " " : "", reg + i, (unsigned)regs[i], (int)(int16_t)regs[i]);
    }
  }
}

void app_mb_reg_cli(char rw, const char *args)
{
  static char buf[300];
  app_mb_reg_str(rw, args, buf, sizeof(buf));
  if ((rw == 'w') && (strncmp(buf, "mbw ok", 6) == 0))   /* config audit trail (Data_Logging_and_Event_Journal.md) */
  {
    extern void app_log_event_src(const char *type, const char *src, const char *fmt, ...);
    app_log_event_src("CONFIG", "cli", "mbw %s -> %s", args, buf);
  }
  printf("%s\n\r", buf);
}

/* ---- scheduler status query (task ③): op=4 -> CM4 online table / counters ---- */
void app_mb_bus_status(void)
{
  uint8_t req[1] = { 0x04U };
  uint8_t rsp[16];
  uint16_t r = app_rpc_transact(req, 1, rsp, sizeof(rsp), 150);
  if (r < 11U) { printf("mbus: no reply (%u)\n\r", (unsigned)r); return; }
  uint16_t bitmap = (uint16_t)(rsp[0] | ((uint16_t)rsp[1] << 8));
  uint32_t polls  = (uint32_t)rsp[2] | ((uint32_t)rsp[3] << 8) | ((uint32_t)rsp[4] << 16) | ((uint32_t)rsp[5] << 24);
  uint16_t fails  = (uint16_t)(rsp[6] | ((uint16_t)rsp[7] << 8));
  uint16_t bcasts = (uint16_t)(rsp[8] | ((uint16_t)rsp[9] << 8));
  printf("bus(M4 scheduler): online=");
  int any = 0;
  for (int a = 1; a <= 16; a++)
  {
    if ((bitmap & (1U << (a - 1))) != 0U) { printf("%s%d", any ? "," : "", a); any = 1; }
  }
  if (!any) { printf("(none)"); }
  printf("  polls=%lu fails=%u time-bcasts=%u\n\r",
         (unsigned long)polls, (unsigned)fails, (unsigned)bcasts);
}

/* public transaction entry (used by app_mota streaming; same semantics as master_transact) */
uint16_t app_mb_transact(uint8_t addr, const uint8_t *pdu, uint16_t pn,
                         uint8_t *rsp_pdu, uint8_t expect_rsp)
{
  return master_transact(addr, pdu, pn, rsp_pdu, expect_rsp);
}

/* ---- feed time to M4 (mqtt main loop every 10s): M4 broadcasts it onto the bus on receipt (contract §5) ---- */
void app_mb_time_push(void)
{
  uint32_t now = app_time_now();
  if (now == 0U) { return; }
  uint8_t req[9], ack[4];
  int16_t temp;
  uint16_t flags;
  { extern int app_temp_read(void); temp = (int16_t)app_temp_read(); }
  { extern uint8_t mqtt_cloud_ok(void); flags = (uint16_t)((mqtt_cloud_ok() ? 1U : 0U) | 2U); }  /* bit0=cloud online bit1=scheduler alive (alive as soon as fed) */
  req[0] = 0x03U;
  req[1] = (uint8_t)(now & 0xFFU);
  req[2] = (uint8_t)((now >> 8) & 0xFFU);
  req[3] = (uint8_t)((now >> 16) & 0xFFU);
  req[4] = (uint8_t)((now >> 24) & 0xFFU);
  req[5] = (uint8_t)((uint16_t)temp & 0xFFU);      /* v1.3 extension: data source for board-level slave map IR 0x0006/0x0007 */
  req[6] = (uint8_t)(((uint16_t)temp >> 8) & 0xFFU);
  req[7] = (uint8_t)(flags & 0xFFU);
  req[8] = (uint8_t)(flags >> 8);
  (void)app_rpc_transact(req, 9, ack, sizeof(ack), 50);
}

/* ---- nucleo jumper-continuity tools: retired on 757 (those pins belong to other functions, can't grab them) ---- */
int app_mb_wires_test(char *report, int cap)
{
  if ((report != 0) && (cap > 0)) { snprintf(report, (size_t)cap, "n/a on 757 (nucleo-only tool)"); }
  return -1;
}

void app_mb_wire_scan(void)
{
  printf("wscan: n/a on 757 (nucleo-only tool)\n\r");
}

/* ---- baud-rate sweep (CLI 'mbspeed'; contract 0x0011 + RPMsg op=7) ----
 * Per step: FC16 write slave 0x0011 (ACK at old rate) -> op7 switch master port -> quiet 40ms to clear both sides' switch window
 *          -> N ident attempts to tally success rate -> if all dead, master switches back to 1M and waits for the slave's 3s
 *             safety window to auto-revert, then re-enrolls it. */
static const uint32_t s_baud_disp[] = { 1000000U, 250000U, 500000U, 2000000U, 3000000U, 5000000U,
                                        4800U, 9600U, 19200U, 38400U, 57600U, 115200U };

static int speed_set_master(uint8_t code)
{
  uint8_t req[2] = { 0x07U, code }, ack[4];
  uint16_t r = app_rpc_transact(req, 2, ack, sizeof(ack), 150);
  return ((r >= 1U) && (ack[0] == 0U)) ? 0 : -1;
}

static int speed_ident_ok(void)
{
  uint8_t pdu[8], rsp[MB_PDU_MAX];
  uint16_t regs[4] = {0};
  int n = mb_req_read(pdu, MB_FC_READ_INPUT, 0, 4);
  uint16_t rl = master_transact(SLAVE_ADDR, pdu, (uint16_t)n, rsp, 1);
  return ((rl > 0U) && (mb_rsp_regs(rsp, rl, MB_FC_READ_INPUT, regs, 4) == 4)
          && (regs[0] == 1U) && (regs[1] == 1U)) ? 1 : 0;
}

static void speed_back_to_default(void)
{
  (void)speed_set_master(0);
  HAL_Delay(3300);                      /* wait for the slave's 3s safety window to auto-revert to 1M */
  if (speed_ident_ok()) { printf("  (recovered @1M via slave auto-revert)\n\r"); }
  else { printf("  **WARN: slave not back at 1M**\n\r"); }
}

void app_mb_speed_sweep(void)
{
  static const uint8_t order[] = { 1, 2, 0, 3, 4, 5 };   /* 250K->500K->1M->2M->3M->5M */
  enum { TRIES = 50 };
  printf("mbspeed: BL3085 3.3V rated 250K; sweeping (contract 0x0011)...\n\r");
  for (unsigned i = 0; i < sizeof(order); i++)
  {
    uint8_t code = order[i];
    uint8_t pdu[16], rsp[MB_PDU_MAX];
    uint16_t one = code;
    int n = mb_req_write_regs(pdu, 0x0011, 1, &one);
    uint16_t rl = master_transact(SLAVE_ADDR, pdu, (uint16_t)n, rsp, 1);   /* slave switches first (ACK at old rate) */
    if (rl == 0U)
    {
      printf("  %7lubps: slave switch NO-ACK, skip\n\r", (unsigned long)s_baud_disp[code]);
      speed_back_to_default();
      continue;
    }
    if (speed_set_master(code) != 0) { printf("  op7 fail\n\r"); speed_back_to_default(); continue; }
    HAL_Delay(40);                      /* 20ms switch window on each side */
    int ok = 0, tries = 0;
    for (int k = 0; k < TRIES; k++)
    {
      ok += speed_ident_ok(); tries++;
      if ((k == 9) && (ok == 0)) { break; }   /* first 10 all dead = dead rate, early-out to save the IWDG window */
    }
    printf("  %7lubps: %d/%d %s\n\r", (unsigned long)s_baud_disp[code], ok, tries,
           (ok == tries) ? "PASS" : (ok == 0) ? "DEAD" : "**MARGINAL**");
    if (ok == 0) { speed_back_to_default(); }
  }
  /* wrap-up: make sure both sides are back at the default 1M */
  {
    uint8_t pdu[16], rsp[MB_PDU_MAX];
    uint16_t zero = 0;
    int n = mb_req_write_regs(pdu, 0x0011, 1, &zero);
    (void)master_transact(SLAVE_ADDR, pdu, (uint16_t)n, rsp, 1);
    (void)speed_set_master(0);
    HAL_Delay(60);
    printf("mbspeed done, back @1M: ident %s\n\r", speed_ident_ok() ? "OK" : "**FAIL**");
  }
}

/* ---- A8 acceptance sequence (same semantics as the single-core version, transport = RPMsg->CM4) ---- */
#define STEP(name, cond) do { total++; int ok_ = (cond); if (ok_) pass++; \
  printf("  %-28s %s\n\r", name, ok_ ? "PASS" : "FAIL"); } while (0)

void app_mbx_run(void)
{
  static uint8_t pdu[MB_PDU_MAX], rsp[MB_PDU_MAX];
  int total = 0, pass = 0;
  uint16_t rl;
  printf("mbx via RPMsg->CM4: master=UART8(backplane) slave=EX_16DO(addr %u) 1Mbps\n\r", SLAVE_ADDR);

  {
    uint16_t regs[4] = {0};
    int n = mb_req_read(pdu, MB_FC_READ_INPUT, 0, 4);
    rl = master_transact(SLAVE_ADDR, pdu, (uint16_t)n, rsp, 1);
    STEP("ident(FC04 0x0000..3)", (rl > 0U) && (mb_rsp_regs(rsp, rl, MB_FC_READ_INPUT, regs, 4) == 4)
                                  && (regs[0] == 1U) && (regs[1] == 1U));
  }
  {
    uint8_t bits[2] = { 0xA5, 0x5A }, back[2] = {0};
    int n = mb_req_write_coils(pdu, 0, 16, bits);
    rl = master_transact(SLAVE_ADDR, pdu, (uint16_t)n, rsp, 1);
    STEP("write coils(FC15)", (rl > 0U) && (mb_rsp_write_ok(rsp, rl, MB_FC_WRITE_COILS, 0, 16) == 0));
    n = mb_req_read(pdu, MB_FC_READ_COILS, 0, 16);
    rl = master_transact(SLAVE_ADDR, pdu, (uint16_t)n, rsp, 1);
    STEP("readback coils(FC01)", (rl > 0U) && (mb_rsp_bits(rsp, rl, MB_FC_READ_COILS, back, 2) == 2)
                                 && (back[0] == 0xA5U) && (back[1] == 0x5AU));
  }
  {
    uint32_t now = app_time_now();
    uint16_t t[3] = { (uint16_t)(now >> 16), (uint16_t)(now & 0xFFFFU), 0 };
    uint16_t back[3] = {0};
    int n = mb_req_write_regs(pdu, 0, 3, t);
    rl = master_transact(MB_ADDR_BCAST, pdu, (uint16_t)n, rsp, 0);
    STEP("time bcast no-response", rl == 0U);
    n = mb_req_read(pdu, MB_FC_READ_HOLD, 0, 3);
    rl = master_transact(SLAVE_ADDR, pdu, (uint16_t)n, rsp, 1);
    STEP("time readback(FC03)", (rl > 0U) && (mb_rsp_regs(rsp, rl, MB_FC_READ_HOLD, back, 3) == 3)
                                && (back[0] == t[0]) && (back[1] == t[1]));
  }
  {
    int n = mb_req_read(pdu, MB_FC_READ_INPUT, 0, 1);
    rl = master_transact(9, pdu, (uint16_t)n, rsp, 1);
    STEP("absent addr timeout", rl == 0U);
  }
  {
    uint16_t regs[8] = {0};
    int n = mb_req_read(pdu, MB_FC_READ_INPUT, 0, 8);
    rl = master_transact(SLAVE_ADDR, pdu, (uint16_t)n, rsp, 1);
    uint16_t errs_before = (rl > 0U) ? ((mb_rsp_regs(rsp, rl, MB_FC_READ_INPUT, regs, 8) == 8) ? regs[6] : 0xFFFFU) : 0xFFFFU;
    static uint8_t evil[16];
    n = mb_adu_build(evil, sizeof(evil), SLAVE_ADDR, pdu, 5);
    evil[n - 1] ^= 0xFFU;
    { static uint8_t raw[24]; uint8_t ack[4];
      raw[0] = 0x02U; memcpy(&raw[1], evil, (size_t)n);
      app_rpc_transact(raw, (uint16_t)(n + 1), ack, sizeof(ack), 150); }
    n = mb_req_read(pdu, MB_FC_READ_INPUT, 0, 8);
    rl = master_transact(SLAVE_ADDR, pdu, (uint16_t)n, rsp, 1);
    uint16_t errs_after = (rl > 0U) ? ((mb_rsp_regs(rsp, rl, MB_FC_READ_INPUT, regs, 8) == 8) ? regs[6] : 0U) : 0U;
    STEP("bad-CRC ignored+counted", (errs_before != 0xFFFFU) && (errs_after == (uint16_t)(errs_before + 1U)));
  }
  {
    uint8_t ex = 0;
    int n = mb_req_read(pdu, MB_FC_READ_COILS, 10, 10);
    rl = master_transact(SLAVE_ADDR, pdu, (uint16_t)n, rsp, 1);
    STEP("exception(ILLEGAL_ADDR)", (rl > 0U) && (mb_rsp_is_exception(rsp, rl, &ex) == 1)
                                    && (ex == MB_EXC_ILLEGAL_ADDR));
  }
  {
    uint8_t zeros[2] = { 0, 0 };                  /* wrap-up: zero the real relays (16O is live ammo) */
    int n = mb_req_write_coils(pdu, 0, 16, zeros);
    rl = master_transact(SLAVE_ADDR, pdu, (uint16_t)n, rsp, 1);
    STEP("coils cleanup(all off)", (rl > 0U) && (mb_rsp_write_ok(rsp, rl, MB_FC_WRITE_COILS, 0, 16) == 0));
  }
  printf("mbx result: %d/%d %s (via CM4)\n\r", pass, total,
         (pass == total) ? "ALL PASS" : "**FAIL**");
}
