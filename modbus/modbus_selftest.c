/* SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Edgron. Licensed under the Apache License, Version 2.0 (see LICENSE in this directory). */
/* modbus_selftest.c — Modbus frame core unit tests (pure C assertion set, runs on-board at power-on = permanent regression)
 * Authoritative vectors: (1) byte-by-byte compare against Modbus spec classic example frames; (2) other frames first pre-run through an independent Python implementation
 * (guards against implementation and test sharing the same bug). Returns failed-case count (0=all pass), *total=total number of cases. */
#include "modbus_core.h"
#include "modbus_slave.h"
#include <string.h>

static int s_fail, s_total;
#define CHECK(cond) do { s_total++; if (!(cond)) { s_fail++; } } while (0)

int mb_selftest(int *total)
{
  /* Large buffers as static go into .bss: runs once at power-on and single-threaded, does not stress the task stack */
  static uint8_t adu[MB_ADU_MAX], pdu[MB_PDU_MAX];
  uint8_t addr;
  const uint8_t *rp;
  uint16_t rn;
  int n;
  s_fail = 0;
  s_total = 0;

  /* 1. CRC standard vector */
  CHECK(mb_crc16((const uint8_t *)"123456789", 9) == 0x4B37U);

  /* 2. Spec classic example: slave 1 FC03 read holding 0x006B qty3 -> 01 03 00 6B 00 03 74 17 */
  {
    static const uint8_t ref[] = { 0x01, 0x03, 0x00, 0x6B, 0x00, 0x03, 0x74, 0x17 };
    n = mb_req_read(pdu, MB_FC_READ_HOLD, 0x006B, 3);
    CHECK(n == 5);
    n = mb_adu_build(adu, sizeof(adu), 1, pdu, (uint16_t)n);
    CHECK((n == 8) && (memcmp(adu, ref, 8) == 0));
    CHECK(mb_adu_check(adu, 8, &addr, &rp, &rn) == 0);
    CHECK((addr == 1) && (rn == 5) && (rp[0] == MB_FC_READ_HOLD));
  }

  /* 3. FC03 response frame (Python pre-run: 010306ae4156524340846d) parsing */
  {
    static const uint8_t ref[] = { 0x01, 0x03, 0x06, 0xAE, 0x41, 0x56, 0x52, 0x43, 0x40, 0x84, 0x6D };
    uint16_t regs[4];
    CHECK(mb_adu_check(ref, sizeof(ref), &addr, &rp, &rn) == 0);
    CHECK(mb_rsp_regs(rp, rn, MB_FC_READ_HOLD, regs, 4) == 3);
    CHECK((regs[0] == 0xAE41U) && (regs[1] == 0x5652U) && (regs[2] == 0x4340U));
  }

  /* 4. FC15 write 16 coils (Python pre-run: 020f0000001002ff00b720) */
  {
    static const uint8_t ref[] = { 0x02, 0x0F, 0x00, 0x00, 0x00, 0x10, 0x02, 0xFF, 0x00, 0xB7, 0x20 };
    uint8_t bits[2] = { 0xFF, 0x00 };
    n = mb_req_write_coils(pdu, 0, 16, bits);
    CHECK(n == 8);
    n = mb_adu_build(adu, sizeof(adu), 2, pdu, (uint16_t)n);
    CHECK((n == (int)sizeof(ref)) && (memcmp(adu, ref, sizeof(ref)) == 0));
  }

  /* 5. FC16 broadcast write time (Python pre-run: 00100000000306686c2a4001f4753b) */
  {
    static const uint8_t ref[] = { 0x00, 0x10, 0x00, 0x00, 0x00, 0x03, 0x06,
                                   0x68, 0x6C, 0x2A, 0x40, 0x01, 0xF4, 0x75, 0x3B };
    uint16_t t[3] = { 0x686CU, 0x2A40U, 0x01F4U };
    n = mb_req_write_regs(pdu, 0, 3, t);
    CHECK(n == 12);   /* fc1+start2+quantity2+byte-count1+data6 (once mistyped as 13; implementation was right, the assertion was wrong—caught by full-frame compare) */
    n = mb_adu_build(adu, sizeof(adu), MB_ADDR_BCAST, pdu, (uint16_t)n);
    CHECK((n == (int)sizeof(ref)) && (memcmp(adu, ref, sizeof(ref)) == 0));
  }

  /* 6. Exception response (Python pre-run: 0383026131) recognition */
  {
    static const uint8_t ref[] = { 0x03, 0x83, 0x02, 0x61, 0x31 };
    uint8_t exc = 0;
    CHECK(mb_adu_check(ref, sizeof(ref), &addr, &rp, &rn) == 0);
    CHECK((mb_rsp_is_exception(rp, rn, &exc) == 1) && (exc == MB_EXC_ILLEGAL_ADDR));
    /* slave-side constructs the same exception response -> consistent */
    n = mb_slv_exception(pdu, MB_FC_READ_HOLD, MB_EXC_ILLEGAL_ADDR);
    CHECK((n == 2) && (pdu[0] == 0x83) && (pdu[1] == 0x02));
  }

  /* 7. Bad CRC / short frame rejection */
  {
    uint8_t bad[8] = { 0x01, 0x03, 0x00, 0x6B, 0x00, 0x03, 0x74, 0x16 };  /* CRC tail byte wrong */
    CHECK(mb_adu_check(bad, 8, &addr, &rp, &rn) == -1);
    CHECK(mb_adu_check(bad, 3, &addr, &rp, &rn) == -1);                   /* <4B */
  }

  /* 8. FC05/06 single-write round trip + echo check (coil ON normalized to 0xFF00) */
  {
    n = mb_req_write_single(pdu, MB_FC_WRITE_COIL, 7, 1);
    CHECK((n == 5) && (mb_get16(&pdu[3]) == 0xFF00U));
    CHECK(mb_rsp_write_ok(pdu, 5, MB_FC_WRITE_COIL, 7, 1) == 0);          /* response = request echo */
    n = mb_req_write_single(pdu, MB_FC_WRITE_REG, 0x0010, 0x0003);        /* EVENTS W1C */
    CHECK((n == 5) && (mb_rsp_write_ok(pdu, 5, MB_FC_WRITE_REG, 0x0010, 0x0003) == 0));
    CHECK(mb_rsp_write_ok(pdu, 5, MB_FC_WRITE_REG, 0x0011, 0x0003) == -1);/* echo mismatch must be reported */
  }

  /* 9. slave response construction -> master parsing closed loop (FC04 read IDENT 4 registers) */
  {
    uint16_t ident[4] = { 1, 1, 0x0102U, 0x0001U };   /* MAP_VER/EX_16DO/FW/CAPS=DO */
    uint16_t back[4];
    n = mb_slv_rsp_regs(pdu, MB_FC_READ_INPUT, ident, 4);
    CHECK(n == 10);
    CHECK(mb_rsp_regs(pdu, (uint16_t)n, MB_FC_READ_INPUT, back, 4) == 4);
    CHECK(memcmp(back, ident, sizeof(ident)) == 0);
  }

  /* 10. slave bit-response construction -> master parsing closed loop (FC01 read 16 coils) */
  {
    uint8_t bits[2] = { 0xA5, 0x01 };
    uint8_t back[2];
    n = mb_slv_rsp_bits(pdu, MB_FC_READ_COILS, bits, 16);
    CHECK(n == 4);
    CHECK(mb_rsp_bits(pdu, (uint16_t)n, MB_FC_READ_COILS, back, 2) == 2);
    CHECK((back[0] == 0xA5) && (back[1] == 0x01));
  }

  /* 11. Parameter validation: quantity limit / zero quantity / null pointer */
  CHECK(mb_req_read(pdu, MB_FC_READ_HOLD, 0, 126) == -1);
  CHECK(mb_req_read(pdu, MB_FC_READ_COILS, 0, 2001) == -1);
  CHECK(mb_req_read(pdu, MB_FC_READ_HOLD, 0, 0) == -1);
  CHECK(mb_req_write_regs(pdu, 0, 124, (const uint16_t *)pdu) == -1);
  CHECK(mb_req_write_coils(pdu, 0, 0, (const uint8_t *)pdu) == -1);
  CHECK(mb_adu_build(adu, 4, 1, pdu, 5) == -1);                           /* insufficient capacity */

  /* 12. Full-length response (125 registers) round trip */
  {
    static uint16_t big[MB_MAX_READ_REGS], back[MB_MAX_READ_REGS];
    for (uint16_t i = 0; i < MB_MAX_READ_REGS; i++) { big[i] = (uint16_t)(i * 257U); }
    n = mb_slv_rsp_regs(pdu, MB_FC_READ_HOLD, big, MB_MAX_READ_REGS);
    CHECK(n == (int)(2U + 250U));
    n = mb_adu_build(adu, sizeof(adu), 16, pdu, (uint16_t)n);
    CHECK(n == 255);
    CHECK(mb_adu_check(adu, (uint16_t)n, &addr, &rp, &rn) == 0);
    CHECK(mb_rsp_regs(rp, rn, MB_FC_READ_HOLD, back, MB_MAX_READ_REGS) == (int)MB_MAX_READ_REGS);
    CHECK(memcmp(back, big, sizeof(big)) == 0);
  }

  /* ---- Slave PDU dispatcher (modbus_slave.c, contract §3 data model) ---- */
  {
    static mb_slave_t sl;
    static uint8_t req[64], rsp[64];
    uint16_t rl;
    mb_slave_defaults(&sl);
    sl.uptime_s = 0x00012345UL;

    /* 13. IDENT: FC04 read 0x0000 qty4 -> map_ver/type/fw/caps */
    n = mb_req_read(req, MB_FC_READ_INPUT, 0, 4);
    rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
    {
      uint16_t regs[4];
      CHECK(mb_rsp_regs(rsp, rl, MB_FC_READ_INPUT, regs, 4) == 4);
      CHECK((regs[0] == 1U) && (regs[1] == 1U) && (regs[3] == 1U));
    }

    /* 14. Not addressed to me (address mismatch) -> no reply */
    CHECK(mb_slave_handle(&sl, 5, 6, req, (uint16_t)n, rsp) == 0U);

    /* 15. FC15 write 16 coils 0xA5A5 -> FC01 read back consistent */
    {
      uint8_t bits[2] = { 0xA5, 0xA5 };
      n = mb_req_write_coils(req, 0, 16, bits);
      rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      CHECK(mb_rsp_write_ok(rsp, rl, MB_FC_WRITE_COILS, 0, 16) == 0);
      CHECK(sl.coils == 0xA5A5U);
      n = mb_req_read(req, MB_FC_READ_COILS, 0, 16);
      rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      uint8_t back[2];
      CHECK(mb_rsp_bits(rsp, rl, MB_FC_READ_COILS, back, 2) == 2);
      CHECK((back[0] == 0xA5U) && (back[1] == 0xA5U));
    }

    /* 16. Time broadcast FC16 (address 0): execute + no reply + time_valid set */
    {
      uint16_t t[3] = { 0x1234U, 0x5678U, 0x01F4U };
      CHECK(sl.time_valid == 0U);
      n = mb_req_write_regs(req, 0, 3, t);
      CHECK(mb_slave_handle(&sl, 5, MB_ADDR_BCAST, req, (uint16_t)n, rsp) == 0U);
      CHECK((sl.time_valid == 1U) && (sl.time_regs[0] == 0x1234U) && (sl.time_regs[2] == 0x01F4U));
      /* FC03 read back time */
      n = mb_req_read(req, MB_FC_READ_HOLD, 0, 3);
      rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      uint16_t regs[3];
      CHECK(mb_rsp_regs(rsp, rl, MB_FC_READ_HOLD, regs, 3) == 3);
      CHECK(regs[1] == 0x5678U);
    }

    /* 17. Event W1C: set events -> FC06 write-1-to-clear -> FLAGS.bit1 goes off */
    {
      sl.events = 0x0005U;
      n = mb_req_write_single(req, MB_FC_WRITE_REG, 0x0010, 0x0001);
      rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      CHECK((rl == 5U) && (sl.events == 0x0004U));   /* clears bit0 only */
      n = mb_req_write_single(req, MB_FC_WRITE_REG, 0x0010, 0xFFFF);
      (void)mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      CHECK(sl.events == 0U);
    }

    /* 18. Exception paths: illegal function code / coil out of range */
    {
      uint8_t ex;
      req[0] = 0x2B;                               /* unsupported FC */
      rl = mb_slave_handle(&sl, 5, 5, req, 1, rsp);
      CHECK((mb_rsp_is_exception(rsp, rl, &ex) == 1) && (ex == MB_EXC_ILLEGAL_FC));
      n = mb_req_read(req, MB_FC_READ_COILS, 10, 10);   /* 10+10>16 out of range */
      rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      CHECK((mb_rsp_is_exception(rsp, rl, &ex) == 1) && (ex == MB_EXC_ILLEGAL_ADDR));
    }

    /* ---- Contract v0.25 DI extension (first user = EX_16DI) ---- */

    /* 19. FC02 on a DI-less module -> ILLEGAL_FC; with DI -> reads the debounced bitmap */
    {
      uint8_t ex, back[2];
      n = mb_req_read(req, MB_FC_READ_DISC, 0, 16);
      rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);   /* defaults: di_cnt=0 */
      CHECK((mb_rsp_is_exception(rsp, rl, &ex) == 1) && (ex == MB_EXC_ILLEGAL_FC));
      sl.di_cnt = 16;
      sl.di = 0x5A81U;
      rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      CHECK(mb_rsp_bits(rsp, rl, MB_FC_READ_DISC, back, 2) == 2);
      CHECK((back[0] == 0x81U) && (back[1] == 0x5AU));
      n = mb_req_read(req, MB_FC_READ_DISC, 8, 9);              /* 8+9>16 out of range */
      rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      CHECK((mb_rsp_is_exception(rsp, rl, &ex) == 1) && (ex == MB_EXC_ILLEGAL_ADDR));
    }

    /* 20. FC04 0x0200 counter block: u32 hi-word-first shadow readout + range guard */
    {
      static const uint16_t cnt[4] = { 0x0001U, 0x86A0U, 0x0000U, 0x002AU };   /* ch1=100000, ch2=42 */
      uint16_t regs[4];
      uint8_t ex;
      sl.cnt = cnt;
      sl.cnt_regs = 4;
      n = mb_req_read(req, MB_FC_READ_INPUT, 0x0200, 4);
      rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      CHECK(mb_rsp_regs(rsp, rl, MB_FC_READ_INPUT, regs, 4) == 4);
      CHECK((regs[0] == 0x0001U) && (regs[1] == 0x86A0U) && (regs[3] == 0x002AU));
      n = mb_req_read(req, MB_FC_READ_INPUT, 0x0203, 2);        /* crosses block end */
      rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      CHECK((mb_rsp_is_exception(rsp, rl, &ex) == 1) && (ex == MB_EXC_ILLEGAL_ADDR));
    }

    /* 21. Debounce 0x0020~: default 10, FC06 write + FC03 readback, >255 -> ILLEGAL_VAL,
     *     FC16 bulk write validates all before applying any */
    {
      uint16_t regs[16];
      uint8_t ex;
      n = mb_req_read(req, MB_FC_READ_HOLD, 0x0020, 16);
      rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      CHECK(mb_rsp_regs(rsp, rl, MB_FC_READ_HOLD, regs, 16) == 16);
      CHECK((regs[0] == 10U) && (regs[15] == 10U));
      n = mb_req_write_single(req, MB_FC_WRITE_REG, 0x0025, 50);
      rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      CHECK((rl == 5U) && (sl.debounce_ms[5] == 50U));
      n = mb_req_write_single(req, MB_FC_WRITE_REG, 0x0025, 256);
      rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      CHECK((mb_rsp_is_exception(rsp, rl, &ex) == 1) && (ex == MB_EXC_ILLEGAL_VAL));
      CHECK(sl.debounce_ms[5] == 50U);                          /* rejected write left value alone */
      {
        uint16_t vals[3] = { 1U, 300U, 3U };                    /* one bad value poisons the batch */
        n = mb_req_write_regs(req, 0x0020, 3, vals);
        rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
        CHECK((mb_rsp_is_exception(rsp, rl, &ex) == 1) && (ex == MB_EXC_ILLEGAL_VAL));
        CHECK(sl.debounce_ms[0] == 10U);                        /* nothing applied */
        vals[1] = 2U;
        n = mb_req_write_regs(req, 0x0020, 3, vals);
        rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
        CHECK(mb_rsp_write_ok(rsp, rl, MB_FC_WRITE_REGS, 0x0020, 3) == 0);
        CHECK((sl.debounce_ms[0] == 1U) && (sl.debounce_ms[1] == 2U) && (sl.debounce_ms[2] == 3U));
      }
    }

    /* 22. CNT_CLR 0x0030: write ORs the pending bitmap, readback is always 0 */
    {
      uint16_t regs[1];
      n = mb_req_write_single(req, MB_FC_WRITE_REG, 0x0030, 0x0005);
      rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      CHECK((rl == 5U) && (sl.cnt_clr == 0x0005U));
      n = mb_req_write_single(req, MB_FC_WRITE_REG, 0x0030, 0x0008);
      (void)mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      CHECK(sl.cnt_clr == 0x000DU);                             /* OR, not overwrite */
      n = mb_req_read(req, MB_FC_READ_HOLD, 0x0030, 1);
      rl = mb_slave_handle(&sl, 5, 5, req, (uint16_t)n, rsp);
      CHECK(mb_rsp_regs(rsp, rl, MB_FC_READ_HOLD, regs, 1) == 1);
      CHECK(regs[0] == 0U);
      sl.cnt_clr = 0;
    }
  }

  if (total != 0) { *total = s_total; }
  return s_fail;
}
