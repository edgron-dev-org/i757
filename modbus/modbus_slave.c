/* SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Edgron. Licensed under the Apache License, Version 2.0 (see LICENSE in this directory). */
/* modbus_slave.c — Modbus slave PDU dispatcher implementation (contract v0.2 §3/§4)
 * Broadcast (address 0) only executes write-class commands and does not reply (standard semantics); read-class broadcasts are ignored outright.
 * Out of range=ILLEGAL_ADDR, unknown function code=ILLEGAL_FC, illegal value=ILLEGAL_VAL. */
#include "modbus_slave.h"
#include "modbus_core.h"
#include <string.h>

void mb_slave_defaults(mb_slave_t *s)
{
  memset(s, 0, sizeof(*s));
  s->map_ver     = 1;
  s->module_type = 1;        /* bench default (selftest asserts it; nucleo test slave uses it) —
                              * real modules overwrite the whole ident block from g_slave_module
                              * in slave_core_init, so these are never seen on a real board */
  s->fw_ver      = 0x0100;
  s->caps        = 0x0001;   /* DO only (same bench default) */
  s->safe_state  = 1;        /* power-on default safe state, awaiting master enrollment (contract §4) */
  for (uint8_t i = 0; i < 16U; i++) { s->debounce_ms[i] = 10U; }   /* contract v0.25 default */
}

static uint16_t exc(uint8_t *rsp, uint8_t fc, uint8_t code)
{
  return (uint16_t)mb_slv_exception(rsp, fc, code);
}

static void coils_apply(mb_slave_t *s, uint16_t val)
{
  s->coils = val;
  if (s->on_coils_write != 0) { s->on_coils_write(val); }
}

uint16_t mb_slave_handle(mb_slave_t *s, uint8_t self_addr, uint8_t frame_addr,
                         const uint8_t *pdu, uint16_t n, uint8_t *rsp)
{
  uint8_t bcast = (frame_addr == MB_ADDR_BCAST) ? 1U : 0U;
  if (!bcast && (frame_addr != self_addr)) { return 0; }   /* not addressed to me */
  if (n < 1U) { return 0; }
  uint8_t fc = pdu[0];

  switch (fc)
  {
    case MB_FC_READ_COILS:                       /* FC01: coils 0~15 */
    {
      if (bcast) { return 0; }                   /* read-class broadcast is meaningless, ignore */
      if (n != 5U) { return exc(rsp, fc, MB_EXC_ILLEGAL_VAL); }
      uint16_t start = mb_get16(&pdu[1]), qty = mb_get16(&pdu[3]);
      if ((qty == 0U) || (start > 15U) || ((start + qty) > 16U)) { return exc(rsp, fc, MB_EXC_ILLEGAL_ADDR); }
      uint8_t bits[2];
      uint16_t v = (uint16_t)(s->coils >> start);
      bits[0] = (uint8_t)(v & 0xFFU);
      bits[1] = (uint8_t)((v >> 8) & 0xFFU);
      return (uint16_t)mb_slv_rsp_bits(rsp, fc, bits, qty);
    }
    case MB_FC_READ_DISC:                        /* FC02: discrete inputs (contract v0.25) */
    {
      if (bcast) { return 0; }
      if (s->di_cnt == 0U) { return exc(rsp, fc, MB_EXC_ILLEGAL_FC); }   /* module has no DI */
      if (n != 5U) { return exc(rsp, fc, MB_EXC_ILLEGAL_VAL); }
      uint16_t start = mb_get16(&pdu[1]), qty = mb_get16(&pdu[3]);
      if ((qty == 0U) || (start >= s->di_cnt) || ((start + qty) > s->di_cnt)) { return exc(rsp, fc, MB_EXC_ILLEGAL_ADDR); }
      uint8_t bits[2];
      uint16_t v = (uint16_t)(s->di >> start);
      bits[0] = (uint8_t)(v & 0xFFU);
      bits[1] = (uint8_t)((v >> 8) & 0xFFU);
      return (uint16_t)mb_slv_rsp_bits(rsp, fc, bits, qty);
    }
    case MB_FC_WRITE_COIL:                       /* FC05: single coil */
    {
      if (n != 5U) { return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_VAL); }
      uint16_t addr = mb_get16(&pdu[1]), val = mb_get16(&pdu[3]);
      if (addr > 15U) { return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_ADDR); }
      if ((val != 0xFF00U) && (val != 0x0000U)) { return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_VAL); }
      uint16_t c = s->coils;
      if (val == 0xFF00U) { c |= (uint16_t)(1U << addr); } else { c &= (uint16_t)~(1U << addr); }
      coils_apply(s, c);
      if (bcast) { return 0; }
      memcpy(rsp, pdu, 5);                       /* response = request echo */
      return 5;
    }
    case MB_FC_WRITE_COILS:                      /* FC15: multiple coils (full overwrite = idempotent) */
    {
      if (n < 7U) { return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_VAL); }
      uint16_t start = mb_get16(&pdu[1]), qty = mb_get16(&pdu[3]);
      uint8_t bc = pdu[5];
      if ((qty == 0U) || (start > 15U) || ((start + qty) > 16U)) { return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_ADDR); }
      if ((bc != ((qty + 7U) / 8U)) || (n != (uint16_t)(6U + bc))) { return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_VAL); }
      uint16_t v = (uint16_t)(pdu[6] | ((bc > 1U) ? ((uint16_t)pdu[7] << 8) : 0U));
      uint16_t mask = (uint16_t)(((qty >= 16U) ? 0xFFFFU : ((1U << qty) - 1U)) << start);
      coils_apply(s, (uint16_t)((s->coils & ~mask) | ((uint16_t)(v << start) & mask)));
      if (bcast) { return 0; }
      rsp[0] = fc;
      mb_put16(&rsp[1], start);
      mb_put16(&rsp[3], qty);
      return 5;
    }
    case MB_FC_READ_INPUT:                       /* FC04: 0x0000~0x0007 system area / 0x0100~ AI area */
    {
      if (bcast) { return 0; }
      if (n != 5U) { return exc(rsp, fc, MB_EXC_ILLEGAL_VAL); }
      uint16_t start = mb_get16(&pdu[1]), qty = mb_get16(&pdu[3]);
      if ((start >= 0x0100U) && (start < 0x0200U) && (s->ai != 0) && (qty >= 1U) && (qty <= MB_MAX_READ_REGS)
          && (((uint32_t)start + qty) <= (0x0100U + (uint32_t)s->ai_cnt)))
      {
        return (uint16_t)mb_slv_rsp_regs(rsp, fc, &s->ai[start - 0x0100U], qty);
      }
      if ((start >= 0x0200U) && (start < 0x0300U) && (s->cnt != 0) && (qty >= 1U) && (qty <= MB_MAX_READ_REGS)
          && (((uint32_t)start + qty) <= (0x0200U + (uint32_t)s->cnt_regs)))
      {
        return (uint16_t)mb_slv_rsp_regs(rsp, fc, &s->cnt[start - 0x0200U], qty);   /* DI counters (contract v0.25) */
      }
      if ((start >= 0x0300U) && (s->diag != 0) && (qty >= 1U) && (qty <= MB_MAX_READ_REGS)
          && (((uint32_t)start + qty) <= (0x0300U + (uint32_t)s->diag_cnt)))
      {
        return (uint16_t)mb_slv_rsp_regs(rsp, fc, &s->diag[start - 0x0300U], qty);   /* diagnostic block (contract v0.27) */
      }
      if ((qty == 0U) || (start > 8U) || ((start + qty) > 9U)) { return exc(rsp, fc, MB_EXC_ILLEGAL_ADDR); }
      uint16_t regs[9];
      regs[0] = s->map_ver;
      regs[1] = s->module_type;
      regs[2] = s->fw_ver;
      regs[3] = s->caps;
      regs[4] = (uint16_t)(s->uptime_s >> 16);
      regs[5] = (uint16_t)(s->uptime_s & 0xFFFFU);
      regs[6] = s->crc_errs;
      regs[7] = (uint16_t)((s->safe_state ? 1U : 0U) | ((s->events != 0U) ? 2U : 0U)
                          | (s->time_valid ? 0U : 4U));
      regs[8] = s->hw_ver;                     /* 0x0008 hardware version (contract v0.24) */
      return (uint16_t)mb_slv_rsp_regs(rsp, fc, &regs[start], qty);
    }
    case MB_FC_READ_HOLD:                        /* FC03: 0x0000~0x0002 time / 0x0010 events */
    {
      if (bcast) { return 0; }
      if (n != 5U) { return exc(rsp, fc, MB_EXC_ILLEGAL_VAL); }
      uint16_t start = mb_get16(&pdu[1]), qty = mb_get16(&pdu[3]);
      if ((start <= 2U) && (qty >= 1U) && ((start + qty) <= 3U))
      {
        return (uint16_t)mb_slv_rsp_regs(rsp, fc, &s->time_regs[start], qty);
      }
      if ((start == 0x0010U) && (qty == 1U))
      {
        return (uint16_t)mb_slv_rsp_regs(rsp, fc, &s->events, 1);
      }
      if ((start == 0x0011U) && (qty == 1U))
      {
        return (uint16_t)mb_slv_rsp_regs(rsp, fc, &s->baud_sel, 1);
      }
      if ((start >= 0x0020U) && (start < 0x0030U) && (s->di_cnt != 0U) && (qty >= 1U)
          && (((uint32_t)start + qty) <= 0x0030U))
      {
        return (uint16_t)mb_slv_rsp_regs(rsp, fc, &s->debounce_ms[start - 0x0020U], qty);   /* v0.25 */
      }
      if ((start == 0x0030U) && (qty == 1U) && (s->cnt != 0))
      {
        uint16_t zero = 0;                       /* CNT_CLR: W1C self-clearing, always reads 0 (v0.25) */
        return (uint16_t)mb_slv_rsp_regs(rsp, fc, &zero, 1);
      }
      /* config/cal area 0x0200~0x02FF (contract v0.28): forwarded register-by-register */
      if ((start >= 0x0200U) && (start < 0x0300U) && (qty >= 1U) && (qty <= MB_MAX_READ_REGS)
          && (((uint32_t)start + qty) <= 0x0300U) && (s->on_cfg_read != 0))
      {
        uint16_t regs[MB_MAX_READ_REGS];
        for (uint16_t i = 0; i < qty; i++)
        {
          uint8_t e = s->on_cfg_read((uint16_t)(start + i), &regs[i]);
          if (e != 0U) { return exc(rsp, fc, e); }
        }
        return (uint16_t)mb_slv_rsp_regs(rsp, fc, regs, qty);
      }
      return exc(rsp, fc, MB_EXC_ILLEGAL_ADDR);
    }
    case MB_FC_WRITE_REG:                        /* FC06: single write (a time cell / event W1C / baud rate) */
    {
      if (n != 5U) { return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_VAL); }
      uint16_t addr = mb_get16(&pdu[1]), val = mb_get16(&pdu[3]);
      if (addr <= 2U)         { s->time_regs[addr] = val; s->time_valid = 1; }
      else if (addr == 0x0010U) { s->events = (uint16_t)(s->events & ~val); }   /* write-1-to-clear */
      else if (addr == 0x0011U)                  /* contract v0.22: switch 20ms after ACK (at old baud rate) */
      {
        if (val > MB_BAUD_CODE_MAX) { return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_VAL); }
        s->baud_sel = val; s->baud_pending = 1;
      }
      else if ((addr >= 0x0020U) && (addr < 0x0030U) && (s->di_cnt != 0U))
      {
        if (val > 255U) { return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_VAL); }   /* v0.25: 0~255 ms */
        s->debounce_ms[addr - 0x0020U] = val;
      }
      else if ((addr == 0x0030U) && (s->cnt != 0))
      {
        s->cnt_clr |= val;                       /* CNT_CLR: module sampler consumes & clears (v0.25) */
      }
      else if ((addr >= 0x0200U) && (addr < 0x0300U) && (s->on_cfg_write != 0))
      {
        uint8_t e = s->on_cfg_write(addr, val);  /* config/cal area (contract v0.28) */
        if (e != 0U) { return bcast ? 0U : exc(rsp, fc, e); }
      }
      else { return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_ADDR); }
      if (bcast) { return 0; }
      memcpy(rsp, pdu, 5);
      return 5;
    }
    case MB_FC_WRITE_REGS:                       /* FC16: multiple write (the front door for time broadcast) */
    {
      if (n < 8U) { return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_VAL); }
      uint16_t start = mb_get16(&pdu[1]), qty = mb_get16(&pdu[3]);
      uint8_t bc = pdu[5];
      if ((bc != (uint8_t)(qty * 2U)) || (n != (uint16_t)(6U + bc))) { return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_VAL); }
      if ((start <= 2U) && (qty >= 1U) && ((start + qty) <= 3U))
      {
        for (uint16_t i = 0; i < qty; i++) { s->time_regs[start + i] = mb_get16(&pdu[6U + 2U * i]); }
        s->time_valid = 1;
      }
      else if ((start == 0x0010U) && (qty == 1U))
      {
        s->events = (uint16_t)(s->events & ~mb_get16(&pdu[6]));
      }
      else if ((start == 0x0011U) && (qty == 1U))
      {
        uint16_t v = mb_get16(&pdu[6]);
        if (v > MB_BAUD_CODE_MAX) { return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_VAL); }
        s->baud_sel = v; s->baud_pending = 1;
      }
      else if ((start >= 0x0020U) && (start < 0x0030U) && (s->di_cnt != 0U)
               && (((uint32_t)start + qty) <= 0x0030U))
      {
        /* debounce bulk download (v0.25, master re-sends after enrollment): validate all
         * values first so an oversized one rejects the whole write instead of half-applying */
        for (uint16_t i = 0; i < qty; i++)
        {
          if (mb_get16(&pdu[6U + 2U * i]) > 255U) { return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_VAL); }
        }
        for (uint16_t i = 0; i < qty; i++)
        {
          s->debounce_ms[start - 0x0020U + i] = mb_get16(&pdu[6U + 2U * i]);
        }
      }
      else { return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_ADDR); }
      if (bcast) { return 0; }
      rsp[0] = fc;
      mb_put16(&rsp[1], start);
      mb_put16(&rsp[3], qty);
      return 5;
    }
    default:
      if ((fc >= 0x41U) && (fc <= 0x48U) && (s->on_user_fc != 0))   /* contract v0.23: OTA user function codes */
      {
        uint16_t r = s->on_user_fc(fc, pdu, n, rsp);
        if (r != 0xFFFFU) { return bcast ? 0U : r; }
      }
      return bcast ? 0U : exc(rsp, fc, MB_EXC_ILLEGAL_FC);
  }
}
