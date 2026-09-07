/* SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Edgron. Licensed under the Apache License, Version 2.0 (see LICENSE in this directory). */
/* modbus_core.c — Modbus RTU frame core implementation (contract=software/docs/Backplane_Bus_Protocol.md v0.2)
 * Pure C99, no HAL; complete frames in/out (framing = driver-layer UART idle interrupt's job); registers big-endian / CRC little-endian. */
#include "modbus_core.h"
#include <string.h>

uint16_t mb_crc16(const uint8_t *d, uint16_t n)
{
  uint16_t crc = 0xFFFFU;
  while (n--)
  {
    crc ^= *d++;
    for (int k = 0; k < 8; k++)
    {
      crc = (crc & 1U) ? ((crc >> 1) ^ 0xA001U) : (crc >> 1);
    }
  }
  return crc;
}

uint16_t mb_get16(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0] << 8) | p[1]); }
void     mb_put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)(v & 0xFFU); }

/* ---- ADU ---- */

int mb_adu_build(uint8_t *dst, uint16_t cap, uint8_t addr,
                 const uint8_t *pdu, uint16_t pdu_len)
{
  uint16_t total = (uint16_t)(1U + pdu_len + 2U);
  if ((pdu == 0) || (pdu_len == 0U) || (pdu_len > MB_PDU_MAX) || (cap < total)) { return -1; }
  dst[0] = addr;
  memcpy(&dst[1], pdu, pdu_len);
  uint16_t crc = mb_crc16(dst, (uint16_t)(1U + pdu_len));
  dst[1U + pdu_len] = (uint8_t)(crc & 0xFFU);          /* CRC low byte first */
  dst[2U + pdu_len] = (uint8_t)(crc >> 8);
  return (int)total;
}

int mb_adu_check(const uint8_t *adu, uint16_t len,
                 uint8_t *addr, const uint8_t **pdu, uint16_t *pdu_len)
{
  if ((len < 4U) || (len > MB_ADU_MAX)) { return -1; }  /* shortest = address+function code+CRC */
  uint16_t calc = mb_crc16(adu, (uint16_t)(len - 2U));
  uint16_t got  = (uint16_t)(adu[len - 2U] | ((uint16_t)adu[len - 1U] << 8));
  if (calc != got) { return -1; }
  *addr = adu[0];
  *pdu = &adu[1];
  *pdu_len = (uint16_t)(len - 3U);
  return 0;
}

/* ---- master-side request ---- */

int mb_req_read(uint8_t *pdu, uint8_t fc, uint16_t start, uint16_t qty)
{
  uint16_t lim;
  switch (fc)
  {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISC:  lim = MB_MAX_READ_BITS; break;
    case MB_FC_READ_HOLD:
    case MB_FC_READ_INPUT: lim = MB_MAX_READ_REGS; break;
    default: return -1;
  }
  if ((qty == 0U) || (qty > lim)) { return -1; }
  pdu[0] = fc;
  mb_put16(&pdu[1], start);
  mb_put16(&pdu[3], qty);
  return 5;
}

int mb_req_write_single(uint8_t *pdu, uint8_t fc, uint16_t addr, uint16_t val)
{
  if (fc == MB_FC_WRITE_COIL) { val = (val != 0U) ? 0xFF00U : 0x0000U; }
  else if (fc != MB_FC_WRITE_REG) { return -1; }
  pdu[0] = fc;
  mb_put16(&pdu[1], addr);
  mb_put16(&pdu[3], val);
  return 5;
}

int mb_req_write_coils(uint8_t *pdu, uint16_t start, uint16_t qty, const uint8_t *bits)
{
  if ((qty == 0U) || (qty > MB_MAX_WRITE_BITS) || (bits == 0)) { return -1; }
  uint8_t bc = (uint8_t)((qty + 7U) / 8U);
  pdu[0] = MB_FC_WRITE_COILS;
  mb_put16(&pdu[1], start);
  mb_put16(&pdu[3], qty);
  pdu[5] = bc;
  memcpy(&pdu[6], bits, bc);
  return (int)(6U + bc);
}

int mb_req_write_regs(uint8_t *pdu, uint16_t start, uint16_t qty, const uint16_t *vals)
{
  if ((qty == 0U) || (qty > MB_MAX_WRITE_REGS) || (vals == 0)) { return -1; }
  uint8_t bc = (uint8_t)(qty * 2U);
  pdu[0] = MB_FC_WRITE_REGS;
  mb_put16(&pdu[1], start);
  mb_put16(&pdu[3], qty);
  pdu[5] = bc;
  for (uint16_t i = 0; i < qty; i++) { mb_put16(&pdu[6U + 2U * i], vals[i]); }
  return (int)(6U + bc);
}

/* ---- master-side response parsing ---- */

int mb_rsp_is_exception(const uint8_t *pdu, uint16_t n, uint8_t *exc)
{
  if ((n >= 2U) && ((pdu[0] & MB_FC_EXC_FLAG) != 0U))
  {
    if (exc != 0) { *exc = pdu[1]; }
    return 1;
  }
  return 0;
}

int mb_rsp_regs(const uint8_t *pdu, uint16_t n, uint8_t fc, uint16_t *out, uint16_t max)
{
  if ((n < 2U) || (pdu[0] != fc)) { return -1; }
  uint8_t bc = pdu[1];
  if (((bc & 1U) != 0U) || (n != (uint16_t)(2U + bc))) { return -1; }
  uint16_t qty = (uint16_t)(bc / 2U);
  if (qty > max) { return -1; }
  for (uint16_t i = 0; i < qty; i++) { out[i] = mb_get16(&pdu[2U + 2U * i]); }
  return (int)qty;
}

int mb_rsp_bits(const uint8_t *pdu, uint16_t n, uint8_t fc, uint8_t *out, uint16_t max)
{
  if ((n < 2U) || (pdu[0] != fc)) { return -1; }
  uint8_t bc = pdu[1];
  if ((n != (uint16_t)(2U + bc)) || (bc > max)) { return -1; }
  memcpy(out, &pdu[2], bc);
  return (int)bc;
}

int mb_rsp_write_ok(const uint8_t *pdu, uint16_t n, uint8_t fc, uint16_t a, uint16_t b)
{
  if ((n != 5U) || (pdu[0] != fc)) { return -1; }
  if (fc == MB_FC_WRITE_COIL) { b = (b != 0U) ? 0xFF00U : 0x0000U; }
  return ((mb_get16(&pdu[1]) == a) && (mb_get16(&pdu[3]) == b)) ? 0 : -1;
}

/* ---- slave-side response ---- */

int mb_slv_exception(uint8_t *pdu, uint8_t req_fc, uint8_t exc)
{
  pdu[0] = (uint8_t)(req_fc | MB_FC_EXC_FLAG);
  pdu[1] = exc;
  return 2;
}

int mb_slv_rsp_regs(uint8_t *pdu, uint8_t fc, const uint16_t *vals, uint16_t qty)
{
  if ((qty == 0U) || (qty > MB_MAX_READ_REGS) || (vals == 0)) { return -1; }
  pdu[0] = fc;
  pdu[1] = (uint8_t)(qty * 2U);
  for (uint16_t i = 0; i < qty; i++) { mb_put16(&pdu[2U + 2U * i], vals[i]); }
  return (int)(2U + qty * 2U);
}

int mb_slv_rsp_bits(uint8_t *pdu, uint8_t fc, const uint8_t *bits, uint16_t qty)
{
  if ((qty == 0U) || (qty > MB_MAX_READ_BITS) || (bits == 0)) { return -1; }
  uint8_t bc = (uint8_t)((qty + 7U) / 8U);
  pdu[0] = fc;
  pdu[1] = bc;
  memcpy(&pdu[2], bits, bc);
  return (int)(2U + bc);
}
