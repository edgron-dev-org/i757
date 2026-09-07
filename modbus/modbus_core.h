/* SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Edgron. Licensed under the Apache License, Version 2.0 (see LICENSE in this directory). */
/* modbus_core.h — Modbus RTU frame core (CRC/ADU/PDU frame build & parse)
 * Contract = software/docs/Backplane_Bus_Protocol.md v0.2 (backplane application spec); PDU layer is standard Modbus,
 * same core serves: front-panel 6×485 master stack / backplane master-slave / future Modbus TCP (drop ADU, swap in MBAP).
 * Pure C99, no HAL/RTOS deps; framing relies on UART idle interrupt (driver-layer job), this core handles complete frames only.
 * Byte order: register values big-endian on the wire (Modbus convention), CRC little-endian on the wire. */
#ifndef MODBUS_CORE_H
#define MODBUS_CORE_H
#include <stdint.h>

#define MB_ADU_MAX      256U            /* Standard RTU ADU upper limit (address+PDU+CRC) */
#define MB_PDU_MAX      (MB_ADU_MAX - 3U)
#define MB_ADDR_BCAST   0U              /* Broadcast: write-class function codes only, slave does not reply */

/* Function codes (subset used by this project) */
#define MB_FC_READ_COILS      0x01U
#define MB_FC_READ_DISC       0x02U
#define MB_FC_READ_HOLD       0x03U
#define MB_FC_READ_INPUT      0x04U
#define MB_FC_WRITE_COIL      0x05U
#define MB_FC_WRITE_REG       0x06U
#define MB_FC_WRITE_COILS     0x0FU
#define MB_FC_WRITE_REGS      0x10U
#define MB_FC_OTA_BASE        0x41U     /* 0x41~0x48 user-defined: reserved for slave OTA */
#define MB_FC_EXC_FLAG        0x80U     /* Exception response flag bit */

/* Exception codes */
#define MB_EXC_ILLEGAL_FC     0x01U
#define MB_EXC_ILLEGAL_ADDR   0x02U
#define MB_EXC_ILLEGAL_VAL    0x03U
#define MB_EXC_DEVICE_FAIL    0x04U

/* Quantity limits (Modbus spec) */
#define MB_MAX_READ_BITS      2000U
#define MB_MAX_WRITE_BITS     1968U
#define MB_MAX_READ_REGS      125U
#define MB_MAX_WRITE_REGS     123U

uint16_t mb_crc16(const uint8_t *d, uint16_t n);      /* CRC-16/MODBUS */
uint16_t mb_get16(const uint8_t *p);                  /* Read 16-bit big-endian */
void     mb_put16(uint8_t *p, uint16_t v);            /* Write 16-bit big-endian */

/* ---- ADU layer (RTU: address+PDU+CRC) ---- */
/* Build ADU: returns total length (1+pdu_len+2); returns -1 on illegal args / insufficient capacity */
int mb_adu_build(uint8_t *dst, uint16_t cap, uint8_t addr,
                 const uint8_t *pdu, uint16_t pdu_len);
/* Validate complete frame (driver delivers per idle gap): returns 0 if CRC/length valid, yielding address and PDU view (points inside adu),
 * bad frame returns -1 (caller just discards; RTU has no re-sync concept—the next silent gap naturally restarts) */
int mb_adu_check(const uint8_t *adu, uint16_t len,
                 uint8_t *addr, const uint8_t **pdu, uint16_t *pdu_len);

/* ---- PDU layer: master-side request construction (returns PDU length, -1 on illegal args) ---- */
int mb_req_read(uint8_t *pdu, uint8_t fc, uint16_t start, uint16_t qty);          /* FC 01/02/03/04 */
int mb_req_write_single(uint8_t *pdu, uint8_t fc, uint16_t addr, uint16_t val);   /* FC 05(val=0/non-0)/06 */
int mb_req_write_coils(uint8_t *pdu, uint16_t start, uint16_t qty,
                       const uint8_t *bits);                                      /* FC 15, bits packed LSB-first */
int mb_req_write_regs(uint8_t *pdu, uint16_t start, uint16_t qty,
                      const uint16_t *vals);                                      /* FC 16 */

/* ---- PDU layer: master-side response parsing ---- */
/* Exception response? if so returns 1 and fills exception code */
int mb_rsp_is_exception(const uint8_t *pdu, uint16_t n, uint8_t *exc);
/* Read-register response (FC03/04): validates structure, converts big-endian to host order into out, returns register count; -1 on bad structure */
int mb_rsp_regs(const uint8_t *pdu, uint16_t n, uint8_t fc, uint16_t *out, uint16_t max);
/* Read-bit response (FC01/02): validates structure, copies out packed bytes, returns byte count; -1 on bad structure */
int mb_rsp_bits(const uint8_t *pdu, uint16_t n, uint8_t fc, uint8_t *out, uint16_t max);
/* Write-response echo check (FC05/06: address+value; FC15/16: start+quantity): returns 0 on match, else -1 */
int mb_rsp_write_ok(const uint8_t *pdu, uint16_t n, uint8_t fc, uint16_t a, uint16_t b);

/* ---- PDU layer: slave-side response construction (for slave firmware / loopback test) ---- */
int mb_slv_exception(uint8_t *pdu, uint8_t req_fc, uint8_t exc);                  /* Exception response (2B) */
int mb_slv_rsp_regs(uint8_t *pdu, uint8_t fc, const uint16_t *vals, uint16_t qty);/* FC03/04 response */
int mb_slv_rsp_bits(uint8_t *pdu, uint8_t fc, const uint8_t *bits, uint16_t qty); /* FC01/02 response (qty=bit count) */

int mb_selftest(int *total);            /* Power-on self-test: returns failure count (0=all pass), *total=number of cases */

#endif
