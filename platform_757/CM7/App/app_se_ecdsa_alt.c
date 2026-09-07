/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_se_ecdsa_alt.c — MBEDTLS_ECDSA_SIGN_ALT implementation: forward ECDSA signing to the 608A
 *
 * Why: the device identity private key is locked in 608A slot0 (generated on-chip, physically unreadable).
 *   mbedTLS 2.16 has no opaque pk, but the only place the device needs to "sign with the private key"
 *   is the CertificateVerify of the TLS handshake (client authentication).
 *   ECDSA_SIGN_ALT globally replaces mbedtls_ecdsa_sign, ignores the software private key d passed in,
 *   and always routes through the 608A.
 *   → The handshake signature is produced by the chip private key; the VPS verifies with the device
 *   certificate (= 608A public key, CA-signed) = true hardware mTLS identity.
 *   A cloner's 608A lacks our private key → signature fails verification → VPS refuses the connection.
 *
 * Preconditions: no MBEDTLS_ECDSA_DETERMINISTIC (otherwise the _det branch bypasses
 *   this function); no other ECDSA sign call on the device (fwsign/anticlone/ota are all verify).
 * Threading/timing: handshake runs on the tcpip thread; the 608A soft-I2C sign busy-waits ~100ms,
 *   acceptable once per handshake (IWDG 32s is fed by defaultTask, unaffected). The placeholder devkey's d
 *   is not used (altcp only needs the parse to succeed). */
#include "mbedtls/ecdsa.h"
#include <string.h>

extern int atecc_sign_slot(uint16_t slot, const uint8_t dgst[32], uint8_t sig64[64]);

/* Which 608A slot signs the NEXT handshake: 0 = factory identity (default), or
 * CFG_SE_CUSTOMER_SLOT when the connection presents the customer identity cert
 * (identity2.der). Set by app_mqtt right before each connect attempt; one MQTT
 * client + handshake and setup both on the tcpip thread = no race. */
static volatile uint16_t s_sign_slot = 0;
void app_se_sign_slot_set(uint16_t slot) { s_sign_slot = slot; }

/* 608A handshake-signature observability: +1 each time the 608A completes one TLS handshake ECDSA sign
 * (reported by heartbeat/dash). Since ECDSA_SIGN_ALT is the device's only sign path, count>0 and broker
 * online = this connection's handshake was signed by the 608A and accepted by the VPS server (otherwise
 * the handshake fails and cannot connect). app_se_hs608_count() is read by the heartbeat. */
static volatile uint32_t s_hs608 = 0;
static volatile uint32_t s_hs608_fail = 0;
uint32_t app_se_hs608_count(void) { return s_hs608; }
uint32_t app_se_hs608_fail(void)  { return s_hs608_fail; }

int mbedtls_ecdsa_sign(mbedtls_ecp_group *grp, mbedtls_mpi *r, mbedtls_mpi *s,
                       const mbedtls_mpi *d, const unsigned char *buf, size_t blen,
                       int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
  uint8_t dgst[32], sig[64];
  size_t n = (blen < 32U) ? blen : 32U;   /* SHA256=32; if digest is shorter, left-align and zero-pad (ECDSA takes the leftmost bits) */
  (void)grp; (void)d; (void)f_rng; (void)p_rng;
  memset(dgst, 0, sizeof(dgst));
  memcpy(dgst, buf, n);
  if (atecc_sign_slot(s_sign_slot, dgst, sig) != 0) { s_hs608_fail++; return MBEDTLS_ERR_ECP_HW_ACCEL_FAILED; }
  int ret = mbedtls_mpi_read_binary(r, sig, 32);
  if (ret == 0) { ret = mbedtls_mpi_read_binary(s, sig + 32, 32); }
  if (ret == 0) { s_hs608++; }
  return ret;
}
