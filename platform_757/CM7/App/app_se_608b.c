/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_se_608b.c — ATECC608B secure-element backend (production board; currently a placeholder stub)
 *
 * ⚠️ Fill in the implementation during production-board bring-up. Hardware: ATECC608B on I2C3
 *   (on the board's I2C3, PH7/8).
 *   CRYPTO_EN=PH6 enable. Default address 0x60 (SE050 shares the bus at 0x48, choose which to place per BOM).
 *
 * Production provisioning (one-time, see docs/Production_Security_Checklist.md):
 *   1. Generate the ECC P-256 key pair on-chip (GenKey; private key goes into the slot and never leaves the chip);
 *   2. Export the public key -> have your CA sign a device certificate (or register the public key in the shipping list);
 *   3. Lock the config zone + data zone (Lock); from then on the private key cannot be changed or read.
 *
 * This file needs to implement (using ATECC commands; suggest porting a trimmed Microchip CryptoAuthLib):
 *   app_se_probe:  Wake + Info command, confirm the chip is present and locked;
 *   app_se_sign:   Nonce (load challenge into TempKey) -> Sign (sign TempKey with the slot private key) -> read back 64B R||S.
 * I2C timing note: the 608B needs a Wake pulse (SDA held low >60us) + 1.5ms settle after wake-up.
 */
#include "app_se.h"

#if defined(APP_SE_BACKEND_608B)

/* Implemented in app_sec_test.c (software-emulated I2C 25kHz + CryptoAuth command layer)
 * provisioning fully done: slot0 P-256 private key generated on-chip and locked, public key =
 * keys/anticlone/dev_se.pub (unique to this board). */
extern int atecc_sign(const uint8_t dgst[32], uint8_t sig64[64]);
extern int atecc_pubkey(uint8_t pub64[64]);

int app_se_probe(void)
{
  uint8_t p[64];
  return (atecc_pubkey(p) == 0) ? APP_SE_OK : APP_SE_ABSENT;
}

int app_se_sign(const uint8_t challenge[32], uint8_t sig64[64])
{
  return (atecc_sign(challenge, sig64) == 0) ? APP_SE_OK : APP_SE_ERR;
}

#endif /* APP_SE_BACKEND_608B */
