/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_se.h — Secure Element abstraction interface (user file)
 *
 * This is the "anti-clone" hardware abstraction layer: a safe that can only sign and whose private key never leaves the chip.
 *   - Production board: ATECC608B (I2C3), private key generated/used on-chip, unreadable even by a physical probe -> app_se_608b.c
 *   - Development/test: software emulation (dev key, no real protection, flow verification only) -> app_se_soft.c
 * The backend is selected by the CMake macro APP_SE_BACKEND; the upper layer (app_anticlone) does not care which.
 *
 * Challenge-response: the upper layer issues a 32B random challenge -> app_se_sign has the SE sign with its internal private key -> the upper layer verifies with the embedded public key.
 * A cloner copies the firmware + swaps in a 608B, but that 608B lacks your private key => signature fails verification => firmware judges it a clone.
 */
#ifndef APP_SE_H
#define APP_SE_H
#include <stdint.h>
#include <stddef.h>

#define APP_SE_OK          0
#define APP_SE_ABSENT     -1   /* No secure element detected (not fitted, or a fault) */
#define APP_SE_ERR        -2   /* Communication/operation failure */

int app_se_probe(void);        /* Probe whether the SE is present; APP_SE_OK / APP_SE_ABSENT */

/* Sign a 32B digest with the SE's internal private key; output 64B raw r||s (ATECC608B Sign native format).
 * challenge: 32B; sig64: output buffer (>=64B). Returns APP_SE_OK / APP_SE_ERR */
int app_se_sign(const uint8_t challenge[32], uint8_t sig64[64]);

#endif
