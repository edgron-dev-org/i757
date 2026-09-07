/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_identity.h — device identity partition (one universal firmware image, per-board certificate)
 *
 * The device certificate is DATA, not code: the production jig writes it once into
 * littlefs ("identity.der", raw DER) over the USB-C console; every board then runs the
 * SAME hex. Priority at boot:
 *   1. littlefs identity.der  (provisioned boards)
 *   2. build-embedded app_tls_devcert_pem  (dev boards / legacy per-board builds)
 *   3. none -> unprovisioned: cloud stays silent, everything local keeps working
 * The matching PRIVATE key is never stored anywhere: it lives inside the ATECC608A
 * (TLS handshake signing is routed there by MBEDTLS_ECDSA_SIGN_ALT).
 *
 * Production flow (tools/provision.ps1 drives these console commands):
 *   sec608cfg / sec608lock / sec608gen / sec608lockdata   -- one-time 608A key ceremony
 *   id-pub               -> board prints the 608A slot0 public key (128 hex chars)
 *   (PC: CA signs a certificate for that public key, CN = serial number)
 *   id-cert <hex-DER>    -> board checks the cert's public key against the 608A,
 *                           stores identity.der, prints the CN
 */
#ifndef APP_IDENTITY_H
#define APP_IDENTITY_H
#include <stddef.h>

#define APP_ID_NONE      0   /* no usable certificate (stub build, unprovisioned board) */
#define APP_ID_EMBEDDED  1   /* using the build-embedded certificate */
#define APP_ID_LFS       2   /* using the provisioned identity partition */

void app_identity_init(void);   /* early littlefs mount + load; call after app_tls_init, before TLS config creation */
const unsigned char *app_identity_cert(size_t *len);  /* active device cert (DER or PEM; len has mbedTLS parse semantics) */
const unsigned char *app_identity2_cert(size_t *len); /* customer identity cert (DER, slot CFG_SE_CUSTOMER_SLOT); NULL = not provisioned */
int  app_identity_source(void);                       /* APP_ID_* */
int  app_identity_cn(char *out, size_t cap);          /* subject CN of the active cert; 0=OK, -1=none */
int  app_identity_cli(char *line);                    /* id / id-pub / id-cert <hex> / id-clear yes / fwkey2 ...; 1=consumed */

/* ---- customer firmware-signing key (2026-09-07; OTA_Update_and_Brick_Proofing.md §2 gate ②bis) ----
 * A second OTA signature-verification key. Registered ONLY over the USB console (`fwkey2 set`,
 * deliberately not a cloud command: whoever holds the board decides whom it trusts), after which
 * the board accepts firmware signed by EITHER the build-embedded Edgron key OR this key.
 * Stored raw (65-byte uncompressed P-256 point 0x04||X||Y) in littlefs "fwkey2.bin". */
#define APP_FWKEY2_LEN 65U
const unsigned char *app_fwkey2(size_t *len);  /* registered customer key (raw point) or NULL; lock-free read from the tcpip thread */
const char *app_fwkey2_fp(void);               /* fingerprint = first 4 bytes of SHA256(raw key) as 8 hex chars; "-" = none */

#endif
