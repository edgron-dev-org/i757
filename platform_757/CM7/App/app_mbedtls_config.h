/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_mbedtls_config.h — mbedTLS 2.16 trimmed config (user file; applied via CMake MBEDTLS_CONFIG_FILE)
 * Goal: TLS1.2 client, single suite ECDHE-ECDSA-AES128-GCM-SHA256, P-256, X.509+PEM
 * Decision: do NOT define MBEDTLS_HAVE_TIME* => skip certificate validity-period checks
 *   —— certs are all year 9999 + "brick-proof first", avoids SNTP dependency and the
 *      "power-on clock 1970 earlier than notBefore" pitfall
 * Memory: built-in buffer allocator uses a dedicated static pool (initialized in app_init.c),
 *   never touches the newlib/FreeRTOS heap */
#ifndef APP_MBEDTLS_CONFIG_H
#define APP_MBEDTLS_CONFIG_H

/* Platform */
#define MBEDTLS_PLATFORM_C
#define MBEDTLS_HAVE_TIME               /* needed by altcp session struct; timestamp only, cert validity-period check (HAVE_TIME_DATE) still off */
#define MBEDTLS_PLATFORM_MEMORY
#define MBEDTLS_MEMORY_BUFFER_ALLOC_C   /* static-pool allocator: single-threaded (tcpip) use, lock-free OK */
#define MBEDTLS_MEMORY_DEBUG            /* enable cur/max pool-usage getters: field forensics for the TLS-pool-exhaustion wedge (2026-07-24, app_mqtt offline watchdog autopsy). A few size_t counters, negligible cost. */
#define MBEDTLS_NO_PLATFORM_ENTROPY     /* no /dev/urandom */
#define MBEDTLS_ENTROPY_HARDWARE_ALT    /* H7 hardware RNG, implemented in app_init.c */

/* RNG */
#define MBEDTLS_ENTROPY_C
#define MBEDTLS_CTR_DRBG_C

/* Symmetric / hash */
#define MBEDTLS_AES_C
#define MBEDTLS_GCM_C
#define MBEDTLS_CIPHER_C
#define MBEDTLS_MD_C
#define MBEDTLS_SHA256_C

/* Elliptic curve */
#define MBEDTLS_BIGNUM_C
#define MBEDTLS_ECP_C
#define MBEDTLS_ECDH_C
#define MBEDTLS_ECDSA_C
#define MBEDTLS_ECDSA_SIGN_ALT    /* device-side ECDSA signing forwarded to 608A (private key in chip).
                                     ecdsa.c's built-in mbedtls_ecdsa_sign is excluded by this macro, implementation=app_se_ecdsa_alt.c.
                                     The device's only sign = TLS handshake CertificateVerify -> 608A. */
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
#define MBEDTLS_ECP_NIST_OPTIM

/* RSA: our certs are all ECC, but the AWS server cert chain is RSA-2048 (Amazon Root CA 1 / intermediate CA)——
 * verifying AWS server identity needs RSA. Device side still uses ECDSA certs, only server verification goes via RSA. */
#define MBEDTLS_RSA_C
#define MBEDTLS_PKCS1_V15

/* ASN.1 / PEM / X.509 / PK */
#define MBEDTLS_ASN1_PARSE_C
#define MBEDTLS_ASN1_WRITE_C
#define MBEDTLS_BASE64_C
#define MBEDTLS_PEM_PARSE_C
#define MBEDTLS_OID_C
#define MBEDTLS_PK_C
#define MBEDTLS_PK_PARSE_C
#define MBEDTLS_X509_USE_C
#define MBEDTLS_X509_CRT_PARSE_C

/* TLS 1.2 client */
#define MBEDTLS_SSL_TLS_C
#define MBEDTLS_SSL_CLI_C
#define MBEDTLS_SSL_PROTO_TLS1_2
/* SNI: must be on, otherwise the client won't send the server_name extension in ClientHello (mbedtls_ssl_set_hostname only sets the hostname used for verification).
 * AWS IoT routes by SNI: missing SNI => TLS completes but the front end refuses entry to the MQTT broker */
#define MBEDTLS_SSL_SERVER_NAME_INDICATION
#define MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED   /* our VPS: ECC server cert */
#define MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED     /* AWS: RSA server cert (client still uses ECDSA cert) */
#define MBEDTLS_SSL_CIPHERSUITES \
  MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256, \
  MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256

/* Record buffers: application data is only 1KB chunks, but the AWS handshake CertificateRequest (a long list of acceptable CAs) in a single record
 * can reach ~5-16K, and 4K causes INVALID_RECORD. IN must be large enough to hold the AWS handshake record; OUT we only send small packets */
#define MBEDTLS_SSL_IN_CONTENT_LEN   8192
#define MBEDTLS_SSL_OUT_CONTENT_LEN  2048

#include "mbedtls/check_config.h"

#endif /* APP_MBEDTLS_CONFIG_H */
