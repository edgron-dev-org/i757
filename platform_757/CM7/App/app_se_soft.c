/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_se_soft.c — software-emulated secure-element backend (development/test only; no real protection)
 *
 * ⚠️ Emulates the 608B's signing with a dev private key compiled into the firmware — the key sits in flash
 *    and a cloner can read it out, so this backend has "zero anti-clone value"; it only serves to verify the
 *    challenge-response flow is correct. The production board does not compile this file (uses the 608B backend).
 * APP_SE_SIMULATE_CLONE macro: when defined, signs with the wrong slot key -> upper-layer verification fails
 *    -> simulates "a clone board being caught".
 */
#include "app_se.h"

#if defined(APP_SE_BACKEND_SOFT)

#include <string.h>
#include "app_certs.h"
#include "app_init.h"       /* app_rng */
#include "mbedtls/pk.h"
#include "mbedtls/ecdsa.h"

int app_se_probe(void)
{
  return APP_SE_OK;   /* software backend is always "present" */
}

int app_se_sign(const uint8_t challenge[32], uint8_t sig64[64])
{
  mbedtls_pk_context pk;
  mbedtls_mpi r, s;
  int ret;
  mbedtls_pk_init(&pk);
  mbedtls_mpi_init(&r);
  mbedtls_mpi_init(&s);

#if defined(APP_SE_SIMULATE_CLONE)
  /* Clone simulation: sign with a key not matching the verification public key (the device TLS private key) -> verification must fail */
  ret = mbedtls_pk_parse_key(&pk, (const uint8_t *)app_tls_devkey_pem, app_tls_devkey_len, NULL, 0);
#else
  ret = mbedtls_pk_parse_key(&pk, (const uint8_t *)app_se_dev_key_pem, app_se_dev_key_len, NULL, 0);
#endif
  if (ret == 0)
  {
    mbedtls_ecp_keypair *kp = mbedtls_pk_ec(pk);
    ret = mbedtls_ecdsa_sign(&kp->grp, &r, &s, &kp->d, challenge, 32, app_rng, NULL);
  }
  if (ret == 0)
  {
    /* Output 64B raw R||S (same as ATECC608B Sign), each 32B big-endian */
    if (mbedtls_mpi_write_binary(&r, sig64, 32) != 0) { ret = APP_SE_ERR; }
    if (mbedtls_mpi_write_binary(&s, sig64 + 32, 32) != 0) { ret = APP_SE_ERR; }
  }
  mbedtls_mpi_free(&r);
  mbedtls_mpi_free(&s);
  mbedtls_pk_free(&pk);
  return (ret == 0) ? APP_SE_OK : APP_SE_ERR;
}

#endif /* APP_SE_BACKEND_SOFT */
