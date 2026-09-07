/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_anticlone.c — anti-clone challenge-response verification (user file)
 *
 * ONE identity system (2026-07-24 redesign): the device's identity CERTIFICATE is the
 * anti-clone anchor — no separate per-board embedded public key any more (the old
 * app_anticlone_pub_pem scheme broke on the universal image: every board has a different
 * SE key, so a single baked-in key can only ever match one board).
 *
 * Flow: ① verify the identity certificate against the embedded CA (proves the cert —
 *       and therefore its public key — was blessed by us at provisioning);
 *       ② HW RNG 32B random challenge -> app_se_sign (SE signs with its internal private
 *       key, key never leaves the chip) -> verify against the CERT's public key.
 * Pass = this chip holds the private key matching a CA-signed identity. A cloner copying
 * flash (cert included) fails ②; a cloner with his own SE fails ① (no CA signature).
 * The random challenge prevents replay. No identity / no CA / no SE -> ABSENT (inert).
 *
 * ⚠️ Production-board hardening points (do not just return the result, see docs/Production_Security_Checklist.md):
 *   - Entangle the verification logic with real functionality (let the result feed into key computations / decrypt key parameters),
 *     not a single if(!ok)halt — that is trivially located in a disassembly and broken by a one-byte patch; bury it in many places and actually "use" the result.
 *   - Pair with RDP2 (firmware unreadable = the check cannot be modified); a pure software check alone will not stop someone who can modify the firmware.
 */
#include "app_anticlone.h"
#include "app_se.h"
#include "app_certs.h"
#include "app_identity.h"
#include "app_init.h"       /* app_rng */
#include "mbedtls/pk.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/x509_crt.h"

int app_anticlone_verify(void)
{
  uint8_t challenge[32];
  uint8_t sig64[64];
  int r;

  if (app_se_probe() != APP_SE_OK) { return APP_AC_ABSENT; }
  size_t idlen = 0;
  const unsigned char *idcert = app_identity_cert(&idlen);
  if ((idcert == NULL) || (app_tls_ca_len <= 1U)) { return APP_AC_ABSENT; }   /* unprovisioned board / stub build: feature inert */

  /* ① identity certificate must chain to our CA (locally, independent of the cloud) */
  mbedtls_x509_crt crt, ca;
  mbedtls_x509_crt_init(&crt);
  mbedtls_x509_crt_init(&ca);
  uint32_t flags = 0;
  r = mbedtls_x509_crt_parse(&crt, idcert, idlen);
  if (r == 0) { r = mbedtls_x509_crt_parse(&ca, (const unsigned char *)app_tls_ca_pem, app_tls_ca_len); }
  if (r == 0) { r = mbedtls_x509_crt_verify(&crt, &ca, NULL, NULL, &flags, NULL, NULL); }

  /* ② random challenge -> SE signs -> verify against the certificate's public key */
  if (r == 0) { r = (app_rng(NULL, challenge, sizeof(challenge)) == 0) ? 0 : -1; }
  if (r == 0) { r = (app_se_sign(challenge, sig64) == APP_SE_OK) ? 0 : -1; }
  if (r == 0)
  {
    mbedtls_mpi sr, ss;
    mbedtls_mpi_init(&sr);
    mbedtls_mpi_init(&ss);
    r = mbedtls_mpi_read_binary(&sr, sig64, 32);
    if (r == 0) { r = mbedtls_mpi_read_binary(&ss, sig64 + 32, 32); }
    if (r == 0)
    {
      mbedtls_ecp_keypair *kp = mbedtls_pk_ec(crt.pk);
      r = (kp != NULL) ? mbedtls_ecdsa_verify(&kp->grp, challenge, sizeof(challenge), &kp->Q, &sr, &ss) : -1;
    }
    mbedtls_mpi_free(&sr);
    mbedtls_mpi_free(&ss);
  }
  mbedtls_x509_crt_free(&crt);
  mbedtls_x509_crt_free(&ca);
  return (r == 0) ? APP_AC_GENUINE : APP_AC_CLONE;
}
