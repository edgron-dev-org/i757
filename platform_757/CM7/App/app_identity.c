/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_identity.c — device identity partition (see app_identity.h for the design).
 * Runs entirely on defaultTask (CLI + boot init); littlefs single-task discipline holds. */
#include <stdio.h>
#include <string.h>
#include "app_identity.h"
#include "app_certs.h"
#include "app_cfg.h"
#include "app_qflash.h"
#include "lfs.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/oid.h"
#include "mbedtls/base64.h"
#include "mbedtls/ecp.h"
#include "mbedtls/pk.h"
#include "mbedtls/sha256.h"

extern lfs_t *app_lfs(void);
extern int atecc_pubkey(unsigned char pub64[64]);   /* 608A slot0 public key (app_sec_test.c); !=0 when chip absent */
extern int atecc_pubkey_slot(uint16_t slot, unsigned char pub64[64]);
extern int atecc_genkey_slot(uint16_t slot, unsigned char pub64[64]);

#define ID_FILE      "identity.der"
#define ID2_FILE     "identity2.der"   /* customer identity: cert for the CFG_SE_CUSTOMER_SLOT key */
#define FWK2_FILE    "fwkey2.bin"      /* customer firmware-signing public key, raw 65-byte P-256 point */
#define ID_CERT_MAX  1024U          /* ECC P-256 device cert DER is ~600B; headroom for extensions */

static unsigned char s_cert[ID_CERT_MAX];   /* loaded identity.der */
static size_t        s_len = 0;
static int           s_src = APP_ID_NONE;
static unsigned char s_cert2[ID_CERT_MAX];  /* loaded identity2.der ("" = not provisioned) */
static size_t        s_len2 = 0;
static uint8_t       s_fbuf[256];
static const struct lfs_file_config s_fcfg = { .buffer = s_fbuf };
static unsigned char   s_fwk2[APP_FWKEY2_LEN];   /* customer firmware-signing public key (raw uncompressed point) */
static volatile size_t s_fwk2_len = 0;           /* written LAST on set / FIRST on clear: ota_verify_signature (tcpip thread) reads lock-free */
static char            s_fwk2_fp[9] = "-";

/* ---- helpers ---- */

/* subject CN of a parsed cert -> out; 0=OK */
static int cert_cn(const mbedtls_x509_crt *crt, char *out, size_t cap)
{
  const mbedtls_x509_name *n;
  for (n = &crt->subject; n != NULL; n = n->next)
  {
    if (MBEDTLS_OID_CMP(MBEDTLS_OID_AT_CN, &n->oid) == 0)
    {
      size_t l = (n->val.len < (cap - 1U)) ? n->val.len : (cap - 1U);
      memcpy(out, n->val.p, l);
      out[l] = 0;
      return 0;
    }
  }
  return -1;
}

/* parse the ACTIVE cert (DER from lfs, or embedded PEM) into crt; caller frees. 0=OK */
static int active_parse(mbedtls_x509_crt *crt)
{
  mbedtls_x509_crt_init(crt);
  if (s_src == APP_ID_LFS)
  {
    return mbedtls_x509_crt_parse_der(crt, s_cert, s_len);
  }
  if (s_src == APP_ID_EMBEDDED)
  {
    return mbedtls_x509_crt_parse(crt, (const unsigned char *)app_tls_devcert_pem, app_tls_devcert_len);
  }
  return -1;
}

static int hex_nib(char c)
{
  if ((c >= '0') && (c <= '9')) { return c - '0'; }
  if ((c >= 'a') && (c <= 'f')) { return c - 'a' + 10; }
  if ((c >= 'A') && (c <= 'F')) { return c - 'A' + 10; }
  return -1;
}

/* ---- public API ---- */

/* read + validate one DER cert file into buf; returns length or 0 */
static size_t id_file_load(const char *file, unsigned char *buf, size_t cap)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  lfs_ssize_t n;
  if ((fs == NULL) ||
      (lfs_file_opencfg(fs, &f, file, LFS_O_RDONLY, (struct lfs_file_config *)&s_fcfg) != 0))
  {
    return 0;
  }
  n = lfs_file_read(fs, &f, buf, cap);
  (void)lfs_file_close(fs, &f);
  if (n > 0)
  {
    mbedtls_x509_crt crt;
    mbedtls_x509_crt_init(&crt);
    int ok = (mbedtls_x509_crt_parse_der(&crt, buf, (size_t)n) == 0);
    mbedtls_x509_crt_free(&crt);
    if (ok) { return (size_t)n; }
    printf("[ID] %s corrupt -> ignored\n\r", file);
  }
  return 0;
}

/* ---- customer firmware-signing key ---- */

/* the raw 65-byte point must be a valid secp256r1 public key (rejects garbage, wrong curve, off-curve points) */
static int fwk2_valid(const unsigned char *raw, size_t n)
{
  mbedtls_ecp_group grp;
  mbedtls_ecp_point q;
  int ok;
  if ((n != APP_FWKEY2_LEN) || (raw[0] != 0x04U)) { return 0; }
  mbedtls_ecp_group_init(&grp);
  mbedtls_ecp_point_init(&q);
  ok = (mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1) == 0) &&
       (mbedtls_ecp_point_read_binary(&grp, &q, raw, n) == 0) &&
       (mbedtls_ecp_check_pubkey(&grp, &q) == 0);
  mbedtls_ecp_point_free(&q);
  mbedtls_ecp_group_free(&grp);
  return ok;
}

static void fwk2_fp_calc(void)
{
  unsigned char h[32];
  if (s_fwk2_len == 0U) { strcpy(s_fwk2_fp, "-"); return; }
  (void)mbedtls_sha256_ret(s_fwk2, s_fwk2_len, h, 0);
  snprintf(s_fwk2_fp, sizeof(s_fwk2_fp), "%02X%02X%02X%02X", h[0], h[1], h[2], h[3]);
}

static void fwk2_load(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  lfs_ssize_t n;
  unsigned char raw[APP_FWKEY2_LEN + 1U];   /* +1: a longer file is detected as corrupt, not truncated into "valid" */
  s_fwk2_len = 0;
  if ((fs != NULL) &&
      (lfs_file_opencfg(fs, &f, FWK2_FILE, LFS_O_RDONLY, (struct lfs_file_config *)&s_fcfg) == 0))
  {
    n = lfs_file_read(fs, &f, raw, sizeof(raw));
    (void)lfs_file_close(fs, &f);
    if ((n > 0) && fwk2_valid(raw, (size_t)n))
    {
      memcpy(s_fwk2, raw, APP_FWKEY2_LEN);
      s_fwk2_len = APP_FWKEY2_LEN;
    }
    else if (n > 0) { printf("[ID] %s corrupt -> ignored\n\r", FWK2_FILE); }
  }
  fwk2_fp_calc();
}

const unsigned char *app_fwkey2(size_t *len)
{
  size_t l = s_fwk2_len;
  *len = l;
  return (l > 0U) ? s_fwk2 : NULL;
}

const char *app_fwkey2_fp(void) { return s_fwk2_fp; }

void app_identity_init(void)
{
  /* early storage bring-up: QSPI + littlefs (idempotent; P5 self-test reuses the mount) */
  if (app_qflash_init() == 0xEF4019UL) { (void)app_lfs_mount(); }

  s_src = APP_ID_NONE;
  s_len = id_file_load(ID_FILE, s_cert, sizeof(s_cert));
  if (s_len > 0U) { s_src = APP_ID_LFS; }
  if ((s_src == APP_ID_NONE) && (app_tls_devcert_len > 1U))   /* stub build embeds "" (len 1) */
  {
    s_src = APP_ID_EMBEDDED;
  }
  s_len2 = id_file_load(ID2_FILE, s_cert2, sizeof(s_cert2));  /* customer identity (optional) */
  fwk2_load();                                                  /* customer OTA signing key (optional) */
  {
    char cn[32];
    printf("[ID] source=%s cn=%s id2=%s fwkey2=%s\n\r",
           (s_src == APP_ID_LFS) ? "lfs" : (s_src == APP_ID_EMBEDDED) ? "embedded" : "NONE(unprovisioned)",
           (app_identity_cn(cn, sizeof cn) == 0) ? cn : "-",
           (s_len2 > 0U) ? "yes" : "no", s_fwk2_fp);
  }
}

const unsigned char *app_identity_cert(size_t *len)
{
  if (s_src == APP_ID_LFS)      { *len = s_len; return s_cert; }
  if (s_src == APP_ID_EMBEDDED) { *len = app_tls_devcert_len; return (const unsigned char *)app_tls_devcert_pem; }
  *len = 0;
  return NULL;
}

const unsigned char *app_identity2_cert(size_t *len)   /* customer identity; NULL = not provisioned */
{
  if (s_len2 > 0U) { *len = s_len2; return s_cert2; }
  *len = 0;
  return NULL;
}

int app_identity_source(void) { return s_src; }

int app_identity_cn(char *out, size_t cap)
{
  mbedtls_x509_crt crt;
  int rc = active_parse(&crt);
  if (rc == 0) { rc = cert_cn(&crt, out, cap); }
  mbedtls_x509_crt_free(&crt);
  return (rc == 0) ? 0 : -1;
}

/* ---- CLI (production jig protocol; all prints single-line, machine-parsable) ---- */

static void id_show(void)
{
  char cn[32];
  printf("id: source=%s cn=%s len=%u\n\r",
         (s_src == APP_ID_LFS) ? "lfs" : (s_src == APP_ID_EMBEDDED) ? "embedded" : "none",
         (app_identity_cn(cn, sizeof cn) == 0) ? cn : "-",
         (unsigned)((s_src == APP_ID_LFS) ? s_len : app_tls_devcert_len));
}

static void id_pub(void)
{
  unsigned char pub[64];
  if (atecc_pubkey(pub) != 0) { printf("id-pub: FAIL (608A absent?)\n\r"); return; }
  printf("id-pub: ");
  for (int i = 0; i < 64; i++) { printf("%02X", pub[i]); }
  printf("\n\r");
}

/* hex-DER -> validate CN + "cert pubkey == this board's 608A slot key" -> write file + RAM copy.
 * slot: which 608A key the cert must certify (0=factory, CFG_SE_CUSTOMER_SLOT=customer).
 * require_chip: factory path tolerates an absent 608A (dev boards, warn+skip); the customer
 * identity is meaningless without the chip key, so id2-cert refuses outright. */
static void cert_write(const char *tag, const char *hex, const char *file, uint16_t slot,
                       int require_chip, unsigned char *ram, size_t *ram_len)
{
  static unsigned char der[ID_CERT_MAX];   /* static: 1KB doesn't belong on this stack */
  size_t n = 0;
  while (hex[2U * n] != 0)
  {
    int hi = hex_nib(hex[2U * n]);
    int lo = (hi >= 0) ? hex_nib(hex[2U * n + 1U]) : -1;
    if ((lo < 0) || (n >= sizeof(der))) { printf("%s: FAIL bad-hex\n\r", tag); return; }
    der[n++] = (unsigned char)((hi << 4) | lo);
  }
  if (n == 0U) { printf("%s: FAIL empty\n\r", tag); return; }

  /* parse + extract CN + pull the cert's public key for the chip cross-check */
  mbedtls_x509_crt crt;
  char cn[32];
  mbedtls_x509_crt_init(&crt);
  if (mbedtls_x509_crt_parse_der(&crt, der, n) != 0)
  {
    mbedtls_x509_crt_free(&crt);
    printf("%s: FAIL parse\n\r", tag);
    return;
  }
  if (cert_cn(&crt, cn, sizeof cn) != 0) { strcpy(cn, "-"); }

  /* safety: the cert must certify THIS board's chip key in THIS slot (prevents
   * cert/board mix-ups on the bench, and customer certs signed over the wrong key). */
  {
    unsigned char chip[64];
    if (atecc_pubkey_slot(slot, chip) == 0)
    {
      unsigned char certpub[65];
      size_t olen = 0;
      mbedtls_ecp_keypair *ec = mbedtls_pk_ec(crt.pk);
      if ((ec == NULL) ||
          (mbedtls_ecp_point_write_binary(&ec->grp, &ec->Q, MBEDTLS_ECP_PF_UNCOMPRESSED,
                                          &olen, certpub, sizeof certpub) != 0) ||
          (olen != 65U) || (memcmp(&certpub[1], chip, 64) != 0))
      {
        mbedtls_x509_crt_free(&crt);
        printf("%s: FAIL pubkey-mismatch (cert is not for this board's 608A slot %u)\n\r",
               tag, (unsigned)slot);
        return;
      }
    }
    else if (require_chip)
    {
      mbedtls_x509_crt_free(&crt);
      printf("%s: FAIL no-608A (customer identity needs the chip key)\n\r", tag);
      return;
    }
    else
    {
      printf("%s: warn no-608A, pubkey check skipped\n\r", tag);
    }
  }
  mbedtls_x509_crt_free(&crt);

  lfs_t *fs = app_lfs();
  lfs_file_t f;
  if ((fs == NULL) ||
      (lfs_file_opencfg(fs, &f, file, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                        (struct lfs_file_config *)&s_fcfg) != 0))
  {
    printf("%s: FAIL lfs\n\r", tag);
    return;
  }
  lfs_ssize_t w = lfs_file_write(fs, &f, der, n);
  (void)lfs_file_close(fs, &f);
  if (w != (lfs_ssize_t)n) { printf("%s: FAIL write\n\r", tag); return; }

  memcpy(ram, der, n);
  *ram_len = n;
  printf("%s: OK cn=%s len=%u (takes effect for TLS on next boot)\n\r", tag, cn, (unsigned)n);
}

/* print a cert as PEM on the console (copy-paste into a .crt file; needed to register the
 * board with a cloud, e.g. AWS `register-certificate-without-ca`). DER -> base64/64-col. */
static void cert_show(const char *tag, const unsigned char *cert, size_t len)
{
  if ((cert == NULL) || (len == 0U)) { printf("%s: none\n\r", tag); return; }
  if (cert[0] == '-')   /* embedded PEM string: print as-is */
  {
    printf("%.*s\n\r", (int)len, (const char *)cert);
    return;
  }
  static unsigned char b64[((ID_CERT_MAX + 2U) / 3U) * 4U + 4U];
  size_t olen = 0;
  if (mbedtls_base64_encode(b64, sizeof(b64), &olen, cert, len) != 0)
  {
    printf("%s: FAIL b64\n\r", tag);
    return;
  }
  printf("-----BEGIN CERTIFICATE-----\n\r");
  for (size_t i = 0; i < olen; i += 64U)
  {
    size_t l = ((olen - i) < 64U) ? (olen - i) : 64U;
    printf("%.*s\n\r", (int)l, &b64[i]);
  }
  printf("-----END CERTIFICATE-----\n\r");
}

static void id2_show(void)
{
  char cn[32] = "-";
  if (s_len2 > 0U)
  {
    mbedtls_x509_crt crt;
    mbedtls_x509_crt_init(&crt);
    if ((mbedtls_x509_crt_parse_der(&crt, s_cert2, s_len2) == 0)) { (void)cert_cn(&crt, cn, sizeof cn); }
    mbedtls_x509_crt_free(&crt);
  }
  printf("id2: slot=%u cert=%s cn=%s len=%u\n\r", (unsigned)CFG_SE_CUSTOMER_SLOT,
         (s_len2 > 0U) ? "lfs" : "none", cn, (unsigned)s_len2);
}

static void id2_pub(int generate)
{
  unsigned char pub[64];
  const char *tag = generate ? "id2-gen" : "id2-pub";
  int rc = generate ? atecc_genkey_slot(CFG_SE_CUSTOMER_SLOT, pub)
                    : atecc_pubkey_slot(CFG_SE_CUSTOMER_SLOT, pub);
  if (rc != 0) { printf("%s: FAIL (608A absent, or slot %u has no key yet — run id2-gen yes)\n\r",
                        tag, (unsigned)CFG_SE_CUSTOMER_SLOT); return; }
  if (generate && (s_len2 > 0U))
  {
    printf("id2-gen: warn slot key REPLACED -- the stored identity2 cert no longer matches, re-sign and id2-cert it\n\r");
  }
  printf("%s: ", tag);
  for (int i = 0; i < 64; i++) { printf("%02X", pub[i]); }
  printf("\n\r");
}

static void fwk2_show(void)
{
  printf("fwkey2: %s fp=%s (customer OTA signing key; Edgron key always verifies too)\n\r",
         (s_fwk2_len > 0U) ? "lfs" : "none", s_fwk2_fp);
}

/* fwkey2 set <hex>: either the 65-byte raw uncompressed point (130 hex chars) or the SubjectPublicKeyInfo
 * DER of the key (`openssl ec -in key.pem -pubout -outform DER | xxd -p`, 91 bytes). Both normalise to
 * the raw point on disk, so the verifier never parses ASN.1 at OTA time. */
static void fwk2_set(const char *hex)
{
  unsigned char in[128];
  unsigned char raw[APP_FWKEY2_LEN];
  size_t n = 0;
  while (hex[2U * n] != 0)
  {
    int hi = hex_nib(hex[2U * n]);
    int lo = (hi >= 0) ? hex_nib(hex[2U * n + 1U]) : -1;
    if ((lo < 0) || (n >= sizeof(in))) { printf("fwkey2: FAIL bad-hex\n\r"); return; }
    in[n++] = (unsigned char)((hi << 4) | lo);
  }
  if (n == APP_FWKEY2_LEN) { memcpy(raw, in, n); }
  else
  {
    mbedtls_pk_context pk;
    mbedtls_ecp_keypair *ec;
    size_t olen = 0;
    mbedtls_pk_init(&pk);
    if ((mbedtls_pk_parse_public_key(&pk, in, n) != 0) || ((ec = mbedtls_pk_ec(pk)) == NULL) ||
        (ec->grp.id != MBEDTLS_ECP_DP_SECP256R1) ||
        (mbedtls_ecp_point_write_binary(&ec->grp, &ec->Q, MBEDTLS_ECP_PF_UNCOMPRESSED,
                                        &olen, raw, sizeof(raw)) != 0) ||
        (olen != APP_FWKEY2_LEN))
    {
      mbedtls_pk_free(&pk);
      printf("fwkey2: FAIL parse (want 65-byte raw P-256 point or SPKI DER, as hex)\n\r");
      return;
    }
    mbedtls_pk_free(&pk);
  }
  if (!fwk2_valid(raw, sizeof(raw))) { printf("fwkey2: FAIL not a valid P-256 public key\n\r"); return; }

  lfs_t *fs = app_lfs();
  lfs_file_t f;
  if ((fs == NULL) ||
      (lfs_file_opencfg(fs, &f, FWK2_FILE, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                        (struct lfs_file_config *)&s_fcfg) != 0))
  {
    printf("fwkey2: FAIL lfs\n\r");
    return;
  }
  lfs_ssize_t w = lfs_file_write(fs, &f, raw, sizeof(raw));
  (void)lfs_file_close(fs, &f);
  if (w != (lfs_ssize_t)sizeof(raw)) { printf("fwkey2: FAIL write\n\r"); return; }
  s_fwk2_len = 0;                       /* publish order: retire the old key, copy, re-arm (tcpip reader sees old-or-new, never a mix) */
  memcpy(s_fwk2, raw, sizeof(raw));
  s_fwk2_len = APP_FWKEY2_LEN;
  fwk2_fp_calc();
  printf("fwkey2: OK fp=%s (effective immediately for ota-sign)\n\r", s_fwk2_fp);
}

int app_identity_cli(char *line)
{
  /* ---- customer firmware-signing key. USB console only BY CONSTRUCTION: this dispatcher is reached from
   * app_cli.c alone; the cloud command path (app_mqtt -> ota_cmd) has no route here. Keep it that way. ---- */
  if (strcmp(line, "fwkey2") == 0)            { fwk2_show(); return 1; }
  if (strncmp(line, "fwkey2 set ", 11) == 0)  { fwk2_set(&line[11]); return 1; }
  if (strcmp(line, "fwkey2 clear yes") == 0)
  {
    lfs_t *fs = app_lfs();
    s_fwk2_len = 0;
    fwk2_fp_calc();
    if ((fs != NULL) && (lfs_remove(fs, FWK2_FILE) == 0)) { printf("fwkey2: cleared (only the Edgron key verifies OTA now)\n\r"); }
    else { printf("fwkey2: FAIL (nothing stored?)\n\r"); }
    return 1;
  }
  if (strncmp(line, "fwkey2", 6) == 0)
  {
    printf("usage: fwkey2 | fwkey2 set <hex: 65-byte raw point or SPKI DER> | fwkey2 clear yes\n\r");
    return 1;
  }

  if (strcmp(line, "id") == 0)      { id_show(); return 1; }
  if (strcmp(line, "id-pub") == 0)  { id_pub(); return 1; }
  if (strncmp(line, "id-cert ", 8) == 0)
  {
    cert_write("id-cert", &line[8], ID_FILE, 0, 0, s_cert, &s_len);
    if (s_len > 0U) { s_src = APP_ID_LFS; }
    return 1;
  }
  if (strcmp(line, "id-cert-show") == 0)
  {
    size_t l = 0;
    const unsigned char *c = app_identity_cert(&l);
    cert_show("id-cert-show", c, l);
    return 1;
  }
  if (strcmp(line, "id-clear yes") == 0)
  {
    lfs_t *fs = app_lfs();
    if ((fs != NULL) && (lfs_remove(fs, ID_FILE) == 0)) { printf("id-clear: OK (reboot to fall back)\n\r"); }
    else { printf("id-clear: FAIL\n\r"); }
    return 1;
  }
  /* ---- customer identity (slot CFG_SE_CUSTOMER_SLOT; guide: Connect_Your_Own_AWS_IoT.md) ---- */
  if (strcmp(line, "id2") == 0)          { id2_show(); return 1; }
  if (strcmp(line, "id2-pub") == 0)      { id2_pub(0); return 1; }
  if (strcmp(line, "id2-gen yes") == 0)  { id2_pub(1); return 1; }   /* confirm word: destroys the previous slot key */
  if (strcmp(line, "id2-gen") == 0)
  {
    printf("id2-gen: generates a NEW key in slot %u and destroys the old one forever -- confirm with `id2-gen yes`\n\r",
           (unsigned)CFG_SE_CUSTOMER_SLOT);
    return 1;
  }
  if (strncmp(line, "id2-cert ", 9) == 0)
  {
    cert_write("id2-cert", &line[9], ID2_FILE, CFG_SE_CUSTOMER_SLOT, 1, s_cert2, &s_len2);
    return 1;
  }
  if (strcmp(line, "id2-cert-show") == 0)
  {
    cert_show("id2-cert-show", (s_len2 > 0U) ? s_cert2 : NULL, s_len2);
    return 1;
  }
  if (strcmp(line, "id2-clear yes") == 0)
  {
    lfs_t *fs = app_lfs();
    s_len2 = 0;
    if ((fs != NULL) && (lfs_remove(fs, ID2_FILE) == 0)) { printf("id2-clear: OK (reboot: TLS back to factory identity)\n\r"); }
    else { printf("id2-clear: FAIL\n\r"); }
    return 1;
  }
  return 0;
}
