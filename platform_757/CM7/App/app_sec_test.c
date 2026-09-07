/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_sec_test.c — dual crypto-chip first test v2: software-emulated I2C
 * Drops the hardware I2C3 peripheral (too many uncertain factors during debugging), pure GPIO bit-bang:
 * PH7=SCL PH8=SDA, open-drain output + internal pull-up (no external pull-ups on the board = hardware
 * erratum, add 4.7k×2 next revision), ~25kHz steady-state rate.
 * Netlist: U6=ATECC608A (always powered) U7=SE050E2 (ENA=PH6 active-high), both on this bus. */
#include <stdio.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/ecp.h"

#define SCL_PIN GPIO_PIN_7
#define SDA_PIN GPIO_PIN_8

static inline void dly(void)                 /* half-bit ~20µs => ~25kHz */
{
  uint32_t t0 = DWT->CYCCNT;
  while ((DWT->CYCCNT - t0) < (20U * 480U)) {}
}
static inline void scl(uint8_t h) { HAL_GPIO_WritePin(GPIOH, SCL_PIN, h ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static inline void sda(uint8_t h) { HAL_GPIO_WritePin(GPIOH, SDA_PIN, h ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static inline uint8_t sda_rd(void) { return (HAL_GPIO_ReadPin(GPIOH, SDA_PIN) == GPIO_PIN_SET) ? 1U : 0U; }
static inline uint8_t scl_rd(void) { return (HAL_GPIO_ReadPin(GPIOH, SCL_PIN) == GPIO_PIN_SET) ? 1U : 0U; }

static void sw_init(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOH_CLK_ENABLE();
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  scl(1); sda(1);
  g.Pin = SCL_PIN | SDA_PIN;
  g.Mode = GPIO_MODE_OUTPUT_OD;              /* open-drain+pull-up: write 1=release (pulled up), write 0=drive low; IDR readable anytime */
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &g);
  dly();
}

static void i2c_start(void) { sda(1); scl(1); dly(); sda(0); dly(); scl(0); dly(); }
static void i2c_stop(void)  { sda(0); scl(1); dly(); sda(1); dly(); }

static uint8_t i2c_wbyte(uint8_t b)          /* returns 1=ACK */
{
  for (int i = 7; i >= 0; i--)
  {
    sda((uint8_t)((b >> i) & 1U));
    dly(); scl(1); dly(); scl(0);
  }
  sda(1); dly(); scl(1); dly();
  uint8_t ack = (sda_rd() == 0U) ? 1U : 0U;
  scl(0); dly();
  return ack;
}

static uint8_t i2c_rbyte(uint8_t ack)
{
  uint8_t b = 0;
  sda(1);
  for (int i = 7; i >= 0; i--)
  {
    dly(); scl(1); dly();
    b = (uint8_t)((b << 1) | sda_rd());
    scl(0);
  }
  sda(ack ? 0U : 1U);
  dly(); scl(1); dly(); scl(0); sda(1); dly();
  return b;
}

static int probe(uint8_t a7)                 /* START+addr(W)+STOP; 1=ACK */
{
  i2c_start();
  uint8_t ack = i2c_wbyte((uint8_t)(a7 << 1));
  i2c_stop();
  return ack;
}

/* ---- CryptoAuth command layer (608A; ATCA CRC16=poly 0x8005 LSB-first) ---- */
static uint16_t atca_crc(const uint8_t *d, uint8_t n)
{
  uint16_t crc = 0;
  for (uint8_t i = 0; i < n; i++)
  {
    for (uint8_t b = 1; b != 0U; b = (uint8_t)(b << 1))
    {
      uint8_t din = ((d[i] & b) != 0U) ? 1U : 0U;
      uint8_t msb = (uint8_t)((crc >> 15) & 1U);
      crc = (uint16_t)(crc << 1);
      if (din != msb) { crc ^= 0x8005U; }
    }
  }
  return crc;
}

static void a608_wake(void)
{
  sda(0);
  for (int i = 0; i < 5; i++) { dly(); }   /* ≥60µs low */
  sda(1);
  HAL_Delay(2);                            /* tWHI 1.5ms */
}

/* Send command packet + read response after tEXEC; returns response data length (excluding count/crc), <0=failure */
static int a608_cmd(uint8_t op, uint8_t p1, uint16_t p2,
                    uint8_t exec_ms, uint8_t *rsp, uint8_t cap)
{
  uint8_t pkt[8];
  pkt[0] = 0x03;                           /* word addr: command */
  pkt[1] = 7;                              /* count..crc */
  pkt[2] = op; pkt[3] = p1;
  pkt[4] = (uint8_t)(p2 & 0xFFU); pkt[5] = (uint8_t)(p2 >> 8);
  uint16_t c = atca_crc(&pkt[1], 5);
  pkt[6] = (uint8_t)c; pkt[7] = (uint8_t)(c >> 8);
  i2c_start();
  uint8_t ok = i2c_wbyte(0x60U << 1);
  for (int i = 0; ok && (i < 8); i++) { ok = i2c_wbyte(pkt[i]); }
  i2c_stop();
  if (!ok) { return -1; }
  HAL_Delay(exec_ms);
  i2c_start();
  if (!i2c_wbyte((0x60U << 1) | 1U)) { i2c_stop(); return -2; }
  uint8_t cnt = i2c_rbyte(1);
  if ((cnt < 4U) || (cnt > (uint8_t)(cap + 3U))) { i2c_rbyte(0); i2c_stop(); return -3; }
  uint8_t buf[72];   /* must be ≥ cap+3 (Sign/GenKey response cnt=67); original buf[40] overflowed 27B and smashed the stack = r5 wild-pointer crash */
  buf[0] = cnt;
  for (uint8_t i = 1; i < cnt; i++) { buf[i] = i2c_rbyte(i < (uint8_t)(cnt - 1U)); }
  i2c_stop();
  uint16_t want = (uint16_t)(buf[cnt - 2U] | ((uint16_t)buf[cnt - 1U] << 8));
  if (atca_crc(buf, (uint8_t)(cnt - 2U)) != want) { return -4; }
  uint8_t n = (uint8_t)(cnt - 3U);
  if (n > cap) { n = cap; }
  memcpy(rsp, &buf[1], n);
  return (int)n;
}

static void a608_idle(void)
{
  i2c_start(); (void)i2c_wbyte(0x60U << 1); (void)i2c_wbyte(0x02); i2c_stop();
}

/* Command carrying a data body (used for Nonce passthrough): [03][count][op][p1][p2lo][p2hi][data..][crclo][crchi] */
static int a608_cmd_data(uint8_t op, uint8_t p1, uint16_t p2, const uint8_t *data, uint8_t dlen,
                         uint8_t exec_ms, uint8_t *rsp, uint8_t cap)
{
  uint8_t pkt[80];
  uint8_t cnt = (uint8_t)(7U + dlen);
  pkt[0] = 0x03; pkt[1] = cnt; pkt[2] = op; pkt[3] = p1;
  pkt[4] = (uint8_t)p2; pkt[5] = (uint8_t)(p2 >> 8);
  if (dlen) { memcpy(&pkt[6], data, dlen); }
  uint16_t c = atca_crc(&pkt[1], (uint8_t)(cnt - 2U));
  pkt[cnt - 1U] = (uint8_t)c; pkt[cnt] = (uint8_t)(c >> 8);
  i2c_start();
  uint8_t ok = i2c_wbyte(0x60U << 1);
  for (int i = 0; ok && (i <= cnt); i++) { ok = i2c_wbyte(pkt[i]); }
  i2c_stop();
  if (!ok) { return -1; }
  HAL_Delay(exec_ms);
  i2c_start();
  if (!i2c_wbyte((0x60U << 1) | 1U)) { i2c_stop(); return -2; }
  uint8_t rc = i2c_rbyte(1);
  if ((rc < 4U) || (rc > (uint8_t)(cap + 3U))) { i2c_rbyte(0); i2c_stop(); return -3; }
  uint8_t buf[72];
  buf[0] = rc;
  for (uint8_t i = 1; i < rc; i++) { buf[i] = i2c_rbyte(i < (uint8_t)(rc - 1U)); }
  i2c_stop();
  uint16_t want = (uint16_t)(buf[rc - 2U] | ((uint16_t)buf[rc - 1U] << 8));
  if (atca_crc(buf, (uint8_t)(rc - 2U)) != want) { return -4; }
  uint8_t n = (uint8_t)(rc - 3U);
  if (n > cap) { n = cap; }
  memcpy(rsp, &buf[1], n);
  return (int)n;
}

/* Exported: sign a 32B digest with a slot's internal private key -> 64B R||S.
 * slot 0 = factory identity; CFG_SE_CUSTOMER_SLOT = customer identity (config blueprint
 * leaves slots 1..7 P-256/GenKey-open for the customer, contract §5bis). */
int atecc_sign_slot(uint16_t slot, const uint8_t dgst[32], uint8_t sig64[64])
{
  int n;
  sw_init();
  a608_wake();
  n = a608_cmd_data(0x16, 0x03, 0x0000, dgst, 32, 7, sig64, 64);   /* Nonce passthrough -> TempKey */
  if (n < 0) { a608_idle(); return -1; }
  n = a608_cmd(0x41, 0x80, slot, 90, sig64, 64);                   /* Sign external, keyID=slot */
  a608_idle();
  return (n == 64) ? 0 : -2;
}
int atecc_sign(const uint8_t dgst[32], uint8_t sig64[64]) { return atecc_sign_slot(0, dgst, sig64); }

/* Exported: read a slot's public key (GenKey mode 0 = derive from existing private key, no regeneration) */
int atecc_pubkey_slot(uint16_t slot, uint8_t pub64[64])
{
  int n;
  sw_init();
  a608_wake();
  n = a608_cmd(0x40, 0x00, slot, 90, pub64, 64);
  a608_idle();
  return (n == 64) ? 0 : -1;
}
int atecc_pubkey(uint8_t pub64[64]) { return atecc_pubkey_slot(0, pub64); }

/* Exported: GENERATE a new P-256 key pair inside a slot (GenKey mode 0x04) -> public key.
 * DESTRUCTIVE: the slot's previous private key is gone forever. Works on slots the config
 * blueprint left GenKey-open (1..7) even after the data zone is locked; slot 0 is SlotLocked
 * at production and the chip itself refuses regeneration there. */
int atecc_genkey_slot(uint16_t slot, uint8_t pub64[64])
{
  int n;
  sw_init();
  a608_wake();
  n = a608_cmd(0x40, 0x04, slot, 120, pub64, 64);
  a608_idle();
  return (n == 64) ? 0 : -1;
}

static void sec_608_identify(void)
{
  uint8_t r[32];
  a608_wake();
  int n = a608_cmd(0x30, 0x00, 0x0000, 5, r, sizeof(r));    /* Info(Revision) */
  if (n == 4) { printf("608A rev: %02X %02X %02X %02X\n\r", r[0], r[1], r[2], r[3]); }
  else { printf("608A info fail (%d)\n\r", n); }
  n = a608_cmd(0x02, 0x80, 0x0000, 5, r, sizeof(r));        /* Read32 config[0]: contains SN */
  if (n == 32)
  {
    printf("608A SN: %02X%02X%02X%02X %02X%02X%02X%02X%02X (SN[0..1]=0123/SN[8]=EE=%s)\n\r",
           r[0], r[1], r[2], r[3], r[8], r[9], r[10], r[11], r[12],
           ((r[0] == 0x01U) && (r[1] == 0x23U) && (r[12] == 0xEEU)) ? "OK" : "odd");
  }
  else { printf("608A read cfg fail (%d)\n\r", n); }
  /* Lock status: config block2 bytes 86/87 (LockValue/LockConfig) — read block2 again */
  n = a608_cmd(0x02, 0x80, 0x0010, 5, r, sizeof(r));        /* Read32 config block2: address=word-addressed, block2=word16(0x10) */
  if (n == 32)
  {
    printf("608A lock: LockValue=%02X LockConfig=%02X (55=unlocked/configurable, 00=locked)\n\r", r[22], r[23]);
  }
  a608_idle();
}

/* ---- provisioning: config blueprint (contract `Anti_Clone_and_Secure_Element_Selection.md`§5bis) ----
 * bytes16-127 target values; slot0=platform identity (GenKey + secret + single-slot lockable), 1~7=customer P-256 fully open,
 * 8=416B data store freely read/write, 9~15=customer small slots (data type). bytes84-87 hardware-unwritable = skipped. */
static const uint8_t s_cfg_blue[112] = {
  /* 16-19: I2C=0xC0, rsv, CountMatch=0, ChipMode=0 */
  0xC0, 0x00, 0x00, 0x00,
  /* 20-51: SlotConfig[0..15] LE: slot0=0x2083; 1~7=0x2083; 8=0x0000; 9~15=0x0000 */
  0x83, 0x20,  0x83, 0x20,  0x83, 0x20,  0x83, 0x20,
  0x83, 0x20,  0x83, 0x20,  0x83, 0x20,  0x83, 0x20,
  0x00, 0x00,  0x00, 0x00,  0x00, 0x00,  0x00, 0x00,
  0x00, 0x00,  0x00, 0x00,  0x00, 0x00,  0x00, 0x00,
  /* 52-59: Counter0/1 defaults */
  0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
  /* 60-67: Counter1 + UseLock */
  0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
  /* 68-83: VolatileKey/SecureBoot etc. all default 0 */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  /* 84-87: UserExtra/lock bytes (unwritable, placeholder not sent) */
  0x00, 0x00, 0x00, 0x00,
  /* 88-91: SlotLocked=FFFF (all unlocked), ChipOptions=0 */
  0xFF, 0xFF, 0x00, 0x00,
  /* 92-95: X509format */
  0x00, 0x00, 0x00, 0x00,
  /* 96-127: KeyConfig[0..15] LE: 0~7=0x0033 (P256 private key+PubInfo+lockable); 8~15=0x001C (data type) */
  0x33, 0x00,  0x33, 0x00,  0x33, 0x00,  0x33, 0x00,
  0x33, 0x00,  0x33, 0x00,  0x33, 0x00,  0x33, 0x00,
  0x1C, 0x00,  0x1C, 0x00,  0x1C, 0x00,  0x1C, 0x00,
  0x1C, 0x00,  0x1C, 0x00,  0x1C, 0x00,  0x1C, 0x00,
};

static int a608_write4(uint16_t word, const uint8_t *d4)   /* Write(0x12) config 4B */
{
  uint8_t pkt[13];
  pkt[0] = 0x03; pkt[1] = 11; pkt[2] = 0x12; pkt[3] = 0x00;
  pkt[4] = (uint8_t)word; pkt[5] = (uint8_t)(word >> 8);
  memcpy(&pkt[6], d4, 4);
  uint16_t c = atca_crc(&pkt[1], 9);
  pkt[10] = (uint8_t)c; pkt[11] = (uint8_t)(c >> 8);
  i2c_start();
  uint8_t ok = i2c_wbyte(0x60U << 1);
  for (int i = 0; ok && (i < 12); i++) { ok = i2c_wbyte(pkt[i]); }
  i2c_stop();
  if (!ok) { return -1; }
  HAL_Delay(26);
  i2c_start();
  if (!i2c_wbyte((0x60U << 1) | 1U)) { i2c_stop(); return -2; }
  uint8_t cnt = i2c_rbyte(1), st = i2c_rbyte(1);
  i2c_rbyte(1); i2c_rbyte(0);
  i2c_stop();
  return ((cnt == 4U) && (st == 0x00U)) ? 0 : (int)(0x100U + st);
}

static void sec_608_cfg(void)
{
  uint8_t r[32];
  int fails = 0;
  a608_wake();
  for (uint16_t w = 4; w < 32U; w++)          /* words 4..31 = bytes16..127; word21 (84-87) skipped */
  {
    if (w == 21U) { continue; }
    int rc = a608_write4(w, &s_cfg_blue[(w - 4U) * 4U]);
    if (rc != 0) { printf("cfg word %u FAIL(%d)\n\r", w, rc); fails++; }
  }
  printf("config write: %s\n\r", fails ? "HAS FAILS" : "all ok");
  printf("read-back 128B (after review, run 'sec608lock yes' to lock):\n\r");
  for (uint16_t b = 0; b < 4U; b++)
  {
    int n = a608_cmd(0x02, 0x80, (uint16_t)(b << 3), 5, r, sizeof(r));
    if (n == 32)
    {
      printf("  [%03u]", b * 32U);
      for (int i = 0; i < 32; i++) { printf(" %02X", r[i]); }
      printf("\n\r");
    }
  }
  a608_idle();
}

static void sec_608_lock_config(void)
{
  uint8_t r[4];
  a608_wake();
  int n = a608_cmd(0x17, 0x80, 0x0000, 35, r, sizeof(r));   /* Lock config, CRC-skip mode */
  printf("LockConfig: %s (rc=%d st=%02X)\n\r",
         ((n == 1) && (r[0] == 0x00U)) ? "LOCKED" : "FAIL", n, (n > 0) ? r[0] : 0xFF);
  a608_idle();
}

static void sec_608_lock_data(void)
{
  uint8_t r[4];
  a608_wake();
  int n = a608_cmd(0x17, 0x81, 0x0000, 35, r, sizeof(r));   /* Lock data zone */
  printf("LockData: %s (rc=%d st=%02X)\n\r",
         ((n == 1) && (r[0] == 0x00U)) ? "LOCKED" : "FAIL", n, (n > 0) ? r[0] : 0xFF);
  a608_idle();
}

static void sec_608_lock_slot0(void)
{
  /* Individual slot lock (works AFTER the config/data zone locks — that is its purpose):
   * permanently disables GenKey-create and writes on slot0, Sign stays allowed. Field
   * measurement 2026-07-24: without this, GenKey could still REGENERATE the identity key
   * on a fully zone-locked chip (board-2 provisioning re-run silently swapped the key). */
  uint8_t r[4];
  a608_wake();
  int n = a608_cmd(0x17, 0x82, 0x0000, 35, r, sizeof(r));   /* Lock: zone=slot(0b10), slot=0, CRC-skip */
  printf("LockSlot0: %s (rc=%d st=%02X)\n\r",
         ((n == 1) && (r[0] == 0x00U)) ? "LOCKED" : "FAIL", n, (n > 0) ? r[0] : 0xFF);
  a608_idle();
}

static void sec_608_genkey(void)
{
  uint8_t pub[64];
  a608_wake();
  int n = a608_cmd(0x40, 0x04, 0x0000, 120, pub, sizeof(pub));  /* GenKey slot0, returns 64B public key */
  if (n == 64)
  {
    printf("slot0 pubkey X: ");
    for (int i = 0; i < 32; i++) { printf("%02X", pub[i]); }
    printf("\n\rslot0 pubkey Y: ");
    for (int i = 32; i < 64; i++) { printf("%02X", pub[i]); }
    printf("\n\r");
  }
  else { printf("GenKey FAIL (%d)\n\r", n); }
  a608_idle();
}

static void sec_608_signtest(void)
{
  uint8_t dgst[32], sig[64], pub[64];
  for (int i = 0; i < 32; i++) { dgst[i] = (uint8_t)(0x5AU ^ i); }   /* fixed digest = reproducible */
  if (atecc_pubkey(pub) != 0) { printf("pubkey read FAIL\n\r"); return; }
  if (atecc_sign(dgst, sig) != 0) { printf("sign FAIL\n\r"); return; }
  printf("sig R: "); for (int i = 0; i < 32; i++) { printf("%02X", sig[i]); }
  printf("\n\rsig S: "); for (int i = 32; i < 64; i++) { printf("%02X", sig[i]); } printf("\n\r");
  /* mbedtls verifies using the 608A public key just read out */
  {
    extern int app_rng(void *, unsigned char *, size_t);
    mbedtls_ecdsa_context ctx;
    mbedtls_mpi r, s;
    mbedtls_ecdsa_init(&ctx);
    mbedtls_mpi_init(&r); mbedtls_mpi_init(&s);
    int rc = mbedtls_ecp_group_load(&ctx.grp, MBEDTLS_ECP_DP_SECP256R1);
    uint8_t point[65]; point[0] = 0x04; memcpy(&point[1], pub, 64);
    if (rc == 0) { rc = mbedtls_ecp_point_read_binary(&ctx.grp, &ctx.Q, point, 65); }
    if (rc == 0) { rc = mbedtls_mpi_read_binary(&r, sig, 32); }
    if (rc == 0) { rc = mbedtls_mpi_read_binary(&s, sig + 32, 32); }
    if (rc == 0) { rc = mbedtls_ecdsa_verify(&ctx.grp, dgst, 32, &ctx.Q, &r, &s); }
    printf("mbedtls verify: %s (rc=%d)\n\r", (rc == 0) ? "PASS = hardware key signs, pubkey verifies" : "FAIL", rc);
    mbedtls_mpi_free(&r); mbedtls_mpi_free(&s); mbedtls_ecdsa_free(&ctx);
  }
}

int app_sec_cli(char *line)
{
  if (strcmp(line, "sec608sign") == 0) { sec_608_signtest(); return 1; }
  if (strcmp(line, "sec608cfg") == 0) { sw_init(); sec_608_cfg(); return 1; }
  if (strcmp(line, "sec608lock yes") == 0) { sw_init(); sec_608_lock_config(); return 1; }
  if (strcmp(line, "sec608lockdata yes") == 0) { sw_init(); sec_608_lock_data(); return 1; }
  if (strcmp(line, "sec608lockslot yes") == 0) { sw_init(); sec_608_lock_slot0(); return 1; }
  if (strcmp(line, "sec608gen") == 0) { sw_init(); sec_608_genkey(); return 1; }
  if (strcmp(line, "sec608") == 0)
  {
    sw_init();
    sec_608_identify();
    return 1;
  }
  if (strcmp(line, "sec") != 0) { return 0; }
  printf("--- crypto chips first contact (soft I2C @25kHz) ---\n\r");
  sw_init();
  printf("idle: SCL=%u SDA=%u\n\r", scl_rd(), sda_rd());
  /* SE050 enable (netlist: PH6 goes only to SE050.ENA, active-high) */
  {
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_6; g.Mode = GPIO_MODE_OUTPUT_PP; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &g);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(50);
  }
  {
    int found = 0;
    printf("scan:");
    for (uint8_t a = 0x08; a < 0x78U; a++)
    {
      if (probe(a)) { printf(" 0x%02X", a); found++; }
    }
    printf("%s\n\r", found ? "" : " (none)");
  }
  /* 608A: wake = drive SDA low 100µs -> release -> 2ms -> read 4B (genuine fingerprint 04 11 33 43) */
  {
    sda(0);
    for (int i = 0; i < 5; i++) { dly(); }
    sda(1);
    HAL_Delay(2);
    i2c_start();
    if (i2c_wbyte((uint8_t)((0x60U << 1) | 1U)))
    {
      uint8_t r[4];
      for (int i = 0; i < 4; i++) { r[i] = i2c_rbyte(i < 3); }
      i2c_stop();
      printf("608A wake: %02X %02X %02X %02X %s\n\r", r[0], r[1], r[2], r[3],
             ((r[0] == 0x04U) && (r[1] == 0x11U)) ? "= GENUINE wake-ack" : "(unexpected)");
    }
    else
    {
      i2c_stop();
      printf("608A wake: addr 0x60 no ACK\n\r");
    }
    printf("post-wake ");
    int found = 0;
    printf("scan:");
    for (uint8_t a = 0x08; a < 0x78U; a++) { if (probe(a)) { printf(" 0x%02X", a); found++; } }
    printf("%s\n\r", found ? "" : " (none)");
  }
  printf("SE050 @0x48: %s (ENA=PH6 held high)\n\r", probe(0x48) ? "ACK" : "nack");
  return 1;
}
