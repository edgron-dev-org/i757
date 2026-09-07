/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_pwrfail_test.c — Three-layer power-fail retention mechanism implementation (API = app_pwrfail.h)
 * Hardware: PJ4 = LM393 power-fail interrupt (both edges, polarity self-learned at boot); BKPSRAM 0x38800000/4K (MPU Region5 non-cacheable
 * = write-through iron rule); save area = last 2MB of QFLASH, slot rotation (slot = 3 sectors 12KB, 170 slots).
 * The platform's own ledger lives entirely in RTC backup registers (BKP0~3R, VBAT-fed), the whole 4KB BKPSRAM belongs to the application (resource table commitment). */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "app_qflash.h"
#include "app_pwrfail.h"

/* ---- Slot layout: page0 = header (written last = natural tear protection), page1.. = data ---- */
#define SLOT_SECT   3U
#define SLOT_BYTES  (SLOT_SECT * QFLASH_SECTOR)          /* 12KB */
#define NSLOT       (QFLASH_PF_SIZE / SLOT_BYTES)        /* 170 */
#define BX_MAGIC    0x58424650UL                          /* 'PFBX' */

typedef struct {
  uint32_t magic, seq;
  uint16_t len, flags;                                    /* flags bit0=dryrun */
  uint32_t us_total;
  uint16_t sum;                                           /* data sum16 */
  uint16_t rsv;
} bx_hdr_t;                                               /* 20B, occupies page0 exclusively */

/* ---- RTC backup register roles (platform ledger, VBAT-fed) ---- */
#define BKP_BATT   (RTC->BKP0R)   /* battery-present magic 'PFBT' */
#define BKP_RETAIN (RTC->BKP1R)   /* retention-region-initialized magic 'PFRT' */
#define BKP_LASTUS (RTC->BKP2R)   /* most recent power-fail event detect→blackbox complete µs */
#define PF_BATT_MAGIC   0x50464254UL
#define PF_RETAIN_MAGIC 0x50465254UL

extern uint32_t __bkpsram_start, __bkpsram_end;

static uint8_t  s_powergood = 1, s_batt_ok = 0, s_retain_valid = 0;
static volatile uint8_t s_fired = 0;
static app_pf_hook_t s_hooks[8];
static uint8_t  s_nhook = 0;
static const uint8_t *s_bx_buf = 0;
static uint16_t s_bx_len = 0;
static uint32_t s_seq_next = 1;                           /* next record sequence number */
static uint32_t s_slot_next = 0;
static uint8_t  s_slot_ready = 0;                         /* next slot pre-erased */
static uint8_t  s_erase_left = 0;                         /* sectors remaining in background pre-erase */
static uint32_t s_last_seq = 0, s_last_slot = 0;          /* most recent valid record */

/* Self-proving example (also the usage template for application developers): if retention region valid, +1 each power-up */
PF_RETAIN static uint32_t s_retain_boots;

static inline uint32_t cyc(void) { return DWT->CYCCNT; }
#define CYC_PER_US 480U

static uint16_t sum16(const uint8_t *d, uint32_t n)
{
  uint32_t s = 0;
  for (uint32_t i = 0; i < n; i++) { s += d[i]; }
  return (uint16_t)s;
}

uint8_t  app_pf_battery_ok(void)   { return s_batt_ok; }
uint8_t  app_pf_retain_valid(void) { return s_retain_valid; }
uint32_t app_pf_last_event_us(void){ return (BKP_LASTUS == 0xFFFFFFFFUL) ? 0U : BKP_LASTUS; }

int app_pf_hook_register(app_pf_hook_t fn)
{
  if ((fn == 0) || (s_nhook >= 8U)) { return -1; }
  s_hooks[s_nhook++] = fn;
  return 0;
}

int app_pf_blackbox_set(const void *buf, uint16_t len)
{
  if ((buf == 0) || (len == 0U) || (len > APP_PF_BLACKBOX_MAX)) { return -1; }
  s_bx_buf = (const uint8_t *)buf;
  s_bx_len = len;
  return 0;
}

/* ---- Dying routine (EXTI highest priority / pftest dry run; no RTOS, no printf) ---- */
static void pf_dying(uint32_t dry)
{
  uint32_t t0 = cyc();
  for (uint8_t i = 0; i < s_nhook; i++) { s_hooks[i](); }          /* Layer 1: application copies live state */
  if ((s_bx_buf != 0) && s_slot_ready)                              /* Layer 2: blackbox transfer */
  {
    uint32_t base = QFLASH_PF_BASE + s_slot_next * SLOT_BYTES;
    bx_hdr_t h;
    app_qflash_dying_prep();
    (void)app_qflash_write(base + QFLASH_PAGE, s_bx_buf, s_bx_len); /* data page written first */
    h.magic = BX_MAGIC; h.seq = s_seq_next;
    h.len = s_bx_len; h.flags = (uint16_t)(dry & 1U);
    h.us_total = (cyc() - t0) / CYC_PER_US;
    h.sum = sum16(s_bx_buf, s_bx_len); h.rsv = 0;
    (void)app_qflash_write(base, &h, sizeof(h));                    /* header written last = record-complete proof */
    BKP_LASTUS = (cyc() - t0) / CYC_PER_US;
    s_last_seq = s_seq_next; s_last_slot = s_slot_next;
    s_seq_next++;
    s_slot_next = (s_slot_next + 1U) % NSLOT;
    s_slot_ready = 0;                                               /* survival path (dry run / power restored) re-pre-erased by poll */
    s_erase_left = SLOT_SECT;
  }
}

void EXTI4_IRQHandler(void)
{
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
  uint32_t lvl = (HAL_GPIO_ReadPin(GPIOJ, GPIO_PIN_4) == GPIO_PIN_SET) ? 1U : 0U;
  if ((lvl != s_powergood) && !s_fired)
  {
    s_fired = 1;
    pf_dying(0);
    s_fired = 0;
  }
}

static int slot_hdr(uint32_t idx, bx_hdr_t *h)
{
  return app_qflash_read(QFLASH_PF_BASE + idx * SLOT_BYTES, h, sizeof(*h));
}

static void app_pf_demo_install(void);   /* example at end of file */

void app_pf_init(void)
{
  GPIO_InitTypeDef g = {0};
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_BKPRAM_CLK_ENABLE();
  HAL_PWREx_EnableBkUpReg();
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* battery-present self-proof + Layer 0 retention region adjudication */
  s_batt_ok = (BKP_BATT == PF_BATT_MAGIC) ? 1U : 0U;
  BKP_BATT = PF_BATT_MAGIC;
  s_retain_valid = (s_batt_ok && (BKP_RETAIN == PF_RETAIN_MAGIC)) ? 1U : 0U;
  if (!s_retain_valid)
  {
    uint32_t *p = &__bkpsram_start;                       /* first boot / battery replaced: whole-region zeroing handed to the application */
    while (p < &__bkpsram_end) { *p++ = 0; }
    BKP_RETAIN = PF_RETAIN_MAGIC;
  }
  s_retain_boots++;                                       /* example variable: monotonically increasing within its validity */
  /* Blackbox slot scan: find the largest seq */
  {
    bx_hdr_t h;
    uint32_t best = 0;
    for (uint32_t i = 0; i < NSLOT; i++)
    {
      if ((slot_hdr(i, &h) == 0) && (h.magic == BX_MAGIC) && (h.seq > best))
      {
        best = h.seq; s_last_seq = h.seq; s_last_slot = i;
      }
    }
    s_seq_next = best + 1U;
    s_slot_next = (best == 0U) ? 0U : ((s_last_slot + 1U) % NSLOT);
    s_slot_ready = 0;
    s_erase_left = SLOT_SECT;                             /* poll pre-erases the next slot */
  }
  /* PJ4 power-fail interrupt */
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  g.Pin = GPIO_PIN_4; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOJ, &g);
  HAL_Delay(2);
  s_powergood = (HAL_GPIO_ReadPin(GPIOJ, GPIO_PIN_4) == GPIO_PIN_SET) ? 1U : 0U;
  g.Mode = GPIO_MODE_IT_RISING_FALLING;
  HAL_GPIO_Init(GPIOJ, &g);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  printf("[PF] armed(PJ4=%u) battery=%s retain=%s boots=%lu bbox: last seq=%lu next slot=%lu\n\r",
         s_powergood, s_batt_ok ? "OK" : "absent/first",
         s_retain_valid ? "valid" : "cleared",
         (unsigned long)s_retain_boots, (unsigned long)s_last_seq, (unsigned long)s_slot_next);
  app_pf_demo_install();
}

/* ---- Platform built-in example (= application-side usage template: copy it and swap in your own process data) ----
 * The Layer 1 hook copies the volatile live state (tick/uptime) into the snapshot struct, Layer 2 registers that struct as the blackbox payload */
static struct { uint32_t magic, tick, uptime_s, retain_boots; } s_demo;

static void demo_hook(void)              /* dying callback: only write memory, no RTOS/printf */
{
  s_demo.magic        = 0x4F4D4544UL;    /* 'DEMO' */
  s_demo.tick         = HAL_GetTick();
  s_demo.uptime_s     = HAL_GetTick() / 1000U;
  s_demo.retain_boots = s_retain_boots;
}

static void app_pf_demo_install(void)
{
  (void)app_pf_hook_register(demo_hook);
  (void)app_pf_blackbox_set(&s_demo, sizeof(s_demo));
}

void app_pf_poll(void)   /* defaultTask every tick: pre-erase next slot, ≤1 sector per tick (45ms typical, does not block the tick) */
{
  if (s_erase_left > 0U)
  {
    uint32_t base = QFLASH_PF_BASE + s_slot_next * SLOT_BYTES;
    if (app_qflash_erase4k(base + (uint32_t)(SLOT_SECT - s_erase_left) * QFLASH_SECTOR) == 0)
    {
      s_erase_left--;
      if (s_erase_left == 0U) { s_slot_ready = 1; }
    }
  }
}

int app_pf_blackbox_last(void *dst, uint16_t cap)
{
  bx_hdr_t h;
  if (s_last_seq == 0U) { return 0; }
  if ((slot_hdr(s_last_slot, &h) != 0) || (h.magic != BX_MAGIC)) { return 0; }
  uint16_t n = (h.len > cap) ? cap : h.len;
  static uint8_t tmp[APP_PF_BLACKBOX_MAX];
  if (app_qflash_read(QFLASH_PF_BASE + s_last_slot * SLOT_BYTES + QFLASH_PAGE, tmp, h.len) != 0) { return 0; }
  if (sum16(tmp, h.len) != h.sum) { return -1; }          /* torn / bad record */
  memcpy(dst, tmp, n);
  return (int)n;
}

/* ---- CLI diagnostics ---- */
int app_pf_cli(char *line)
{
  if (strcmp(line, "pfstat") == 0)
  {
    printf("battery=%s retain=%s boots=%lu hooks=%u bbox_buf=%uB slot next=%lu ready=%u last seq=%lu evt=%luus\n\r",
           s_batt_ok ? "OK" : "absent/first", s_retain_valid ? "valid" : "cleared",
           (unsigned long)s_retain_boots, s_nhook, s_bx_len,
           (unsigned long)s_slot_next, s_slot_ready, (unsigned long)s_last_seq,
           (unsigned long)app_pf_last_event_us());
    return 1;
  }
  if (strcmp(line, "pftest") == 0)
  {
    if ((s_bx_buf == 0) || (!s_slot_ready)) { printf("pftest: %s\n\r", (s_bx_buf == 0) ? "no bbox buf registered" : "slot not pre-erased yet"); return 1; }
    pf_dying(1);
    printf("dry-run done: seq=%lu us=%lu\n\r", (unsigned long)s_last_seq, (unsigned long)app_pf_last_event_us());
    return 1;
  }
  if (strcmp(line, "pfreport") == 0)
  {
    bx_hdr_t h;
    if ((s_last_seq == 0U) || (slot_hdr(s_last_slot, &h) != 0) || (h.magic != BX_MAGIC))
    {
      printf("no blackbox record\n\r");
      return 1;
    }
    static uint8_t buf[APP_PF_BLACKBOX_MAX];
    int n = app_pf_blackbox_last(buf, sizeof(buf));
    printf("last record: seq=%lu slot=%lu len=%u %s us=%lu sum=%s\n\r  head:",
           (unsigned long)h.seq, (unsigned long)s_last_slot, h.len,
           (h.flags & 1U) ? "(dry-run)" : "(REAL power-fail)",
           (unsigned long)h.us_total, (n > 0) ? "OK" : "BAD");
    for (int i = 0; (i < 16) && (i < n); i++) { printf(" %02X", buf[i]); }
    printf("\n\r");
    return 1;
  }
  return 0;
}
