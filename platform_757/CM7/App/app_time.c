/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_time.c — time service implementation (user file; RTC via bare registers, same as ADC3 precedent, does not touch CubeMX)
 * RAM anchor = authoritative time (unix anchor + tick delta); RTC only appears at two moments:
 *   boot —— if calendar is valid (INITS), seed the anchor (seamless time continuation after reset/OTA swap);
 *   SNTP sync —— also write a copy into the RTC.
 * Threads: app_time_sntp_cb runs on the tcpip thread (RTC register write ~hundreds of µs, non-blocking, no printf);
 *       the rest run on defaultTask. now() is lock-free: 32-bit reads/writes are naturally atomic, anchor update order = seconds then tick, error ≤1s is negligible. */
#include "app_time.h"
#include "app_mqtt.h"     /* mqtt_mark_dirty */
#include "stm32h7xx_hal.h"

static volatile uint32_t s_anchor_unix = 0;   /* anchor: unix seconds at the moment of sync */
static volatile uint32_t s_anchor_tick = 0;   /* anchor: HAL_GetTick at the moment of sync */
static volatile uint8_t  s_src = 0;           /* 0 not synced / 1 RTC continuation / 2 SNTP */
static uint8_t  s_rtc_ok = 0;

/* ---- calendar conversion (Howard Hinnant algorithm, no libc dependency) ---- */
static uint32_t days_from_civil(int y, unsigned m, unsigned d)
{
  y -= (m <= 2);
  unsigned era = (unsigned)y / 400U;          /* 2000~2099 always positive, negative eras not handled */
  unsigned yoe = (unsigned)y - era * 400U;
  unsigned doy = (153U * (m + ((m > 2U) ? (unsigned)-3 : 9U)) + 2U) / 5U + d - 1U;
  unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
  return era * 146097U + doe - 719468U;       /* days relative to 1970-01-01 */
}
static void civil_from_days(uint32_t days, int *y, unsigned *m, unsigned *d)
{
  uint32_t z = days + 719468U;
  unsigned era = z / 146097U;
  unsigned doe = z - era * 146097U;
  unsigned yoe = (doe - doe / 1460U + doe / 36524U - doe / 146096U) / 365U;
  int yy = (int)(yoe + era * 400U);
  unsigned doy = doe - (365U * yoe + yoe / 4U - yoe / 100U);
  unsigned mp = (5U * doy + 2U) / 153U;
  *d = doy - (153U * mp + 2U) / 5U + 1U;
  *m = mp + ((mp < 10U) ? 3U : (unsigned)-9);
  *y = yy + (int)(*m <= 2U);
}
static uint8_t to_bcd(unsigned v)   { return (uint8_t)(((v / 10U) << 4) | (v % 10U)); }
static unsigned from_bcd(uint8_t b) { return ((unsigned)(b >> 4)) * 10U + (b & 0x0FU); }

/* ---- RTC bare registers ---- */
static int rtc_write_calendar(uint32_t unix_s)   /* 0 = written, -1 = INIT mode never entered
                                                  * (2026-09-02: this used to fail SILENTLY —
                                                  * TR/DR writes outside INIT are ignored, the
                                                  * RTC free-ran and showed up 13 min slow in
                                                  * that day's boot-event timestamps) */
{
  int y; unsigned mo, d;
  int ok = 0;
  uint32_t days = unix_s / 86400U, rem = unix_s % 86400U;
  unsigned hh = rem / 3600U, mi = (rem % 3600U) / 60U, ss = rem % 60U;
  civil_from_days(days, &y, &mo, &d);
  if ((y < 2000) || (y > 2099)) { return -1; }   /* RTC year field 00~99 (base 2000) */
  unsigned wdu = ((days + 3U) % 7U) + 1U;     /* 1=Monday..7=Sunday (1970-01-01=Thursday) */
  RTC->WPR = 0xCAU; RTC->WPR = 0x53U;         /* unlock write protection */
  RTC->ISR |= RTC_ISR_INIT;
  { uint32_t t = 1000000U; while (((RTC->ISR & RTC_ISR_INITF) == 0U) && (t > 0U)) { t--; }
    ok = ((RTC->ISR & RTC_ISR_INITF) != 0U) ? 1 : 0; }
  if (ok)
  {
    RTC->PRER = (127UL << 16) | 255UL;        /* LSE 32768 = (127+1)*(255+1) -> 1Hz */
    RTC->TR = ((uint32_t)to_bcd(hh) << 16) | ((uint32_t)to_bcd(mi) << 8) | to_bcd(ss);
    RTC->DR = ((uint32_t)to_bcd((unsigned)(y - 2000)) << 16) | ((uint32_t)wdu << 13)
            | ((uint32_t)to_bcd(mo) << 8) | to_bcd(d);
  }
  RTC->ISR &= ~RTC_ISR_INIT;
  RTC->WPR = 0xFFU;                           /* re-lock */
  return ok ? 0 : -1;
}

static uint32_t rtc_read_unix(void)
{
  { uint32_t t = 1000000U; while (((RTC->ISR & RTC_ISR_RSF) == 0U) && (t > 0U)) { t--; } }
  uint32_t tr = RTC->TR;
  uint32_t dr = RTC->DR;                      /* reading DR releases the shadow lock, fixed order TR->DR */
  unsigned ss = from_bcd((uint8_t)(tr & 0x7FU));
  unsigned mi = from_bcd((uint8_t)((tr >> 8) & 0x7FU));
  unsigned hh = from_bcd((uint8_t)((tr >> 16) & 0x3FU));
  unsigned d  = from_bcd((uint8_t)(dr & 0x3FU));
  unsigned mo = from_bcd((uint8_t)((dr >> 8) & 0x1FU));
  int      y  = 2000 + (int)from_bcd((uint8_t)((dr >> 16) & 0xFFU));
  return days_from_civil(y, mo, d) * 86400U + hh * 3600U + mi * 60U + ss;
}

void app_time_init(void)
{
  __HAL_RCC_RTC_CLK_ENABLE();                 /* RTC APB register access clock */
  HAL_PWR_EnableBkUpAccess();                 /* DBP: allow writing the backup domain */
  /* LSE drive strength (2026-09-02 field finding): nothing ever configured LSEDRV, so
   * the oscillator has run at the backup-domain default 00 = LOWEST drive since first
   * power-up. The greenhouse board's RTC ticked at ~45-55% of real time with hour-to-
   * hour jitter — marginal oscillation dropping edges, the textbook low-drive symptom.
   * Raise to medium-high (10). LSEDRV is writable only while LSEON=0, so a wrong
   * setting costs one controlled oscillator restart (~2 s calendar pause, once ever —
   * the bits live in the battery backup domain). */
  if ((RCC->BDCR & RCC_BDCR_LSEDRV) != RCC_BDCR_LSEDRV_1) /* target = 10 = medium-high.
   * Case history (2026-09-02/03): the RTC "ran" at ~55% of real time since first
   * power-up — root cause was a cargo-cult 1 M parallel resistor across X2 (needed
   * for discrete-gate Pierce oscillators, fatal load for the nW-class LSE amp; the
   * ~18 kHz half-speed "clock" was a relaxation mode through that resistor). Fix =
   * remove the resistor (board rework; deleted in the next PCB rev along with
   * 10 pF -> 15 pF load caps). Drive: lowest was marginal even without the resistor
   * (FC-135 is a heavy crystal), highest risks overdriving the tuning fork at the
   * present effective CL ~9 pF — medium-high is the AN2867-sane setting. */
  {
    RCC->BDCR &= ~RCC_BDCR_LSEON;
    { uint32_t t0 = HAL_GetTick();
      while (((RCC->BDCR & RCC_BDCR_LSERDY) != 0U) && ((HAL_GetTick() - t0) < 8000U)) { } }
    MODIFY_REG(RCC->BDCR, RCC_BDCR_LSEDRV, RCC_BDCR_LSEDRV_1);
  }
  if ((RCC->BDCR & RCC_BDCR_LSERDY) == 0U)
  {
    RCC->BDCR |= RCC_BDCR_LSEON;              /* LSE startup can take 1~2s, paid only once on cold boot */
    uint32_t t0 = HAL_GetTick();
    while (((RCC->BDCR & RCC_BDCR_LSERDY) == 0U) && ((HAL_GetTick() - t0) < 8000U)) { }
  }
  if ((RCC->BDCR & RCC_BDCR_LSERDY) == 0U)
  {
    extern void app_log_event(const char *type, const char *fmt, ...);
    s_rtc_ok = 0;
    app_log_event("FAULT", "rtc unavailable (LSE not ready) - pure SNTP mode");
    return;
  }
  if ((RCC->BDCR & RCC_BDCR_RTCSEL) != RCC_BDCR_RTCSEL_0)
  {
    /* RTCSEL can only be written once after a backup-domain reset; if another source is already selected, reset the backup domain and reselect (SRAM4 guard is not in the backup domain, so no harm) */
    if ((RCC->BDCR & RCC_BDCR_RTCSEL) != 0U)
    {
      RCC->BDCR |= RCC_BDCR_BDRST;
      RCC->BDCR &= ~RCC_BDCR_BDRST;
      RCC->BDCR |= RCC_BDCR_LSEON;
      uint32_t t0 = HAL_GetTick();
      while (((RCC->BDCR & RCC_BDCR_LSERDY) == 0U) && ((HAL_GetTick() - t0) < 8000U)) { }
    }
    RCC->BDCR |= RCC_BDCR_RTCSEL_0;           /* 01 = LSE */
  }
  RCC->BDCR |= RCC_BDCR_RTCEN;
  s_rtc_ok = 1;
  if ((RTC->ISR & RTC_ISR_INITS) != 0U)       /* calendar valid (set before reset) -> seed anchor, seamless time continuation */
  {
    s_anchor_unix = rtc_read_unix();
    s_anchor_tick = HAL_GetTick();
    s_src = 1;
  }
}

void app_time_sntp_cb(unsigned long s)        /* tcpip thread: no printf (event queue is fine) */
{
  extern void app_log_event(const char *type, const char *fmt, ...);
  if (s < 1750000000UL) { return; }           /* reply earlier than 2025-06 = bad packet, discard */
  s_anchor_unix = (uint32_t)s;
  s_anchor_tick = HAL_GetTick();
  s_src = 2;
  if (s_rtc_ok)
  {
    /* RTC discipline made LOUD (2026-09-02 user ruling, after boot events showed up
     * 13 min in the past): measure how far the RTC had wandered since the last sync
     * before rewriting it — >10 s on an LSE clock that is disciplined every SNTP poll
     * (1 h) means the discipline is NOT taking, and a failed INIT-mode entry is a
     * FAULT, not a shrug. */
    if ((RTC->ISR & RTC_ISR_INITS) != 0U)     /* only meaningful against a set calendar */
    {
      int32_t off = (int32_t)(rtc_read_unix() - (uint32_t)s);
      if ((off > 10) || (off < -10))
      { app_log_event("SYSTEM", "rtc was %+lds off at sync (rewritten)", (long)off); }
    }
    if (rtc_write_calendar((uint32_t)s) != 0)
    { app_log_event("FAULT", "rtc calendar write failed (INIT mode not entered)"); }
  }
  mqtt_mark_dirty();                          /* report on change: dash immediately sees the time jump */
}

uint32_t app_time_now(void)
{
  if (s_src == 0U) { return 0U; }
  return s_anchor_unix + (HAL_GetTick() - s_anchor_tick) / 1000U;
}

uint8_t app_time_source(void) { return s_src; }
