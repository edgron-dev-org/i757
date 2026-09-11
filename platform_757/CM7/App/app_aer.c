/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_aer.c — reservoir aeration: an air pump switched on water temperature.
 *
 * Field wiring contract (greenhouse install, 2026-09-08):
 *   - Air pump = mains aquarium pump behind a 24 V interposing relay. Relay-module output
 *     O9 (coil 8) drives that relay's coil; the air stone sits on the reservoir floor.
 *   - Water temperature = PH_EC engineering-block indices 4 and 5 (hardware channels
 *     T_PH1 / T_PH2, published as t3 / t4): the two probes that live in the reservoir.
 *     The WARMER valid reading is used — aeration errs on the side of running.
 *
 * Why temperature: dissolved oxygen falls as water warms while root demand rises, so a
 * warm reservoir is the one that needs bubbling; below the setpoint the pump rests.
 *
 * Control law (setpoint and hysteresis are settable and persisted, "aer.cfg"):
 *   on     : water >= T_on sustained 1 min              -> pump on
 *   off    : water <= T_on - hyst sustained 5 min       -> pump off
 *   boot   : the FIRST valid sample decides at once (a reset fail-safes the coil off,
 *            a warm reservoir must not sit out the sustain)
 *   sensor : both probes invalid / scan dead for 10 min -> pump ON + FAULT (fail
 *            aerating: bubbling a cool reservoir costs nothing, not bubbling a warm one
 *            costs roots)
 *   T_on = 0 makes auto mode "always on" (every reading is >= 0).
 * Every switch is journaled as an ACTION event, so it lands on the history charts.
 *
 * Modes ("aer" command, CLI + cloud): auto (default) / on / off (forced) / rel (release
 * the coil to the dashboard). Mode is RAM-only — a reboot returns to auto, which is the
 * right default for an unattended reservoir (the temperature law re-derives the pump
 * state within seconds of the first scan, so a reset never leaves the pump silently off).
 * In auto/on/off the coil is re-asserted every tick, so a stray dashboard write is
 * corrected within one control period. */
#include "app_user.h"
#include "app_platform.h"
#include "app_user_points.h"
#include "app_aer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lfs.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* journal writer with an explicit source tag (cli/cloud), app_datalog.c */
extern void app_log_event_src(const char *type, const char *src, const char *fmt, ...);

/* ---- coil / probes ---- */
#define AER_COIL           8U       /* panel O9 */
#define AER_PROBE_A        4U       /* PT_PHEC eng-block index: T_PH1 (published t3) */
#define AER_PROBE_B        5U       /* PT_PHEC eng-block index: T_PH2 (published t4) */

/* ---- tuning (tenths of °C / seconds) ---- */
#define AER_TICK_S         5U       /* control period */
#define AER_ON_SUS         60U      /* >= setpoint this long -> on */
#define AER_OFF_SUS        300U     /* <= setpoint - hyst this long -> off */
#define AER_SENS_INVAL     0x7FFF   /* PH_EC "channel invalid" marker */
#define AER_SENS_SUS       600U     /* invalid this long -> fail aerating + FAULT */
#define AER_DEF_T_ON       220      /* 22.0 °C */
#define AER_DEF_HYST       10       /* 1.0 °C */
#define AER_T_ON_MAX       400      /* 40.0 °C */
#define AER_HYST_MIN       1        /* 0.1 °C */
#define AER_HYST_MAX       50       /* 5.0 °C */

#define AER_MODE_AUTO      0U
#define AER_MODE_ON        1U
#define AER_MODE_OFF       2U
#define AER_MODE_REL       3U       /* released: we do not touch the coil at all */

static volatile uint8_t s_mode = AER_MODE_AUTO;
static int16_t  s_t_on = AER_DEF_T_ON;     /* persisted */
static int16_t  s_hyst = AER_DEF_HYST;     /* persisted */
static uint8_t  s_run = 0;                 /* auto-law commanded pump state */
static int16_t  s_water = AER_SENS_INVAL;  /* last reading, tenths of °C */
static uint32_t s_hot_s = 0, s_cool_s = 0, s_bad_s = 0;
static uint8_t  s_faulted = 0;             /* FAULT journaled once per sensor loss */
static uint8_t  s_boot_done = 0;           /* first-valid-sample decision taken */
static char     s_mstr[32];                /* heartbeat mode string */

static const char *mode_name(uint8_t m)
{
  return (m == AER_MODE_AUTO) ? "auto" : (m == AER_MODE_ON) ? "on" : (m == AER_MODE_OFF) ? "off" : "rel";
}

/* ---- persisted settings: littlefs "aer.cfg" = "<t_on> <hyst>\n" in tenths (vent.cfg pattern) ---- */
#define AER_FILE "aer.cfg"
extern lfs_t *app_lfs(void);
static uint8_t s_afbuf[256];
static const struct lfs_file_config s_afcfg = { .buffer = s_afbuf };
static uint8_t s_aloaded;

static void aer_save(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[24];
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, AER_FILE, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                       (struct lfs_file_config *)&s_afcfg) != 0) { return; }
  snprintf(buf, sizeof buf, "%d %d\n", (int)s_t_on, (int)s_hyst);
  (void)lfs_file_write(fs, &f, buf, strlen(buf));
  (void)lfs_file_close(fs, &f);
}
static void aer_load(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[24];
  int t, h;
  if (s_aloaded || (fs == NULL)) { return; }
  s_aloaded = 1;
  if (lfs_file_opencfg(fs, &f, AER_FILE, LFS_O_RDONLY, (struct lfs_file_config *)&s_afcfg) != 0) { return; }
  lfs_ssize_t n = lfs_file_read(fs, &f, buf, sizeof(buf) - 1U);
  (void)lfs_file_close(fs, &f);
  if (n <= 0) { return; }
  buf[n] = 0;
  if (sscanf(buf, "%d %d", &t, &h) == 2)
  {
    if ((t >= 0) && (t <= AER_T_ON_MAX)) { s_t_on = (int16_t)t; }
    if ((h >= AER_HYST_MIN) && (h <= AER_HYST_MAX)) { s_hyst = (int16_t)h; }
  }
}

/* warmer of the two valid reservoir probes; INVAL when neither is usable */
static int16_t water_temp(void)
{
  int16_t a = (int16_t)app_io_ai(PT_PHEC, AER_PROBE_A);
  int16_t b = (int16_t)app_io_ai(PT_PHEC, AER_PROBE_B);
  int va = (a != (int16_t)AER_SENS_INVAL), vb = (b != (int16_t)AER_SENS_INVAL);
  if (va && vb) { return (a > b) ? a : b; }
  if (va) { return a; }
  if (vb) { return b; }
  return (int16_t)AER_SENS_INVAL;
}

static void aer_assert(uint8_t on) { app_io_do_set(PT_RELAY, AER_COIL, on); }

static void aer_go(uint8_t on, const char *why)
{
  if (on == s_run) { return; }
  if (s_water == (int16_t)AER_SENS_INVAL)
  { app_log_event("ACTION", "aer %s water=invalid %s", on ? "on" : "off", why); }
  else
  { app_log_event("ACTION", "aer %s water=%d.%dC %s", on ? "on" : "off",
                  (int)(s_water / 10), (int)abs(s_water % 10), why); }
  s_run = on;
}

static void aer_tick(void)
{
  int     ok    = (app_io_ok(PT_PHEC) > 0);
  int16_t t     = ok ? water_temp() : (int16_t)AER_SENS_INVAL;
  int     valid = (t != (int16_t)AER_SENS_INVAL);
  int16_t t_off = (int16_t)(s_t_on - s_hyst);
  char    why[24];

  s_water = t;

  /* ---- sensor watchdog (runs in every mode except released) ---- */
  if (!valid)
  {
    s_bad_s += AER_TICK_S;
    if ((s_bad_s >= AER_SENS_SUS) && !s_faulted)
    {
      s_faulted = 1;
      app_log_event("FAULT", "aer water sensor lost -> pump on");
      if (s_mode == AER_MODE_AUTO) { aer_go(1, "sensor-lost"); }
    }
    if (s_mode == AER_MODE_AUTO)     { aer_assert(s_run); }
    else if (s_mode != AER_MODE_REL) { aer_assert((uint8_t)(s_mode == AER_MODE_ON)); }
    return;                                       /* no law decisions on stale data */
  }
  if (s_faulted) { s_faulted = 0; app_log_event("SYSTEM", "aer water sensor back water=%d.%dC",
                                                (int)(t / 10), (int)abs(t % 10)); }
  s_bad_s = 0;

  if (s_mode == AER_MODE_ON)  { aer_assert(1); return; }
  if (s_mode == AER_MODE_OFF) { aer_assert(0); return; }
  if (s_mode == AER_MODE_REL) { return; }

  /* ---- automatic law ---- */
  s_hot_s  = (t >= s_t_on) ? s_hot_s  + AER_TICK_S : 0U;
  s_cool_s = (t <= t_off)  ? s_cool_s + AER_TICK_S : 0U;

  if (!s_boot_done)
  {
    /* first valid sample after boot (or after re-entering auto): decide at once */
    s_boot_done = 1U;
    aer_go((uint8_t)(t >= s_t_on), "boot");
  }
  else if (!s_run && (s_hot_s >= AER_ON_SUS))
  {
    snprintf(why, sizeof why, ">=%d.%dC", (int)(s_t_on / 10), (int)(s_t_on % 10));
    aer_go(1, why);
  }
  else if (s_run && (s_cool_s >= AER_OFF_SUS))
  {
    snprintf(why, sizeof why, "<=%d.%dC", (int)(t_off / 10), (int)abs(t_off % 10));
    aer_go(0, why);
  }

  aer_assert(s_run);
}

static void aer_task(void *arg)
{
  (void)arg;
  vTaskDelay(pdMS_TO_TICKS(15000));   /* let the scanner produce first real values */
  for (;;)
  {
    aer_load();                       /* no-op once loaded; covers a late fs mount */
    aer_tick();
    vTaskDelay(pdMS_TO_TICKS(AER_TICK_S * 1000U));
  }
}

/* True pump state from the relay module's coil READ-BACK (FC01), not the commanded value —
 * the dashboard and the datalog report what the relay is actually doing, whatever drove it. */
uint8_t app_aer_state(void) { return app_io_di(PT_RELAY_RB, AER_COIL) ? 1U : 0U; }

const char *app_aer_mode(void)
{
  if (s_mode == AER_MODE_AUTO)
  { snprintf(s_mstr, sizeof s_mstr, "auto %d.%dC hyst %d.%d", (int)(s_t_on / 10), (int)(s_t_on % 10),
             (int)(s_hyst / 10), (int)(s_hyst % 10)); return s_mstr; }
  return mode_name(s_mode);
}

/* "22", "22.5", "0.8" -> tenths; one decimal max; 0 = bad */
static int parse_tenths(const char *s, int16_t *out, const char **end)
{
  int v = 0, n = 0;
  while ((*s >= '0') && (*s <= '9')) { v = v * 10 + (*s - '0'); s++; n++; }
  if (n == 0) { return 0; }
  v *= 10;
  if (*s == '.')
  {
    s++;
    if ((*s >= '0') && (*s <= '9')) { v += (*s - '0'); s++; }
    while ((*s >= '0') && (*s <= '9')) { s++; }   /* extra decimals ignored */
  }
  if ((*s != 0) && (*s != ' ')) { return 0; }
  *out = (int16_t)v;
  *end = s;
  return 1;
}

int app_aer_cmd(const char *line, const char *src, char *out, uint16_t cap)
{
  if (strncmp(line, "aer", 3) != 0) { return 0; }
  const char *a = line + 3;
  if ((*a != 0) && (*a != ' ')) { return 0; }     /* not ours (e.g. some longer word) */
  while (*a == ' ') { a++; }

  if (*a == 0 || strcmp(a, "stat") == 0) { /* status only */ }
  else if (strcmp(a, "auto") == 0)
  {
    s_mode = AER_MODE_AUTO;
    s_boot_done = 0; s_hot_s = 0; s_cool_s = 0;   /* next valid sample decides at once */
    app_log_event_src("CONFIG", src, "aer mode auto");
  }
  else if (strcmp(a, "on") == 0)
  {
    s_mode = AER_MODE_ON;
    app_log_event_src("CONFIG", src, "aer forced on");
  }
  else if (strcmp(a, "off") == 0)
  {
    s_mode = AER_MODE_OFF;
    app_log_event_src("CONFIG", src, "aer forced off");
  }
  else if (strcmp(a, "rel") == 0)
  {
    s_mode = AER_MODE_REL;
    app_log_event_src("CONFIG", src, "aer released (coil to dashboard)");
  }
  else if (strncmp(a, "set ", 4) == 0)
  {
    const char *p = a + 4, *e;
    int16_t t, h = s_hyst;
    while (*p == ' ') { p++; }
    if (!parse_tenths(p, &t, &e) || (t > AER_T_ON_MAX))
    { snprintf(out, cap, "aer set: setpoint 0..40.0 [hyst 0.1..5.0]"); return 1; }
    while (*e == ' ') { e++; }
    if (*e != 0)
    {
      if (!parse_tenths(e, &h, &e) || (h < AER_HYST_MIN) || (h > AER_HYST_MAX))
      { snprintf(out, cap, "aer set: setpoint 0..40.0 [hyst 0.1..5.0]"); return 1; }
    }
    s_t_on = t; s_hyst = h;
    s_hot_s = 0; s_cool_s = 0;
    aer_save();
    app_log_event_src("CONFIG", src, "aer set on>=%d.%dC off<=%d.%dC", (int)(t / 10), (int)(t % 10),
                      (int)((t - h) / 10), (int)abs((t - h) % 10));
  }
  else { snprintf(out, cap, "aer: use auto|on|off|rel|set <C> [<hystC>]|stat"); return 1; }

  { uint8_t pump = (s_mode == AER_MODE_AUTO) ? s_run : (s_mode == AER_MODE_ON) ? 1U :
                   (s_mode == AER_MODE_OFF) ? 0U : app_aer_state();
    if (s_water == (int16_t)AER_SENS_INVAL)
    { snprintf(out, cap, "aer %s pump=%s water=invalid set=%d.%dC hyst=%d.%dC", mode_name(s_mode),
               pump ? "on" : "off", (int)(s_t_on / 10), (int)(s_t_on % 10), (int)(s_hyst / 10), (int)(s_hyst % 10)); }
    else
    { snprintf(out, cap, "aer %s pump=%s water=%d.%dC set=%d.%dC hyst=%d.%dC", mode_name(s_mode),
               pump ? "on" : "off", (int)(s_water / 10), (int)abs(s_water % 10),
               (int)(s_t_on / 10), (int)(s_t_on % 10), (int)(s_hyst / 10), (int)(s_hyst % 10)); } }
  return 1;
}

int app_aer_cli(char *line)
{
  char out[128];
  if (!app_aer_cmd(line, "cli", out, sizeof out)) { return 0; }
  printf("%s\n\r", out);
  return 1;
}

void app_aer_init(void)
{
  aer_load();                        /* setpoint/hysteresis from flash (retried in the task if fs is late) */
  xTaskCreate(aer_task, "aer", 512, NULL, APP_TASK_PRIO_LOW, NULL);
}
