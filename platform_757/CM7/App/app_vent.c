/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_vent.c — greenhouse temperature control: three-stage roof window + two-speed fan.
 *
 * Field wiring contract (greenhouse install, 2026-08-27; fan stages added 2026-09-04):
 *   - Window = 24 V linear actuator behind a stage-select driver board. Relay-module
 *     outputs O14/O15/O16 (coils 13/14/15) are EXCLUSIVE level signals: asserting one
 *     drives the window to that opening and holds it; ALL OFF = window closes (retracts).
 *     That makes "all off" the fail-safe: if this controller dies or the backplane drops,
 *     the module's 3 s safe-state clears the coils and the window closes on its own.
 *   - Fan = Turbo 315 mixed-flow exhaust fan, two speeds on an SPDT-exclusive pair:
 *     O12 = low (coil 11), O13 = high (coil 12), both off = fan off. Stages 4 and 5 of
 *     the same ladder: the window is held FULL open underneath a running fan. The fan
 *     stages exist only while `vent fan on` (persisted, default OFF): until the fan is
 *     wired the ladder tops out at stage 3 and the two coils are left alone.
 *   - Greenhouse air temperature = PH_EC engineering-block index 7 = hardware channel
 *     T_EC2 (the temp-EC2 terminal carries the air probe since the 2026-09-01 re-plan).
 *     NOTE the published heartbeat names are reordered: that channel is "t2" on the
 *     dashboard, while t1 = outdoor (T_EC1) and t3/t4 = water (T_PH1/T_PH2). See
 *     the greenhouse NFT system configuration document (plan/nft). Polled into the process image by the scan table row
 *     PT_PHEC (app_user.c).
 *
 * Control law (pure temperature ladder — no clock, so no timezone/DST dependency; the
 * night close falls out naturally because the greenhouse cools):
 *   open      : air >= 25.0 °C sustained 3 min           -> stage 1
 *   boot-warm : FIRST valid sample after boot >= 25.0 °C -> stage 1 immediately (09-02:
 *               a reboot fail-safes the window shut; a hot greenhouse must not wait 3 min)
 *   jump      : air >= 28.0 °C instantly                 -> FULL open (stage 3; 09-02 ruling,
 *               was "30.0 -> at least stage 2" — no staged dwell when it is already hot)
 *   escalate  : >= 10 min at a stage AND air >= 25.5 °C AND cooling slower than
 *               0.5 °C per 10 min                        -> stage +1 (max 3, or 5 with
 *               the fan enabled: 4 = window full + fan low, 5 = window full + fan high)
 *   step down : air <= 22.0 °C sustained 10 min          -> stage -1
 *   close     : air <= 20.0 °C sustained 2 min           -> stage 0
 *   sensor    : air invalid/scan dead for 10 min         -> stage 0 + FAULT (fail closed)
 * Every stage change is journaled as an ACTION event, so it lands on the history charts
 * (and the public page) as an annotation — the interlock is auditable by design.
 *
 * Modes ("vent" command, CLI + cloud): auto (default) / manual stage 0..3 (0..5 with the
 * fan) / off (release the coils to the dashboard). Mode is RAM-only — a reboot returns to
 * auto, which is the safe default for an unattended greenhouse. In auto/manual the coils
 * are re-asserted every tick, so a stray dashboard write is corrected within one control
 * period. `vent fan on|off` is the one persisted setting (littlefs "vent.cfg"). */
#include "app_user.h"
#include "app_platform.h"
#include "app_user_points.h"
#include "app_vent.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lfs.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* journal writer with an explicit source tag (cli/cloud), app_datalog.c */
extern void app_log_event_src(const char *type, const char *src, const char *fmt, ...);

/* ---- stages / coils ---- */
#define VENT_WIN_STAGES    3        /* window stages 1..3 */
#define VENT_FAN_STAGES    2        /* fan low/high = stages 4..5 (only with `vent fan on`) */
#define VENT_STAGES_MAX    (VENT_WIN_STAGES + VENT_FAN_STAGES)
static const uint16_t s_win_coil[VENT_WIN_STAGES] = { 13, 14, 15 };   /* panel O14..O16 */
static const uint16_t s_fan_coil[VENT_FAN_STAGES] = { 11, 12 };       /* panel O12 (low), O13 (high) */

/* ---- tuning (tenths of °C / seconds) ---- */
#define VENT_TICK_S        5U       /* control period */
#define VENT_T_OPEN        250      /* >= -> arm opening */
#define VENT_T_OPEN_SUS    180U     /* ... sustained this long -> stage 1 */
#define VENT_T_JUMP        280      /* >= -> FULL open, no dwell (09-02: 30.0 -> 28.0 same day) */
#define VENT_ESC_HOLD      600U     /* min seconds at a stage before escalating */
#define VENT_ESC_T         255      /* still at/above this after the hold ... */
#define VENT_ESC_SLOPE     -5       /* ... and cooling slower than 0.5 °C/10 min -> +1 */
#define VENT_T_STEPDN      220      /* <= sustained 10 min -> stage -1 */
#define VENT_STEPDN_SUS    600U
#define VENT_T_CLOSE       200      /* <= sustained 2 min -> stage 0 */
#define VENT_CLOSE_SUS     120U
#define VENT_SENS_INVAL    0x7FFF   /* PH_EC "channel invalid" marker */
#define VENT_SENS_SUS      600U     /* invalid this long -> fail closed + FAULT */

#define VENT_MODE_AUTO     0U
#define VENT_MODE_MANUAL   1U
#define VENT_MODE_OFF      2U       /* released: we do not touch the coils at all */

static volatile uint8_t s_mode = VENT_MODE_AUTO;
static volatile uint8_t s_manual_stage = 0;
static uint8_t  s_fan_en = 0;             /* fan stages enabled (persisted); 0 until the fan is wired */
static uint8_t  s_stage = 0;              /* current commanded stage (auto or manual) */
static int16_t  s_air = VENT_SENS_INVAL;   /* last reading, tenths of °C */
static uint32_t s_hot_s = 0, s_cool_s = 0, s_cold_s = 0, s_bad_s = 0, s_dwell_s = 0;
static uint8_t  s_faulted = 0;            /* FAULT journaled once per sensor loss */

/* 1-minute temperature history for the 10-min cooling slope (index 0 = newest minute) */
static int16_t  s_hist[12];
static uint8_t  s_hist_n = 0;
static uint32_t s_min_acc_s = 0;

static const char *mode_name(uint8_t m)
{
  return (m == VENT_MODE_AUTO) ? "auto" : (m == VENT_MODE_MANUAL) ? "manual" : "off";
}

static uint8_t stages_max(void) { return s_fan_en ? (uint8_t)VENT_STAGES_MAX : (uint8_t)VENT_WIN_STAGES; }

/* ---- the one persisted setting: littlefs "vent.cfg" = "fan\n" (flow.cfg pattern) ---- */
#define VENT_FILE "vent.cfg"
extern lfs_t *app_lfs(void);
static uint8_t s_vfbuf[256];
static const struct lfs_file_config s_vfcfg = { .buffer = s_vfbuf };
static uint8_t s_vloaded;

static void vent_save(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[16];
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, VENT_FILE, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                       (struct lfs_file_config *)&s_vfcfg) != 0) { return; }
  snprintf(buf, sizeof buf, "%u\n", (unsigned)s_fan_en);
  (void)lfs_file_write(fs, &f, buf, strlen(buf));
  (void)lfs_file_close(fs, &f);
}
static void vent_load(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[16];
  unsigned fan;
  if (s_vloaded || (fs == NULL)) { return; }
  s_vloaded = 1;
  if (lfs_file_opencfg(fs, &f, VENT_FILE, LFS_O_RDONLY, (struct lfs_file_config *)&s_vfcfg) != 0) { return; }
  lfs_ssize_t n = lfs_file_read(fs, &f, buf, sizeof(buf) - 1U);
  (void)lfs_file_close(fs, &f);
  if (n <= 0) { return; }
  buf[n] = 0;
  if (sscanf(buf, "%u", &fan) == 1) { s_fan_en = (uint8_t)(fan != 0U); }
}

/* window level = min(stage, 3), fan speed = stage - 3; exclusive within each group;
 * every tick, RMW-safe. Fan coils are only driven while the fan stages are enabled, so
 * O12/O13 stay free for the dashboard until the fan is actually wired. */
static void vent_assert(uint8_t stage)
{
  uint8_t win = (stage > (uint8_t)VENT_WIN_STAGES) ? (uint8_t)VENT_WIN_STAGES : stage;
  uint8_t fan = (stage > (uint8_t)VENT_WIN_STAGES) ? (uint8_t)(stage - VENT_WIN_STAGES) : 0U;
  for (uint8_t k = 0; k < VENT_WIN_STAGES; k++)
  { app_io_do_set(PT_RELAY, s_win_coil[k], (uint8_t)(win == (uint8_t)(k + 1U))); }
  if (s_fan_en)
  {
    for (uint8_t k = 0; k < VENT_FAN_STAGES; k++)
    { app_io_do_set(PT_RELAY, s_fan_coil[k], (uint8_t)(fan == (uint8_t)(k + 1U))); }
  }
}

static void vent_go(uint8_t stage, const char *why)
{
  if (stage == s_stage) { return; }
  app_log_event("ACTION", "vent %u->%u air=%d.%dC %s", (unsigned)s_stage, (unsigned)stage,
                (int)(s_air / 10), (int)abs(s_air % 10), why);
  s_stage = stage;
  s_dwell_s = 0;
}

/* 10-min slope in tenths of °C: newest minute minus ~10 minutes ago; INT16_MAX = not enough history */
static int16_t slope_10min(void)
{
  if (s_hist_n < 11U) { return INT16_MAX; }
  return (int16_t)(s_hist[0] - s_hist[10]);
}

static void vent_tick(void)
{
  int      ok = (app_io_ok(PT_PHEC) > 0);
  int16_t  t  = (int16_t)app_io_ai(PT_PHEC, 7);   /* eng block index 7 = t4: the air probe sits on
                                                   * the temp-EC2 terminal since the 2026-09-01
                                                   * probe re-plan (t2 is a water temp now) */
  int      valid = ok && (t != (int16_t)VENT_SENS_INVAL);

  s_air = valid ? t : (int16_t)VENT_SENS_INVAL;

  /* ---- sensor watchdog (runs in every mode except released) ---- */
  if (!valid)
  {
    s_bad_s += VENT_TICK_S;
    if ((s_bad_s >= VENT_SENS_SUS) && !s_faulted)
    {
      s_faulted = 1;
      app_log_event("FAULT", "vent air sensor lost -> close");
      if (s_mode == VENT_MODE_AUTO) { vent_go(0, "sensor-lost"); }
    }
    if (s_mode != VENT_MODE_OFF) { vent_assert((s_mode == VENT_MODE_MANUAL) ? s_manual_stage : s_stage); }
    return;                                       /* no ladder decisions on stale data */
  }
  if (s_faulted) { s_faulted = 0; app_log_event("SYSTEM", "vent air sensor back air=%d.%dC",
                                                (int)(t / 10), (int)abs(t % 10)); }
  s_bad_s = 0;

  /* ---- minute history for the slope ---- */
  s_min_acc_s += VENT_TICK_S;
  if (s_min_acc_s >= 60U)
  {
    s_min_acc_s = 0;
    memmove(&s_hist[1], &s_hist[0], sizeof(s_hist) - sizeof(s_hist[0]));
    s_hist[0] = t;
    if (s_hist_n < (uint8_t)(sizeof(s_hist) / sizeof(s_hist[0]))) { s_hist_n++; }
  }

  if (s_mode == VENT_MODE_MANUAL) { vent_assert(s_manual_stage); return; }
  if (s_mode == VENT_MODE_OFF)    { return; }

  /* ---- automatic ladder ---- */
  s_dwell_s += VENT_TICK_S;
  s_hot_s  = (t >= VENT_T_OPEN)   ? s_hot_s  + VENT_TICK_S : 0U;
  s_cool_s = (t <= VENT_T_STEPDN) ? s_cool_s + VENT_TICK_S : 0U;
  s_cold_s = (t <= VENT_T_CLOSE)  ? s_cold_s + VENT_TICK_S : 0U;

  /* boot grace (09-02 ruling): a reboot fail-safes the window shut; if the FIRST valid
   * sample already reads warm (>= open threshold), open one stage immediately instead
   * of sitting out the 3-min sustain in a hot greenhouse. One-shot; the jump rule below
   * takes over in the same tick when it is outright hot. */
  { static uint8_t s_boot_done;
    if (!s_boot_done)
    {
      s_boot_done = 1U;
      if ((t >= VENT_T_OPEN) && (s_stage == 0U)) { vent_go(1, "boot-warm"); }
    } }
  /* >=28 C: straight to FULL open, no staged dwell (2026-09-02 user ruling "it's already
   * hot, stop opening one step at a time"; threshold 30->28 same day) */
  /* the jump lands on the window FULL open; the fan (stages 4/5) only ever comes in
   * through the slow-cool escalation, never as a first reaction */
  if ((t >= VENT_T_JUMP) && (s_stage < VENT_WIN_STAGES))       { vent_go(VENT_WIN_STAGES, "jump-full"); }
  else if ((s_stage == 0U) && (s_hot_s >= VENT_T_OPEN_SUS))    { vent_go(1, "open"); }
  else if ((s_stage > 0U) && (s_cold_s >= VENT_CLOSE_SUS))     { vent_go(0, "close"); }
  else if ((s_stage > 0U) && (s_cool_s >= VENT_STEPDN_SUS))    { vent_go((uint8_t)(s_stage - 1U), "step-down"); s_cool_s = 0; }
  else if ((s_stage > 0U) && (s_stage < stages_max()) && (s_dwell_s >= VENT_ESC_HOLD) &&
           (t >= VENT_ESC_T) && (slope_10min() != INT16_MAX) && (slope_10min() > VENT_ESC_SLOPE))
  { vent_go((uint8_t)(s_stage + 1U), "slow-cool"); }

  vent_assert(s_stage);
}

static void vent_task(void *arg)
{
  (void)arg;
  vTaskDelay(pdMS_TO_TICKS(15000));   /* let the scanner produce first real values */
  for (;;)
  {
    vent_load();                      /* no-op once loaded; covers a late fs mount */
    vent_tick();
    vTaskDelay(pdMS_TO_TICKS(VENT_TICK_S * 1000U));
  }
}

/* True window stage derived from the relay module's coil READ-BACK (FC01), not from the
 * commanded value — the dashboard and the datalog report where the window actually is,
 * whatever drove it (auto ladder, manual mode, released-mode dashboard writes). */
uint8_t app_vent_stage(void)
{
  if (s_fan_en)                             /* a running fan implies the window is full open */
  {
    for (uint8_t k = VENT_FAN_STAGES; k > 0U; k--)
    { if (app_io_di(PT_RELAY_RB, s_fan_coil[k - 1U])) { return (uint8_t)(VENT_WIN_STAGES + k); } }
  }
  for (uint8_t k = 0; k < VENT_WIN_STAGES; k++)
  { if (app_io_di(PT_RELAY_RB, s_win_coil[k])) { return (uint8_t)(k + 1U); } }
  return 0;
}
const char *app_vent_mode(void) { return mode_name(s_mode); }

int app_vent_cmd(const char *line, const char *src, char *out, uint16_t cap)
{
  if (strncmp(line, "vent", 4) != 0) { return 0; }
  const char *a = line + 4;
  while (*a == ' ') { a++; }

  if (*a == 0 || strcmp(a, "stat") == 0) { /* status only */ }
  else if (strcmp(a, "auto") == 0)
  {
    s_mode = VENT_MODE_AUTO;
    app_log_event_src("CONFIG", src, "vent mode auto");
  }
  else if (strcmp(a, "off") == 0)
  {
    s_mode = VENT_MODE_OFF;
    app_log_event_src("CONFIG", src, "vent released (coils to dashboard)");
  }
  else if ((a[0] >= '0') && (a[0] <= '0' + stages_max()) && (a[1] == 0))
  {
    s_manual_stage = (uint8_t)(a[0] - '0');
    s_mode = VENT_MODE_MANUAL;
    app_log_event_src("CONFIG", src, "vent manual %u", (unsigned)s_manual_stage);
  }
  else if ((strcmp(a, "fan on") == 0) || (strcmp(a, "fan off") == 0))
  {
    s_fan_en = (uint8_t)(a[4] == 'o' && a[5] == 'n');
    if (!s_fan_en)                            /* fan gone: fall back to window full, release its coils */
    {
      if (s_stage > (uint8_t)VENT_WIN_STAGES) { vent_go(VENT_WIN_STAGES, "fan-disabled"); }
      if (s_manual_stage > (uint8_t)VENT_WIN_STAGES) { s_manual_stage = VENT_WIN_STAGES; }
      for (uint8_t k = 0; k < VENT_FAN_STAGES; k++) { app_io_do_set(PT_RELAY, s_fan_coil[k], 0U); }
    }
    vent_save();
    app_log_event_src("CONFIG", src, "vent fan stages %s (O12 low / O13 high)", s_fan_en ? "on" : "off");
  }
  else { snprintf(out, cap, "vent: use auto|off|0..%u|fan on|off|stat", (unsigned)stages_max()); return 1; }

  if (s_air == (int16_t)VENT_SENS_INVAL)
  { snprintf(out, cap, "vent %s stage=%u/%u fan=%s air=invalid", mode_name(s_mode),
             (unsigned)((s_mode == VENT_MODE_MANUAL) ? s_manual_stage : s_stage),
             (unsigned)stages_max(), s_fan_en ? "on" : "off"); }
  else
  { snprintf(out, cap, "vent %s stage=%u/%u fan=%s air=%d.%dC slope10=%d", mode_name(s_mode),
             (unsigned)((s_mode == VENT_MODE_MANUAL) ? s_manual_stage : s_stage),
             (unsigned)stages_max(), s_fan_en ? "on" : "off",
             (int)(s_air / 10), (int)abs(s_air % 10), (int)slope_10min()); }
  return 1;
}

int app_vent_cli(char *line)
{
  char out[128];
  if (!app_vent_cmd(line, "cli", out, sizeof out)) { return 0; }
  printf("%s\n\r", out);
  return 1;
}

void app_vent_init(void)
{
  vent_load();                       /* fan-stage enable from flash (retried in the task if fs is late) */
  xTaskCreate(vent_task, "vent", 512, NULL, APP_TASK_PRIO_LOW, NULL);
}
