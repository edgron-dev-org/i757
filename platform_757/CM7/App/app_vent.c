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
 *   - Fan = gable-end exhaust fan (2000 m3/h, mains, installed 2026-10-20 together with
 *     the 50 % external shade cloth). O12 (coil 11) drives the 24 V coil of an
 *     intermediate relay that switches the mains to the fan — same pattern as the air
 *     pump on O9. O13 (coil 12) is reserved for a second speed / second fan. Stages 4
 *     and 5 of the same ladder: the window is held FULL open underneath a running fan.
 *     `vent fan 1|2` (persisted, default 0 = no fan) says how many fan speeds are wired;
 *     with 0 the ladder tops out at stage 3 and the two coils are left alone.
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
 *               0.5 °C per 10 min                        -> stage +1 (window stages only, max 3)
 *   step down : air <= 22.0 °C sustained 10 min          -> stage -1 (window stages only)
 *   close     : air <= 20.0 °C sustained 2 min           -> stage 0
 *   sensor    : air invalid/scan dead for 10 min         -> stage 0 + FAULT (fail closed)
 * Fan stages (2026-10-20, only with `vent fan 1|2`) sit on top of the window ladder with
 * their OWN two-point hysteresis — a mains fan is not "one more window notch", it costs
 * power and noise, so it never comes in through the slow-cool escalation:
 *   fan on    : air >= FAN_ON  (default 29.0 °C) sustained 2 min -> stage 4 (window full + fan)
 *   fan off   : air <= FAN_OFF (default 26.0 °C) sustained 5 min -> stage 3 (window full)
 *   two-speed : stage 4 and air >= FAN_ON + 2.0 sustained 2 min  -> stage 5 (high)
 *               stage 5 and air <= FAN_OFF + 2.0 sustained 5 min -> stage 4 (low)
 * Defaults come from the first shaded day (10-20): under the cloth and the full-open
 * window the house sat 25..29.6 °C, so the fan is there to shave the peaks above 29.
 * Every stage change is journaled as an ACTION event, so it lands on the history charts
 * (and the public page) as an annotation — the interlock is auditable by design.
 *
 * Night cooling (2026-11-08, user ruling: option A, windows only, with a rain guard). The
 * day ladder above closes the window as soon as the AIR is cool (<= 20 C), while the NFT
 * water is still 24..26 C and the RDWC buckets 25..27 C at 20:00 and only lose 3..5 C by
 * dawn (11-03..11-08 nights). So a second, WATER-driven rule puts a FLOOR under the window
 * stage at night. "Season" is not a state: in winter the water never reaches W_HI at night
 * and the cold guards hold, so the rule is silent by physics and switches itself on again
 * when the nights turn warm.
 *   start   : water (max of t3/t4) >= W_HI (22.0) AND outdoor <= water - 3.0 (something to
 *             gain) AND outdoor >= OUT_MIN (12.0, frost guard) AND air >= AIR_MIN (15.0,
 *             crop chill guard), all sustained 5 min -> floor = stage 3 (window FULL open)
 *   stop    : water <= W_LO (20.0) OR outdoor < OUT_MIN OR air < AIR_MIN OR outdoor >
 *             water - 1.0, sustained 5 min -> floor = 0 (the day ladder then closes as usual)
 *   fan     : NEVER from this rule (the floor caps at the window stages)
 *   rain    : outdoor fell >= 3.0 C within the last 60 min (front / squall proxy; there is
 *             no rain sensor) -> floor limited to stage 1 for 60 min after the last such drop.
 *             Side effect: the sun-heated outdoor probe cools fast at dusk and trips this
 *             for ~1 h after sunset; harmless, the day ladder still rules while air >= 22.
 *   sensor  : water or outdoor invalid -> floor = 0 (fail to the day ladder, no FAULT)
 * The floor only clamps the AUTO ladder's step-down/close (vent_go); manual/off untouched.
 * `vent night on|off [whiC wloC outminC airminC]` is persisted in vent.cfg (line 2); the
 * heartbeat field "ventn" says why the floor is where it is.
 *
 * Modes ("vent" command, CLI + cloud): auto (default) / manual stage 0..3 (0..5 with the
 * fan) / off (release the coils to the dashboard). Mode is RAM-only — a reboot returns to
 * auto, which is the safe default for an unattended greenhouse. In auto/manual the coils
 * are re-asserted every tick, so a stray dashboard write is corrected within one control
 * period. `vent fan <speeds> [on off]` is the one persisted setting (littlefs "vent.cfg"). */
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
/* fan hysteresis (tenths of °C / seconds); on/off thresholds are persisted and settable */
#define VENT_FAN_ON_DEF    290      /* >= sustained FAN_ON_SUS -> fan on (stage 4) */
#define VENT_FAN_OFF_DEF   260      /* <= sustained FAN_OFF_SUS -> fan off (back to stage 3) */
#define VENT_FAN_ON_SUS    120U
#define VENT_FAN_OFF_SUS   300U
#define VENT_FAN_HI_DT     20       /* two-speed: high at ON+2.0, back to low at OFF+2.0 */
#define VENT_FAN_T_MIN     150      /* sanity window for `vent fan <n> <on> <off>` */
#define VENT_FAN_T_MAX     450

/* night cooling (tenths of °C / seconds); W_HI/W_LO/OUT_MIN/AIR_MIN are persisted and settable */
#define VENT_NIGHT_W_HI_DEF    220  /* water >= -> start (with the other conditions) */
#define VENT_NIGHT_W_LO_DEF    200  /* water <= -> stop */
#define VENT_NIGHT_OUT_MIN_DEF 120  /* outdoor below this: frost guard, never open at night */
#define VENT_NIGHT_AIR_MIN_DEF 150  /* house air below this: crop chill guard */
#define VENT_NIGHT_GAP_IN      30   /* outdoor must sit >= 3.0 C under the water to start */
#define VENT_NIGHT_GAP_OUT     10   /* outdoor within 1.0 C of the water -> nothing to gain */
#define VENT_NIGHT_SUS         300U /* both directions */
#define VENT_NIGHT_FLOOR       VENT_WIN_STAGES   /* window FULL open; the fan is never a night stage */
#define VENT_NIGHT_T_MIN       50   /* sanity window for `vent night on a b c d` */
#define VENT_NIGHT_T_MAX       350
#define VENT_RAIN_DROP         30   /* outdoor fell >= 3.0 C inside the last hour ... */
#define VENT_RAIN_FLOOR        1U   /* ... -> night floor capped at stage 1 ... */
#define VENT_RAIN_HOLD_S       3600U/* ... for an hour after the last such drop */
#define VENT_OUT_HIST_N        61   /* 1-min outdoor history = 60-min span */

#define VENT_MODE_AUTO     0U
#define VENT_MODE_MANUAL   1U
#define VENT_MODE_OFF      2U       /* released: we do not touch the coils at all */

static volatile uint8_t s_mode = VENT_MODE_AUTO;
static volatile uint8_t s_manual_stage = 0;
static uint8_t  s_fan_spd = 0;            /* fan speeds wired 0/1/2 (persisted); 0 until the fan is wired */
static int16_t  s_fan_on  = VENT_FAN_ON_DEF;   /* persisted with s_fan_spd */
static int16_t  s_fan_off = VENT_FAN_OFF_DEF;
static uint8_t  s_stage = 0;              /* current commanded stage (auto or manual) */
static int16_t  s_air = VENT_SENS_INVAL;   /* last reading, tenths of °C */
static uint32_t s_hot_s = 0, s_cool_s = 0, s_cold_s = 0, s_bad_s = 0, s_dwell_s = 0;
static uint32_t s_fon_s = 0, s_foff_s = 0, s_fhi_s = 0, s_flo_s = 0;   /* fan hysteresis timers */
static uint8_t  s_faulted = 0;            /* FAULT journaled once per sensor loss */

/* 1-minute temperature history for the 10-min cooling slope (index 0 = newest minute) */
static int16_t  s_hist[12];
static uint8_t  s_hist_n = 0;
static uint32_t s_min_acc_s = 0;

/* night cooling state */
static uint8_t  s_night_en      = 1;                       /* persisted (vent.cfg line 2) */
static int16_t  s_night_w_hi    = VENT_NIGHT_W_HI_DEF;     /* persisted */
static int16_t  s_night_w_lo    = VENT_NIGHT_W_LO_DEF;
static int16_t  s_night_out_min = VENT_NIGHT_OUT_MIN_DEF;
static int16_t  s_night_air_min = VENT_NIGHT_AIR_MIN_DEF;
static uint8_t  s_night_active  = 0;                       /* rule currently holding a floor */
static uint8_t  s_floor         = 0;                       /* min stage the auto ladder may settle at */
static uint32_t s_non_s = 0, s_noff_s = 0;                 /* start/stop sustain timers */
static uint32_t s_rain_hold_s   = 0;                       /* > 0: rain guard active, counts down */
static int16_t  s_water = VENT_SENS_INVAL, s_out = VENT_SENS_INVAL;   /* last readings, tenths */
static int16_t  s_out_hist[VENT_OUT_HIST_N];               /* 1-min outdoor history, [0] = newest */
static uint8_t  s_out_hist_n = 0;
static const char *s_night_why = "off";                    /* heartbeat "ventn" */

/* "29" / "29.5" / "26.0" -> tenths of °C; -1 on garbage (integer parse, no float scanf linked) */
static int tenths(const char *s)
{
  int v = 0, nd = 0;
  while ((*s >= '0') && (*s <= '9')) { v = v * 10 + (*s - '0'); s++; nd++; }
  if (nd == 0) { return -1; }
  v *= 10;
  if (*s == '.')
  {
    s++;
    if ((*s >= '0') && (*s <= '9')) { v += (*s - '0'); s++; }
    while ((*s >= '0') && (*s <= '9')) { s++; }       /* ignore further decimals */
  }
  return (*s == 0) ? v : -1;
}

static int night_thr_ok(int whi, int wlo, int omin, int amin)
{
  return (whi > wlo) && (wlo >= VENT_NIGHT_T_MIN) && (whi <= VENT_NIGHT_T_MAX) &&
         (omin >= VENT_NIGHT_T_MIN) && (omin <= VENT_NIGHT_T_MAX) &&
         (amin >= VENT_NIGHT_T_MIN) && (amin <= VENT_NIGHT_T_MAX);
}

static const char *mode_name(uint8_t m)
{
  return (m == VENT_MODE_AUTO) ? "auto" : (m == VENT_MODE_MANUAL) ? "manual" : "off";
}

static uint8_t stages_max(void) { return (uint8_t)(VENT_WIN_STAGES + s_fan_spd); }

/* ---- the one persisted setting: littlefs "vent.cfg" = "speeds on off\n" (flow.cfg pattern);
 * a pre-10-20 file holds a single "0|1" (fan enable) and loads as 1 speed + default thresholds ---- */
#define VENT_FILE "vent.cfg"
extern lfs_t *app_lfs(void);
static uint8_t s_vfbuf[256];
static const struct lfs_file_config s_vfcfg = { .buffer = s_vfbuf };
static uint8_t s_vloaded;

static void vent_save(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[64];
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, VENT_FILE, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                       (struct lfs_file_config *)&s_vfcfg) != 0) { return; }
  snprintf(buf, sizeof buf, "%u %d %d\n%u %d %d %d %d\n", (unsigned)s_fan_spd, (int)s_fan_on, (int)s_fan_off,
           (unsigned)s_night_en, (int)s_night_w_hi, (int)s_night_w_lo, (int)s_night_out_min, (int)s_night_air_min);
  (void)lfs_file_write(fs, &f, buf, strlen(buf));
  (void)lfs_file_close(fs, &f);
}
static void vent_load(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[64];
  unsigned spd;
  int on, off;
  if (s_vloaded || (fs == NULL)) { return; }
  s_vloaded = 1;
  if (lfs_file_opencfg(fs, &f, VENT_FILE, LFS_O_RDONLY, (struct lfs_file_config *)&s_vfcfg) != 0) { return; }
  lfs_ssize_t n = lfs_file_read(fs, &f, buf, sizeof(buf) - 1U);
  (void)lfs_file_close(fs, &f);
  if (n <= 0) { return; }
  buf[n] = 0;
  int got = sscanf(buf, "%u %d %d", &spd, &on, &off);
  if (got >= 1) { s_fan_spd = (uint8_t)((spd > (unsigned)VENT_FAN_STAGES) ? VENT_FAN_STAGES : spd); }
  if ((got == 3) && (on > off) && (off >= VENT_FAN_T_MIN) && (on <= VENT_FAN_T_MAX))
  { s_fan_on = (int16_t)on; s_fan_off = (int16_t)off; }
  const char *l2 = strchr(buf, '\n');          /* line 2 (11-08): night enable + four thresholds */
  if (l2 != NULL)
  {
    unsigned en; int whi, wlo, omin, amin;
    if (sscanf(l2 + 1, "%u %d %d %d %d", &en, &whi, &wlo, &omin, &amin) == 5)
    {
      s_night_en = (uint8_t)(en ? 1U : 0U);
      if (night_thr_ok(whi, wlo, omin, amin))
      { s_night_w_hi = (int16_t)whi; s_night_w_lo = (int16_t)wlo; s_night_out_min = (int16_t)omin; s_night_air_min = (int16_t)amin; }
    }
  }
}

/* window level = min(stage, 3), fan speed = stage - 3; exclusive within each group;
 * every tick, RMW-safe. Only the wired fan coils are driven (speeds 0/1/2), so O13 (and
 * O12 with no fan) stay free for the dashboard. */
static void vent_assert(uint8_t stage)
{
  uint8_t win = (stage > (uint8_t)VENT_WIN_STAGES) ? (uint8_t)VENT_WIN_STAGES : stage;
  uint8_t fan = (stage > (uint8_t)VENT_WIN_STAGES) ? (uint8_t)(stage - VENT_WIN_STAGES) : 0U;
  for (uint8_t k = 0; k < VENT_WIN_STAGES; k++)
  { app_io_do_set(PT_RELAY, s_win_coil[k], (uint8_t)(win == (uint8_t)(k + 1U))); }
  for (uint8_t k = 0; k < s_fan_spd; k++)
  { app_io_do_set(PT_RELAY, s_fan_coil[k], (uint8_t)(fan == (uint8_t)(k + 1U))); }
}

static void vent_go(uint8_t stage, const char *why)
{
  if ((s_mode == VENT_MODE_AUTO) && (stage < s_floor)) { stage = s_floor; why = "night-floor"; }
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

/* outdoor fall over the last hour in tenths: hottest minute in the window minus now */
static int16_t out_drop_60min(void)
{
  if (s_out_hist_n < 2U) { return 0; }
  int16_t mx = s_out_hist[0];
  for (uint8_t i = 1; i < s_out_hist_n; i++) { if (s_out_hist[i] > mx) { mx = s_out_hist[i]; } }
  return (int16_t)(mx - s_out_hist[0]);
}

/* water (max of the two water probes, t3/t4 = eng block 4/5) and outdoor (t1 = eng block 6) */
static void night_read(void)
{
  int ok = (app_io_ok(PT_PHEC) > 0);
  int16_t w3 = (int16_t)app_io_ai(PT_PHEC, 4), w4 = (int16_t)app_io_ai(PT_PHEC, 5), o = (int16_t)app_io_ai(PT_PHEC, 6);
  int v3 = ok && (w3 != (int16_t)VENT_SENS_INVAL), v4 = ok && (w4 != (int16_t)VENT_SENS_INVAL);
  s_water = (v3 && v4) ? ((w3 > w4) ? w3 : w4) : (v3 ? w3 : (v4 ? w4 : (int16_t)VENT_SENS_INVAL));
  s_out   = (ok && (o != (int16_t)VENT_SENS_INVAL)) ? o : (int16_t)VENT_SENS_INVAL;
}

/* AUTO mode only. Decides s_night_active / s_floor / s_night_why; journals every change. */
static void night_tick(int16_t air)
{
  if (!s_night_en || (s_water == (int16_t)VENT_SENS_INVAL) || (s_out == (int16_t)VENT_SENS_INVAL))
  {
    s_non_s = s_noff_s = 0;
    if (s_night_active)
    { s_night_active = 0; app_log_event("ACTION", "vent night-cool off (%s)", s_night_en ? "sensor" : "disabled"); }
    s_night_why = s_night_en ? "sensor" : "off";
    s_floor = 0;
    return;
  }

  /* rain guard: a >= 3.0 C outdoor fall inside the last hour holds the floor at stage 1 for an hour */
  if (out_drop_60min() >= VENT_RAIN_DROP) { s_rain_hold_s = VENT_RAIN_HOLD_S; }
  else if (s_rain_hold_s > 0U)             { s_rain_hold_s = (s_rain_hold_s > VENT_TICK_S) ? s_rain_hold_s - VENT_TICK_S : 0U; }

  int start = (s_water >= s_night_w_hi) && (s_out <= (int16_t)(s_water - VENT_NIGHT_GAP_IN)) &&
              (s_out >= s_night_out_min) && (air >= s_night_air_min);
  int stop  = (s_water <= s_night_w_lo) || (s_out < s_night_out_min) || (air < s_night_air_min) ||
              (s_out > (int16_t)(s_water - VENT_NIGHT_GAP_OUT));
  s_non_s  = start ? s_non_s  + VENT_TICK_S : 0U;
  s_noff_s = stop  ? s_noff_s + VENT_TICK_S : 0U;

  if (!s_night_active && (s_non_s >= VENT_NIGHT_SUS))
  {
    s_night_active = 1;
    app_log_event("ACTION", "vent night-cool on water=%d.%dC out=%d.%dC air=%d.%dC",
                  (int)(s_water / 10), (int)abs(s_water % 10), (int)(s_out / 10), (int)abs(s_out % 10),
                  (int)(air / 10), (int)abs(air % 10));
  }
  else if (s_night_active && (s_noff_s >= VENT_NIGHT_SUS))
  {
    s_night_active = 0;
    app_log_event("ACTION", "vent night-cool off water=%d.%dC out=%d.%dC air=%d.%dC (%s)",
                  (int)(s_water / 10), (int)abs(s_water % 10), (int)(s_out / 10), (int)abs(s_out % 10),
                  (int)(air / 10), (int)abs(air % 10),
                  (s_water <= s_night_w_lo) ? "water-low" :
                  ((s_out < s_night_out_min) || (air < s_night_air_min)) ? "cold" : "no-gain");
  }

  uint8_t floor = 0;
  if (s_night_active)
  {
    floor = (s_rain_hold_s > 0U) ? (uint8_t)VENT_RAIN_FLOOR : (uint8_t)VENT_NIGHT_FLOOR;
    s_night_why = (s_rain_hold_s > 0U) ? "rain" : "on";
  }
  else
  {
    s_night_why = (s_water < s_night_w_hi) ? "idle-water" :
                  ((s_out < s_night_out_min) || (air < s_night_air_min)) ? "idle-cold" :
                  (s_out > (int16_t)(s_water - VENT_NIGHT_GAP_IN)) ? "idle-outdoor" : "arming";
  }
  if (floor != s_floor)
  {
    if (s_night_active)
    { app_log_event("SYSTEM", "vent night floor %u->%u%s", (unsigned)s_floor, (unsigned)floor,
                    (s_rain_hold_s > 0U) ? " (rain-guard: outdoor fell >=3.0C/60min)" : ""); }
    s_floor = floor;
    if ((s_floor > 0U) && (s_stage < s_floor)) { vent_go(s_floor, "night-cool"); }
    /* a lowered floor is not forced: the day ladder steps down / closes to it on its own clock */
  }
}

static void vent_tick(void)
{
  int      ok = (app_io_ok(PT_PHEC) > 0);
  int16_t  t  = (int16_t)app_io_ai(PT_PHEC, 7);   /* eng block index 7 = t4: the air probe sits on
                                                   * the temp-EC2 terminal since the 2026-09-01
                                                   * probe re-plan (t2 is a water temp now) */
  int      valid = ok && (t != (int16_t)VENT_SENS_INVAL);

  s_air = valid ? t : (int16_t)VENT_SENS_INVAL;
  night_read();

  /* ---- sensor watchdog (runs in every mode except released) ---- */
  if (!valid)
  {
    s_bad_s += VENT_TICK_S;
    if ((s_bad_s >= VENT_SENS_SUS) && !s_faulted)
    {
      s_faulted = 1;
      app_log_event("FAULT", "vent air sensor lost -> close");
      /* The night floor must not hold the window open blind: rain and frost guards read the same
       * module. Drop it first, or vent_go() would lift the close back to the floor (review P4). */
      if (s_night_active) { app_log_event("ACTION", "vent night-cool off (air sensor)"); }
      s_night_active = 0; s_floor = 0; s_non_s = 0; s_noff_s = 0; s_night_why = "sensor";
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
    if (s_out != (int16_t)VENT_SENS_INVAL)
    {
      memmove(&s_out_hist[1], &s_out_hist[0], sizeof(s_out_hist) - sizeof(s_out_hist[0]));
      s_out_hist[0] = s_out;
      if (s_out_hist_n < (uint8_t)VENT_OUT_HIST_N) { s_out_hist_n++; }
    }
  }

  if (s_mode == VENT_MODE_MANUAL) { vent_assert(s_manual_stage); return; }
  if (s_mode == VENT_MODE_OFF)    { return; }

  /* ---- night cooling: water-driven floor under the window stage (see header) ---- */
  night_tick(t);

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
  /* ---- fan stages: own two-point hysteresis on top of the window ladder (10-20) ---- */
  int fan_moved = 0;
  if (s_fan_spd > 0U)
  {
    s_fon_s  = (t >= s_fan_on)                  ? s_fon_s  + VENT_TICK_S : 0U;
    s_foff_s = (t <= s_fan_off)                 ? s_foff_s + VENT_TICK_S : 0U;
    s_fhi_s  = (t >= s_fan_on + VENT_FAN_HI_DT) ? s_fhi_s  + VENT_TICK_S : 0U;
    s_flo_s  = (t <= s_fan_off + VENT_FAN_HI_DT)? s_flo_s  + VENT_TICK_S : 0U;
    if ((s_stage <= VENT_WIN_STAGES) && (s_fon_s >= VENT_FAN_ON_SUS))
    { vent_go(VENT_WIN_STAGES + 1U, "fan-on"); fan_moved = 1; }
    else if ((s_stage > VENT_WIN_STAGES) && (s_foff_s >= VENT_FAN_OFF_SUS))
    { vent_go(VENT_WIN_STAGES, "fan-off"); s_cool_s = 0; fan_moved = 1; }
    else if ((s_fan_spd >= 2U) && (s_stage == VENT_WIN_STAGES + 1U) && (s_fhi_s >= VENT_FAN_ON_SUS))
    { vent_go(VENT_WIN_STAGES + 2U, "fan-high"); fan_moved = 1; }
    else if ((s_stage == VENT_WIN_STAGES + 2U) && (s_flo_s >= VENT_FAN_OFF_SUS))
    { vent_go(VENT_WIN_STAGES + 1U, "fan-low"); fan_moved = 1; }
  }
  else if (s_stage > VENT_WIN_STAGES) { vent_go(VENT_WIN_STAGES, "fan-disabled"); fan_moved = 1; }

  /* ---- window ladder (step-down / slow-cool act on window stages only; the close
   * rule covers every stage so a cold snap still shuts everything, fan included) ---- */
  if (fan_moved) { /* one move per tick */ }
  else if ((t >= VENT_T_JUMP) && (s_stage < VENT_WIN_STAGES))  { vent_go(VENT_WIN_STAGES, "jump-full"); }
  else if ((s_stage == 0U) && (s_hot_s >= VENT_T_OPEN_SUS))    { vent_go(1, "open"); }
  else if ((s_stage > 0U) && (s_cold_s >= VENT_CLOSE_SUS))     { vent_go(0, "close"); }
  else if ((s_stage > 0U) && (s_stage <= VENT_WIN_STAGES) && (s_cool_s >= VENT_STEPDN_SUS))
  { vent_go((uint8_t)(s_stage - 1U), "step-down"); s_cool_s = 0; }
  else if ((s_stage > 0U) && (s_stage < VENT_WIN_STAGES) && (s_dwell_s >= VENT_ESC_HOLD) &&
           (t >= VENT_ESC_T) && (slope_10min() != INT16_MAX) && (slope_10min() > VENT_ESC_SLOPE))
  { vent_go((uint8_t)(s_stage + 1U), "slow-cool"); }

  vent_assert(s_stage);
}

static void vent_task(void *arg)
{
  (void)arg;
  int wdg = app_wdg_register("vent", 60000U);   /* owns the roof-window motor relays */
  vTaskDelay(pdMS_TO_TICKS(15000));   /* let the scanner produce first real values */
  for (;;)
  {
    app_wdg_checkin(wdg);
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
  for (uint8_t k = s_fan_spd; k > 0U; k--)  /* a running fan implies the window is full open */
  { if (app_io_di(PT_RELAY_RB, s_fan_coil[k - 1U])) { return (uint8_t)(VENT_WIN_STAGES + k); } }
  for (uint8_t k = 0; k < VENT_WIN_STAGES; k++)
  { if (app_io_di(PT_RELAY_RB, s_win_coil[k])) { return (uint8_t)(k + 1U); } }
  return 0;
}
const char *app_vent_mode(void) { return mode_name(s_mode); }
const char *app_vent_night(void) { return (s_mode == VENT_MODE_AUTO) ? s_night_why : "n/a"; }

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
  else if (strncmp(a, "fan", 3) == 0)
  {
    /* vent fan off|on|0|1|2 [onC offC]   e.g. "vent fan 1 29 26" (on = 1 speed, °C may carry .x) */
    char w[8] = {0}, won[8] = {0}, woff[8] = {0};
    int n = sscanf(a + 3, "%7s %7s %7s", w, won, woff);   /* no %f: newlib scanf floats are not linked */
    unsigned spd;
    if (n < 1)                       { snprintf(out, cap, "vent fan: off|on|1|2 [onC offC]"); return 1; }
    if      (strcmp(w, "off") == 0)  { spd = 0; }
    else if (strcmp(w, "on") == 0)   { spd = 1; }
    else if ((w[0] >= '0') && (w[0] <= '2') && (w[1] == 0)) { spd = (unsigned)(w[0] - '0'); }
    else                             { snprintf(out, cap, "vent fan: off|on|1|2 [onC offC]"); return 1; }
    if (n == 3)
    {
      int on = tenths(won), off = tenths(woff);
      if (!((on > off) && (off >= VENT_FAN_T_MIN) && (on <= VENT_FAN_T_MAX)))
      { snprintf(out, cap, "vent fan: need %d.0 <= off < on <= %d.0 C", VENT_FAN_T_MIN / 10, VENT_FAN_T_MAX / 10); return 1; }
      s_fan_on = (int16_t)on; s_fan_off = (int16_t)off;
    }
    else if (n == 2)                 { snprintf(out, cap, "vent fan: give both onC and offC"); return 1; }
    s_fan_spd = (uint8_t)spd;
    s_fon_s = s_foff_s = s_fhi_s = s_flo_s = 0;
    if (s_stage > stages_max())        { vent_go(stages_max(), "fan-reconfig"); }
    if (s_manual_stage > stages_max()) { s_manual_stage = stages_max(); }
    for (uint8_t k = s_fan_spd; k < VENT_FAN_STAGES; k++) { app_io_do_set(PT_RELAY, s_fan_coil[k], 0U); }  /* release unused coils */
    vent_save();
    app_log_event_src("CONFIG", src, "vent fan speeds=%u on=%d.%d off=%d.%d (O12 low / O13 high)",
                      (unsigned)s_fan_spd, s_fan_on / 10, s_fan_on % 10, s_fan_off / 10, s_fan_off % 10);
  }
  else if (strncmp(a, "night", 5) == 0)
  {
    /* vent night on|off [whiC wloC outminC airminC]   e.g. "vent night on 22 20 12 15" */
    char w[8] = {0}, p1[8] = {0}, p2[8] = {0}, p3[8] = {0}, p4[8] = {0};
    int n = sscanf(a + 5, "%7s %7s %7s %7s %7s", w, p1, p2, p3, p4);
    unsigned en;
    if (n < 1)                      { snprintf(out, cap, "vent night: on|off [whiC wloC outminC airminC]"); return 1; }
    if      (strcmp(w, "on") == 0)  { en = 1; }
    else if (strcmp(w, "off") == 0) { en = 0; }
    else                            { snprintf(out, cap, "vent night: on|off [whiC wloC outminC airminC]"); return 1; }
    if (n == 5)
    {
      int whi = tenths(p1), wlo = tenths(p2), omin = tenths(p3), amin = tenths(p4);
      if (!night_thr_ok(whi, wlo, omin, amin))
      { snprintf(out, cap, "vent night: need %d.0 <= wlo < whi <= %d.0, outmin/airmin in the same range",
                 VENT_NIGHT_T_MIN / 10, VENT_NIGHT_T_MAX / 10); return 1; }
      s_night_w_hi = (int16_t)whi; s_night_w_lo = (int16_t)wlo; s_night_out_min = (int16_t)omin; s_night_air_min = (int16_t)amin;
    }
    else if (n != 1)                { snprintf(out, cap, "vent night: give all four of whiC wloC outminC airminC"); return 1; }
    s_night_en = (uint8_t)en;
    s_non_s = s_noff_s = 0;
    if (!en && s_night_active) { s_night_active = 0; s_floor = 0; s_night_why = "off"; }
    vent_save();
    app_log_event_src("CONFIG", src, "vent night %s whi=%d.%d wlo=%d.%d outmin=%d.%d airmin=%d.%d",
                      en ? "on" : "off", s_night_w_hi / 10, s_night_w_hi % 10, s_night_w_lo / 10, s_night_w_lo % 10,
                      s_night_out_min / 10, s_night_out_min % 10, s_night_air_min / 10, s_night_air_min % 10);
  }
  else { snprintf(out, cap, "vent: use auto|off|0..%u|fan off|on|1|2 [onC offC]|night on|off [whi wlo outmin airmin]|stat", (unsigned)stages_max()); return 1; }

  if (s_air == (int16_t)VENT_SENS_INVAL)
  { snprintf(out, cap, "vent %s stage=%u/%u fan=%u@%d.%d/%d.%d air=invalid", mode_name(s_mode),
             (unsigned)((s_mode == VENT_MODE_MANUAL) ? s_manual_stage : s_stage),
             (unsigned)stages_max(), (unsigned)s_fan_spd,
             s_fan_on / 10, s_fan_on % 10, s_fan_off / 10, s_fan_off % 10); }
  else
  { snprintf(out, cap, "vent %s stage=%u/%u fan=%u@%d.%d/%d.%d air=%d.%dC slope10=%d", mode_name(s_mode),
             (unsigned)((s_mode == VENT_MODE_MANUAL) ? s_manual_stage : s_stage),
             (unsigned)stages_max(), (unsigned)s_fan_spd,
             s_fan_on / 10, s_fan_on % 10, s_fan_off / 10, s_fan_off % 10,
             (int)(s_air / 10), (int)abs(s_air % 10), (int)slope_10min()); }
  { size_t l = strlen(out);
    if (l < cap)
    {
      char ws[8] = "-", os[8] = "-";
      if (s_water != (int16_t)VENT_SENS_INVAL) { snprintf(ws, sizeof ws, "%d.%d", (int)(s_water / 10), (int)abs(s_water % 10)); }
      if (s_out   != (int16_t)VENT_SENS_INVAL) { snprintf(os, sizeof os, "%d.%d", (int)(s_out / 10),   (int)abs(s_out % 10)); }
      snprintf(out + l, cap - l, " night=%s/%s floor=%u water=%sC out=%sC thr=%d.%d/%d.%d/%d.%d/%d.%d",
               s_night_en ? "on" : "off", s_night_why, (unsigned)s_floor, ws, os,
               s_night_w_hi / 10, s_night_w_hi % 10, s_night_w_lo / 10, s_night_w_lo % 10,
               s_night_out_min / 10, s_night_out_min % 10, s_night_air_min / 10, s_night_air_min % 10);
    } }
  return 1;
}

int app_vent_cli(char *line)
{
  char out[256];
  if (!app_vent_cmd(line, "cli", out, sizeof out)) { return 0; }
  printf("%s\n\r", out);
  return 1;
}

void app_vent_init(void)
{
  vent_load();                       /* fan-stage enable from flash (retried in the task if fs is late) */
  xTaskCreate(vent_task, "vent", 512, NULL, APP_TASK_PRIO_LOW, NULL);
}
