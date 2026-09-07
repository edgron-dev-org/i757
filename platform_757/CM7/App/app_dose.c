/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_dose.c — nutrient/pH dosing control: three peristaltic pumps on the relay module.
 *
 * Wiring contract (bench rig 2026-08-31, greenhouse trial next day):
 *   - Pump A (nutrient part A) = relay-module panel O1 (coil 0)
 *   - Pump B (nutrient part B) = relay-module panel O2 (coil 1)
 *   - Pump ACID (pH-down)      = relay-module panel O3 (coil 2)
 *   Pumps are ~5 W peristaltic, 2 mm tubing — tens of ml/min. Volume dosing is TIMED:
 *   ml -> seconds through a per-pump flow calibration (`dose cal`, default 30 ml/min,
 *   MEASURE IT with a beaker before trusting any volume).
 *   - EC / pH process values = PH_EC module engineering block (PT_PHEC scan row):
 *     channel 2 = ec1 (µS/cm, EC25), channel 0 = ph1 (pH ×100).
 *
 * Safety posture (dosing = chemicals, so opt-in, fail-silent):
 *   - Boot mode = OFF: the app owns the three coils and holds them off; nothing doses
 *     until a human says `dose auto` (or fires a manual shot). A reboot lands back in OFF.
 *   - Module safe-state: if this controller or the backplane dies, the relay module's own
 *     3 s watchdog clears the coils — pumps stop without our help.
 *   - One pump runs at a time (queue); every start is journaled as an ACTION event.
 *   - Auto loop only doses on a VALID, PLAUSIBLE reading: scan alive, value != 0x7FFF,
 *     and EC above a dry-tank floor (a probe in air reads near zero — dosing on that
 *     would pump the whole bottle in).
 *   - Daily caps per pump; hitting one journals FAULT and drops auto to OFF.
 *
 * Auto control law (mirrors the manual greenhouse SOP: dose -> wait for mixing -> re-read):
 *   EC low  : ec1 < target-db sustained 5 min -> queue A then B, DOSE_EC_ML each,
 *             then a mixing wait (default 20 min) before the next auto decision.
 *   pH high : ph1 > target+db sustained 5 min -> queue ACID, DOSE_ACID_ML (small!),
 *             then the same mixing wait. EC has priority over pH in one cycle.
 *   Parameters (`dose cal/ec/ph/mix`) persist in littlefs ("dose.cfg", netcfg pattern):
 *   every change saves immediately, boot reloads. MODE persists too since 2026-09-02
 *   (user ruling, after a pair of silent resets killed the closed loop unnoticed):
 *   a boot whose cfg says auto re-arms itself, but only after one full mixing wait —
 *   a reset storm can never machine-gun shots, and the sensor-valid guard still gates
 *   every evaluation. `dose off`/`stop` persist OFF, so an explicit stop stays stopped
 *   across reboots. (The original design landed every boot in OFF and a human re-armed
 *   auto — chemicals: fail-quiet, never fail-active; the mixing-wait grace keeps that
 *   spirit while surviving unattended resets.)
 *
 * Circulation interlock (2026-09-04): the probe pot sits in the RETURN trough, so with
 *   the feed pump down it holds stale, unmixed water — dosing on that measures nothing
 *   real and drops chemicals into a trough nobody is stirring. app_flowmon's "all
 *   gutters low" alarm is the pump/supply-lost signal: while it is active the auto loop
 *   is HELD (running/queued shots aborted, FAULT journaled once); when it clears, the
 *   loop sits out a grace period so the pot has seen fresh return water before the
 *   sustain timers start again. A board without flow meters never raises that alarm,
 *   so the interlock costs nothing there. A single blocked gutter does NOT hold dosing —
 *   circulation through the other three is intact. */
#include "app_user.h"
#include "app_platform.h"
#include "app_user_points.h"
#include "app_dose.h"
#include "app_flowmon.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "lfs.h"

extern void app_log_event_src(const char *type, const char *src, const char *fmt, ...);

/* ---- wiring ---- */
#define DOSE_PUMPS         3
#define DOSE_COIL_BASE     0U       /* pump k = relay coil k (panel O1..O3) */
#define CH_EC              2U       /* PT_PHEC channel: ec1 */
#define CH_EC2             3U       /* PT_PHEC channel: ec2 */
#define CH_PH              0U       /* PT_PHEC channel: ph1 */
#define CH_PH2             1U       /* PT_PHEC channel: ph2 */

/* ---- timing ---- */
#define DOSE_TICK_MS       500U     /* pump timing granularity */
#define DOSE_EVAL_TICKS    10U      /* control-law evaluation every 5 s */

/* ---- defaults (RAM, set over `dose ...`) ---- */
#define DEF_MLMIN          30U      /* 5 W / 2 mm-tube peristaltic ballpark — CALIBRATE */
#define DEF_EC_TARGET      1150U    /* µS/cm */
#define DEF_EC_DB          50U
#define DEF_PH_TARGET      600U     /* pH x100 */
#define DEF_PH_DB          20U
#define DEF_EC_ML          50U      /* auto shot, A and B each */
#define DEF_ACID_ML        5U       /* auto shot, acid — small on purpose */
#define DEF_MIX_S          1200U    /* mixing wait after an auto dose */
#define SUSTAIN_S          300U     /* out-of-band this long before acting */
#define SENS_BAD_S         600U     /* invalid this long -> FAULT (journal once) */
#define EC_DRY_FLOOR       300U     /* below this = probe dry / tank empty: no dosing */
#define CAP_AB_ML          500U     /* per pump per day */
#define CAP_ACID_ML        50U
#define MAN_MAX_ML         200U     /* biggest single manual shot (A/B) */
#define MAN_MAX_ACID_ML    20U
#define SENS_INVAL         0x7FFF
#define FLOW_RESUME_S      300U     /* after circulation returns: fresh return water must reach the pot */
#define FLOW_SUPPLY_BIT    0x10U    /* app_flowmon_alarms(): all four gutters low = pump/supply lost */

#define M_OFF     0U                /* own the coils, hold them off; manual shots allowed */
#define M_AUTO    1U
#define M_RELEASE 2U                /* dashboard owns the coils */

static const char  *s_pump_name[DOSE_PUMPS] = { "a", "b", "acid" };
static volatile uint8_t s_mode = M_OFF;
static uint16_t s_mlmin[DOSE_PUMPS] = { DEF_MLMIN, DEF_MLMIN, DEF_MLMIN };
static uint16_t s_ec_target = DEF_EC_TARGET, s_ec_db = DEF_EC_DB;
/* pH source policy (2026-09-02 pull-test ruling): a pulled-but-moist glass electrode
 * publishes plausible drift for minutes — undetectable electrically (that day pH1 in
 * air drifted to within 0.04 of the acid trigger). Cross-checking the two probes is
 * the honest detector, but it must be CONFIGURABLE: some installs split the probes
 * across two zones to save money, where divergence is normal and a mean is nonsense.
 *   phsrc: 0 = mean of ph1+ph2 with divergence guard (same-tank), 1/2 = that probe only
 *   phdiv: divergence alarm/hold threshold, pH x100 (default 10 = 0.10); 0 = off       */
static uint8_t  s_ph_src = 0U;
static uint16_t s_ph_div = 10U;
static uint8_t  s_div_bad = 0U;     /* latched: probes disagreeing, acid loop held */
static uint8_t  s_ec_src = 0U;      /* EC source (2026-09-04, same policy as pH): 0 = mean of ec1+ec2 with divergence guard, 1/2 = that probe */
static uint16_t s_ec_div = 100U;    /* divergence guard, uS/cm; 0 = off */
static uint8_t  s_ecdiv_bad = 0U;   /* latched: EC probes disagreeing, fertilizer loop held */
static uint16_t s_ph_target = DEF_PH_TARGET, s_ph_db = DEF_PH_DB;
static uint16_t s_ec_ml = DEF_EC_ML, s_acid_ml = DEF_ACID_ML;
static uint32_t s_mix_s = DEF_MIX_S;

/* run state: one pump at a time + a small shot queue (A-then-B is two entries) */
static int8_t   s_run = -1;               /* pump index, -1 = idle */
static uint32_t s_run_ms = 0;             /* remaining */
static struct { uint8_t pump; uint16_t ml; } s_q[4];
static uint8_t  s_qn = 0;

static uint32_t s_ml_today[DOSE_PUMPS];   /* commanded volume, reset daily / on `dose auto` */
static uint32_t s_day_s = 0;
static uint32_t s_low_s = 0, s_high_s = 0, s_wait_s = 0, s_bad_s = 0;
static uint8_t  s_faulted = 0;
static uint8_t  s_flowhold = 0;           /* circulation lost: auto loop held (journaled once) */
static uint16_t s_ec = SENS_INVAL, s_ph = SENS_INVAL;   /* last readings for status */

static const char *mode_name(uint8_t m)
{ return (m == M_AUTO) ? "auto" : (m == M_RELEASE) ? "release" : "off"; }

/* ---- parameter persistence (littlefs "dose.cfg", app_netcfg.c pattern) ----
 * One text line: cal_a cal_b cal_acid ec_target ec_db ph_target ph_db ec_ml acid_ml mix_s
 * [boot_auto] [phsrc phdiv]. Fields 11+ are append-only (both appended 2026-09-02): 11 =
 * boot mode (0/1), 12/13 = pH source policy (0=mean 1/2=probe, divergence x100). An older,
 * shorter file reads fine and the missing tail keeps defaults. Saved on every parameter
 * AND mode command; loaded once at startup (retried from the task if fs mounted late). */
#define DOSE_FILE "dose.cfg"
extern lfs_t *app_lfs(void);
static uint8_t s_dfbuf[256];                 /* LFS_NO_MALLOC: bring your own buffer */
static const struct lfs_file_config s_dfcfg = { .buffer = s_dfbuf };
static uint8_t s_ploaded = 0;

static void dose_save(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[96];
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, DOSE_FILE, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                       (struct lfs_file_config *)&s_dfcfg) != 0) { return; }
  snprintf(buf, sizeof(buf), "%u %u %u %u %u %u %u %u %u %lu %u %u %u %u %u\n",
           (unsigned)s_mlmin[0], (unsigned)s_mlmin[1], (unsigned)s_mlmin[2],
           (unsigned)s_ec_target, (unsigned)s_ec_db,
           (unsigned)s_ph_target, (unsigned)s_ph_db,
           (unsigned)s_ec_ml, (unsigned)s_acid_ml, (unsigned long)s_mix_s,
           (unsigned)((s_mode == M_AUTO) ? 1U : 0U),
           (unsigned)s_ph_src, (unsigned)s_ph_div,
           (unsigned)s_ec_src, (unsigned)s_ec_div);       /* fields 14/15: EC source policy (2026-09-04) */
  (void)lfs_file_write(fs, &f, buf, strlen(buf));
  (void)lfs_file_close(fs, &f);
}

static void dose_load(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[96];
  unsigned ca, cb, cc, ect, ecd, pht, phd, eml, aml, bmode = 0, psrc = 0, pdiv = 10, esrc = 0, ediv = 100;
  unsigned long mix;
  int nf;
  if (s_ploaded || (fs == NULL)) { return; }
  s_ploaded = 1;                             /* fs is up: one attempt, file may not exist yet */
  if (lfs_file_opencfg(fs, &f, DOSE_FILE, LFS_O_RDONLY,
                       (struct lfs_file_config *)&s_dfcfg) != 0) { return; }
  lfs_ssize_t n = lfs_file_read(fs, &f, buf, sizeof(buf) - 1U);
  (void)lfs_file_close(fs, &f);
  if (n <= 0) { return; }
  buf[n] = 0;
  nf = sscanf(buf, "%u %u %u %u %u %u %u %u %u %lu %u %u %u %u %u",
              &ca, &cb, &cc, &ect, &ecd, &pht, &phd, &eml, &aml, &mix, &bmode, &psrc, &pdiv, &esrc, &ediv);
  if (nf < 10) { return; }                   /* 10 fields = pre-boot-mode file, mode stays OFF */
  if (nf >= 13)                              /* pH source policy (append-only tail, 09-02) */
  {
    if (psrc <= 2U)   { s_ph_src = (uint8_t)psrc; }
    if (pdiv <= 100U) { s_ph_div = (uint16_t)pdiv; }
  }
  if (nf >= 15)                              /* EC source policy (append-only tail, 09-04) */
  {
    if (esrc <= 2U)    { s_ec_src = (uint8_t)esrc; }
    if (ediv <= 1000U) { s_ec_div = (uint16_t)ediv; }
  }
  /* the same bounds the commands enforce — a corrupt field keeps its compile-time default */
  if ((ca >= 5U) && (ca <= 600U)) { s_mlmin[0] = (uint16_t)ca; }
  if ((cb >= 5U) && (cb <= 600U)) { s_mlmin[1] = (uint16_t)cb; }
  if ((cc >= 5U) && (cc <= 600U)) { s_mlmin[2] = (uint16_t)cc; }
  if ((ect >= 300U) && (ect <= 3000U)) { s_ec_target = (uint16_t)ect; }
  if ((ecd >= 5U) && (ecd <= 500U))    { s_ec_db = (uint16_t)ecd; }
  if ((pht >= 400U) && (pht <= 700U))  { s_ph_target = (uint16_t)pht; }
  if ((phd >= 5U) && (phd <= 100U))    { s_ph_db = (uint16_t)phd; }
  if ((eml >= 1U) && (eml <= MAN_MAX_ML))      { s_ec_ml = (uint16_t)eml; }
  if ((aml >= 1U) && (aml <= MAN_MAX_ACID_ML)) { s_acid_ml = (uint16_t)aml; }
  if ((mix >= 60UL) && (mix <= 7200UL)) { s_mix_s = (uint32_t)mix; }
  if ((nf >= 11) && (bmode == 1U) && (s_mode == M_OFF))
  {
    /* boot restore (2026-09-02): re-arm auto, but sit out one full mixing wait first —
     * a reset storm gets at most zero shots per boot, and the very first evaluation
     * still needs valid sensors + the sustained-low dwell like any other. */
    s_mode = M_AUTO;
    s_wait_s = s_mix_s;
    app_log_event("SYSTEM", "dose auto restored from cfg (grace %lus before first eval)",
                  (unsigned long)s_mix_s);
  }
  printf("[DOSE] params from " DOSE_FILE ": cal=%u/%u/%u ec=%u\xC2\xB1%u ph=%u\xC2\xB1%u mix=%lus mode=%s\n\r",
         (unsigned)s_mlmin[0], (unsigned)s_mlmin[1], (unsigned)s_mlmin[2],
         (unsigned)s_ec_target, (unsigned)s_ec_db, (unsigned)s_ph_target, (unsigned)s_ph_db,
         (unsigned long)s_mix_s, mode_name(s_mode));
}

static void pump_assert(void)             /* every tick, corrects stray writes (vent pattern) */
{
  for (uint8_t k = 0; k < DOSE_PUMPS; k++)
  { app_io_do_set(PT_RELAY, (uint16_t)(DOSE_COIL_BASE + k), (uint8_t)(s_run == (int8_t)k)); }
}

static int shot_queue(uint8_t pump, uint16_t ml, const char *src, const char *why)
{
  if (s_qn >= (uint8_t)(sizeof(s_q) / sizeof(s_q[0]))) { return -1; }
  s_q[s_qn].pump = pump; s_q[s_qn].ml = ml; s_qn++;
  app_log_event_src("ACTION", src, "dose %s %uml (%lus) %s ec=%u ph=%u",
                    s_pump_name[pump], (unsigned)ml,
                    (unsigned long)(ml * 60UL / s_mlmin[pump]), why,
                    (unsigned)s_ec, (unsigned)s_ph);
  return 0;
}

static void shot_start_next(void)
{
  if ((s_run >= 0) || (s_qn == 0U)) { return; }
  s_run    = (int8_t)s_q[0].pump;
  s_run_ms = (uint32_t)s_q[0].ml * 60000UL / s_mlmin[s_q[0].pump];
  s_ml_today[s_q[0].pump] += s_q[0].ml;   /* count what was commanded (caps err on the safe side) */
  memmove(&s_q[0], &s_q[1], sizeof(s_q) - sizeof(s_q[0]));
  s_qn--;
}

static void shots_abort(void)
{
  s_run = -1; s_run_ms = 0; s_qn = 0;
}

/* effective pH per the source policy. Returns SENS_INVAL when the chosen source is
 * dead OR (mean mode) the probes disagree beyond phdiv — the acid loop then simply
 * never accumulates its trigger, while the EC loop keeps running on its own probe. */
static uint16_t dose_ph_read(int ok)
{
  uint16_t p1 = ok ? app_io_ai(PT_PHEC, CH_PH)  : (uint16_t)SENS_INVAL;
  uint16_t p2 = ok ? app_io_ai(PT_PHEC, CH_PH2) : (uint16_t)SENS_INVAL;
  if (s_ph_src == 1U) { return p1; }
  if (s_ph_src == 2U) { return p2; }
  if (p1 == (uint16_t)SENS_INVAL) { return p2; }  /* one probe faulted: the survivor serves */
  if (p2 == (uint16_t)SENS_INVAL) { return p1; }
  {
    uint16_t d = (p1 > p2) ? (uint16_t)(p1 - p2) : (uint16_t)(p2 - p1);
    if ((s_ph_div != 0U) && (d > s_ph_div))
    {
      if (!s_div_bad)
      {
        s_div_bad = 1U;
        app_log_event("FAULT", "ph probes disagree %u.%02u (ph1=%u ph2=%u) -> acid hold",
                      (unsigned)(d / 100U), (unsigned)(d % 100U), (unsigned)p1, (unsigned)p2);
      }
      return (uint16_t)SENS_INVAL;
    }
    if (s_div_bad)
    {
      s_div_bad = 0U;
      app_log_event("SYSTEM", "ph probes agree again (ph1=%u ph2=%u)", (unsigned)p1, (unsigned)p2);
    }
    return (uint16_t)((p1 + p2) / 2U);
  }
}

/* effective EC per the source policy (2026-09-04): same shape as dose_ph_read. In mean mode a
 * disagreement beyond ecdiv returns SENS_INVAL, which the eval treats as bad data = no dosing. */
static uint16_t dose_ec_read(int ok)
{
  uint16_t e1 = ok ? app_io_ai(PT_PHEC, CH_EC)  : (uint16_t)SENS_INVAL;
  uint16_t e2 = ok ? app_io_ai(PT_PHEC, CH_EC2) : (uint16_t)SENS_INVAL;
  if (s_ec_src == 1U) { return e1; }
  if (s_ec_src == 2U) { return e2; }
  if (e1 == (uint16_t)SENS_INVAL) { return e2; }
  if (e2 == (uint16_t)SENS_INVAL) { return e1; }
  {
    uint16_t d = (e1 > e2) ? (uint16_t)(e1 - e2) : (uint16_t)(e2 - e1);
    if ((s_ec_div != 0U) && (d > s_ec_div))
    {
      if (!s_ecdiv_bad)
      {
        s_ecdiv_bad = 1U;
        app_log_event("FAULT", "ec probes disagree %uuS (ec1=%u ec2=%u) -> dosing hold", (unsigned)d, (unsigned)e1, (unsigned)e2);
      }
      return (uint16_t)SENS_INVAL;
    }
    if (s_ecdiv_bad)
    {
      s_ecdiv_bad = 0U;
      app_log_event("SYSTEM", "ec probes agree again (ec1=%u ec2=%u)", (unsigned)e1, (unsigned)e2);
    }
    return (uint16_t)((e1 + e2) / 2U);
  }
}

/* circulation interlock, every 5 s in auto mode (also while a shot is running: a pump
 * that stops mid-shot must not keep the peristaltic going into a dead trough) */
static void dose_flowlock(void)
{
  if (app_flowmon_alarms() & FLOW_SUPPLY_BIT)
  {
    if (!s_flowhold)
    {
      s_flowhold = 1U;
      shots_abort();
      app_log_event("FAULT", "dose hold: feed flow lost on all gutters -> pumps off, no dosing");
    }
    s_low_s = 0; s_high_s = 0;
    return;
  }
  if (s_flowhold)
  {
    s_flowhold = 0U;
    s_low_s = 0; s_high_s = 0;
    if (s_wait_s < FLOW_RESUME_S) { s_wait_s = FLOW_RESUME_S; }
    app_log_event("SYSTEM", "dose resume: feed flow back, %us grace for fresh return water",
                  (unsigned)FLOW_RESUME_S);
  }
}

static void dose_eval(void)               /* every 5 s, auto mode, pumps idle, circulation ok */
{
  int ok = (app_io_ok(PT_PHEC) > 0);
  uint16_t ec = dose_ec_read(ok), ph = dose_ph_read(ok);
  s_ec = ok ? ec : (uint16_t)SENS_INVAL;
  s_ph = ph;
  /* EC gates the eval (fertilizer is the primary loop); a dead/untrusted pH only
   * stops the acid trigger from accumulating — A/B dosing continues */
  int valid = ok && (ec != SENS_INVAL) && (ec >= EC_DRY_FLOOR);

  if (!valid)                             /* fail-silent: never dose on bad data */
  {
    s_low_s = 0; s_high_s = 0;
    s_bad_s += 5U;
    if ((s_bad_s >= SENS_BAD_S) && !s_faulted)
    {
      s_faulted = 1;
      app_log_event("FAULT", "dose sensors invalid (ec=%u ph=%u) -> no dosing",
                    (unsigned)ec, (unsigned)ph);
    }
    return;
  }
  if (s_faulted) { s_faulted = 0; app_log_event("SYSTEM", "dose sensors back ec=%u ph=%u",
                                                (unsigned)ec, (unsigned)ph); }
  s_bad_s = 0;

  if (s_wait_s > 0U)                      /* mixing wait after the previous auto dose */
  { s_wait_s = (s_wait_s > 5U) ? (s_wait_s - 5U) : 0U; return; }

  s_low_s  = (ec < (uint16_t)(s_ec_target - s_ec_db)) ? s_low_s  + 5U : 0U;
  s_high_s = ((ph != (uint16_t)SENS_INVAL) && (ph > (uint16_t)(s_ph_target + s_ph_db)))
           ? s_high_s + 5U : 0U;

  if (s_low_s >= SUSTAIN_S)               /* EC first: fertilizer is the primary loop */
  {
    if ((s_ml_today[0] + s_ec_ml > CAP_AB_ML) || (s_ml_today[1] + s_ec_ml > CAP_AB_ML))
    {
      app_log_event("FAULT", "dose A/B daily cap %uml reached -> auto off", (unsigned)CAP_AB_ML);
      s_mode = M_OFF;
      return;
    }
    (void)shot_queue(0, s_ec_ml, "auto", "ec-low");
    (void)shot_queue(1, s_ec_ml, "auto", "ec-low");
    s_low_s = 0; s_wait_s = s_mix_s;
  }
  else if (s_high_s >= SUSTAIN_S)
  {
    if (s_ml_today[2] + s_acid_ml > CAP_ACID_ML)
    {
      app_log_event("FAULT", "dose acid daily cap %uml reached -> auto off", (unsigned)CAP_ACID_ML);
      s_mode = M_OFF;
      return;
    }
    (void)shot_queue(2, s_acid_ml, "auto", "ph-high");
    s_high_s = 0; s_wait_s = s_mix_s;
  }
}

static void dose_task(void *arg)
{
  uint8_t evl = 0;
  (void)arg;
  vTaskDelay(pdMS_TO_TICKS(15000));       /* let the scanner produce first real values */
  dose_load();                            /* no-op if init already loaded; covers a late fs mount */
  for (;;)
  {
    if (s_run >= 0)                       /* pump timing, 500 ms granularity */
    {
      s_run_ms = (s_run_ms > DOSE_TICK_MS) ? (s_run_ms - DOSE_TICK_MS) : 0U;
      if (s_run_ms == 0U) { s_run = -1; }
    }
    if (s_run < 0) { shot_start_next(); }

    if (++evl >= DOSE_EVAL_TICKS)
    {
      evl = 0;
      s_day_s += 5U;
      if (s_day_s >= 86400U) { s_day_s = 0; memset(s_ml_today, 0, sizeof(s_ml_today)); }
      if (s_mode == M_AUTO) { dose_flowlock(); }
      if ((s_mode == M_AUTO) && !s_flowhold && (s_run < 0) && (s_qn == 0U)) { dose_eval(); }
      else if (s_mode != M_RELEASE)       /* keep status readings fresh even when not evaluating */
      {
        int ok = (app_io_ok(PT_PHEC) > 0);
        s_ec = dose_ec_read(ok);
        s_ph = dose_ph_read(ok);
      }
    }
    if (s_mode != M_RELEASE) { pump_assert(); }
    vTaskDelay(pdMS_TO_TICKS(DOSE_TICK_MS));
  }
}

const char *app_dose_hb(void)
{
  static char b[48];
  if (s_run >= 0)
  { snprintf(b, sizeof(b), "%s run:%s %lus A%lu/B%lu/ac%lu", mode_name(s_mode),
             s_pump_name[s_run], (unsigned long)((s_run_ms + 999U) / 1000U),
             (unsigned long)s_ml_today[0], (unsigned long)s_ml_today[1],
             (unsigned long)s_ml_today[2]); }
  else
  { snprintf(b, sizeof(b), "%s%s A%lu/B%lu/ac%lu", mode_name(s_mode), s_flowhold ? "!flow" : "",
             (unsigned long)s_ml_today[0], (unsigned long)s_ml_today[1],
             (unsigned long)s_ml_today[2]); }
  return b;
}

static int pump_by_name(const char *s, int len)
{
  for (int k = 0; k < DOSE_PUMPS; k++)
  { if ((strncmp(s, s_pump_name[k], (size_t)len) == 0) &&
        ((int)strlen(s_pump_name[k]) == len)) { return k; } }
  return -1;
}

int app_dose_cmd(const char *line, const char *src, char *out, uint16_t cap)
{
  if (strncmp(line, "dose", 4) != 0) { return 0; }
  const char *a = line + 4;
  while (*a == ' ') { a++; }

  char t1[8] = "", t2[8] = "";
  unsigned long v = 0;
  int nt = 0;
  {
    const char *p = a; int k = 0;
    while (*p && (*p != ' ') && (k < 7)) { t1[k++] = *p++; } t1[k] = 0; if (k) { nt = 1; }
    while (*p == ' ') { p++; }
    k = 0;
    while (*p && (*p != ' ') && (k < 7)) { t2[k++] = *p++; } t2[k] = 0; if (k) { nt = 2; }
    while (*p == ' ') { p++; }
    if (*p) { v = strtoul(p, 0, 10); nt = 3; }
    else if (nt == 2 && (t2[0] >= '0') && (t2[0] <= '9')) { v = strtoul(t2, 0, 10); }
  }

  if ((nt == 0) || (strcmp(t1, "stat") == 0)) { /* status below */ }
  else if (strcmp(t1, "auto") == 0)
  {
    memset(s_ml_today, 0, sizeof(s_ml_today)); s_day_s = 0;
    s_low_s = 0; s_high_s = 0; s_wait_s = 0;
    s_mode = M_AUTO;
    dose_save();                             /* boot mode rides the cfg since 2026-09-02 */
    app_log_event_src("CONFIG", src, "dose auto (ec %u±%u ph %u±%u shots %u/%uml mix %lus)",
                      (unsigned)s_ec_target, (unsigned)s_ec_db, (unsigned)s_ph_target,
                      (unsigned)s_ph_db, (unsigned)s_ec_ml, (unsigned)s_acid_ml,
                      (unsigned long)s_mix_s);
  }
  else if (strcmp(t1, "off") == 0)
  { shots_abort(); s_mode = M_OFF; dose_save(); app_log_event_src("CONFIG", src, "dose off"); }
  else if (strcmp(t1, "stop") == 0)
  { shots_abort(); s_mode = M_OFF; dose_save(); app_log_event_src("ACTION", src, "dose STOP (pumps off)"); }
  else if (strcmp(t1, "release") == 0)
  { shots_abort(); s_mode = M_RELEASE; dose_save(); app_log_event_src("CONFIG", src, "dose released (coils to dashboard)"); }
  else if (strcmp(t1, "cal") == 0)
  {
    int k = pump_by_name(t2, (int)strlen(t2));
    if ((k < 0) || (v < 5UL) || (v > 600UL)) { snprintf(out, cap, "dose cal a|b|acid <5..600 ml/min>"); return 1; }
    s_mlmin[k] = (uint16_t)v;
    dose_save();
    app_log_event_src("CONFIG", src, "dose cal %s %luml/min", s_pump_name[k], v);
  }
  else if (strcmp(t1, "ec") == 0)
  {
    /* `dose ec <target> [deadband]` — the optional 2nd value sets the +-band (2026-09-02
     * user request "tighten to +-20": the band had lived only in dose.cfg with no command) */
    unsigned long tgt = (nt == 3) ? strtoul(t2, 0, 10) : v;
    if ((tgt < 300UL) || (tgt > 3000UL)) { snprintf(out, cap, "dose ec <300..3000 uS> [db 5..500]"); return 1; }
    if (nt == 3)
    {
      if ((v < 5UL) || (v > 500UL)) { snprintf(out, cap, "dose ec <300..3000 uS> [db 5..500]"); return 1; }
      s_ec_db = (uint16_t)v;
    }
    s_ec_target = (uint16_t)tgt;
    dose_save();
    app_log_event_src("CONFIG", src, "dose ec target %lu±%uuS", tgt, (unsigned)s_ec_db);
  }
  else if (strcmp(t1, "ph") == 0)
  {
    unsigned long tgt = (nt == 3) ? strtoul(t2, 0, 10) : v;   /* same optional-deadband form */
    if ((tgt < 400UL) || (tgt > 700UL)) { snprintf(out, cap, "dose ph <400..700 x100> [db 5..100]"); return 1; }
    if (nt == 3)
    {
      if ((v < 5UL) || (v > 100UL)) { snprintf(out, cap, "dose ph <400..700 x100> [db 5..100]"); return 1; }
      s_ph_db = (uint16_t)v;
    }
    s_ph_target = (uint16_t)tgt;
    dose_save();
    app_log_event_src("CONFIG", src, "dose ph target %lu±%u", tgt, (unsigned)s_ph_db);
  }
  else if (strcmp(t1, "phsrc") == 0)         /* pH source: avg (mean + divergence guard) | 1 | 2 */
  {
    uint8_t v8;
    if (strcmp(t2, "avg") == 0)      { v8 = 0U; }
    else if (strcmp(t2, "1") == 0)   { v8 = 1U; }
    else if (strcmp(t2, "2") == 0)   { v8 = 2U; }
    else { snprintf(out, cap, "dose phsrc avg|1|2"); return 1; }
    s_ph_src = v8; s_div_bad = 0U;
    dose_save();
    app_log_event_src("CONFIG", src, "dose phsrc %s", (v8 == 0U) ? "avg" : ((v8 == 1U) ? "1" : "2"));
  }
  else if (strcmp(t1, "phdiv") == 0)         /* divergence hold threshold, pH x100; 0 = off */
  {
    if (v > 100UL) { snprintf(out, cap, "dose phdiv <0..100 x100, 0=off>"); return 1; }
    s_ph_div = (uint16_t)v; s_div_bad = 0U;
    dose_save();
    app_log_event_src("CONFIG", src, "dose phdiv 0.%02lu%s", v, v ? "" : " (off)");
  }
  else if (strcmp(t1, "ecsrc") == 0)         /* EC source: avg (mean + divergence guard) | 1 | 2 */
  {
    uint8_t v8;
    if (strcmp(t2, "avg") == 0)      { v8 = 0U; }
    else if (strcmp(t2, "1") == 0)   { v8 = 1U; }
    else if (strcmp(t2, "2") == 0)   { v8 = 2U; }
    else { snprintf(out, cap, "dose ecsrc avg|1|2"); return 1; }
    s_ec_src = v8; s_ecdiv_bad = 0U;
    dose_save();
    app_log_event_src("CONFIG", src, "dose ecsrc %s", (v8 == 0U) ? "avg" : ((v8 == 1U) ? "1" : "2"));
  }
  else if (strcmp(t1, "ecdiv") == 0)         /* divergence hold threshold, uS/cm; 0 = off */
  {
    if (v > 1000UL) { snprintf(out, cap, "dose ecdiv <0..1000 uS, 0=off>"); return 1; }
    s_ec_div = (uint16_t)v; s_ecdiv_bad = 0U;
    dose_save();
    app_log_event_src("CONFIG", src, "dose ecdiv %luuS%s", v, v ? "" : " (off)");
  }
  else if (strcmp(t1, "mix") == 0)
  {
    if ((v < 1UL) || (v > 120UL)) { snprintf(out, cap, "dose mix <1..120 min>"); return 1; }
    s_mix_s = (uint32_t)v * 60U;
    dose_save();
    app_log_event_src("CONFIG", src, "dose mix %lumin", v);
  }
  else if (strcmp(t1, "ab") == 0)
  {
    if (s_mode == M_RELEASE) { snprintf(out, cap, "dose: released — `dose off` first"); return 1; }
    if ((v < 1UL) || (v > MAN_MAX_ML)) { snprintf(out, cap, "dose ab <1..%u ml>", MAN_MAX_ML); return 1; }
    if (s_qn > (uint8_t)(sizeof(s_q) / sizeof(s_q[0])) - 2U) { snprintf(out, cap, "dose: queue full"); return 1; }
    (void)shot_queue(0, (uint16_t)v, src, "manual");
    (void)shot_queue(1, (uint16_t)v, src, "manual");
  }
  else
  {
    int k = pump_by_name(t1, (int)strlen(t1));
    unsigned long maxml = (k == 2) ? MAN_MAX_ACID_ML : MAN_MAX_ML;
    if (k < 0)
    { snprintf(out, cap, "dose: a|b|acid|ab <ml> | stop|auto|off|release | cal|ec|ph|mix|phsrc|phdiv|ecsrc|ecdiv"); return 1; }
    if (s_mode == M_RELEASE) { snprintf(out, cap, "dose: released — `dose off` first"); return 1; }
    if ((v < 1UL) || (v > maxml)) { snprintf(out, cap, "dose %s <1..%lu ml>", s_pump_name[k], maxml); return 1; }
    if (shot_queue((uint8_t)k, (uint16_t)v, src, "manual") != 0)
    { snprintf(out, cap, "dose: queue full"); return 1; }
  }

  snprintf(out, cap,
           "dose %s%s%s%s ec=%u(src=%s div=%u) ph=%u(src=%s div=%u) tgt=%u±%u/%u±%u today A%lu/B%lu/ac%luml cal=%u/%u/%u mix=%lum",
           mode_name(s_mode), s_flowhold ? "(FLOWHOLD)" : "",
           (s_run >= 0) ? " run:" : "", (s_run >= 0) ? s_pump_name[s_run] : "",
           (unsigned)s_ec, (s_ec_src == 0U) ? "avg" : ((s_ec_src == 1U) ? "1" : "2"), (unsigned)s_ec_div,
           (unsigned)s_ph,
           (s_ph_src == 0U) ? "avg" : ((s_ph_src == 1U) ? "1" : "2"), (unsigned)s_ph_div,
           (unsigned)s_ec_target, (unsigned)s_ec_db, (unsigned)s_ph_target, (unsigned)s_ph_db,
           (unsigned long)s_ml_today[0], (unsigned long)s_ml_today[1], (unsigned long)s_ml_today[2],
           (unsigned)s_mlmin[0], (unsigned)s_mlmin[1], (unsigned)s_mlmin[2],
           (unsigned long)(s_mix_s / 60U));
  return 1;
}

int app_dose_cli(char *line)
{
  char out[192];
  if (!app_dose_cmd(line, "cli", out, sizeof out)) { return 0; }
  printf("%s\n\r", out);
  return 1;
}

void app_dose_init(void)
{
  dose_load();                       /* params from flash before anything runs (retried in the task) */
  xTaskCreate(dose_task, "dose", 640, NULL, APP_TASK_PRIO_LOW, NULL);
}
