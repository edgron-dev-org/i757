/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_dose.c — nutrient/pH dosing control: four peristaltic pumps on the relay module.
 *
 * Wiring contract (bench rig 2026-08-31, greenhouse trial next day; BASE added 2026-09-11):
 *   - Pump A (nutrient part A) = relay-module panel O1 (coil 0)
 *   - Pump B (nutrient part B) = relay-module panel O2 (coil 1)
 *   - Pump ACID (pH-down)      = relay-module panel O3 (coil 2)
 *   - Pump BASE (pH-up)        = relay-module panel O4 (coil 3)
 *   Why both pH directions: the direction a reservoir drifts is set by the fertilizer's
 *   ammonium share, not by the water. A nitrate-only feed lets nitrate uptake push pH UP
 *   (acid pump), an ammonium-bearing feed (typ. >10 % of N, e.g. Kristalon 25 %) pushes it
 *   DOWN every day (base pump). A product controller must carry both.
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
 * Auto control law v2 (2026-09-11, user ruling "the software should tune dose size and
 * cadence itself, minimum ripple"): feed-forward + proportional + on-line identification.
 *   The reservoir is a first-order process with ~12 min dead time (mixing), so a PID is the
 *   wrong tool (integral action + dead time = overshoot/oscillation); greenhouse dosing
 *   machines (Priva/Argus class) do what is done here instead:
 *   - one DECISION per cycle (`dose mix`, default 12 min = measured settling time), pumps
 *     idle, sensors valid, circulation ok;
 *   - GAIN per pump (response per ml: A+B -> uS/cm, acid/base -> pH) is LEARNED from every
 *     auto shot: delta over the cycle minus the expected drift, exponentially smoothed,
 *     clamped to [1/5, 5x] of the factory seed so a stuck probe can never teach a runaway;
 *     three consecutive "no response" shots -> FAULT, auto off (empty bottle, tube off,
 *     dead probe all land here);
 *   - DRIFT per variable (pH/h, uS/h: fertilizer acidity, plant uptake, CO2) is learned
 *     from idle cycles (no shot, no top-up), also smoothed and clamped;
 *   - each decision: need = (target - value) + carried feed-forward (drift x cycle, so the
 *     value is held BEFORE it leaves the band, not pulled back after); ml = need / gain,
 *     fired when it reaches the minimum shot (2 ml pH, 10 ml A/B), capped per cycle by
 *     `dose shot`, one variable per cycle (EC first) so learning attribution stays clean.
 *     Ripple therefore ~ one minimum shot (2 ml base = +0.026 pH here), not the band.
 *   - top-up (water meter moved during the cycle) and manual shots invalidate learning
 *     for that cycle (float-valve dilution and hand pours are not the pump's doing);
 *   - learned gains/drifts persist in littlefs "dose.lrn" (user ruling: never relearn
 *     from scratch after a reboot); factory seeds = the greenhouse measurements:
 *     A/B liquid +14 uS per 50+50 ml, acid 30 ml -> -0.33 pH, base 1:31 KOH 20 ml -> +0.26,
 *     pH drift -0.15/day (Kristalon ammonium), EC drift ~ -100 uS/day uptake.
 *   Parameters (`dose cal/ec/ph/mix/shot`) persist in littlefs ("dose.cfg", netcfg pattern):
 *   every change saves immediately, boot reloads. MODE persists too since 2026-09-02
 *   (user ruling, after a pair of silent resets killed the closed loop unnoticed):
 *   a boot whose cfg says auto re-arms itself, but only after one full cycle —
 *   a reset storm can never machine-gun shots, and the sensor-valid guard still gates
 *   every evaluation. `dose off`/`stop` persist OFF, so an explicit stop stays stopped
 *   across reboots. (The original design landed every boot in OFF and a human re-armed
 *   auto — chemicals: fail-quiet, never fail-active; the cycle grace keeps that spirit
 *   while surviving unattended resets.)
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
extern uint32_t app_flow_dl(void);          /* app_user.c: top-up water meter, 0.1 L since boot */

/* ---- wiring ---- */
#define DOSE_PUMPS         4
#define DOSE_COIL_BASE     0U       /* pump k = relay coil k (panel O1..O4) */
#define P_A                0U
#define P_B                1U
#define P_ACID             2U
#define P_BASE             3U
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
#define DEF_BASE_ML        15U      /* auto shot, base: 1:31 KOH working solution measured +0.013 pH/ml @165 L (2026-09-11) -> ~+0.2 */
#define DEF_MIX_S          720U     /* decision cycle = mixing/settling time (measured 12 min, 2026-09-11) */
/* learning (v2). Gains are stored x1000: A/B = uS per ml of A (B follows equal), pH pumps =
 * pH x100 per ml. Drifts x1000 per hour, signed. Seeds = greenhouse measurements. */
#define SEED_GAIN_AB       280U     /* +14 uS per 50 ml A + 50 ml B  -> 0.28 uS/ml */
#define SEED_GAIN_ACID     1100U    /* 30 ml -> -0.33 pH (2026-08-26) -> 1.1 (x100)/ml */
#define SEED_GAIN_BASE     1300U    /* 20 ml -> +0.26 pH (2026-09-11) -> 1.3 (x100)/ml */
#define SEED_DRIFT_PH      (-625)   /* -0.15 pH/day = -0.625 (x100)/h */
#define SEED_DRIFT_EC      (-4000)  /* ~ -100 uS/day uptake = -4 uS/h */
#define DRIFT_PH_MAX       5000     /* |0.05 pH/h| plausibility clamp */
#define DRIFT_EC_MAX       50000    /* |50 uS/h| */
#define LEARN_ALPHA_PCT    30U      /* gain: 30 % step toward each observation */
#define DRIFT_ALPHA_PCT    20U      /* drift: 20 % */
#define LEARN_MIN_PH       4        /* learn gain only when the expected move is >= 0.04 pH (probe noise 0.01) */
#define LEARN_MIN_EC       4        /* ... >= 4 uS (EC noise ~2) */
#define STRIKE_MIN_PH      8        /* count a "no response" only on shots expected to move >= 0.08 pH */
#define STRIKE_MIN_EC      10       /* ... >= 10 uS */
#define DRIFT_WIN_S        10800U   /* drift is learned over 3 h windows (12 min deltas are quantization noise) */
#define MIN_SHOT_PH_ML     2U       /* smallest auto shot (2.5 s at 47 ml/min) -> ripple floor */
#define MIN_SHOT_AB_ML     15U
#define NORESP_STRIKES     3U       /* consecutive no-response shots -> FAULT, auto off */
#define SENS_BAD_S         600U     /* invalid this long -> FAULT (journal once) */
#define EC_DRY_FLOOR       300U     /* below this = probe dry / tank empty: no dosing */
#define CAP_AB_ML          500U     /* per pump per day */
#define CAP_ACID_ML        50U
#define CAP_BASE_ML        60U      /* per day: ~5x the measured daily need (12 ml), ~+0.8 pH worst case on a lying probe */
#define MAN_MAX_ML         200U     /* biggest single manual shot (A/B) */
#define MAN_MAX_ACID_ML    20U
#define MAN_MAX_BASE_ML    40U
#define SENS_INVAL         0x7FFF
#define FLOW_RESUME_S      300U     /* after circulation returns: fresh return water must reach the pot */
#define FLOW_SUPPLY_BIT    0x10U    /* app_flowmon_alarms(): all four gutters low = pump/supply lost */

#define M_OFF     0U                /* own the coils, hold them off; manual shots allowed */
#define M_AUTO    1U
#define M_RELEASE 2U                /* dashboard owns the coils */

static const char  *s_pump_name[DOSE_PUMPS] = { "a", "b", "acid", "base" };
static volatile uint8_t s_mode = M_OFF;
static uint16_t s_mlmin[DOSE_PUMPS] = { DEF_MLMIN, DEF_MLMIN, DEF_MLMIN, DEF_MLMIN };
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
static uint16_t s_ec_ml = DEF_EC_ML, s_acid_ml = DEF_ACID_ML, s_base_ml = DEF_BASE_ML;
static uint32_t s_mix_s = DEF_MIX_S;

/* run state: one pump at a time + a small shot queue (A-then-B is two entries) */
static int8_t   s_run = -1;               /* pump index, -1 = idle */
static uint32_t s_run_ms = 0;             /* remaining */
static struct { uint8_t pump; uint16_t ml; } s_q[4];
static uint8_t  s_qn = 0;

static uint32_t s_ml_today[DOSE_PUMPS];   /* commanded volume, reset daily / on `dose auto` */
static uint32_t s_day_s = 0;
static uint32_t s_wait_s = 0, s_bad_s = 0;
static uint8_t  s_faulted = 0;
static uint8_t  s_flowhold = 0;           /* circulation lost: auto loop held (journaled once) */
static uint16_t s_ec = SENS_INVAL, s_ph = SENS_INVAL;   /* last readings for status */

static const char *mode_name(uint8_t m)
{ return (m == M_AUTO) ? "auto" : (m == M_RELEASE) ? "release" : "off"; }

/* ---- learned model (v2) ---- */
static uint32_t s_gain[3]  = { SEED_GAIN_AB, SEED_GAIN_ACID, SEED_GAIN_BASE };  /* index: 0 A/B, 1 acid, 2 base */
static int32_t  s_drift_ph = SEED_DRIFT_PH, s_drift_ec = SEED_DRIFT_EC;
static uint8_t  s_strikes[3];             /* no-response counters per gain slot */
static int32_t  s_ff_ph = 0, s_ff_ec = 0; /* carried feed-forward need, FINE units (pH x100 x1000 / uS x1000) */
static struct { uint8_t kind; uint8_t slot; uint16_t ml; uint16_t ec0, ph0; uint32_t flow0; } s_last;
                                          /* what the previous cycle did: kind 0 idle, 1 EC, 2 pH */
static uint8_t  s_confound = 1U;          /* manual shot / hold / boot: do not learn from this cycle */
static int32_t  s_acc_dph = 0, s_acc_dec = 0;     /* drift window: sum of observed deltas (fine units) ... */
static int32_t  s_acc_dose_ph = 0, s_acc_dose_ec = 0;  /* ... minus what the shots were expected to do (fine) */
static uint32_t s_acc_t = 0;                       /* ... over this many seconds */
#define K_IDLE 0U
#define K_EC   1U
#define K_PH   2U

/* ---- parameter persistence (littlefs "dose.cfg", app_netcfg.c pattern) ----
 * One text line: cal_a cal_b cal_acid ec_target ec_db ph_target ph_db ec_ml acid_ml mix_s
 * [boot_auto] [phsrc phdiv] [ecsrc ecdiv] [cal_base base_ml]. Fields 11+ are append-only:
 * 11 = boot mode (0/1), 12/13 = pH source policy (0=mean 1/2=probe, divergence x100),
 * 14/15 = EC source policy, 16/17 = base pump (2026-09-11). An older, shorter file reads
 * fine and the missing tail keeps defaults. Saved on every parameter AND mode command;
 * loaded once at startup (retried from the task if fs mounted late). */
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
  snprintf(buf, sizeof(buf), "%u %u %u %u %u %u %u %u %u %lu %u %u %u %u %u %u %u\n",
           (unsigned)s_mlmin[P_A], (unsigned)s_mlmin[P_B], (unsigned)s_mlmin[P_ACID],
           (unsigned)s_ec_target, (unsigned)s_ec_db,
           (unsigned)s_ph_target, (unsigned)s_ph_db,
           (unsigned)s_ec_ml, (unsigned)s_acid_ml, (unsigned long)s_mix_s,
           (unsigned)((s_mode == M_AUTO) ? 1U : 0U),
           (unsigned)s_ph_src, (unsigned)s_ph_div,
           (unsigned)s_ec_src, (unsigned)s_ec_div,        /* fields 14/15: EC source policy (2026-09-04) */
           (unsigned)s_mlmin[P_BASE], (unsigned)s_base_ml); /* fields 16/17: base pump (2026-09-11) */
  (void)lfs_file_write(fs, &f, buf, strlen(buf));
  (void)lfs_file_close(fs, &f);
}

static void dose_load(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[96];
  unsigned ca, cb, cc, ect, ecd, pht, phd, eml, aml, bmode = 0, psrc = 0, pdiv = 10, esrc = 0, ediv = 100;
  unsigned cbs = DEF_MLMIN, bml = DEF_BASE_ML;
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
  nf = sscanf(buf, "%u %u %u %u %u %u %u %u %u %lu %u %u %u %u %u %u %u",
              &ca, &cb, &cc, &ect, &ecd, &pht, &phd, &eml, &aml, &mix, &bmode, &psrc, &pdiv, &esrc, &ediv,
              &cbs, &bml);
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
  if (nf >= 17)                              /* base pump (append-only tail, 09-11) */
  {
    if ((cbs >= 5U) && (cbs <= 600U))            { s_mlmin[P_BASE] = (uint16_t)cbs; }
    if ((bml >= 1U) && (bml <= MAN_MAX_BASE_ML)) { s_base_ml = (uint16_t)bml; }
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
  printf("[DOSE] params from " DOSE_FILE ": cal=%u/%u/%u/%u ec=%u\xC2\xB1%u ph=%u\xC2\xB1%u mix=%lus mode=%s\n\r",
         (unsigned)s_mlmin[0], (unsigned)s_mlmin[1], (unsigned)s_mlmin[2], (unsigned)s_mlmin[3],
         (unsigned)s_ec_target, (unsigned)s_ec_db, (unsigned)s_ph_target, (unsigned)s_ph_db,
         (unsigned long)s_mix_s, mode_name(s_mode));
}

/* ---- learned model persistence (littlefs "dose.lrn", one line, all integers) ----
 * gain_ab gain_acid gain_base drift_ph drift_ec. Written on every learning update (a few
 * times a day); a missing/corrupt file keeps the factory seeds. `dose learn reset` deletes it. */
#define LEARN_FILE "dose.lrn"
static void learn_save(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[64];
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, LEARN_FILE, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                       (struct lfs_file_config *)&s_dfcfg) != 0) { return; }
  snprintf(buf, sizeof(buf), "%lu %lu %lu %ld %ld\n",
           (unsigned long)s_gain[0], (unsigned long)s_gain[1], (unsigned long)s_gain[2],
           (long)s_drift_ph, (long)s_drift_ec);
  (void)lfs_file_write(fs, &f, buf, strlen(buf));
  (void)lfs_file_close(fs, &f);
}

static void learn_load(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[64];
  unsigned long g0, g1, g2; long dp, de;
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, LEARN_FILE, LFS_O_RDONLY, (struct lfs_file_config *)&s_dfcfg) != 0) { return; }
  lfs_ssize_t n = lfs_file_read(fs, &f, buf, sizeof(buf) - 1U);
  (void)lfs_file_close(fs, &f);
  if (n <= 0) { return; }
  buf[n] = 0;
  if (sscanf(buf, "%lu %lu %lu %ld %ld", &g0, &g1, &g2, &dp, &de) != 5) { return; }
  /* same clamps the learner enforces: a corrupt value falls back to its seed */
  if ((g0 >= SEED_GAIN_AB / 5U) && (g0 <= SEED_GAIN_AB * 5U))     { s_gain[0] = (uint32_t)g0; }
  if ((g1 >= SEED_GAIN_ACID / 5U) && (g1 <= SEED_GAIN_ACID * 5U)) { s_gain[1] = (uint32_t)g1; }
  if ((g2 >= SEED_GAIN_BASE / 5U) && (g2 <= SEED_GAIN_BASE * 5U)) { s_gain[2] = (uint32_t)g2; }
  if ((dp >= -DRIFT_PH_MAX) && (dp <= DRIFT_PH_MAX)) { s_drift_ph = (int32_t)dp; }
  if ((de >= -DRIFT_EC_MAX) && (de <= DRIFT_EC_MAX)) { s_drift_ec = (int32_t)de; }
  printf("[DOSE] learned from " LEARN_FILE ": gain ab=%lu acid=%lu base=%lu drift ph=%ld ec=%ld\n\r",
         (unsigned long)s_gain[0], (unsigned long)s_gain[1], (unsigned long)s_gain[2],
         (long)s_drift_ph, (long)s_drift_ec);
}

static void learn_reset(void)
{
  lfs_t *fs = app_lfs();
  s_gain[0] = SEED_GAIN_AB; s_gain[1] = SEED_GAIN_ACID; s_gain[2] = SEED_GAIN_BASE;
  s_drift_ph = SEED_DRIFT_PH; s_drift_ec = SEED_DRIFT_EC;
  memset(s_strikes, 0, sizeof(s_strikes));
  s_ff_ph = 0; s_ff_ec = 0; s_last.kind = K_IDLE;
  s_acc_dph = 0; s_acc_dec = 0; s_acc_dose_ph = 0; s_acc_dose_ec = 0; s_acc_t = 0;
  if (fs != NULL) { (void)lfs_remove(fs, LEARN_FILE); }
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
  if (strcmp(src, "auto") != 0) { s_confound = 1U; }   /* hand shots are not the model's doing */
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
        app_log_event("FAULT", "ph probes disagree %u.%02u (ph1=%u ph2=%u) -> pH dosing hold",
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
      s_confound = 1U;
      app_log_event("FAULT", "dose hold: feed flow lost on all gutters -> pumps off, no dosing");
    }
    return;
  }
  if (s_flowhold)
  {
    s_flowhold = 0U;
    s_confound = 1U;
    if (s_wait_s < FLOW_RESUME_S) { s_wait_s = FLOW_RESUME_S; }
    app_log_event("SYSTEM", "dose resume: feed flow back, %us grace for fresh return water",
                  (unsigned)FLOW_RESUME_S);
  }
}

/* one learning step from what the previous cycle did (v2) */
static void learn_step(uint16_t ec, uint16_t ph)
{
  int32_t dt_s  = (int32_t)s_mix_s;                      /* cycle length = what elapsed */
  int     topup = (app_flow_dl() != s_last.flow0);       /* float valve fed the tank: dilution + alkalinity */
  int     ph_ok = (ph != (uint16_t)SENS_INVAL) && (s_last.ph0 != (uint16_t)SENS_INVAL);
  int32_t d_ec  = (int32_t)ec - (int32_t)s_last.ec0;                   /* sensor units */
  int32_t d_ph  = ph_ok ? ((int32_t)ph - (int32_t)s_last.ph0) : 0;
  int32_t dosed_ec = 0, dosed_ph = 0;                     /* what the last shot was expected to do (fine units) */

  if (s_confound || topup)                               /* not the pumps' doing: drop the window too */
  {
    s_confound = 0U; s_last.kind = K_IDLE;
    s_acc_dph = 0; s_acc_dec = 0; s_acc_dose_ph = 0; s_acc_dose_ec = 0; s_acc_t = 0;
    return;
  }

  if (s_last.kind != K_IDLE)                             /* a shot fired: learn that pump's gain */
  {
    uint8_t  slot = s_last.slot;
    uint32_t seed = (slot == 0U) ? SEED_GAIN_AB : (slot == 1U) ? SEED_GAIN_ACID : SEED_GAIN_BASE;
    const char *nm = (slot == 0U) ? "ab" : s_pump_name[(slot == 1U) ? P_ACID : P_BASE];
    int32_t  expect = (int32_t)(s_gain[slot] * s_last.ml);           /* fine units (uS x1000 / pH x100 x1000) */
    int32_t  lmin   = ((s_last.kind == K_EC) ? LEARN_MIN_EC  : LEARN_MIN_PH)  * 1000L;
    int32_t  smin   = ((s_last.kind == K_EC) ? STRIKE_MIN_EC : STRIKE_MIN_PH) * 1000L;
    int32_t  resp, obs;
    if (s_last.kind == K_EC) { dosed_ec = expect; resp = d_ec * 1000L - (s_drift_ec * dt_s / 3600L); }
    else
    {
      dosed_ph = (slot == 1U) ? -expect : expect;
      resp = ph_ok ? (d_ph * 1000L - (s_drift_ph * dt_s / 3600L)) : 0;
      if (slot == 1U) { resp = -resp; }                  /* acid: a good shot moves pH DOWN */
    }
    if ((expect >= lmin) && ((s_last.kind == K_EC) || ph_ok))
    {
      obs = resp / (int32_t)s_last.ml;                   /* x1000 per ml, same scale as s_gain */
      if (obs < (int32_t)(s_gain[slot] / 10U))           /* less than a tenth of the expected move */
      {
        if ((expect >= smin) && (++s_strikes[slot] >= NORESP_STRIKES))
        {
          app_log_event("FAULT", "dose %s: %u shots with no response (last %uml moved %ld) -> auto off",
                        nm, (unsigned)s_strikes[slot], (unsigned)s_last.ml,
                        (long)((s_last.kind == K_EC) ? d_ec : d_ph));
          s_strikes[slot] = 0; s_mode = M_OFF; dose_save();
        }
      }
      else
      {
        uint32_t old = s_gain[slot];
        int32_t  g = (int32_t)old + (obs - (int32_t)old) * (int32_t)LEARN_ALPHA_PCT / 100;
        if (g < (int32_t)(seed / 5U)) { g = (int32_t)(seed / 5U); }
        if (g > (int32_t)(seed * 5U)) { g = (int32_t)(seed * 5U); }
        s_gain[slot] = (uint32_t)g; s_strikes[slot] = 0;
        learn_save();
        app_log_event("SYSTEM", "dose learn %s gain %lu->%lu (x1000/ml, shot %uml moved %ld)",
                      nm, (unsigned long)old, (unsigned long)s_gain[slot], (unsigned)s_last.ml,
                      (long)((s_last.kind == K_EC) ? d_ec : d_ph));
      }
    }
    s_last.kind = K_IDLE;
  }

  /* drift window: every clean cycle adds its delta minus the shot's expected effect; after
   * DRIFT_WIN_S the residual over time is the drift (fertilizer acidity, uptake, CO2) */
  s_acc_dec += d_ec * 1000L; s_acc_dose_ec += dosed_ec;
  if (ph_ok) { s_acc_dph += d_ph * 1000L; s_acc_dose_ph += dosed_ph; }
  s_acc_t += (uint32_t)dt_s;
  if (s_acc_t >= DRIFT_WIN_S)
  {
    int32_t obs_ec = (s_acc_dec - s_acc_dose_ec) / (int32_t)(s_acc_t / 3600U);   /* x1000 per h (window >= 3 h) */
    int32_t obs_ph = (s_acc_dph - s_acc_dose_ph) / (int32_t)(s_acc_t / 3600U);
    if ((obs_ec >= -DRIFT_EC_MAX) && (obs_ec <= DRIFT_EC_MAX))
    { s_drift_ec += (obs_ec - s_drift_ec) * (int32_t)DRIFT_ALPHA_PCT / 100; }
    if ((obs_ph >= -DRIFT_PH_MAX) && (obs_ph <= DRIFT_PH_MAX))
    { s_drift_ph += (obs_ph - s_drift_ph) * (int32_t)DRIFT_ALPHA_PCT / 100; }
    learn_save();
    app_log_event("SYSTEM", "dose learn drift ph %ld ec %ld (x1000/h; %luh window moved ph %ld ec %ld, shots %ld/%ld)",
                  (long)s_drift_ph, (long)s_drift_ec, (unsigned long)(s_acc_t / 3600U),
                  (long)(s_acc_dph / 1000L), (long)(s_acc_dec / 1000L),
                  (long)(s_acc_dose_ph / 1000L), (long)(s_acc_dose_ec / 1000L));
    s_acc_dph = 0; s_acc_dec = 0; s_acc_dose_ph = 0; s_acc_dose_ec = 0; s_acc_t = 0;
  }
}

static void dose_eval(void)               /* every 5 s, auto mode, pumps idle, circulation ok */
{
  int ok = (app_io_ok(PT_PHEC) > 0);
  uint16_t ec = dose_ec_read(ok), ph = dose_ph_read(ok);
  s_ec = ok ? ec : (uint16_t)SENS_INVAL;
  s_ph = ph;
  /* EC gates the eval (fertilizer is the primary loop); a dead/untrusted pH only
   * stops the pH branch — A/B dosing continues */
  int valid = ok && (ec != SENS_INVAL) && (ec >= EC_DRY_FLOOR);

  if (!valid)                             /* fail-silent: never dose on bad data */
  {
    s_confound = 1U;
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

  if (s_wait_s > 0U)                      /* inside the cycle: let the tank settle */
  { s_wait_s = (s_wait_s > 5U) ? (s_wait_s - 5U) : 0U; return; }

  /* ---- decision point: close the books on the last cycle, then decide this one ---- */
  learn_step(ec, ph);

  {
    int32_t dt_s   = (int32_t)s_mix_s;
    int32_t err_ec = (int32_t)s_ec_target - (int32_t)ec;                 /* uS, + = short */
    int32_t need_ec, ml_ab = 0;
    int32_t err_ph = 0, need_ph = 0, ml_ph = 0;
    uint8_t ph_ok  = (ph != (uint16_t)SENS_INVAL);

    s_ff_ec += -(s_drift_ec * dt_s / 3600L);                             /* uptake this cycle, carried (fine) */
    if (s_ff_ec < 0) { s_ff_ec = 0; }                                     /* EC only has an UP pump */
    if (s_ff_ec > (int32_t)(s_ec_db * 2000U)) { s_ff_ec = (int32_t)(s_ec_db * 2000U); }
    need_ec = err_ec * 1000L + s_ff_ec;                                   /* fine units */
    if (err_ec < -(int32_t)s_ec_db) { need_ec = 0; s_ff_ec = 0; }        /* above band: wait for uptake/top-up */
    if (need_ec > 0) { ml_ab = need_ec / (int32_t)s_gain[0]; }            /* fine / (fine per ml) = ml */
    if (ml_ab > (int32_t)s_ec_ml) { ml_ab = (int32_t)s_ec_ml; }          /* `dose shot` = per-cycle cap */

    if (ph_ok)
    {
      err_ph = (int32_t)s_ph_target - (int32_t)ph;                        /* x100, + = too acidic */
      s_ff_ph += -(s_drift_ph * dt_s / 3600L);                           /* fine */
      if (s_ff_ph > (int32_t)(s_ph_db * 2000U))  { s_ff_ph = (int32_t)(s_ph_db * 2000U); }
      if (s_ff_ph < -(int32_t)(s_ph_db * 2000U)) { s_ff_ph = -(int32_t)(s_ph_db * 2000U); }
      need_ph = err_ph * 1000L + s_ff_ph;
      if (need_ph > 0)      { ml_ph = need_ph / (int32_t)s_gain[2];  if (ml_ph > (int32_t)s_base_ml) { ml_ph = (int32_t)s_base_ml; } }
      else if (need_ph < 0) { ml_ph = -need_ph / (int32_t)s_gain[1]; if (ml_ph > (int32_t)s_acid_ml) { ml_ph = (int32_t)s_acid_ml; } }
    }

    s_last.kind = K_IDLE; s_last.ec0 = ec; s_last.ph0 = ph; s_last.flow0 = app_flow_dl();

    if (ml_ab >= (int32_t)MIN_SHOT_AB_ML)                                 /* EC first: fertilizer is the primary loop */
    {
      if ((s_ml_today[P_A] + (uint32_t)ml_ab > CAP_AB_ML) || (s_ml_today[P_B] + (uint32_t)ml_ab > CAP_AB_ML))
      {
        app_log_event("FAULT", "dose A/B daily cap %uml reached -> auto off", (unsigned)CAP_AB_ML);
        s_mode = M_OFF; dose_save();
        return;
      }
      (void)shot_queue(P_A, (uint16_t)ml_ab, "auto", "ec-need");
      (void)shot_queue(P_B, (uint16_t)ml_ab, "auto", "ec-need");
      s_last.kind = K_EC; s_last.slot = 0U; s_last.ml = (uint16_t)ml_ab; s_ff_ec = 0;
    }
    else if (ml_ph >= (int32_t)MIN_SHOT_PH_ML)
    {
      uint8_t  pump = (need_ph > 0) ? P_BASE : P_ACID;
      uint32_t capd = (need_ph > 0) ? CAP_BASE_ML : CAP_ACID_ML;
      if (s_ml_today[pump] + (uint32_t)ml_ph > capd)
      {
        app_log_event("FAULT", "dose %s daily cap %luml reached -> auto off", s_pump_name[pump], (unsigned long)capd);
        s_mode = M_OFF; dose_save();
        return;
      }
      (void)shot_queue(pump, (uint16_t)ml_ph, "auto", "ph-need");
      s_last.kind = K_PH; s_last.slot = (need_ph > 0) ? 2U : 1U; s_last.ml = (uint16_t)ml_ph; s_ff_ph = 0;
    }
    s_wait_s = s_mix_s;                                                   /* next decision one cycle later */
  }
}

static void dose_task(void *arg)
{
  uint8_t evl = 0;
  (void)arg;
  vTaskDelay(pdMS_TO_TICKS(15000));       /* let the scanner produce first real values */
  if (!s_ploaded) { dose_load(); learn_load(); }   /* covers a late fs mount */
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
  static char b[64];
  if (s_run >= 0)
  { snprintf(b, sizeof(b), "%s run:%s %lus A%lu/B%lu/ac%lu/bs%lu", mode_name(s_mode),
             s_pump_name[s_run], (unsigned long)((s_run_ms + 999U) / 1000U),
             (unsigned long)s_ml_today[P_A], (unsigned long)s_ml_today[P_B],
             (unsigned long)s_ml_today[P_ACID], (unsigned long)s_ml_today[P_BASE]); }
  else
  { snprintf(b, sizeof(b), "%s%s A%lu/B%lu/ac%lu/bs%lu", mode_name(s_mode), s_flowhold ? "!flow" : "",
             (unsigned long)s_ml_today[P_A], (unsigned long)s_ml_today[P_B],
             (unsigned long)s_ml_today[P_ACID], (unsigned long)s_ml_today[P_BASE]); }
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
    s_wait_s = 0; s_ff_ph = 0; s_ff_ec = 0; s_last.kind = K_IDLE; s_confound = 1U;
    s_acc_dph = 0; s_acc_dec = 0; s_acc_dose_ph = 0; s_acc_dose_ec = 0; s_acc_t = 0;
    memset(s_strikes, 0, sizeof(s_strikes));
    s_mode = M_AUTO;
    dose_save();                             /* boot mode rides the cfg since 2026-09-02 */
    app_log_event_src("CONFIG", src, "dose auto (ec %u±%u ph %u±%u shots %u/%u/%uml mix %lus)",
                      (unsigned)s_ec_target, (unsigned)s_ec_db, (unsigned)s_ph_target,
                      (unsigned)s_ph_db, (unsigned)s_ec_ml, (unsigned)s_acid_ml, (unsigned)s_base_ml,
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
    if ((k < 0) || (v < 5UL) || (v > 600UL)) { snprintf(out, cap, "dose cal a|b|acid|base <5..600 ml/min>"); return 1; }
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
    if ((v < 3UL) || (v > 120UL)) { snprintf(out, cap, "dose mix <3..120 min> (decision cycle = settling time)"); return 1; }
    s_mix_s = (uint32_t)v * 60U;
    dose_save();
    app_log_event_src("CONFIG", src, "dose mix %lumin", v);
  }
  else if (strcmp(t1, "learn") == 0)         /* learned model: `dose learn` shows, `dose learn reset` -> factory seeds */
  {
    if (strcmp(t2, "reset") == 0)
    {
      learn_reset();
      app_log_event_src("CONFIG", src, "dose learn reset to seeds");
    }
    snprintf(out, cap, "dose learn: gain(x1000/ml) ab=%lu acid=%lu base=%lu drift(x1000/h) ph=%ld ec=%ld ff ph=%ld ec=%ld strikes=%u/%u/%u last=%u wait=%lus win=%lumin ph%+ld ec%+ld",
             (unsigned long)s_gain[0], (unsigned long)s_gain[1], (unsigned long)s_gain[2],
             (long)s_drift_ph, (long)s_drift_ec, (long)(s_ff_ph / 1000L), (long)(s_ff_ec / 1000L),
             (unsigned)s_strikes[0], (unsigned)s_strikes[1], (unsigned)s_strikes[2], (unsigned)s_last.kind,
             (unsigned long)s_wait_s, (unsigned long)(s_acc_t / 60U), (long)(s_acc_dph / 1000L), (long)(s_acc_dec / 1000L));
    return 1;
  }
  else if (strcmp(t1, "shot") == 0)          /* per-cycle auto shot caps: `dose shot acid|base <ml>` (2026-09-11) */
  {
    int k = pump_by_name(t2, (int)strlen(t2));
    if ((k == (int)P_ACID) && (v >= 1UL) && (v <= MAN_MAX_ACID_ML)) { s_acid_ml = (uint16_t)v; }
    else if ((k == (int)P_BASE) && (v >= 1UL) && (v <= MAN_MAX_BASE_ML)) { s_base_ml = (uint16_t)v; }
    else { snprintf(out, cap, "dose shot acid <1..%u> | base <1..%u ml>", MAN_MAX_ACID_ML, MAN_MAX_BASE_ML); return 1; }
    dose_save();
    app_log_event_src("CONFIG", src, "dose shot %s %luml", s_pump_name[k], v);
  }
  else if (strcmp(t1, "ab") == 0)
  {
    if (s_mode == M_RELEASE) { snprintf(out, cap, "dose: released — `dose off` first"); return 1; }
    if ((v < 1UL) || (v > MAN_MAX_ML)) { snprintf(out, cap, "dose ab <1..%u ml>", MAN_MAX_ML); return 1; }
    if (s_qn > (uint8_t)(sizeof(s_q) / sizeof(s_q[0])) - 2U) { snprintf(out, cap, "dose: queue full"); return 1; }
    (void)shot_queue(P_A, (uint16_t)v, src, "manual");
    (void)shot_queue(P_B, (uint16_t)v, src, "manual");
  }
  else
  {
    int k = pump_by_name(t1, (int)strlen(t1));
    unsigned long maxml = (k == (int)P_ACID) ? MAN_MAX_ACID_ML : (k == (int)P_BASE) ? MAN_MAX_BASE_ML : MAN_MAX_ML;
    if (k < 0)
    { snprintf(out, cap, "dose: a|b|acid|base|ab <ml> | stop|auto|off|release | cal|shot|ec|ph|mix|learn|phsrc|phdiv|ecsrc|ecdiv"); return 1; }
    if (s_mode == M_RELEASE) { snprintf(out, cap, "dose: released — `dose off` first"); return 1; }
    if ((v < 1UL) || (v > maxml)) { snprintf(out, cap, "dose %s <1..%lu ml>", s_pump_name[k], maxml); return 1; }
    if (shot_queue((uint8_t)k, (uint16_t)v, src, "manual") != 0)
    { snprintf(out, cap, "dose: queue full"); return 1; }
  }

  snprintf(out, cap,
           "dose %s%s%s%s ec=%u(src=%s div=%u) ph=%u(src=%s div=%u) tgt=%u±%u/%u±%u today A%lu/B%lu/ac%lu/bs%luml shot=%u/%u/%u cal=%u/%u/%u/%u mix=%lum",
           mode_name(s_mode), s_flowhold ? "(FLOWHOLD)" : "",
           (s_run >= 0) ? " run:" : "", (s_run >= 0) ? s_pump_name[s_run] : "",
           (unsigned)s_ec, (s_ec_src == 0U) ? "avg" : ((s_ec_src == 1U) ? "1" : "2"), (unsigned)s_ec_div,
           (unsigned)s_ph,
           (s_ph_src == 0U) ? "avg" : ((s_ph_src == 1U) ? "1" : "2"), (unsigned)s_ph_div,
           (unsigned)s_ec_target, (unsigned)s_ec_db, (unsigned)s_ph_target, (unsigned)s_ph_db,
           (unsigned long)s_ml_today[P_A], (unsigned long)s_ml_today[P_B],
           (unsigned long)s_ml_today[P_ACID], (unsigned long)s_ml_today[P_BASE],
           (unsigned)s_ec_ml, (unsigned)s_acid_ml, (unsigned)s_base_ml,
           (unsigned)s_mlmin[P_A], (unsigned)s_mlmin[P_B], (unsigned)s_mlmin[P_ACID], (unsigned)s_mlmin[P_BASE],
           (unsigned long)(s_mix_s / 60U));
  return 1;
}

int app_dose_cli(char *line)
{
  char out[256];
  if (!app_dose_cmd(line, "cli", out, sizeof out)) { return 0; }
  printf("%s\n\r", out);
  return 1;
}

void app_dose_init(void)
{
  dose_load();                       /* params from flash before anything runs (retried in the task) */
  learn_load();                      /* learned gains/drifts (v2): never start from scratch after a reboot */
  xTaskCreate(dose_task, "dose", 640, NULL, APP_TASK_PRIO_LOW, NULL);
}
