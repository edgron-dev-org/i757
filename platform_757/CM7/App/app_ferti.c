/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_ferti.c — fertigation systems, one instance per reservoir (see app_ferti.h).
 *
 * History: this is app_dose.c (2026-08-31 .. 2026-09-20) with every file-scope static moved
 * into ferti_sys_t and every I/O turned into a (point, channel) binding, plus the hydraulics
 * section (drip-to-waste batch refill / stir / verify / timed irrigation, 2026-10-26). The
 * control law below is UNCHANGED from app_dose.c; the design notes are kept with it.
 *
 * ---- chemistry: control law v2 (2026-09-11, user ruling "the software should tune dose size
 * and cadence itself, minimum ripple"): feed-forward + proportional + on-line identification.
 *   The reservoir is a first-order process with ~12 min dead time (mixing), so a PID is the
 *   wrong tool (integral action + dead time = overshoot/oscillation); greenhouse dosing
 *   machines (Priva/Argus class) do what is done here instead:
 *   - one DECISION per cycle (`mix`, default 12 min = measured settling time), pumps idle,
 *     sensors valid, circulation ok;
 *   - GAIN per pump (response per ml: A+B -> uS/cm, acid/base -> pH) is LEARNED from every
 *     auto shot: delta over the cycle minus the expected drift, as a forgetting sum-ratio,
 *     clamped to [1/5, 5x] of the kind's seed so a stuck probe can never teach a runaway;
 *     three consecutive "no response" shots -> FAULT, auto off (empty bottle, tube off,
 *     dead probe all land here);
 *   - DRIFT per variable (pH/h, uS/h: fertilizer acidity, plant uptake, CO2) is learned
 *     from idle cycles (no shot, no top-up), smoothed and clamped;
 *   - each decision: need = (target - value) + carried feed-forward (drift x cycle, so the
 *     value is held BEFORE it leaves the band, not pulled back after); ml = need / gain,
 *     fired when it reaches the minimum shot (2 ml pH, 15 ml A/B), capped per cycle by
 *     `shot`. Both loops may fire in one cycle (A, B, then the pH pump through the queue,
 *     one pump at a time). The pH shot of a cycle that also fed A/B is drift-accounted only,
 *     never learned (attribution belongs to the EC shot);
 *   - top-up (meter moved during the cycle) and manual shots invalidate learning for that
 *     cycle; learned gains/drifts persist (never relearn from scratch after a reboot);
 *   - PROBE SHOT: after PROBE_AFTER consecutive pH shots too small to learn from, the next
 *     one is enlarged to the smallest learnable size (inside the `shot` cap);
 *   - TOP-UP FEED-FORWARD: fresh water announces itself on the meter (a pulse meter on the
 *     float valve's line, or a batch refill reported by the hydraulics below). Each 0.1 L of
 *     ~0 uS water dilutes the reservoir by target x 0.1 L / tank, and that deficit is owed
 *     back as A/B through the learned gain, pumped WHILE the water comes in, capped per fill
 *     and by the daily cap; the settling clock restarts after each compensation shot. The
 *     typical fill volume is learned and the first drop fires a shot for 85 % of it at once.
 *   - MODE persists: a boot whose cfg says auto re-arms after one full cycle; `off`/`stop`
 *     persist OFF. Chemicals: fail-quiet, never fail-active.
 *   - 24 h caps per pump (rolling): a shot larger than the room left is TRIMMED to the room;
 *     once the room is below a minimum shot the cap HOLDS the loop that owns the pump (shots
 *     refused, FAULT journaled + alarm pushed once), the other loop keeps running. The hold is
 *     sticky and lifts only when the window has freed cap/8 again (hysteresis: hold and release
 *     must not share a threshold, or a cap-limited tank journals a pair every cycle).
 *   - pH / EC source policy: mean of two probes with a divergence guard (pulled electrode
 *     detector), or one probe; a disagreement holds that loop.
 *   - circulation interlock (flow_lock): while app_flowmon reports the feed supply lost the
 *     loop is held and shots aborted; on return a grace period lets fresh water reach the pot.
 *
 * ---- hydraulics (bindings with a `fill` pin; drip-to-waste, 2026-10-26, see
 * the fertigation multi-instance plan of 2026-10-26, section 6):
 *   IDLE -(low float, fresh water ok, not full)-> FILL [fill pump on; no decisions]
 *   FILL -(full float)-> DOSE [fill off; learn fill rate; report batch volume to the chemistry
 *        loop = top-up feed-forward shots; stir on] -(fill time limit)-> FAULT
 *   DOSE -(shots done, episode closed)-> STIR [stir for stir_s]
 *   STIR -> VERIFY [decisions allowed] -(EC and pH in band)-> READY ; else wait up to
 *        verify_rounds mix cycles, alarm, keep checking (irrigation stays blocked)
 *   READY: timed / moisture-triggered irrigation; -(low float)-> FILL (next batch)
 *   any -(overflow float, normally closed, opens)-> FAULT: everything off, dosing off, alarm;
 *        `ack` returns to IDLE.
 *   Protections: fill time limit; no fill while full; full float still asserted after two
 *   irrigations = pump dry-running or drip line blocked; full float drops with no irrigation
 *   since = siphon / leak; no runoff seen runoff_wait_s after an irrigation = no runoff
 *   (blocked line, empty reservoir, dead pump); irrigation rate (batch volume / irrigation
 *   seconds) falling > 20 % batch over batch = drip lines clogging. */
#include "app_user.h"
#include "app_platform.h"
#include "app_user_points.h"
#include "app_ferti.h"
#include "app_dose.h"
#include "app_flowmon.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "lfs.h"

extern void app_log_event_src(const char *type, const char *src, const char *fmt, ...);
extern int  app_mqtt_pub_updata(const char *sub, const void *data, uint16_t len);   /* up/event alarm push (flowmon pattern) */
extern uint8_t app_mqtt_pub_updata_busy(void);
extern uint8_t app_mqtt_ready(void);
extern lfs_t *app_lfs(void);

/* ---- timing ---- */
#define TICK_MS            500U     /* pump timing granularity */
#define EVAL_TICKS         10U      /* control-law evaluation every 5 s */

/* ---- chemistry constants (app_dose.c values, unchanged) ---- */
#define DEF_MLMIN          30U      /* 5 W / 2 mm-tube peristaltic ballpark — CALIBRATE */
#define DEF_EC_TARGET      1150U    /* µS/cm */
#define DEF_EC_DB          50U
#define DEF_PH_TARGET      600U     /* pH x100 */
#define DEF_PH_DB          20U
#define DEF_EC_ML          50U      /* auto shot, A and B each */
#define DEF_ACID_ML        5U       /* auto shot, acid — small on purpose */
#define DEF_BASE_ML        15U      /* auto shot, base: 1:31 KOH working solution measured +0.013 pH/ml @165 L (2026-09-11) -> ~+0.2 */
#define DEF_MIX_S          720U     /* decision cycle = mixing/settling time (measured 12 min, 2026-09-11) */
#define SEED_GAIN_AB       280U     /* +14 uS per 50 ml A + 50 ml B  -> 0.28 uS/ml (165 L) */
#define SEED_GAIN_ACID     1100U    /* 30 ml -> -0.33 pH (2026-08-26) -> 1.1 (x100)/ml */
#define SEED_GAIN_BASE     1300U    /* 20 ml -> +0.26 pH (2026-09-11) -> 1.3 (x100)/ml */
#define SEED_DRIFT_PH      (-625)   /* -0.15 pH/day = -0.625 (x100)/h */
#define SEED_DRIFT_EC      (-4000)  /* ~ -100 uS/day uptake = -4 uS/h */
#define DRIFT_PH_MAX       5000     /* |0.05 pH/h| plausibility clamp */
#define DRIFT_EC_MAX       50000    /* |50 uS/h| */
#define LEARN_WINDOW_ML    250L     /* gain = sum(response) / sum(ml) over a forgetting window measured in EFFECTIVE ml */
#define LEARN_MIN_KEEP_PCT 50L      /* one sample never forgets more than half of the history */
#define LEARN_REF_ML_AB    50U      /* the seed / a reloaded gain enter the sums as one sample of this size */
#define LEARN_REF_ML_PH    5U
#define DRIFT_ALPHA_PCT    20U      /* drift: 20 % */
#define AVG_N              FERTI_AVG_N   /* control/learning use a 60 s mean of the 5 s readings */
#define LEARN_MIN_PH       4        /* learn gain only when the expected move is >= 0.04 pH (probe noise 0.01) */
#define LEARN_MIN_EC       4        /* ... >= 4 uS (EC noise ~2) */
#define STRIKE_MIN_PH      8        /* count a "no response" only on shots expected to move >= 0.08 pH */
#define STRIKE_MIN_EC      10       /* ... >= 10 uS */
#define DRIFT_WIN_S        10800U   /* drift is learned over 3 h windows */
#define MIN_SHOT_PH_ML     2U       /* smallest auto shot -> ripple floor */
#define MIN_SHOT_AB_ML     15U
#define NORESP_STRIKES     3U       /* consecutive no-response shots -> FAULT, auto off */
#define PROBE_AFTER        5U       /* pH shots too small to learn from before one is enlarged to a learnable size */
#define DEF_TANK_DL        1650U    /* reservoir volume, 0.1 L (greenhouse 165 L) -> `tank` */
#define TOPUP_QUIET_S      60U      /* meter silent this long = fill episode over */
#define TOPUP_CAP_ML       200U     /* A and B each, per fill episode */
#define TOPUP_PRED_PCT     85U      /* predicted first shot = this share of the learned typical fill */
#define TOPUP_VOL_ALPHA_PCT 30U     /* typical fill volume: 30 % step per fill */
#define SENS_BAD_S         600U     /* invalid this long -> FAULT (journal once) */
#define EC_DRY_FLOOR       300U     /* below this = probe dry / tank empty: no dosing */
#define DEF_CAP_AB_ML      800U     /* 24 h caps (2026-09-20) */
#define DEF_CAP_ACID_ML    50U
#define DEF_CAP_BASE_ML    150U
#define CAP_MAX_AB_ML      5000U
#define CAP_MAX_PH_ML      1000U
#define CAP_HOURS          FERTI_CAP_HOURS
#define MAN_MAX_ML         200U     /* biggest single manual shot (A/B) */
#define MAN_MAX_ACID_ML    20U
#define MAN_MAX_BASE_ML    40U
#define SENS_INVAL         FERTI_SENS_INVAL
#define FLOW_RESUME_S      300U     /* after circulation returns: fresh return water must reach the pot */
#define FLOW_SUPPLY_BIT    0x10U    /* app_flowmon_alarms(): all four gutters low = pump/supply lost */
#define QLEN               FERTI_QLEN
#define PUMPS              FERTI_PUMPS
#define P_A                FERTI_P_A
#define P_B                FERTI_P_B
#define P_ACID             FERTI_P_ACID
#define P_BASE             FERTI_P_BASE
#define M_OFF              FERTI_M_OFF
#define M_AUTO             FERTI_M_AUTO
#define M_RELEASE          FERTI_M_RELEASE
#define AL_OFF             PUMPS
#define K_IDLE 0U
#define K_EC   1U
#define K_PH   2U

/* ---- hydraulics constants ---- */
#define T_NONE   FERTI_T_NONE
#define T_IDLE   FERTI_T_IDLE
#define T_FILL   FERTI_T_FILL
#define T_DOSE   FERTI_T_DOSE
#define T_STIR   FERTI_T_STIR
#define T_VERIFY FERTI_T_VERIFY
#define T_READY  FERTI_T_READY
#define T_FAULT  FERTI_T_FAULT
#define TAL_OVERFLOW 0U
#define TAL_FILL     1U
#define TAL_VERIFY   2U
#define TAL_RUNOFF   3U
#define TAL_CLOG     4U
#define TAL_LEAK     5U
#define TAL_FRESH    6U
#define TAL_LOW      7U             /* float-valve systems: level low with no inflow = valve failed / supply off */
#define LOW_DWELL_MS 120000UL       /* ... the low float must stay asserted this long before it counts (sloshing while the
                                     * irrigation pump draws, a float bobbing on the stop line; 2 min, user 2026-10-27) */
#define TAL_N        8U
#define CLOG_DROP_PCT 20U           /* irrigation rate down this much batch over batch = clogging */
#define FULL_STUCK_N  2U            /* full float still asserted after this many irrigations = dry run / blocked line */
#define FILL_OVERRUN_PCT 130U       /* inlet meter past this share of the batch with no full float = float stuck: stop, alarm */
#define LEAK_WIN_MS   3600000UL     /* inlet meter movement with the valve commanded off, per hour ... */
#define LEAK_DL       5U            /* ... beyond 0.5 L = valve not closing / leak */

/* ---- instance table ---- */
static ferti_sys_t *s_sys[FERTI_MAX];
static uint8_t      s_n = 0;
static uint8_t      s_started = 0;
static const char  *s_pump_name4[PUMPS] = { "a", "b", "acid", "base" };
static const uint32_t s_lref[3] = { LEARN_REF_ML_AB, LEARN_REF_ML_PH, LEARN_REF_ML_PH };
static const char  *s_tname[8] = { "-", "idle", "fill", "dose", "stir", "verify", "ready", "FAULT" };
static const char  *s_tal_name[TAL_N] = { "overflow", "fill", "verify", "runoff", "clog", "leak", "fresh", "low" };

ferti_sys_t *ferti_first(void) { return s_n ? s_sys[0] : NULL; }
uint8_t ferti_count(void) { return s_n; }

int ferti_owns_coil(uint16_t pt, uint8_t ch)
{
  for (uint8_t k = 0; k < s_n; k++)
  {
    const ferti_bind_t *b = s_sys[k]->b;
    const ferti_pin_t own[] = { b->pump[0], b->pump[1], b->pump[2], b->pump[3], b->fill, b->stir, b->irr, b->circ, b->air };
    _Static_assert(FERTI_PUMPS == 4U, "ferti_owns_coil lists four pumps");
    for (uint8_t i = 0; i < (uint8_t)(sizeof(own) / sizeof(own[0])); i++)
    { if ((own[i].pt == pt) && (own[i].ch == ch)) { return 1; } }
  }
  return 0;
}
ferti_sys_t *ferti_by_name(const char *name, int len)
{
  for (uint8_t k = 0; k < s_n; k++)
  { if (((int)strlen(s_sys[k]->b->name) == len) && (strncmp(name, s_sys[k]->b->name, (size_t)len) == 0)) { return s_sys[k]; } }
  return NULL;
}

static const char *mode_name(uint8_t m)
{ return (m == M_AUTO) ? "auto" : (m == M_RELEASE) ? "release" : "off"; }

static const char *pump_name(const ferti_sys_t *s, uint8_t k)
{ return (s->ph_single && (k >= P_ACID)) ? "ph" : s_pump_name4[k]; }

static int pin_ok(ferti_pin_t p) { return p.pt != FERTI_PIN_NONE; }

/* ---- event journal: text prefixed with the instance tag; the first instance keeps the exact
 * app_dose.c wording and source ("app") so nothing downstream changes ---- */
static void ev(ferti_sys_t *s, const char *type, const char *fmt, ...)
{
  char b[200]; va_list ap;
  va_start(ap, fmt); (void)vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
  if (s->idx == 0U) { app_log_event(type, "%s", b); } else { app_log_event_src(type, s->evsrc, "%s", b); }
}
static void evs(const char *type, const char *src, const char *fmt, ...)   /* explicit source (command origin) */
{
  char b[200]; va_list ap;
  va_start(ap, fmt); (void)vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
  app_log_event_src(type, src, "%s", b);
}

/* ---- I/O through the bindings ---- */
static uint16_t ai_read(const ferti_ai_t *a)
{
  int32_t v;
  uint16_t raw;
  if (!pin_ok(a->pin)) { return (uint16_t)SENS_INVAL; }
  if (app_io_ok(a->pin.pt) <= 0) { return (uint16_t)SENS_INVAL; }
  raw = app_io_ai(a->pin.pt, a->pin.ch);
  if (raw == a->inval) { return (uint16_t)SENS_INVAL; }
  v = (int32_t)raw * a->mul / ((a->div == 0) ? 1 : a->div) + a->off;
  if (v < 0) { v = 0; }
  if (v > 0x7FFE) { v = 0x7FFE; }
  return (uint16_t)v;
}

static uint32_t meter_read(ferti_sys_t *s)      /* 0.1 L since boot: pulse meter, or the virtual meter fed by batch refills / `topup` */
{
  const ferti_bind_t *b = s->b;
  if (b->meter.pt == FERTI_PT_HSDI)
  { return (uint32_t)((uint64_t)app_hsdi_count(b->meter.ch) * 10ULL / (b->meter_ppl ? b->meter_ppl : 1U)); }
  return s->vmeter_dl;
}

static void coil_set(ferti_pin_t p, uint8_t on) { if (pin_ok(p)) { app_io_do_set(p.pt, p.ch, on); } }

static uint8_t di_asserted(const ferti_sys_t *s, ferti_pin_t p, uint8_t invbit)
{
  uint8_t v;
  if (!pin_ok(p)) { return 0U; }
  if (app_io_ok(p.pt) <= 0) { return 0U; }     /* module gone: treat as not asserted (fail-quiet; the FAULT paths need a live module) */
  v = app_io_di(p.pt, p.ch) ? 1U : 0U;
  return (s->b->di_inv & invbit) ? (uint8_t)!v : v;
}

/* ---- seeds by kind ---- */
static void seed_param(uint8_t kind, ferti_param_t *p)
{
  memset(p, 0, sizeof *p);
  for (uint8_t k = 0; k < PUMPS; k++) { p->mlmin[k] = DEF_MLMIN; }
  p->ec_target = DEF_EC_TARGET; p->ec_db = DEF_EC_DB; p->ph_target = DEF_PH_TARGET; p->ph_db = DEF_PH_DB;
  p->ec_ml = DEF_EC_ML; p->acid_ml = DEF_ACID_ML; p->base_ml = DEF_BASE_ML;
  p->mix_s = DEF_MIX_S; p->tank_dl = DEF_TANK_DL; p->ph_dir = 0U;
  p->ph_src = 0U; p->ph_div = 10U; p->ec_src = 0U; p->ec_div = 100U;
  p->cap_ml[P_A] = DEF_CAP_AB_ML; p->cap_ml[P_B] = DEF_CAP_AB_ML; p->cap_ml[P_ACID] = DEF_CAP_ACID_ML; p->cap_ml[P_BASE] = DEF_CAP_BASE_ML;
  p->tz_min = 720;
  switch (kind)
  {
    case FERTI_RDWC:   /* small frequent shots, 90 L buckets, ammonium feed drifts pH down -> base (2026-10-24) */
      p->ec_ml = 20U; p->acid_ml = 5U; p->mix_s = 1800U; p->tank_dl = 900U; p->ph_dir = 1U; break;
    case FERTI_DWC:
      p->ec_ml = 10U; p->acid_ml = 3U; p->base_ml = 5U; p->mix_s = 1800U; p->tank_dl = 200U; p->ph_dir = 2U;
      p->cap_ml[P_A] = 200U; p->cap_ml[P_B] = 200U; p->cap_ml[P_ACID] = 30U; p->cap_ml[P_BASE] = 30U; break;
    case FERTI_DTW:    /* 20 L batch reservoir: feed-forward on 15 L refills, acid bottle (tap water pH 7+) */
      p->ec_target = 2400U; p->ec_db = 100U; p->ec_ml = 75U; p->acid_ml = 10U; p->base_ml = 10U;
      p->mix_s = 180U; p->tank_dl = 200U; p->ph_dir = 2U;
      p->cap_ml[P_A] = 400U; p->cap_ml[P_B] = 400U; p->cap_ml[P_ACID] = 60U; p->cap_ml[P_BASE] = 60U;
      p->batch_dl = 150U; p->fill_max_s = 900U; p->stir_s = 180U; p->verify_rounds = 3U;
      p->irr_per_day = 4U; p->irr_s = 60U; p->irr_start_h = 8U; p->irr_end_h = 18U;
      p->irr_src = 0U; p->irr_on_pct = 350U; p->irr_off_pct = 550U; p->runoff_wait_s = 180U; break;
    case FERTI_EBB:
      p->irr_per_day = 4U; p->irr_s = 600U; p->irr_start_h = 8U; p->irr_end_h = 18U; break;
    default: break;   /* FERTI_NFT = the greenhouse values above */
  }
}
static void seed_gain(uint8_t kind, uint32_t g[3])
{
  /* x1000 per ml: [0] uS per ml of A (B equal), [1] acid pH x100 per ml, [2] base pH x100 per ml.
   * The learning clamp is [1/5, 5x] of these, so a seed must be within a factor of 5 of the
   * truth for that system's bottles and volume. NFT = the 165 L greenhouse measurements with its
   * diluted working solutions. DTW = 20 L batch reservoir fed with concentrate at 5 ml/L for
   * 2.4 mS (2026-10-21): 1 ml of A in 20 L = +24 uS; pH pumps scale with 1 / volume. */
  switch (kind)
  {
    case FERTI_RDWC: g[0] = 513U;   g[1] = 2017U; g[2] = 2383U;  break;   /* 90 L buckets, greenhouse bottles */
    case FERTI_DWC:  g[0] = 2310U;  g[1] = 9075U; g[2] = 10725U; break;   /* 20 L bucket, greenhouse bottles */
    case FERTI_DTW:  g[0] = 24000U; g[1] = 9075U; g[2] = 10725U; break;   /* 20 L, concentrate */
    default:         g[0] = SEED_GAIN_AB; g[1] = SEED_GAIN_ACID; g[2] = SEED_GAIN_BASE; break;
  }
}

/* ---- parameter persistence (one text line, append-only fields; app_netcfg.c pattern) ----
 * 1..22 = the app_dose.c "dose.cfg" layout (cal_a cal_b cal_acid ec_target ec_db ph_target ph_db
 * ec_ml acid_ml mix_s boot_auto phsrc phdiv ecsrc ecdiv cal_base base_ml tank_dl ph_dir cap_ab
 * cap_acid cap_base); 23.. = hydraulics (batch_dl fill_max_s stir_s verify_rounds irr_per_day
 * irr_s irr_start_h irr_end_h irr_src irr_on irr_off runoff_wait_s tz_min). An older, shorter
 * file reads fine and the missing tail keeps the seeds. Saved on every parameter AND mode
 * command; loaded once at startup (retried from the task if the fs mounted late). */
static void cfg_save(ferti_sys_t *s)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  struct lfs_file_config fc = { .buffer = s->dfbuf };
  const ferti_param_t *p = &s->p;
  char buf[256];
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, s->cfg_file, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, &fc) != 0) { return; }
  snprintf(buf, sizeof(buf), "%u %u %u %u %u %u %u %u %u %lu %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %d\n",
           (unsigned)p->mlmin[P_A], (unsigned)p->mlmin[P_B], (unsigned)p->mlmin[P_ACID],
           (unsigned)p->ec_target, (unsigned)p->ec_db, (unsigned)p->ph_target, (unsigned)p->ph_db,
           (unsigned)p->ec_ml, (unsigned)p->acid_ml, (unsigned long)p->mix_s,
           (unsigned)((s->mode == M_AUTO) ? 1U : 0U),
           (unsigned)p->ph_src, (unsigned)p->ph_div, (unsigned)p->ec_src, (unsigned)p->ec_div,
           (unsigned)p->mlmin[P_BASE], (unsigned)p->base_ml, (unsigned)p->tank_dl, (unsigned)p->ph_dir,
           (unsigned)p->cap_ml[P_A], (unsigned)p->cap_ml[P_ACID], (unsigned)p->cap_ml[P_BASE],
           (unsigned)p->batch_dl, (unsigned)p->fill_max_s, (unsigned)p->stir_s, (unsigned)p->verify_rounds,
           (unsigned)p->irr_per_day, (unsigned)p->irr_s, (unsigned)p->irr_start_h, (unsigned)p->irr_end_h,
           (unsigned)p->irr_src, (unsigned)p->irr_on_pct, (unsigned)p->irr_off_pct, (unsigned)p->runoff_wait_s, (int)p->tz_min);
  (void)lfs_file_write(fs, &f, buf, strlen(buf));
  (void)lfs_file_close(fs, &f);
}

static void cfg_load(ferti_sys_t *s)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  struct lfs_file_config fc = { .buffer = s->dfbuf };
  ferti_param_t *p = &s->p;
  char buf[256];
  unsigned ca, cb, cc, ect, ecd, pht, phd, eml, aml, bmode = 0, psrc = 0, pdiv = 10, esrc = 0, ediv = 100;
  unsigned cbs = DEF_MLMIN, bml = DEF_BASE_ML, tank = DEF_TANK_DL, pdir = 0;
  unsigned cap_ab = DEF_CAP_AB_ML, cap_ac = DEF_CAP_ACID_ML, cap_bs = DEF_CAP_BASE_ML;
  unsigned bdl = 0, fmx = 0, sts = 0, vr = 0, ipd = 0, irs = 0, ish = 0, ieh = 0, isrc = 0, ion = 0, ioff = 0, rws = 0;
  int tz = 720;
  unsigned long mix;
  int nf;
  if (s->ploaded || (fs == NULL)) { return; }
  s->ploaded = 1U;                            /* fs is up: one attempt, file may not exist yet */
  if (lfs_file_opencfg(fs, &f, s->cfg_file, LFS_O_RDONLY, &fc) != 0) { return; }
  lfs_ssize_t n = lfs_file_read(fs, &f, buf, sizeof(buf) - 1U);
  (void)lfs_file_close(fs, &f);
  if (n <= 0) { return; }
  buf[n] = 0;
  nf = sscanf(buf, "%u %u %u %u %u %u %u %u %u %lu %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %d",
              &ca, &cb, &cc, &ect, &ecd, &pht, &phd, &eml, &aml, &mix, &bmode, &psrc, &pdiv, &esrc, &ediv,
              &cbs, &bml, &tank, &pdir, &cap_ab, &cap_ac, &cap_bs,
              &bdl, &fmx, &sts, &vr, &ipd, &irs, &ish, &ieh, &isrc, &ion, &ioff, &rws, &tz);
  if (nf < 10) { return; }                   /* 10 fields = pre-boot-mode file, mode stays OFF */
  if (nf >= 13) { if (psrc <= 2U) { p->ph_src = (uint8_t)psrc; } if (pdiv <= 100U) { p->ph_div = (uint16_t)pdiv; } }
  if (nf >= 15) { if (esrc <= 2U) { p->ec_src = (uint8_t)esrc; } if (ediv <= 1000U) { p->ec_div = (uint16_t)ediv; } }
  if (nf >= 17)
  {
    if ((cbs >= 5U) && (cbs <= 600U))            { p->mlmin[P_BASE] = (uint16_t)cbs; }
    if ((bml >= 1U) && (bml <= MAN_MAX_BASE_ML)) { p->base_ml = (uint16_t)bml; }
  }
  if (nf >= 18) { if ((tank >= 200U) && (tank <= 50000U)) { p->tank_dl = (uint16_t)tank; } }
  if (nf >= 19) { if (pdir <= 2U) { p->ph_dir = (uint8_t)pdir; } }
  if (nf >= 22)
  {
    if ((cap_ab >= 50U) && (cap_ab <= CAP_MAX_AB_ML)) { p->cap_ml[P_A] = (uint16_t)cap_ab; p->cap_ml[P_B] = (uint16_t)cap_ab; }
    if ((cap_ac >= 5U)  && (cap_ac <= CAP_MAX_PH_ML)) { p->cap_ml[P_ACID] = (uint16_t)cap_ac; }
    if ((cap_bs >= 5U)  && (cap_bs <= CAP_MAX_PH_ML)) { p->cap_ml[P_BASE] = (uint16_t)cap_bs; }
  }
  if (nf >= 35)                              /* hydraulics tail (2026-10-26) */
  {
    if (bdl <= 10000U) { p->batch_dl = (uint16_t)bdl; }
    if (fmx <= 7200U)  { p->fill_max_s = (uint16_t)fmx; }
    if (sts <= 3600U)  { p->stir_s = (uint16_t)sts; }
    if (vr <= 20U)     { p->verify_rounds = (uint8_t)vr; }
    if (ipd <= 24U)    { p->irr_per_day = (uint8_t)ipd; }
    if (irs <= 3600U)  { p->irr_s = (uint16_t)irs; }
    if ((ish <= 23U) && (ieh <= 24U)) { p->irr_start_h = (uint8_t)ish; p->irr_end_h = (uint8_t)ieh; }
    if (isrc <= 1U)    { p->irr_src = (uint8_t)isrc; }
    if ((ion <= 1000U) && (ioff <= 1000U)) { p->irr_on_pct = (uint16_t)ion; p->irr_off_pct = (uint16_t)ioff; }
    if (rws <= 3600U)  { p->runoff_wait_s = (uint16_t)rws; }
    if ((tz >= -720) && (tz <= 840)) { p->tz_min = (int16_t)tz; }
  }
  /* the same bounds the commands enforce — a corrupt field keeps its seed */
  if ((ca >= 5U) && (ca <= 600U)) { p->mlmin[0] = (uint16_t)ca; }
  if ((cb >= 5U) && (cb <= 600U)) { p->mlmin[1] = (uint16_t)cb; }
  if ((cc >= 5U) && (cc <= 600U)) { p->mlmin[2] = (uint16_t)cc; }
  if ((ect >= 300U) && (ect <= 3000U)) { p->ec_target = (uint16_t)ect; }
  if ((ecd >= 5U) && (ecd <= 500U))    { p->ec_db = (uint16_t)ecd; }
  if ((pht >= 400U) && (pht <= 700U))  { p->ph_target = (uint16_t)pht; }
  if ((phd >= 5U) && (phd <= 100U))    { p->ph_db = (uint16_t)phd; }
  if ((eml >= 1U) && (eml <= MAN_MAX_ML))      { p->ec_ml = (uint16_t)eml; }
  if ((aml >= 1U) && (aml <= MAN_MAX_ACID_ML)) { p->acid_ml = (uint16_t)aml; }
  if ((mix >= 60UL) && (mix <= 7200UL)) { p->mix_s = (uint32_t)mix; }
  if ((nf >= 11) && (bmode == 1U) && (s->mode == M_OFF))
  {
    /* boot restore (2026-09-02): re-arm auto, but sit out one full mixing wait first —
     * a reset storm gets at most zero shots per boot, and the very first evaluation
     * still needs valid sensors + the sustained-low dwell like any other. */
    s->mode = M_AUTO;
    s->wait_s = p->mix_s;
    ev(s, "SYSTEM", "%s auto restored from cfg (grace %lus before first eval)", s->tag, (unsigned long)p->mix_s);
  }
  printf("[DOSE] %s params from %s: cal=%u/%u/%u/%u ec=%u\xC2\xB1%u ph=%u\xC2\xB1%u mix=%lus mode=%s\n\r",
         s->b->name, s->cfg_file,
         (unsigned)p->mlmin[0], (unsigned)p->mlmin[1], (unsigned)p->mlmin[2], (unsigned)p->mlmin[3],
         (unsigned)p->ec_target, (unsigned)p->ec_db, (unsigned)p->ph_target, (unsigned)p->ph_db,
         (unsigned long)p->mix_s, mode_name(s->mode));
}

/* ---- learned model persistence (one line, all integers) ----
 * gain_ab gain_acid gain_base drift_ph drift_ec [fill_dl sum_ml0 sum_resp0 sum_ml1 sum_resp1
 * sum_ml2 sum_resp2] [fill_mls irr_mls]. An older 5-field file re-seeds the sums from its gains.
 * Written on every learning update; a missing/corrupt file keeps the seeds. `learn reset` deletes it. */
static void learn_seed_sums(ferti_sys_t *s, uint8_t slot)   /* make the sums agree with gain[slot] as one reference-size sample */
{
  s->L.lsum_ml[slot]   = (int32_t)(s_lref[slot] * 100U);
  s->L.lsum_resp[slot] = (int32_t)(s_lref[slot] * s->L.gain[slot]);
}

static void learn_save(ferti_sys_t *s)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  struct lfs_file_config fc = { .buffer = s->dfbuf };
  const ferti_lrn_t *L = &s->L;
  char buf[200];
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, s->lrn_file, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, &fc) != 0) { return; }
  /* field 15 = irr_last (Unix s); older files have 14 fields and load with irr_last = 0 */
  snprintf(buf, sizeof(buf), "%lu %lu %lu %ld %ld %u %ld %ld %ld %ld %ld %ld %u %u %lu\n",
           (unsigned long)L->gain[0], (unsigned long)L->gain[1], (unsigned long)L->gain[2],
           (long)L->drift_ph, (long)L->drift_ec, (unsigned)L->tu_typ_dl,
           (long)L->lsum_ml[0], (long)L->lsum_resp[0], (long)L->lsum_ml[1], (long)L->lsum_resp[1],
           (long)L->lsum_ml[2], (long)L->lsum_resp[2], (unsigned)L->fill_mls, (unsigned)L->irr_mls,
           (unsigned long)L->irr_last);
  (void)lfs_file_write(fs, &f, buf, strlen(buf));
  (void)lfs_file_close(fs, &f);
}

static void learn_load(ferti_sys_t *s)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  struct lfs_file_config fc = { .buffer = s->dfbuf };
  ferti_lrn_t *L = &s->L;
  char buf[200];
  unsigned long g0, g1, g2; long dp, de; unsigned fill = 0; long sm[3] = { 0, 0, 0 }, sr[3] = { 0, 0, 0 };
  unsigned fmls = 0, imls = 0; unsigned long ilast = 0;
  int nf;
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, s->lrn_file, LFS_O_RDONLY, &fc) != 0) { return; }
  lfs_ssize_t n = lfs_file_read(fs, &f, buf, sizeof(buf) - 1U);
  (void)lfs_file_close(fs, &f);
  if (n <= 0) { return; }
  buf[n] = 0;
  nf = sscanf(buf, "%lu %lu %lu %ld %ld %u %ld %ld %ld %ld %ld %ld %u %u %lu", &g0, &g1, &g2, &dp, &de, &fill,
              &sm[0], &sr[0], &sm[1], &sr[1], &sm[2], &sr[2], &fmls, &imls, &ilast);
  if (nf < 5) { return; }
  /* same clamps the learner enforces: a corrupt value falls back to its seed */
  if ((g0 >= s->seed_gain[0] / 5U) && (g0 <= s->seed_gain[0] * 5U)) { L->gain[0] = (uint32_t)g0; }
  if ((g1 >= s->seed_gain[1] / 5U) && (g1 <= s->seed_gain[1] * 5U)) { L->gain[1] = (uint32_t)g1; }
  if ((g2 >= s->seed_gain[2] / 5U) && (g2 <= s->seed_gain[2] * 5U)) { L->gain[2] = (uint32_t)g2; }
  if ((dp >= -DRIFT_PH_MAX) && (dp <= DRIFT_PH_MAX)) { L->drift_ph = (int32_t)dp; }
  if ((de >= -DRIFT_EC_MAX) && (de <= DRIFT_EC_MAX)) { L->drift_ec = (int32_t)de; }
  if ((nf >= 6) && (fill <= 5000U)) { L->tu_typ_dl = (uint16_t)fill; }   /* <= 500 L per fill, else not learned */
  for (uint8_t k = 0; k < 3U; k++)
  {
    /* sums are trusted only when they reproduce the stored gain; otherwise re-seed from it. */
    if ((nf >= 12) && (sm[k] > 0) && (sm[k] <= 10000000L) && (sr[k] > 0) &&
        ((unsigned long)(sr[k] * 100L / sm[k]) >= L->gain[k] * 9U / 10U) && ((unsigned long)(sr[k] * 100L / sm[k]) <= L->gain[k] * 11U / 10U))
    { L->lsum_ml[k] = (int32_t)sm[k]; L->lsum_resp[k] = (int32_t)sr[k]; }
    else { learn_seed_sums(s, k); }
  }
  if (nf >= 14) { if (fmls <= 60000U) { L->fill_mls = (uint16_t)fmls; } if (imls <= 60000U) { L->irr_mls = (uint16_t)imls; } }
  if (nf >= 15) { L->irr_last = (uint32_t)ilast; s->irr_day = 0U; }   /* slot bookkeeping is rebuilt from it on the next scheduler pass */
  printf("[DOSE] %s learned from %s: gain ab=%lu acid=%lu base=%lu drift ph=%ld ec=%ld fill=%u.%uL\n\r",
         s->b->name, s->lrn_file, (unsigned long)L->gain[0], (unsigned long)L->gain[1], (unsigned long)L->gain[2],
         (long)L->drift_ph, (long)L->drift_ec, (unsigned)(L->tu_typ_dl / 10U), (unsigned)(L->tu_typ_dl % 10U));
}

static void learn_reset(ferti_sys_t *s)
{
  lfs_t *fs = app_lfs();
  s->L.gain[0] = s->seed_gain[0]; s->L.gain[1] = s->seed_gain[1]; s->L.gain[2] = s->seed_gain[2];
  s->L.drift_ph = SEED_DRIFT_PH; s->L.drift_ec = SEED_DRIFT_EC;
  for (uint8_t k = 0; k < 3U; k++) { learn_seed_sums(s, k); }
  s->L.tu_typ_dl = 0U; s->L.fill_mls = 0U; s->L.irr_mls = 0U;
  memset(s->strikes, 0, sizeof(s->strikes));
  memset(s->unlearned, 0, sizeof(s->unlearned));
  s->ff_ph = 0; s->ff_ec = 0; s->last.kind = K_IDLE;
  s->acc_dph = 0; s->acc_dec = 0; s->acc_dose_ph = 0; s->acc_dose_ec = 0; s->acc_t = 0;
  if (fs != NULL) { (void)lfs_remove(fs, s->lrn_file); }
}

/* ---- pumps and the shot queue (one pump at a time: A and B concentrates must not meet undiluted) ---- */
static void pump_assert(ferti_sys_t *s)       /* every tick, corrects stray writes (vent pattern) */
{
  if (s->ph_single)
  {
    /* one physical pH pump: it runs when EITHER the acid or the base slot is running */
    coil_set(s->b->pump[P_A], (uint8_t)(s->run == (int8_t)P_A));
    coil_set(s->b->pump[P_B], (uint8_t)(s->run == (int8_t)P_B));
    coil_set(s->b->pump[P_ACID], (uint8_t)((s->run == (int8_t)P_ACID) || (s->run == (int8_t)P_BASE)));
    return;
  }
  for (uint8_t k = 0; k < PUMPS; k++) { coil_set(s->b->pump[k], (uint8_t)(s->run == (int8_t)k)); }
}

static int shot_queue(ferti_sys_t *s, uint8_t pump, uint16_t ml, const char *src, const char *why)
{
  /* The queue is shared between this instance's task and the command context (console / MQTT):
   * every edit of q/qn/run is done inside a critical section (2026-11-17, review P5). */
  taskENTER_CRITICAL();
  if (s->qn >= QLEN) { taskEXIT_CRITICAL(); return -1; }
  s->q[s->qn].pump = pump; s->q[s->qn].ml = ml; s->qn++;
  taskEXIT_CRITICAL();
  if (strcmp(src, "auto") != 0) { s->confound = 1U; }   /* hand shots are not the model's doing */
  evs("ACTION", src, "%s %s %uml (%lus) %s ec=%u ph=%u", s->tag, pump_name(s, pump), (unsigned)ml,
      (unsigned long)(ml * 60UL / s->p.mlmin[pump]), why, (unsigned)s->ec, (unsigned)s->ph);
  return 0;
}

static void shot_start_next(ferti_sys_t *s)
{
  taskENTER_CRITICAL();                   /* a stop/off from the command context must not land between the check and the pop */
  if ((s->run >= 0) || (s->qn == 0U)) { taskEXIT_CRITICAL(); return; }
  s->run    = (int8_t)s->q[0].pump;
  s->run_ms = (uint32_t)s->q[0].ml * 60000UL / s->p.mlmin[s->q[0].pump];
  {                                       /* count what was commanded (caps err on the safe side) */
    uint32_t v = (uint32_t)s->ml_h[s->hi][s->q[0].pump] + s->q[0].ml;
    s->ml_h[s->hi][s->q[0].pump] = (v > 65535UL) ? 65535U : (uint16_t)v;
  }
  memmove(&s->q[0], &s->q[1], sizeof(s->q) - sizeof(s->q[0]));
  s->qn--;
  taskEXIT_CRITICAL();
}

static void shots_abort(ferti_sys_t *s)
{
  taskENTER_CRITICAL();
  s->run = -1; s->run_ms = 0; s->qn = 0;
  taskEXIT_CRITICAL();
}

/* ---- 24 h caps (rolling) + cloud alarms ---- */
static uint32_t ml_24h(const ferti_sys_t *s, uint8_t pump)
{
  uint32_t sum = 0;
  for (uint8_t h = 0; h < CAP_HOURS; h++) { sum += s->ml_h[h][pump]; }
  return sum;
}

static uint32_t ml_24h_slot(const ferti_sys_t *s, uint8_t slot)     /* A/B: the larger of the pair */
{
  uint32_t used = ml_24h(s, slot);
  if (slot == P_A) { uint32_t ub = ml_24h(s, P_B); if (ub > used) { used = ub; } }
  return used;
}

static void caps_clear(ferti_sys_t *s) { memset(s->ml_h, 0, sizeof(s->ml_h)); s->hi = 0; s->hour_s = 0; }

static void dose_alarm(ferti_sys_t *s, uint8_t slot, uint8_t active) { s->al_pend[slot] = active ? 1U : 2U; }

static const char *cap_name(const ferti_sys_t *s, uint8_t slot) { return (slot == P_A) ? "A/B" : pump_name(s, slot); }

/* Fit a shot under the pump's 24 h cap. Returns 0 with *ml possibly TRIMMED to the room left in
 * the window, or 1 when the loop that owns the pump is held (shot refused). The hold starts when
 * the room is below a minimum shot (FAULT journaled + alarm pushed once) and is sticky: only
 * caps_release_poll() lifts it, after CAP_RELEASE_DIV-th of the cap has dropped out of the window.
 * 2026-11-16: hold used to test `used + this shot` while release tested `used + minimum shot`, so
 * any shot above the minimum was refused, released 5 s later and refused again next cycle - a
 * FAULT/SYSTEM pair every cycle for as long as the tank stayed cap-limited.
 * A and B always dose the same volume, so the pair is one hold (slot P_A, bits A|B). */
#define CAP_RELEASE_DIV    8U       /* release once cap/8 is free again (never less than a minimum shot) */

static uint32_t cap_room(const ferti_sys_t *s, uint8_t slot)
{
  uint32_t used = ml_24h_slot(s, slot);
  return (used < s->p.cap_ml[slot]) ? ((uint32_t)s->p.cap_ml[slot] - used) : 0U;
}

static int cap_blocked(ferti_sys_t *s, uint8_t pump, uint32_t *ml)
{
  uint8_t  slot  = (pump <= P_B) ? P_A : pump;
  uint8_t  bits  = (slot == P_A) ? 3U : (uint8_t)(1U << slot);
  uint32_t minml = (slot == P_A) ? MIN_SHOT_AB_ML : MIN_SHOT_PH_ML;
  uint32_t room  = cap_room(s, slot);
  if (s->caphold & bits) { return 1; }                       /* sticky: caps_release_poll() decides */
  if (*ml <= room) { return 0; }
  if (room >= minml) { *ml = room; return 0; }               /* use what is left instead of refusing */
  s->caphold |= bits;
  ev(s, "FAULT", "%s %s 24h cap %uml reached (%luml used) -> %s loop held, other loop continues; CHECK tube in tank / stock bottle / pump",
     s->tag, cap_name(s, slot), (unsigned)s->p.cap_ml[slot], (unsigned long)ml_24h_slot(s, slot), (slot == P_A) ? "EC" : "pH");
  dose_alarm(s, slot, 1U);
  return 1;
}

static void caps_release_poll(ferti_sys_t *s)       /* every 5 s in auto: the oldest hour dropping out frees the window */
{
  static const uint8_t slots[3] = { P_A, P_ACID, P_BASE };
  for (uint8_t k = 0; k < 3U; k++)
  {
    uint8_t  slot  = slots[k];
    uint8_t  bits  = (slot == P_A) ? 3U : (uint8_t)(1U << slot);
    uint32_t need  = (slot == P_A) ? MIN_SHOT_AB_ML : MIN_SHOT_PH_ML;
    if (((uint32_t)s->p.cap_ml[slot] / CAP_RELEASE_DIV) > need) { need = (uint32_t)s->p.cap_ml[slot] / CAP_RELEASE_DIV; }
    if ((s->caphold & bits) && (cap_room(s, slot) >= need))
    {
      s->caphold &= (uint8_t)~bits;
      ev(s, "SYSTEM", "%s %s cap released (%luml in last 24h, cap %u)", s->tag, cap_name(s, slot), (unsigned long)ml_24h_slot(s, slot), (unsigned)s->p.cap_ml[slot]);
      dose_alarm(s, slot, 0U);
    }
  }
}

static void dose_off_fault(ferti_sys_t *s, const char *reason)   /* a fault dropped auto to OFF: persist + alarm the phone */
{
  s->mode = M_OFF; cfg_save(s);
  snprintf(s->off_reason, sizeof(s->off_reason), "%s", reason);
  s->off_al = 1U;
  dose_alarm(s, AL_OFF, 1U);
}

/* alarm ids: first instance "al.dose.<slot>.cap" / "al.dose.off", src "dose" (unchanged);
 * others "al.dose.<name>.<slot>.cap" / "al.dose.<name>.off", src "dose.<name>" */
static void alarm_id(const ferti_sys_t *s, char *b, size_t cap, const char *leaf)
{
  if (s->idx == 0U) { snprintf(b, cap, "al.dose.%s", leaf); }
  else { snprintf(b, cap, "al.dose.%s.%s", s->b->name, leaf); }
}

/* one attempt per 5 s tick per pending slot; the uplink slot is shared with the datalog mirror */
static void flush_events(ferti_sys_t *s)
{
  for (uint8_t k = 0; k <= PUMPS; k++)
  {
    char js[320], id[40], leaf[24];
    int  n;
    uint8_t active = (s->al_pend[k] == 1U);
    if (s->al_pend[k] == 0U) { continue; }
    if (!app_mqtt_ready() || app_mqtt_pub_updata_busy()) { return; }
    if (k < PUMPS)
    {
      uint32_t used = ml_24h_slot(s, k);
      snprintf(leaf, sizeof leaf, "%s.cap", (k == P_A) ? "ab" : pump_name(s, k));
      alarm_id(s, id, sizeof id, leaf);
      n = snprintf(js, sizeof js,
                   "{\"pv\":1,\"ts\":%lu,\"id\":\"%s\",\"src\":\"%s\",\"sev\":\"warn\",\"state\":\"%s\","
                   "\"msg\":\"Dosing %s: 24h cap %u ml %s (%lu ml in last 24h)\",\"val\":%lu}",
                   (unsigned long)app_time_now(), id, s->evsrc, active ? "active" : "clear",
                   cap_name(s, k), (unsigned)s->p.cap_ml[k],
                   active ? "reached - loop held. CHECK: tube still in tank, stock bottle not empty, pump drawing" : "released",
                   (unsigned long)used, (unsigned long)used);
    }
    else
    {
      alarm_id(s, id, sizeof id, "off");
      n = snprintf(js, sizeof js,
                   "{\"pv\":1,\"ts\":%lu,\"id\":\"%s\",\"src\":\"%s\",\"sev\":\"alarm\",\"state\":\"%s\","
                   "\"msg\":\"Dosing %s\",\"val\":0}",
                   (unsigned long)app_time_now(), id, s->evsrc, active ? "active" : "clear",
                   active ? s->off_reason : "auto re-armed");
    }
    if ((n > 0) && (app_mqtt_pub_updata("event", js, (uint16_t)n) == 0)) { s->al_pend[k] = 0U; }
    else { return; }
  }
  /* hydraulics alarms: "al.tank.<name>.<what>" */
  for (uint8_t k = 0; k < TAL_N; k++)
  {
    char js[220];
    int n;
    uint8_t active = (s->tal_pend[k] == 1U);
    if (s->tal_pend[k] == 0U) { continue; }
    if (!app_mqtt_ready() || app_mqtt_pub_updata_busy()) { return; }
    n = snprintf(js, sizeof js,
                 "{\"pv\":1,\"ts\":%lu,\"id\":\"al.tank.%s.%s\",\"src\":\"%s\",\"sev\":\"%s\",\"state\":\"%s\","
                 "\"msg\":\"Tank %s: %s %s\",\"val\":%u}",
                 (unsigned long)app_time_now(), s->b->name, s_tal_name[k], s->evsrc,
                 ((k == TAL_OVERFLOW) || (k == TAL_FILL) || (k == TAL_FRESH) || (k == TAL_LOW)) ? "alarm" : "warn",
                 active ? "active" : "clear", s->b->name, s_tal_name[k], active ? s->fault_why : "cleared", (unsigned)s->tstate);
    if ((n > 0) && (app_mqtt_pub_updata("event", js, (uint16_t)n) == 0)) { s->tal_pend[k] = 0U; }
    else { return; }
  }
}

/* effective pH per the source policy. Returns SENS_INVAL when the chosen source is dead OR
 * (mean mode) the probes disagree beyond ph_div — the pH loop then simply never accumulates
 * its trigger, while the EC loop keeps running on its own probe. */
static uint16_t ph_read(ferti_sys_t *s)
{
  uint16_t p1 = ai_read(&s->b->ph[0]);
  uint16_t p2 = ai_read(&s->b->ph[1]);
  if (s->p.ph_src == 1U) { return p1; }
  if (s->p.ph_src == 2U) { return p2; }
  if (p1 == (uint16_t)SENS_INVAL) { return p2; }  /* one probe faulted: the survivor serves */
  if (p2 == (uint16_t)SENS_INVAL) { return p1; }
  {
    uint16_t d = (p1 > p2) ? (uint16_t)(p1 - p2) : (uint16_t)(p2 - p1);
    if ((s->p.ph_div != 0U) && (d > s->p.ph_div))
    {
      if (!s->ph_src_bad)
      {
        s->ph_src_bad = 1U;
        ev(s, "FAULT", "ph probes disagree %u.%02u (ph1=%u ph2=%u) -> pH dosing hold",
           (unsigned)(d / 100U), (unsigned)(d % 100U), (unsigned)p1, (unsigned)p2);
      }
      return (uint16_t)SENS_INVAL;
    }
    if (s->ph_src_bad)
    {
      s->ph_src_bad = 0U;
      ev(s, "SYSTEM", "ph probes agree again (ph1=%u ph2=%u)", (unsigned)p1, (unsigned)p2);
    }
    return (uint16_t)((p1 + p2) / 2U);
  }
}

/* effective EC per the source policy (2026-09-04): same shape as ph_read. */
static uint16_t ec_read(ferti_sys_t *s)
{
  uint16_t e1 = ai_read(&s->b->ec[0]);
  uint16_t e2 = ai_read(&s->b->ec[1]);
  if (s->p.ec_src == 1U) { return e1; }
  if (s->p.ec_src == 2U) { return e2; }
  if (e1 == (uint16_t)SENS_INVAL) { return e2; }
  if (e2 == (uint16_t)SENS_INVAL) { return e1; }
  {
    uint16_t d = (e1 > e2) ? (uint16_t)(e1 - e2) : (uint16_t)(e2 - e1);
    if ((s->p.ec_div != 0U) && (d > s->p.ec_div))
    {
      if (!s->ec_src_bad)
      {
        s->ec_src_bad = 1U;
        ev(s, "FAULT", "ec probes disagree %uuS (ec1=%u ec2=%u) -> dosing hold", (unsigned)d, (unsigned)e1, (unsigned)e2);
      }
      return (uint16_t)SENS_INVAL;
    }
    if (s->ec_src_bad)
    {
      s->ec_src_bad = 0U;
      ev(s, "SYSTEM", "ec probes agree again (ec1=%u ec2=%u)", (unsigned)e1, (unsigned)e2);
    }
    return (uint16_t)((e1 + e2) / 2U);
  }
}

/* circulation interlock, every 5 s in auto mode (also while a shot is running) */
static void flowlock(ferti_sys_t *s)
{
  if (s->b->flow_lock && (app_flowmon_alarms() & FLOW_SUPPLY_BIT))
  {
    if (!s->flowhold)
    {
      s->flowhold = 1U;
      shots_abort(s);
      s->confound = 1U;
      ev(s, "FAULT", "%s hold: feed flow lost on all gutters -> pumps off, no dosing", s->tag);
    }
    return;
  }
  if (s->flowhold)
  {
    s->flowhold = 0U;
    s->confound = 1U;
    if (s->wait_s < FLOW_RESUME_S) { s->wait_s = FLOW_RESUME_S; }
    ev(s, "SYSTEM", "%s resume: feed flow back, %us grace for fresh return water", s->tag, (unsigned)FLOW_RESUME_S);
  }
}

/* one learning step from what the previous cycle did (v2) */
static void learn_step(ferti_sys_t *s, uint16_t ec, uint16_t ph)
{
  ferti_lrn_t *L = &s->L;
  int32_t dt_s  = (int32_t)s->p.mix_s;                      /* cycle length = what elapsed */
  int     topup = (meter_read(s) != s->last.flow0);          /* the valve fed the tank: dilution + alkalinity */
  int     ph_ok = (ph != (uint16_t)SENS_INVAL) && (s->last.ph0 != (uint16_t)SENS_INVAL);
  int32_t d_ec  = (int32_t)ec - (int32_t)s->last.ec0;                   /* sensor units */
  int32_t d_ec_f = d_ec * 1000L + s->last.dil;                          /* fine, with a top-up episode's known dilution added back */
  int32_t d_ph  = ph_ok ? ((int32_t)ph - (int32_t)s->last.ph0) : 0;
  int32_t dosed_ec = 0, dosed_ph = 0;                        /* what the last shot was expected to do (fine units) */

  if (s->confound || topup)                                  /* not the pumps' doing: drop the window too */
  {
    s->confound = 0U; s->last.kind = K_IDLE; s->last.ph_ml = 0U;
    s->acc_dph = 0; s->acc_dec = 0; s->acc_dose_ph = 0; s->acc_dose_ec = 0; s->acc_t = 0;
    return;
  }

  if (s->last.kind != K_IDLE)                                /* a shot fired: learn that pump's gain */
  {
    uint8_t  slot = s->last.slot;
    uint32_t seed = s->seed_gain[slot];
    const char *nm = (slot == 0U) ? "ab" : pump_name(s, (slot == 1U) ? P_ACID : P_BASE);
    int32_t  expect = (int32_t)(L->gain[slot] * s->last.ml);            /* fine units */
    int32_t  lmin   = ((s->last.kind == K_EC) ? LEARN_MIN_EC  : LEARN_MIN_PH)  * 1000L;
    int32_t  smin   = ((s->last.kind == K_EC) ? STRIKE_MIN_EC : STRIKE_MIN_PH) * 1000L;
    int32_t  resp, obs;
    if (s->last.kind == K_EC) { dosed_ec = expect; resp = d_ec_f - (L->drift_ec * dt_s / 3600L); }
    else
    {
      dosed_ph = (slot == 1U) ? -expect : expect;
      resp = ph_ok ? (d_ph * 1000L - (L->drift_ph * dt_s / 3600L)) : 0;
      if (slot == 1U) { resp = -resp; }                      /* acid: a good shot moves pH DOWN */
    }
    if (expect < lmin) { if (s->unlearned[slot] < 255U) { s->unlearned[slot]++; } }   /* too small to teach: count toward a probe */
    if ((expect >= lmin) && ((s->last.kind == K_EC) || ph_ok))
    {
      obs = resp / (int32_t)s->last.ml;                      /* x1000 per ml, same scale as gain */
      if (obs < (int32_t)(L->gain[slot] / 10U))              /* less than a tenth of the expected move */
      {
        if ((expect >= smin) && (++s->strikes[slot] >= NORESP_STRIKES))
        {
          ev(s, "FAULT", "%s %s: %u shots with no response (last %uml moved %ld) -> auto off",
             s->tag, nm, (unsigned)s->strikes[slot], (unsigned)s->last.ml,
             (long)((s->last.kind == K_EC) ? d_ec : d_ph));
          s->strikes[slot] = 0;
          { char r[40]; snprintf(r, sizeof r, "AUTO OFF: %s pump no response", nm); dose_off_fault(s, r); }
        }
      }
      else
      {
        /* sum-ratio estimator with forgetting (2026-09-15): gain = sum(resp) / sum(ml); both sums
         * weighted identically (ml in 0.01 ml fixed point, forgetting in basis points, 2026-09-18) */
        uint32_t old = L->gain[slot];
        int32_t  g;
        int32_t  w_pct = expect * 100L / (lmin * 4L);                  /* sample weight by expected move: full from 4x LEARN_MIN */
        int32_t  wml_c, keep_bp;
        if (w_pct < 10) { w_pct = 10; }
        if (w_pct > 100) { w_pct = 100; }
        wml_c   = (int32_t)s->last.ml * w_pct;                          /* effective ml x100 this sample adds */
        keep_bp = 10000L - wml_c * 100L / LEARN_WINDOW_ML;              /* forget in proportion to what comes in */
        if (keep_bp < LEARN_MIN_KEEP_PCT * 100L) { keep_bp = LEARN_MIN_KEEP_PCT * 100L; }
        L->lsum_ml[slot]   = (int32_t)((int64_t)L->lsum_ml[slot]   * keep_bp / 10000L) + wml_c;
        L->lsum_resp[slot] = (int32_t)((int64_t)L->lsum_resp[slot] * keep_bp / 10000L) + (int32_t)((int64_t)resp * w_pct / 100L);
        g = (L->lsum_ml[slot] > 0) ? (int32_t)((int64_t)L->lsum_resp[slot] * 100L / L->lsum_ml[slot]) : (int32_t)old;
        if ((g < (int32_t)(seed / 5U)) || (g > (int32_t)(seed * 5U)))   /* clamped: keep the sums consistent with the clamp */
        {
          g = (g < (int32_t)(seed / 5U)) ? (int32_t)(seed / 5U) : (int32_t)(seed * 5U);
          L->gain[slot] = (uint32_t)g; learn_seed_sums(s, slot);
        }
        L->gain[slot] = (uint32_t)g; s->strikes[slot] = 0; s->unlearned[slot] = 0;
        learn_save(s);
        ev(s, "SYSTEM", "%s learn %s gain %lu->%lu (x1000/ml, shot %uml moved %ld%s, %ld/ml over %ld.%02ldml)",
           s->tag, nm, (unsigned long)old, (unsigned long)L->gain[slot], (unsigned)s->last.ml,
           (long)((s->last.kind == K_EC) ? (d_ec_f / 1000L) : d_ph), s->last.dil ? " net of top-up" : "",
           (long)obs, (long)(L->lsum_ml[slot] / 100), (long)(L->lsum_ml[slot] % 100));
      }
    }
    s->last.kind = K_IDLE;
  }
  if (s->last.ph_ml != 0U)                                   /* pH shot alongside an A/B shot: expected effect into the drift books only */
  {
    int32_t expect = (int32_t)(L->gain[s->last.ph_slot] * s->last.ph_ml);
    dosed_ph += (s->last.ph_slot == 1U) ? -expect : expect;
    if (s->unlearned[s->last.ph_slot] < 255U) { s->unlearned[s->last.ph_slot]++; }
    s->last.ph_ml = 0U;
  }

  /* drift window: every clean cycle adds its delta minus the shot's expected effect; after
   * DRIFT_WIN_S the residual over time is the drift (fertilizer acidity, uptake, CO2) */
  s->acc_dec += d_ec_f; s->acc_dose_ec += dosed_ec;
  if (ph_ok) { s->acc_dph += d_ph * 1000L; s->acc_dose_ph += dosed_ph; }
  s->acc_t += (uint32_t)dt_s;
  if (s->acc_t >= DRIFT_WIN_S)
  {
    int32_t obs_ec = (s->acc_dec - s->acc_dose_ec) / (int32_t)(s->acc_t / 3600U);   /* x1000 per h (window >= 3 h) */
    int32_t obs_ph = (s->acc_dph - s->acc_dose_ph) / (int32_t)(s->acc_t / 3600U);
    if ((obs_ec >= -DRIFT_EC_MAX) && (obs_ec <= DRIFT_EC_MAX))
    { L->drift_ec += (obs_ec - L->drift_ec) * (int32_t)DRIFT_ALPHA_PCT / 100; }
    if ((obs_ph >= -DRIFT_PH_MAX) && (obs_ph <= DRIFT_PH_MAX))
    { L->drift_ph += (obs_ph - L->drift_ph) * (int32_t)DRIFT_ALPHA_PCT / 100; }
    learn_save(s);
    ev(s, "SYSTEM", "%s learn drift ph %ld ec %ld (x1000/h; %luh window moved ph %ld ec %ld, dosed %ld/%ld)",
       s->tag, (long)L->drift_ph, (long)L->drift_ec, (unsigned long)(s->acc_t / 3600U),
       (long)(s->acc_dph / 1000L), (long)(s->acc_dec / 1000L),
       (long)(s->acc_dose_ph / 1000L), (long)(s->acc_dose_ec / 1000L));
    s->acc_dph = 0; s->acc_dec = 0; s->acc_dose_ph = 0; s->acc_dose_ec = 0; s->acc_t = 0;
  }
}

/* 60 s means (2026-09-15): called once per 5 s reading, in every mode that reads the sensors.
 * A mean is valid once at least half the window holds valid reads; else SENS_INVAL. */
static uint16_t ring_mean(const ferti_sys_t *s, const uint16_t *r)
{
  uint32_t sum = 0; uint8_t n = 0;
  for (uint8_t k = 0; k < s->ring_n; k++) { if (r[k] != (uint16_t)SENS_INVAL) { sum += r[k]; n++; } }
  return (n >= (uint8_t)(AVG_N / 2U)) ? (uint16_t)((sum + n / 2U) / n) : (uint16_t)SENS_INVAL;
}
static void sens_track(ferti_sys_t *s, uint16_t ec, uint16_t ph)
{
  s->ec_ring[s->ring_i] = ec; s->ph_ring[s->ring_i] = ph;
  s->ring_i = (uint8_t)((s->ring_i + 1U) % AVG_N);
  if (s->ring_n < AVG_N) { s->ring_n++; }
  s->ec_avg = ring_mean(s, s->ec_ring);
  s->ph_avg = ring_mean(s, s->ph_ring);
}

static void chem_eval(ferti_sys_t *s)        /* every 5 s, auto mode, pumps idle, circulation ok */
{
  const ferti_param_t *p = &s->p;
  ferti_lrn_t *L = &s->L;
  uint16_t ec = ec_read(s), ph = ph_read(s);
  s->ec = ec;
  s->ph = ph;
  sens_track(s, s->ec, s->ph);
  /* EC gates the eval (fertilizer is the primary loop); a dead/untrusted pH only
   * stops the pH branch — A/B dosing continues */
  int valid = (ec != SENS_INVAL) && (ec >= EC_DRY_FLOOR);

  if (!valid)                             /* fail-silent: never dose on bad data */
  {
    s->confound = 1U;
    s->bad_s += 5U;
    if ((s->bad_s >= SENS_BAD_S) && !s->faulted)
    {
      s->faulted = 1;
      ev(s, "FAULT", "%s sensors invalid (ec=%u ph=%u) -> no dosing", s->tag, (unsigned)ec, (unsigned)ph);
    }
    return;
  }
  if (s->faulted) { s->faulted = 0; ev(s, "SYSTEM", "%s sensors back ec=%u ph=%u", s->tag, (unsigned)ec, (unsigned)ph); }
  s->bad_s = 0;

  if (s->wait_s > 0U)                     /* inside the cycle: let the tank settle */
  { s->wait_s = (s->wait_s > 5U) ? (s->wait_s - 5U) : 0U; return; }
  if (s->tu_in) { return; }               /* valve open: the top-up feed-forward is dosing on volume; decide once it closes */
  if (s->hold) { return; }                /* hydraulics: filling / stirring - the probe does not see a mixed tank yet */

  /* ---- decision point: close the books on the last cycle, then decide this one ---- */
  if (s->ec_avg != (uint16_t)SENS_INVAL) { ec = s->ec_avg; }
  if ((ph != (uint16_t)SENS_INVAL) && (s->ph_avg != (uint16_t)SENS_INVAL)) { ph = s->ph_avg; }
  learn_step(s, ec, ph);

  {
    int32_t dt_s   = (int32_t)p->mix_s;
    int32_t err_ec = (int32_t)p->ec_target - (int32_t)ec;                /* uS, + = short */
    int32_t need_ec, ml_ab = 0;
    int32_t err_ph = 0, need_ph = 0, ml_ph = 0;
    uint8_t ph_ok  = (ph != (uint16_t)SENS_INVAL);

    s->ff_ec = s->tu_carry - (L->drift_ec * dt_s / 3600L);               /* predicted uptake over the coming cycle + top-up remainder (fine) */
    if (s->ff_ec < 0) { s->ff_ec = 0; }                                   /* EC only has an UP pump */
    if (s->ff_ec > (int32_t)(p->ec_db * 2000U)) { s->ff_ec = (int32_t)(p->ec_db * 2000U); }
    need_ec = err_ec * 1000L + s->ff_ec;                                  /* fine units */
    if (err_ec < -(int32_t)p->ec_db) { need_ec = 0; s->ff_ec = 0; s->tu_carry = 0; }   /* above band: wait for uptake/top-up */
    if (need_ec > 0) { ml_ab = need_ec / (int32_t)L->gain[0]; }           /* fine / (fine per ml) = ml */
    if (ml_ab > (int32_t)p->ec_ml) { ml_ab = (int32_t)p->ec_ml; }        /* `shot` = per-cycle cap */

    if (ph_ok)
    {
      err_ph = (int32_t)p->ph_target - (int32_t)ph;                       /* x100, + = too acidic */
      s->ff_ph = -(L->drift_ph * dt_s / 3600L);                           /* one cycle of predicted drift (fine) */
      if (s->ff_ph > (int32_t)(p->ph_db * 2000U))  { s->ff_ph = (int32_t)(p->ph_db * 2000U); }
      if (s->ff_ph < -(int32_t)(p->ph_db * 2000U)) { s->ff_ph = -(int32_t)(p->ph_db * 2000U); }
      need_ph = err_ph * 1000L + s->ff_ph;
      /* (a) `phdir`: one fertiliser drifts pH one way, the opposite pump only fights what the other just put in;
       * (b) inside the band never dose against the learned drift - the drift is the free actuator back to target */
      if (((p->ph_dir == 1U) && (need_ph < 0)) || ((p->ph_dir == 2U) && (need_ph > 0))) { need_ph = 0; }
      if ((err_ph >= -(int32_t)p->ph_db) && (err_ph <= (int32_t)p->ph_db) &&
          (((need_ph < 0) && (L->drift_ph < 0)) || ((need_ph > 0) && (L->drift_ph > 0)))) { need_ph = 0; }
      if (need_ph > 0)      { ml_ph = need_ph / (int32_t)L->gain[2];  if (ml_ph > (int32_t)p->base_ml) { ml_ph = (int32_t)p->base_ml; } }
      else if (need_ph < 0) { ml_ph = -need_ph / (int32_t)L->gain[1]; if (ml_ph > (int32_t)p->acid_ml) { ml_ph = (int32_t)p->acid_ml; } }
    }

    s->last.kind = K_IDLE; s->last.ec0 = ec; s->last.ph0 = ph; s->last.flow0 = meter_read(s); s->last.dil = 0; s->last.ph_ml = 0U;

    if (ml_ab >= (int32_t)MIN_SHOT_AB_ML)
    {
      uint32_t fit = (uint32_t)ml_ab;                                     /* may come back trimmed to the cap's room */
      ml_ab = cap_blocked(s, P_A, &fit) ? 0 : (int32_t)fit;
    }   /* EC loop held by its cap: the pH branch still gets its turn */
    if (ml_ab >= (int32_t)MIN_SHOT_AB_ML)                                 /* EC first: fertilizer is the primary loop (queued first) */
    {
      (void)shot_queue(s, P_A, (uint16_t)ml_ab, "auto", "ec-need");
      (void)shot_queue(s, P_B, (uint16_t)ml_ab, "auto", "ec-need");
      s->last.kind = K_EC; s->last.slot = 0U; s->last.ml = (uint16_t)ml_ab; s->ff_ec = 0; s->tu_carry = 0;
    }
    if (ml_ph >= (int32_t)MIN_SHOT_PH_ML)                                 /* 2026-09-20: pH no longer waits for an A/B-free cycle */
    {
      uint8_t  pump = (need_ph > 0) ? P_BASE : P_ACID;
      uint8_t  slot = (need_ph > 0) ? 2U : 1U;
      int32_t  capc = (need_ph > 0) ? (int32_t)p->base_ml : (int32_t)p->acid_ml;
      const char *why = "ph-need";
      if ((s->last.kind != K_EC) && (s->unlearned[slot] >= PROBE_AFTER))   /* probe: the smallest shot the learner accepts */
      {
        int32_t probe = ((int32_t)LEARN_MIN_PH * 1000L + (int32_t)L->gain[slot] - 1) / (int32_t)L->gain[slot];
        if ((probe > ml_ph) && (probe <= capc)) { ml_ph = probe; why = "ph-need probe"; }
      }
      uint32_t fit = (uint32_t)ml_ph;
      if (!cap_blocked(s, pump, &fit))                                   /* held by its cap: no shot this cycle */
      {
        ml_ph = (int32_t)fit;                                             /* trimmed to the cap's room, never below the minimum */
        (void)shot_queue(s, pump, (uint16_t)ml_ph, "auto", why);
        if (s->last.kind == K_EC) { s->last.ph_ml = (uint16_t)ml_ph; s->last.ph_slot = slot; }   /* rides behind A/B: drift books only */
        else { s->last.kind = K_PH; s->last.slot = slot; s->last.ml = (uint16_t)ml_ph; }           /* alone: the learnable shot */
        s->ff_ph = 0;
      }
    }
    s->wait_s = p->mix_s;                                                 /* next decision one cycle later */
  }
}

/* one top-up compensation shot (A and B, equal). Returns ml queued (0 = nothing / cap hit).
 * Gate = queue room, NOT an idle pump. Same sensor guard as the loop. */
static int32_t topup_shot(ferti_sys_t *s, int32_t ml, const char *why)
{
  if ((ml < (int32_t)MIN_SHOT_AB_ML) || (s->qn > (uint8_t)(QLEN - 2U)) ||
      (s->ec == (uint16_t)SENS_INVAL) || (s->ec < EC_DRY_FLOOR)) { return 0; }
  if (ml > (int32_t)(TOPUP_CAP_ML - s->tu_ep_ml)) { ml = (int32_t)(TOPUP_CAP_ML - s->tu_ep_ml); }
  if (ml < (int32_t)MIN_SHOT_AB_ML) { return 0; }
  { uint32_t fit = (uint32_t)ml; if (cap_blocked(s, P_A, &fit)) { return 0; } ml = (int32_t)fit; }   /* held: the debt stays owed, the loop's feed-forward inherits it once the window frees */
  (void)shot_queue(s, P_A, (uint16_t)ml, "auto", why);
  (void)shot_queue(s, P_B, (uint16_t)ml, "auto", why);
  s->tu_owed  -= ml * (int32_t)s->L.gain[0];
  s->tu_ep_ml += (uint32_t)ml;
  s->wait_s = s->p.mix_s;       /* restart the settling clock: the loop must not double-dose on the unmixed dip */
  return ml;
}

static void topup_reset(ferti_sys_t *s)
{ s->tu_primed = 0U; s->tu_in = 0U; s->tu_owed = 0; s->tu_ep_dl = 0U; s->tu_carry = 0; s->tu_ec0 = (uint16_t)SENS_INVAL; s->tu_pred_dl = 0U; }

/* top-up feed-forward: every 5 s in auto mode (pumps may be running - the deficit keeps
 * accumulating while a compensation shot is still pumping and is served next). The typical fill
 * volume is LEARNED and the first drop queues a shot sized for TOPUP_PRED_PCT of it; the meter
 * keeps the books and anything owed beyond the pre-payment is fired as soon as it is a shot; the
 * remainder below one shot rides the loop's feed-forward; the whole episode is ONE learning sample. */
static void topup_poll(ferti_sys_t *s)
{
  ferti_lrn_t *L = &s->L;
  const ferti_param_t *p = &s->p;
  uint32_t dl = meter_read(s), d;
  if (!s->tu_primed) { s->tu_primed = 1U; s->tu_last_dl = dl; return; }
  d = dl - s->tu_last_dl;
  s->tu_last_dl = dl;
  if (d != 0U)
  {
    /* fresh water at ~0 uS: each 0.1 L lowers EC by target x d / tank - owe that back (fine units) */
    int32_t inc = (int32_t)((uint64_t)d * p->ec_target * 1000ULL / p->tank_dl);
    if (!s->tu_in)
    {
      s->tu_in = 1U; s->tu_ep_ml = 0U; s->tu_ep_dil = 0; s->tu_pred_dl = 0U;
      s->tu_ec0 = (s->ec_avg != (uint16_t)SENS_INVAL) ? s->ec_avg : s->ec;   /* baseline = EC before the first drop (60 s mean) */
      if (L->tu_typ_dl != 0U)                                                /* predicted opening shot */
      {
        uint32_t pdl  = (uint32_t)L->tu_typ_dl * TOPUP_PRED_PCT / 100U;
        int32_t  pml  = (int32_t)((uint64_t)pdl * p->ec_target * 1000ULL / p->tank_dl) / (int32_t)L->gain[0];
        char why[32];
        snprintf(why, sizeof(why), "top-up predict %lu.%luL", (unsigned long)(pdl / 10U), (unsigned long)(pdl % 10U));
        if (topup_shot(s, pml, why) > 0) { s->tu_pred_dl = (uint16_t)pdl; }   /* tu_owed now negative = pre-paid */
      }
    }
    s->tu_owed += inc; s->tu_ep_dil += inc;
    s->tu_ep_dl += d; s->tu_quiet_s = 0U;
  }
  else if (s->tu_in) { s->tu_quiet_s += 5U; }
  if (!s->tu_in) { return; }

  {
    /* metered settlement: whatever is owed beyond the pre-payment, once it is a shot */
    int32_t ml = s->tu_owed / (int32_t)L->gain[0];
    if (ml >= (int32_t)MIN_SHOT_AB_ML)
    {
      char why[24];
      snprintf(why, sizeof(why), "top-up %lu.%luL", (unsigned long)(s->tu_ep_dl / 10U), (unsigned long)(s->tu_ep_dl % 10U));
      if ((topup_shot(s, ml, why) == 0) && (s->tu_ep_ml >= TOPUP_CAP_ML)) { s->tu_owed = 0; }   /* episode cap: the rest is the loop's job */
    }
  }
  if (s->tu_quiet_s >= TOPUP_QUIET_S)          /* fill over */
  {
    uint16_t old_typ = L->tu_typ_dl;
    if (s->tu_owed > 0) { s->tu_carry += s->tu_owed; }   /* below one shot: rides the loop's feed-forward (over-payment: drift's job) */
    /* learn the typical fill volume (EMA; first fill seeds it) */
    if (s->tu_ep_dl <= 5000U)
    {
      if (L->tu_typ_dl == 0U) { L->tu_typ_dl = (uint16_t)s->tu_ep_dl; }
      else { L->tu_typ_dl = (uint16_t)((int32_t)L->tu_typ_dl + ((int32_t)s->tu_ep_dl - (int32_t)L->tu_typ_dl) * (int32_t)TOPUP_VOL_ALPHA_PCT / 100); }
      learn_save(s);
    }
    ev(s, "SYSTEM", "%s top-up %lu.%luL: %uml each (predicted %u.%uL, typical %u.%u->%u.%uL)", s->tag,
       (unsigned long)(s->tu_ep_dl / 10U), (unsigned long)(s->tu_ep_dl % 10U), (unsigned)s->tu_ep_ml,
       (unsigned)(s->tu_pred_dl / 10U), (unsigned)(s->tu_pred_dl % 10U),
       (unsigned)(old_typ / 10U), (unsigned)(old_typ % 10U), (unsigned)(L->tu_typ_dl / 10U), (unsigned)(L->tu_typ_dl % 10U));
    if ((s->tu_ep_ml >= MIN_SHOT_AB_ML) && (s->tu_ec0 != (uint16_t)SENS_INVAL) && (s->tu_ec0 >= EC_DRY_FLOOR))
    {
      /* hand the whole episode to the learner as one big A/B shot: the dilution is known from the
       * meter, so the next decision point can learn the gain from it */
      s->last.kind = K_EC; s->last.slot = 0U; s->last.ml = (uint16_t)s->tu_ep_ml;
      s->last.ec0 = s->tu_ec0; s->last.ph0 = (uint16_t)SENS_INVAL;   /* fresh water moves pH too: keep pH out of this sample */
      s->last.flow0 = dl; s->last.dil = s->tu_ep_dil;
    }
    s->tu_owed = 0; s->tu_in = 0U; s->tu_ep_dl = 0U; s->tu_quiet_s = 0U; s->tu_ep_dil = 0; s->tu_ec0 = (uint16_t)SENS_INVAL; s->tu_pred_dl = 0U;
  }
}

/* ---- chemistry tick: the app_dose.c task body, 500 ms ---- */
static void chem_tick(ferti_sys_t *s)
{
  if (s->run >= 0)                        /* pump timing, 500 ms granularity */
  {
    s->run_ms = (s->run_ms > TICK_MS) ? (s->run_ms - TICK_MS) : 0U;
    if (s->run_ms == 0U) { s->run = -1; }
  }
  if (s->run < 0) { shot_start_next(s); }

  if (++s->evl >= EVAL_TICKS)
  {
    s->evl = 0;
    s->hour_s += 5U;                      /* rolling 24 h caps: a new bucket every hour, the oldest drops out */
    if (s->hour_s >= 3600U) { s->hour_s = 0; s->hi = (uint8_t)((s->hi + 1U) % CAP_HOURS); memset(s->ml_h[s->hi], 0, sizeof(s->ml_h[s->hi])); }
    if (s->mode == M_AUTO) { flowlock(s); caps_release_poll(s); }
    flush_events(s);
    if ((s->mode == M_AUTO) && !s->flowhold && (s->run < 0) && (s->qn == 0U)) { chem_eval(s); }
    else if (s->mode != M_RELEASE)        /* keep status readings fresh even when not evaluating */
    {
      s->ec = ec_read(s);
      s->ph = ph_read(s);
      sens_track(s, s->ec, s->ph);
    }
    if ((s->mode == M_AUTO) && !s->flowhold) { topup_poll(s); }   /* after the readings above are fresh */
    else { topup_reset(s); }              /* off/hold: water that came in then is not ours to replace */
  }
  if (s->mode != M_RELEASE) { s->rel_done = 0U; pump_assert(s); }
  else if (!s->rel_done)                  /* entering release: drop our coils once (a shot may have been running), then hands off */
  { s->rel_done = 1U; pump_assert(s); }
}

/* ================= hydraulics (bindings with a fill pin) ================= */
static int chem_idle(const ferti_sys_t *s) { return (s->run < 0) && (s->qn == 0U) && !s->tu_in; }
static int chem_in_band(const ferti_sys_t *s)
{
  uint16_t ec = (s->ec_avg != (uint16_t)SENS_INVAL) ? s->ec_avg : s->ec;
  uint16_t ph = (s->ph_avg != (uint16_t)SENS_INVAL) ? s->ph_avg : s->ph;
  int32_t e;
  if (ec == (uint16_t)SENS_INVAL) { return 0; }
  e = (int32_t)ec - (int32_t)s->p.ec_target; if ((e < -(int32_t)s->p.ec_db) || (e > (int32_t)s->p.ec_db)) { return 0; }
  if (ph != (uint16_t)SENS_INVAL)
  { e = (int32_t)ph - (int32_t)s->p.ph_target; if ((e < -(int32_t)s->p.ph_db) || (e > (int32_t)s->p.ph_db)) { return 0; } }
  return 1;
}

static void tank_alarm(ferti_sys_t *s, uint8_t what, uint8_t active, const char *why)
{
  if (active && why) { snprintf(s->fault_why, sizeof s->fault_why, "%s", why); }
  s->tal_pend[what] = active ? 1U : 2U;
}

static void tank_goto(ferti_sys_t *s, uint8_t st)
{
  if (s->tstate != st) { ev(s, "SYSTEM", "%s tank %s -> %s", s->tag, s_tname[s->tstate], s_tname[st]); }
  s->tstate = st; s->t_ms = 0U;
}

static void tank_fault(ferti_sys_t *s, uint8_t what, const char *why)
{
  char r[40];
  s->irr_on = 0U; s->irr_ms_left = 0U;
  shots_abort(s);
  tank_goto(s, T_FAULT);
  tank_alarm(s, what, 1U, why);
  ev(s, "FAULT", "%s tank FAULT: %s -> fill/stir/irrigation off, dosing off (ack to clear)", s->tag, why);
  snprintf(r, sizeof r, "TANK %s", why);
  dose_off_fault(s, r);
}

static void tank_outputs(ferti_sys_t *s)
{
  const ferti_bind_t *b = s->b;
  uint8_t run = (s->mode == M_AUTO) ? 1U : 0U;
  coil_set(b->fill, (uint8_t)(run && (s->tstate == T_FILL)));
  if (pin_ok(b->fill)) { coil_set(b->stir, (uint8_t)(run && ((s->tstate == T_DOSE) || (s->tstate == T_STIR)))); }
  else
  {
    /* Float-valve reservoir: stir while any pump runs, then keep stirring for stir_s so the probe sees a
     * mixed tank at the next decision (2026-10-27: the shot read 2205 at rest, 2317 once the irrigation
     * cycle's stirring reached the probe; mix_s = stir_s = 180 s means the stirrer stops at the decision). */
    uint8_t active = (uint8_t)(s->irr_on || s->tu_in || (s->run >= 0));
    if (active) { s->stir_ms_left = (uint32_t)s->p.stir_s * 1000UL; }
    else if (s->stir_ms_left > 0U) { s->stir_ms_left = (s->stir_ms_left > TICK_MS) ? (s->stir_ms_left - TICK_MS) : 0U; }
    coil_set(b->stir, (uint8_t)(run && (active || (s->stir_ms_left > 0U))));
  }
  coil_set(b->irr,  (uint8_t)(s->irr_on && (s->mode != M_RELEASE)));
}

/* local seconds of day from the SNTP/RTC clock; -1 = clock not set */
static int32_t local_sod(const ferti_sys_t *s, uint16_t *day)
{
  uint32_t now = app_time_now();
  int64_t  loc;
  if (now == 0U) { return -1; }
  loc = (int64_t)now + (int64_t)s->p.tz_min * 60;
  if (day) { *day = (uint16_t)(loc / 86400); }
  return (int32_t)(loc % 86400);
}

static void irr_start(ferti_sys_t *s, uint32_t sec, const char *why)
{
  if (!pin_ok(s->b->irr) || s->irr_on || (sec == 0U)) { return; }
  s->irr_on = 1U; s->irr_ms_left = sec * 1000U;
  s->runoff_seen = 0U; s->runoff_sampled = 0U; s->irr_since_full = 1U;
  { uint32_t now = app_time_now();                 /* the irrigation record (ferti_lrn_t.irr_last) */
    if (now != 0U) { s->L.irr_last = now; learn_save(s); } }
  ev(s, "ACTION", "%s irrigate %lus %s (rate %u ml/s = %lu ml)", s->tag, (unsigned long)sec, why,
     (unsigned)s->L.irr_mls, (unsigned long)sec * s->L.irr_mls);
}

static void irr_stop(ferti_sys_t *s)
{
  if (!s->irr_on) { return; }
  s->irr_on = 0U; s->irr_ms_left = 0U; s->since_irr_ms = 0U; s->irr_n_batch++;
}

/* timed / moisture-triggered irrigation, only when the batch is READY */
static void irr_sched(ferti_sys_t *s)
{
  const ferti_param_t *p = &s->p;
  if (s->irr_on || (s->tstate != T_READY) || (s->mode != M_AUTO) || !pin_ok(s->b->irr)) { return; }
  if (p->irr_src == 1U)                   /* moisture closed loop: mean of the bound probes */
  {
    uint32_t sum = 0; uint8_t n = 0;
    for (uint8_t k = 0; k < 4U; k++) { uint16_t v = ai_read(&s->b->moisture[k]); if (v != (uint16_t)SENS_INVAL) { sum += v; n++; } }
    if (n == 0U) { return; }
    if (((sum / n) < p->irr_on_pct) && ((s->since_irr_ms == 0xFFFFFFFFUL) || (s->since_irr_ms >= (uint32_t)p->runoff_wait_s * 1000U)))
    { irr_start(s, p->irr_s, "moisture-low"); }
    return;
  }
  {                                       /* schedule: irr_per_day events spread over [start_h, end_h) */
    uint16_t day; int32_t sod = local_sod(s, &day);
    int32_t  w0 = (int32_t)p->irr_start_h * 3600, w1 = (int32_t)p->irr_end_h * 3600, iv;
    if ((sod < 0) || (p->irr_per_day == 0U) || (w1 <= w0)) { return; }
    iv = (w1 - w0) / (int32_t)p->irr_per_day;
    if (day != s->irr_day)
    {
      /* New local day, or first pass after a reset / learn reload. Start from the irrigation
       * record, not from zero: every slot whose start lies at or before the last recorded
       * irrigation of THIS day counts as done. A reboot inside a slot that already ran must not
       * run it again (2026-11-09: a bank swap at 12:39 re-fired slot 2 at boot); slots that were
       * missed while the board was down are not caught up either — the record decides. */
      s->irr_day = day; s->irr_slot_done = 0U;
      if (s->L.irr_last != 0U)
      {
        int64_t ll = (int64_t)s->L.irr_last + (int64_t)p->tz_min * 60;
        if ((uint16_t)(ll / 86400) == day)
        {
          int32_t lsod = (int32_t)(ll % 86400);
          for (uint8_t k = 0; k < p->irr_per_day; k++)
          { if (w0 + iv * (int32_t)k <= lsod) { s->irr_slot_done |= (1UL << k); } }
        }
      }
    }
    for (uint8_t k = 0; k < p->irr_per_day; k++)
    {
      int32_t t = w0 + iv * (int32_t)k;
      if ((sod >= t) && (sod < t + iv) && !(s->irr_slot_done & (1UL << k)))
      {
        char why[24]; snprintf(why, sizeof why, "slot %u/%u", (unsigned)(k + 1U), (unsigned)p->irr_per_day);
        s->irr_slot_done |= (1UL << k);
        irr_start(s, p->irr_s, why);
        return;
      }
    }
  }
}

static void tank_tick(ferti_sys_t *s)         /* 500 ms */
{
  const ferti_bind_t *b = s->b;
  const ferti_param_t *p = &s->p;
  uint8_t low  = di_asserted(s, b->di_low, FERTI_DI_LOW);
  uint8_t full = di_asserted(s, b->di_full, FERTI_DI_FULL);
  uint8_t ovf  = di_asserted(s, b->di_overflow, FERTI_DI_OVERFLOW);
  uint8_t frsh = di_asserted(s, b->di_fresh_low, FERTI_DI_FRESH_LOW);
  uint8_t rof  = di_asserted(s, b->di_runoff, FERTI_DI_RUNOFF);
  uint8_t prev = s->di_prev;
  s->di_prev = (uint8_t)((low ? FERTI_DI_LOW : 0U) | (full ? FERTI_DI_FULL : 0U) | (ovf ? FERTI_DI_OVERFLOW : 0U) |
                         (frsh ? FERTI_DI_FRESH_LOW : 0U) | (rof ? FERTI_DI_RUNOFF : 0U));
  s->t_ms += TICK_MS;
  if (s->since_irr_ms != 0xFFFFFFFFUL) { s->since_irr_ms += TICK_MS; }

  /* irrigation pump timing (manual `irr now` works in OFF too) */
  if (s->irr_on)
  {
    if (rof) { s->runoff_seen = 1U; }
    s->irr_ms_batch += TICK_MS;
    if (s->mode == M_RELEASE) { irr_stop(s); }
    else { s->irr_ms_left = (s->irr_ms_left > TICK_MS) ? (s->irr_ms_left - TICK_MS) : 0U; if (s->irr_ms_left == 0U) { irr_stop(s); } }
  }

  /* overflow: the normally-closed float opens = reservoir over the brim (or the wire fell off) */
  if (ovf && (s->tstate != T_FAULT) && pin_ok(b->di_overflow)) { tank_fault(s, TAL_OVERFLOW, "overflow"); }

  /* inlet meter moving while the valve is commanded off = valve not closing or a leak downstream */
  if ((b->meter.pt == FERTI_PT_HSDI) && !pin_ok(b->fill))
  {
    uint32_t now_dl = meter_read(s);
    if (s->irr_on || (s->since_irr_ms < 120000UL)) { s->m_leak_dl = 0U; s->m_leak_ms = 0U; }   /* a top-up right after irrigation is normal */
    else
    {
      s->m_leak_dl += now_dl - s->m_last_dl;
      s->m_leak_ms += TICK_MS;
      if ((s->m_leak_dl >= 4U * LEAK_DL) && !s->leak_alarmed)
      { s->leak_alarmed = 1U; tank_alarm(s, TAL_LEAK, 1U, "inlet flow, no irrigation"); ev(s, "FAULT", "%s inlet meter moved %lu.%luL with no irrigation: float valve stuck open or leak", s->tag, (unsigned long)(s->m_leak_dl / 10U), (unsigned long)(s->m_leak_dl % 10U)); }
      if (s->m_leak_ms >= 600000UL) { s->m_leak_dl = 0U; s->m_leak_ms = 0U; }
    }
    s->m_last_dl = now_dl;
  }
  if ((b->meter.pt == FERTI_PT_HSDI) && pin_ok(b->fill))
  {
    uint32_t now_dl = meter_read(s);
    if (s->tstate == T_FILL) { s->m_leak_dl = 0U; s->m_leak_ms = 0U; }
    else
    {
      s->m_leak_dl += now_dl - s->m_last_dl;
      s->m_leak_ms += TICK_MS;
      if ((s->m_leak_dl >= LEAK_DL) && !s->leak_alarmed)
      { s->leak_alarmed = 1U; tank_alarm(s, TAL_LEAK, 1U, "inlet flow, valve off"); ev(s, "FAULT", "%s inlet meter moved %lu.%luL with the valve off: valve stuck open or leak", s->tag, (unsigned long)(s->m_leak_dl / 10U), (unsigned long)(s->m_leak_dl % 10U)); }
      if (s->m_leak_ms >= LEAK_WIN_MS) { s->m_leak_dl = 0U; s->m_leak_ms = 0U; }
    }
    s->m_last_dl = now_dl;
  }

  /* fresh-water tank empty: one alarm, cleared when it refills */
  if (pin_ok(b->di_fresh_low))
  {
    if (frsh && !s->fresh_alarmed) { s->fresh_alarmed = 1U; tank_alarm(s, TAL_FRESH, 1U, "fresh water low"); ev(s, "FAULT", "%s fresh water low: no refill until it is back", s->tag); }
    else if (!frsh && s->fresh_alarmed) { s->fresh_alarmed = 0U; tank_alarm(s, TAL_FRESH, 0U, NULL); ev(s, "SYSTEM", "%s fresh water back", s->tag); }
  }

  if (s->mode != M_AUTO)
  {
    if ((s->tstate != T_FAULT) && (s->tstate != T_IDLE)) { s->hold = 0U; tank_goto(s, T_IDLE); }
    tank_outputs(s);
    return;
  }

  switch (s->tstate)
  {
    case T_IDLE:
    case T_READY:
      if (!pin_ok(b->fill))
      {
        /* float-valve refill (no actuator): the inlet meter feeds the chemistry loop directly;
         * stay READY and irrigate on schedule. A low float here means the valve failed or the
         * supply is off: no irrigation (pump would run dry), alarm until the level is back. */
        if (s->tstate != T_READY) { tank_goto(s, T_READY); }
        if (low && pin_ok(b->di_low)) { if (s->low_ms < LOW_DWELL_MS) { s->low_ms += TICK_MS; } }
        else { s->low_ms = 0U; }
        if (s->low_ms >= LOW_DWELL_MS)              /* sustained low: not a splash */
        {
          irr_stop(s);
          if (!s->low_alarmed) { s->low_alarmed = 1U; tank_alarm(s, TAL_LOW, 1U, "level low, no inflow"); ev(s, "FAULT", "%s reservoir low: float valve failed or supply off -> irrigation held", s->tag); }
        }
        else
        {
          if (s->low_alarmed) { s->low_alarmed = 0U; tank_alarm(s, TAL_LOW, 0U, NULL); ev(s, "SYSTEM", "%s reservoir level back -> irrigation released", s->tag); }
          irr_sched(s);
        }
      }
      else if (low && !full)
      {
        if (frsh && pin_ok(b->di_fresh_low)) { /* wait for fresh water (alarmed above) */ }
        else
        {
          irr_stop(s);
          /* close the books on the last batch: irrigation rate from the volume that went out */
          if ((s->tstate == T_READY) && (s->irr_ms_batch >= 10000U))
          {
            uint16_t old = s->L.irr_mls;
            uint16_t now = (uint16_t)((uint64_t)p->batch_dl * 100ULL * 1000ULL / s->irr_ms_batch);
            s->L.irr_mls = now; learn_save(s);
            ev(s, "SYSTEM", "%s batch out: %u irrigations %lus -> %u ml/s (was %u)", s->tag, (unsigned)s->irr_n_batch,
               (unsigned long)(s->irr_ms_batch / 1000U), (unsigned)now, (unsigned)old);
            if ((old != 0U) && (now < (uint16_t)((uint32_t)old * (100U - CLOG_DROP_PCT) / 100U)) && !s->clog_alarmed)
            { s->clog_alarmed = 1U; tank_alarm(s, TAL_CLOG, 1U, "irrigation rate down"); ev(s, "FAULT", "%s irrigation rate %u->%u ml/s: drip lines clogging?", s->tag, (unsigned)old, (unsigned)now); }
            else if ((s->clog_alarmed) && (now >= old)) { s->clog_alarmed = 0U; tank_alarm(s, TAL_CLOG, 0U, NULL); }
          }
          s->irr_ms_batch = 0U; s->irr_n_batch = 0U; s->full_stuck_n = 0U; s->irr_since_full = 0U;
          if (s->leak_alarmed) { s->leak_alarmed = 0U; tank_alarm(s, TAL_LEAK, 0U, NULL); }
          s->hold = 1U; snprintf(s->hold_why, sizeof s->hold_why, "fill");
          s->fill_dl0 = meter_read(s);
          tank_goto(s, T_FILL);
          ev(s, "ACTION", "%s refill start (batch %u.%uL, limit %us)", s->tag, (unsigned)(p->batch_dl / 10U), (unsigned)(p->batch_dl % 10U), (unsigned)p->fill_max_s);
        }
      }
      else if (s->tstate == T_READY)
      {
        /* protections while irrigating from a full reservoir */
        if ((prev & FERTI_DI_FULL) && !full && !s->irr_since_full && !s->leak_alarmed && pin_ok(b->di_full) && (s->t_ms > 30000U))
        { s->leak_alarmed = 1U; tank_alarm(s, TAL_LEAK, 1U, "level fell, no irrigation"); ev(s, "FAULT", "%s full float dropped with no irrigation since the refill: siphon or leak?", s->tag); }
        irr_sched(s);
      }
      break;

    case T_FILL:
    {
      int      metered = (b->meter.pt == FERTI_PT_HSDI);                 /* inlet pulse meter bound: measured batch */
      uint32_t in_dl   = metered ? (meter_read(s) - s->fill_dl0) : 0U;
      if (frsh && pin_ok(b->di_fresh_low)) { tank_fault(s, TAL_FRESH, "fresh water ran out"); break; }
      if (metered && (in_dl >= (uint32_t)p->batch_dl * FILL_OVERRUN_PCT / 100U))
      { tank_fault(s, TAL_FILL, "fill overrun: float stuck?"); break; }   /* water keeps coming, no full float */
      if (full)
      {
        uint32_t ms = s->t_ms;
        uint32_t dl = (metered && (in_dl >= 10U)) ? in_dl : p->batch_dl;   /* measured litres when the meter saw them, else the design batch */
        s->fill_ms_batch = ms;
        if (ms >= 2000U)
        {
          s->L.fill_mls = (uint16_t)((uint64_t)dl * 100ULL * 1000ULL / ms);
          s->p.fill_max_s = (uint16_t)((ms / 1000U) * 3U / 2U + 30U);   /* next limit = 1.5 x measured + 30 s */
          learn_save(s); cfg_save(s);
        }
        if (!metered) { s->vmeter_dl += p->batch_dl; }   /* no meter: report the design batch to the chemistry loop (feed-forward) */
        s->hold = 0U;
        tank_goto(s, T_DOSE);
        ev(s, "SYSTEM", "%s refill done in %lus: %lu.%luL%s (%u ml/s) -> dosing feed-forward, stir", s->tag,
           (unsigned long)(ms / 1000U), (unsigned long)(dl / 10U), (unsigned long)(dl % 10U), metered ? " metered" : "", (unsigned)s->L.fill_mls);
      }
      else if ((p->fill_max_s != 0U) && (s->t_ms >= (uint32_t)p->fill_max_s * 1000U))
      { tank_fault(s, TAL_FILL, "fill timeout"); }
      break;
    }

    case T_DOSE:
      if (chem_idle(s) && (s->t_ms >= 70000U))   /* the meter episode closes 60 s after the volume event */
      { s->hold = 1U; snprintf(s->hold_why, sizeof s->hold_why, "stir"); tank_goto(s, T_STIR); }
      break;

    case T_STIR:
      if (s->t_ms >= (uint32_t)p->stir_s * 1000U)
      { s->hold = 0U; s->verify_n = 0U; s->verify_alarmed = 0U; s->wait_s = 0U; tank_goto(s, T_VERIFY); }
      break;

    case T_VERIFY:
      if (chem_in_band(s))
      {
        if (s->verify_alarmed) { s->verify_alarmed = 0U; tank_alarm(s, TAL_VERIFY, 0U, NULL); }
        ev(s, "SYSTEM", "%s batch verified ec=%u ph=%u -> ready", s->tag, (unsigned)s->ec_avg, (unsigned)s->ph_avg);
        tank_goto(s, T_READY);
      }
      else if (s->t_ms >= p->mix_s * 1000U)
      {
        s->t_ms = 0U;
        if ((++s->verify_n >= p->verify_rounds) && !s->verify_alarmed)
        {
          s->verify_alarmed = 1U; tank_alarm(s, TAL_VERIFY, 1U, "not in band");
          ev(s, "FAULT", "%s batch not in band after %u cycles (ec=%u/%u ph=%u/%u): irrigation held", s->tag, (unsigned)s->verify_n,
             (unsigned)s->ec_avg, (unsigned)p->ec_target, (unsigned)s->ph_avg, (unsigned)p->ph_target);
        }
      }
      break;

    case T_FAULT:
    default:
      break;
  }

  /* after an irrigation: runoff detection / sample, full-float-stuck check */
  if (!s->irr_on && (s->since_irr_ms != 0xFFFFFFFFUL) && pin_ok(b->irr))
  {
    if (rof) { s->runoff_seen = 1U; }
    if ((s->since_irr_ms >= (uint32_t)p->runoff_wait_s * 1000U) && !s->runoff_sampled)
    {
      s->runoff_sampled = 1U;
      s->ec_runoff = ai_read(&b->ec_runoff); s->ph_runoff = ai_read(&b->ph_runoff); s->runoff_age_s = 0U;
      if (pin_ok(b->di_runoff) && !s->runoff_seen)
      { tank_alarm(s, TAL_RUNOFF, 1U, "no runoff"); ev(s, "FAULT", "%s no runoff %us after irrigation: blocked line / empty tank / dead pump?", s->tag, (unsigned)p->runoff_wait_s); }
      if (pin_ok(b->ec_runoff.pin))
      { ev(s, "SYSTEM", "%s runoff sample ec=%u ph=%u%s", s->tag, (unsigned)s->ec_runoff, (unsigned)s->ph_runoff, s->runoff_seen ? "" : " (no flow seen)"); }
      if (full && pin_ok(b->di_full))
      {
        if ((++s->full_stuck_n >= FULL_STUCK_N) && (s->tal_pend[TAL_CLOG] == 0U) && !s->clog_alarmed)
        { s->clog_alarmed = 1U; tank_alarm(s, TAL_CLOG, 1U, "level not falling"); ev(s, "FAULT", "%s full float still on after %u irrigations: pump dry-running or line blocked", s->tag, (unsigned)s->full_stuck_n); }
      }
      else { s->full_stuck_n = 0U; }
    }
  }
  tank_outputs(s);
}

/* ---- the service task: one task, every instance, 500 ms ---- */
static void ferti_task(void *arg)
{
  (void)arg;
  vTaskDelay(pdMS_TO_TICKS(15000));       /* let the scanner produce first real values */
  for (;;)
  {
    for (uint8_t k = 0; k < s_n; k++)
    {
      ferti_sys_t *s = s_sys[k];
      if (!s->ploaded) { cfg_load(s); learn_load(s); }   /* covers a late fs mount */
      if (pin_ok(s->b->fill) || pin_ok(s->b->irr)) { tank_tick(s); }
      chem_tick(s);
      if (s->evl == 0U) { if (s->runoff_age_s != 0xFFFFFFFFUL) { s->runoff_age_s += 5U; } }
    }
    vTaskDelay(pdMS_TO_TICKS(TICK_MS));
  }
}

/* ---- heartbeat ---- */
const char *ferti_hb(ferti_sys_t *s)
{
  char *b = s->hb; size_t cap = sizeof s->hb; int n;
  char ph[24];
  /* legacy tail "ac<n>/bs<n>" (two pH pumps) or "ph<n>" (one pH pump) */
  if (s->ph_single) { snprintf(ph, sizeof ph, "ph%lu", (unsigned long)(ml_24h(s, P_ACID) + ml_24h(s, P_BASE))); }
  else { snprintf(ph, sizeof ph, "ac%lu/bs%lu", (unsigned long)ml_24h(s, P_ACID), (unsigned long)ml_24h(s, P_BASE)); }
  if (s->run >= 0)
  { n = snprintf(b, cap, "%s run:%s %lus A%lu/B%lu/%s", mode_name(s->mode),
                 pump_name(s, (uint8_t)s->run), (unsigned long)((s->run_ms + 999U) / 1000U),
                 (unsigned long)ml_24h(s, P_A), (unsigned long)ml_24h(s, P_B), ph); }
  else
  { n = snprintf(b, cap, "%s%s%s A%lu/B%lu/%s", mode_name(s->mode), s->flowhold ? "!flow" : "", s->caphold ? "!cap" : "",
                 (unsigned long)ml_24h(s, P_A), (unsigned long)ml_24h(s, P_B), ph); }
  if ((s->tstate != T_NONE) && (n > 0) && ((size_t)n < cap))
  {
    n += snprintf(b + n, cap - (size_t)n, " T:%s%s", s_tname[s->tstate], s->irr_on ? "+irr" : "");
    if (s->L.irr_last != 0U)                     /* last irrigation start, local HH:MM (the record) */
    {
      int64_t ll = (int64_t)s->L.irr_last + (int64_t)s->p.tz_min * 60; int32_t sod = (int32_t)(ll % 86400);
      n += snprintf(b + n, cap - (size_t)n, " li:%02ld:%02ld", (long)(sod / 3600), (long)((sod % 3600) / 60));
    }
    if ((s->ec_runoff != (uint16_t)SENS_INVAL) && ((size_t)n < cap))
    { snprintf(b + n, cap - (size_t)n, " ro:%u/%u@%lus", (unsigned)s->ec_runoff, (unsigned)s->ph_runoff, (unsigned long)s->runoff_age_s); }
  }
  return b;
}

const char *ferti_hb_more(void)
{
  static char b[200];
  int n = 0;
  b[0] = 0;
  for (uint8_t k = 1; k < s_n; k++)
  {
    int m = snprintf(b + n, sizeof(b) - (size_t)n, ",\"dose_%s\":\"%s\"", s_sys[k]->b->name, ferti_hb(s_sys[k]));
    if ((m < 0) || ((size_t)(n + m) >= sizeof b)) { break; }
    n += m;
  }
  return b;
}

const char *ferti_desc_more(void)
{
  static char b[200];
  int n = 0;
  b[0] = 0;
  for (uint8_t k = 1; k < s_n; k++)
  {
    int m = snprintf(b + n, sizeof(b) - (size_t)n,
                     "{\"id\":\"dose.%s.state\",\"name\":\"Dosing %s\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"io\",\"f\":\"dose_%s\"},",
                     s_sys[k]->b->name, s_sys[k]->b->name, s_sys[k]->b->name);
    if ((m < 0) || ((size_t)(n + m) >= sizeof b)) { break; }
    n += m;
  }
  return b;
}

/* ---- commands ---- */
static int pump_by_name(const ferti_sys_t *s, const char *t, int len)
{
  if (s->ph_single && (len == 2) && (strncmp(t, "ph", 2) == 0)) { return (s->p.ph_dir == 1U) ? (int)P_BASE : (int)P_ACID; }
  for (int k = 0; k < (int)PUMPS; k++)
  { if ((strncmp(t, s_pump_name4[k], (size_t)len) == 0) && ((int)strlen(s_pump_name4[k]) == len)) { return k; } }
  return -1;
}

static const char *dir_name(uint8_t d) { return (d == 0U) ? "both" : ((d == 1U) ? "base" : "acid"); }

int ferti_cmd(ferti_sys_t *s, const char *a, const char *src, char *out, uint16_t cap)
{
  ferti_param_t *p = &s->p;
  const char *tag = s->tag;
  while (*a == ' ') { a++; }

  char t1[8] = "", t2[8] = "", t3[8] = "";
  unsigned long v = 0, v2 = 0;
  int nt = 0;
  {
    const char *q = a; int k = 0;
    while (*q && (*q != ' ') && (k < 7)) { t1[k++] = *q++; } t1[k] = 0; if (k) { nt = 1; }
    while (*q == ' ') { q++; }
    k = 0;
    while (*q && (*q != ' ') && (k < 7)) { t2[k++] = *q++; } t2[k] = 0; if (k) { nt = 2; }
    while (*q == ' ') { q++; }
    if (*q)
    {
      v = strtoul(q, 0, 10); nt = 3;
      k = 0; while (*q && (*q != ' ') && (k < 7)) { t3[k++] = *q++; } t3[k] = 0;
      while (*q == ' ') { q++; }
      if (*q) { v2 = strtoul(q, 0, 10); nt = 4; }
    }
    else if (nt == 2 && (t2[0] >= '0') && (t2[0] <= '9')) { v = strtoul(t2, 0, 10); }
  }

  if ((nt == 0) || (strcmp(t1, "stat") == 0)) { /* status below */ }
  else if (strcmp(t1, "auto") == 0)
  {
    caps_clear(s);                           /* operator re-arm = fresh 24 h window (any hold lifts on the next tick) */
    if (s->off_al) { s->off_al = 0U; dose_alarm(s, AL_OFF, 0U); }
    s->wait_s = 0; s->ff_ph = 0; s->ff_ec = 0; s->last.kind = K_IDLE; s->confound = 1U;
    s->acc_dph = 0; s->acc_dec = 0; s->acc_dose_ph = 0; s->acc_dose_ec = 0; s->acc_t = 0;
    memset(s->strikes, 0, sizeof(s->strikes));
    s->mode = M_AUTO;
    cfg_save(s);                             /* boot mode rides the cfg since 2026-09-02 */
    evs("CONFIG", src, "%s auto (ec %u±%u ph %u±%u shots %u/%u/%uml mix %lus)", tag,
        (unsigned)p->ec_target, (unsigned)p->ec_db, (unsigned)p->ph_target,
        (unsigned)p->ph_db, (unsigned)p->ec_ml, (unsigned)p->acid_ml, (unsigned)p->base_ml,
        (unsigned long)p->mix_s);
  }
  else if (strcmp(t1, "off") == 0)
  { shots_abort(s); s->mode = M_OFF; cfg_save(s); evs("CONFIG", src, "%s off", tag); }
  else if (strcmp(t1, "stop") == 0)
  { shots_abort(s); s->irr_on = 0U; s->mode = M_OFF; cfg_save(s); evs("ACTION", src, "%s STOP (pumps off)", tag); }
  else if (strcmp(t1, "release") == 0)
  { shots_abort(s); s->mode = M_RELEASE; cfg_save(s); evs("CONFIG", src, "%s released (coils to dashboard)", tag); }
  else if (strcmp(t1, "cal") == 0)
  {
    int k = pump_by_name(s, t2, (int)strlen(t2));
    if ((k < 0) || (v < 5UL) || (v > 600UL)) { snprintf(out, cap, "dose cal a|b|%s <5..600 ml/min>", s->ph_single ? "ph" : "acid|base"); return 1; }
    p->mlmin[k] = (uint16_t)v;
    if (s->ph_single && (k >= (int)P_ACID)) { p->mlmin[P_ACID] = (uint16_t)v; p->mlmin[P_BASE] = (uint16_t)v; }
    cfg_save(s);
    evs("CONFIG", src, "%s cal %s %luml/min", tag, pump_name(s, (uint8_t)k), v);
  }
  else if (strcmp(t1, "ec") == 0)
  {
    unsigned long tgt = (nt >= 3) ? strtoul(t2, 0, 10) : v;
    if ((tgt < 300UL) || (tgt > 3000UL)) { snprintf(out, cap, "dose ec <300..3000 uS> [db 5..500]"); return 1; }
    if (nt >= 3)
    {
      if ((v < 5UL) || (v > 500UL)) { snprintf(out, cap, "dose ec <300..3000 uS> [db 5..500]"); return 1; }
      p->ec_db = (uint16_t)v;
    }
    p->ec_target = (uint16_t)tgt;
    cfg_save(s);
    evs("CONFIG", src, "%s ec target %lu±%uuS", tag, tgt, (unsigned)p->ec_db);
  }
  else if (strcmp(t1, "ph") == 0)
  {
    unsigned long tgt = (nt >= 3) ? strtoul(t2, 0, 10) : v;   /* same optional-deadband form */
    if ((tgt < 400UL) || (tgt > 700UL)) { snprintf(out, cap, "dose ph <400..700 x100> [db 5..100]"); return 1; }
    if (nt >= 3)
    {
      if ((v < 5UL) || (v > 100UL)) { snprintf(out, cap, "dose ph <400..700 x100> [db 5..100]"); return 1; }
      p->ph_db = (uint16_t)v;
    }
    p->ph_target = (uint16_t)tgt;
    cfg_save(s);
    evs("CONFIG", src, "%s ph target %lu±%u", tag, tgt, (unsigned)p->ph_db);
  }
  else if (strcmp(t1, "phsrc") == 0)
  {
    uint8_t v8;
    if (strcmp(t2, "avg") == 0)      { v8 = 0U; }
    else if (strcmp(t2, "1") == 0)   { v8 = 1U; }
    else if (strcmp(t2, "2") == 0)   { v8 = 2U; }
    else { snprintf(out, cap, "dose phsrc avg|1|2"); return 1; }
    p->ph_src = v8; s->ph_src_bad = 0U;
    cfg_save(s);
    evs("CONFIG", src, "%s phsrc %s", tag, (v8 == 0U) ? "avg" : ((v8 == 1U) ? "1" : "2"));
  }
  else if (strcmp(t1, "phdiv") == 0)
  {
    if (v > 100UL) { snprintf(out, cap, "dose phdiv <0..100 x100, 0=off>"); return 1; }
    p->ph_div = (uint16_t)v; s->ph_src_bad = 0U;
    cfg_save(s);
    evs("CONFIG", src, "%s phdiv 0.%02lu%s", tag, v, v ? "" : " (off)");
  }
  else if (strcmp(t1, "ecsrc") == 0)
  {
    uint8_t v8;
    if (strcmp(t2, "avg") == 0)      { v8 = 0U; }
    else if (strcmp(t2, "1") == 0)   { v8 = 1U; }
    else if (strcmp(t2, "2") == 0)   { v8 = 2U; }
    else { snprintf(out, cap, "dose ecsrc avg|1|2"); return 1; }
    p->ec_src = v8; s->ec_src_bad = 0U;
    cfg_save(s);
    evs("CONFIG", src, "%s ecsrc %s", tag, (v8 == 0U) ? "avg" : ((v8 == 1U) ? "1" : "2"));
  }
  else if (strcmp(t1, "ecdiv") == 0)
  {
    if (v > 1000UL) { snprintf(out, cap, "dose ecdiv <0..1000 uS, 0=off>"); return 1; }
    p->ec_div = (uint16_t)v; s->ec_src_bad = 0U;
    cfg_save(s);
    evs("CONFIG", src, "%s ecdiv %luuS%s", tag, v, v ? "" : " (off)");
  }
  else if (strcmp(t1, "mix") == 0)
  {
    if ((v < 3UL) || (v > 120UL)) { snprintf(out, cap, "dose mix <3..120 min> (decision cycle = settling time)"); return 1; }
    p->mix_s = (uint32_t)v * 60U;
    cfg_save(s);
    evs("CONFIG", src, "%s mix %lumin", tag, v);
  }
  else if (strcmp(t1, "tank") == 0)          /* reservoir volume for the top-up feed-forward */
  {
    if ((v < 20UL) || (v > 5000UL)) { snprintf(out, cap, "dose tank <20..5000 L> (reservoir volume, top-up feed-forward)"); return 1; }
    p->tank_dl = (uint16_t)(v * 10UL);
    cfg_save(s);
    evs("CONFIG", src, "%s tank %luL", tag, v);
  }
  else if (strcmp(t1, "phdir") == 0)
  {
    uint8_t v8;
    if (strcmp(t2, "both") == 0)      { v8 = 0U; }
    else if (strcmp(t2, "base") == 0) { v8 = 1U; }
    else if (strcmp(t2, "acid") == 0) { v8 = 2U; }
    else { snprintf(out, cap, "dose phdir both|base|acid (pumps the auto loop may use)"); return 1; }
    if (s->ph_single && (v8 == 0U)) { snprintf(out, cap, "dose phdir base|acid (one pH pump: what is in the bottle)"); return 1; }
    p->ph_dir = v8;
    cfg_save(s);
    evs("CONFIG", src, "%s phdir %s", tag, dir_name(v8));
  }
  else if (strcmp(t1, "learn") == 0)
  {
    if (strcmp(t2, "reset") == 0)
    {
      learn_reset(s);
      evs("CONFIG", src, "%s learn reset to seeds", tag);
    }
    else if (nt >= 3)                      /* `learn ab|acid|base <x1000/ml>`: operator override, re-seeds that slot's sums */
    {
      uint8_t  slot = (strcmp(t2, "ab") == 0) ? 0U : (strcmp(t2, "acid") == 0) ? 1U : (strcmp(t2, "base") == 0) ? 2U : 3U;
      uint32_t seed = (slot <= 2U) ? s->seed_gain[slot] : 1U;
      if ((slot > 2U) || (v < seed / 5UL) || (v > seed * 5UL))
      { snprintf(out, cap, "dose learn ab|acid|base <x1000/ml> (within 1/5..5x of seed) | reset"); return 1; }
      s->L.gain[slot] = (uint32_t)v; learn_seed_sums(s, slot); s->strikes[slot] = 0; learn_save(s);
      evs("CONFIG", src, "%s learn %s gain set %lu", tag, t2, (unsigned long)v);
    }
    snprintf(out, cap, "%s learn: gain(x1000/ml) ab=%lu acid=%lu base=%lu (over %ld/%ld/%ldml) drift(x1000/h) ph=%ld ec=%ld ff ph=%ld ec=%ld strikes=%u/%u/%u unlearned=%u/%u last=%u wait=%lus win=%lumin ph%+ld ec%+ld topup=%s owed=%lduS fill=%u.%uL tank=%uL avg ec=%u ph=%u",
             tag, (unsigned long)s->L.gain[0], (unsigned long)s->L.gain[1], (unsigned long)s->L.gain[2],
             (long)(s->L.lsum_ml[0] / 100), (long)(s->L.lsum_ml[1] / 100), (long)(s->L.lsum_ml[2] / 100),
             (long)s->L.drift_ph, (long)s->L.drift_ec, (long)(s->ff_ph / 1000L), (long)(s->ff_ec / 1000L),
             (unsigned)s->strikes[0], (unsigned)s->strikes[1], (unsigned)s->strikes[2],
             (unsigned)s->unlearned[1], (unsigned)s->unlearned[2], (unsigned)s->last.kind,
             (unsigned long)s->wait_s, (unsigned long)(s->acc_t / 60U), (long)(s->acc_dph / 1000L), (long)(s->acc_dec / 1000L),
             s->tu_in ? "filling" : "idle", (long)(s->tu_owed / 1000L),
             (unsigned)(s->L.tu_typ_dl / 10U), (unsigned)(s->L.tu_typ_dl % 10U), (unsigned)(p->tank_dl / 10U),
             (unsigned)s->ec_avg, (unsigned)s->ph_avg);
    return 1;
  }
  else if (strcmp(t1, "cap") == 0)           /* 24 h rolling caps per pump */
  {
    if ((strcmp(t2, "ab") == 0) && (v >= 50UL) && (v <= CAP_MAX_AB_ML))         { p->cap_ml[P_A] = (uint16_t)v; p->cap_ml[P_B] = (uint16_t)v; }
    else if ((strcmp(t2, "acid") == 0) && (v >= 5UL) && (v <= CAP_MAX_PH_ML))   { p->cap_ml[P_ACID] = (uint16_t)v; }
    else if ((strcmp(t2, "base") == 0) && (v >= 5UL) && (v <= CAP_MAX_PH_ML))   { p->cap_ml[P_BASE] = (uint16_t)v; }
    else if (s->ph_single && (strcmp(t2, "ph") == 0) && (v >= 5UL) && (v <= CAP_MAX_PH_ML)) { p->cap_ml[P_ACID] = (uint16_t)v; p->cap_ml[P_BASE] = (uint16_t)v; }
    else { snprintf(out, cap, "dose cap ab <50..%u> | acid|base%s <5..%u> (ml per rolling 24h)", CAP_MAX_AB_ML, s->ph_single ? "|ph" : "", CAP_MAX_PH_ML); return 1; }
    cfg_save(s);
    evs("CONFIG", src, "%s cap %s %luml/24h", tag, t2, v);
  }
  else if (strcmp(t1, "shot") == 0)          /* per-cycle auto shot caps: `shot acid|base <ml>` */
  {
    int k = pump_by_name(s, t2, (int)strlen(t2));
    if ((k == (int)P_ACID) && (v >= 1UL) && (v <= MAN_MAX_ACID_ML)) { p->acid_ml = (uint16_t)v; if (s->ph_single) { p->base_ml = (uint16_t)v; } }
    else if ((k == (int)P_BASE) && (v >= 1UL) && (v <= MAN_MAX_BASE_ML)) { p->base_ml = (uint16_t)v; if (s->ph_single) { p->acid_ml = (uint16_t)v; } }
    else { snprintf(out, cap, "dose shot acid <1..%u> | base <1..%u ml>", MAN_MAX_ACID_ML, MAN_MAX_BASE_ML); return 1; }
    cfg_save(s);
    evs("CONFIG", src, "%s shot %s %luml", tag, pump_name(s, (uint8_t)k), v);
  }
  else if (strcmp(t1, "ab") == 0)
  {
    if (s->mode == M_RELEASE) { snprintf(out, cap, "dose: released — `dose off` first"); return 1; }
    if ((v < 1UL) || (v > MAN_MAX_ML)) { snprintf(out, cap, "dose ab <1..%u ml>", MAN_MAX_ML); return 1; }
    if (s->qn > (uint8_t)(QLEN - 2U)) { snprintf(out, cap, "dose: queue full"); return 1; }
    (void)shot_queue(s, P_A, (uint16_t)v, src, "manual");
    (void)shot_queue(s, P_B, (uint16_t)v, src, "manual");
  }
  /* ---- hydraulics verbs (any binding may set them; they act only with the matching pins) ---- */
  else if (strcmp(t1, "topup") == 0)         /* manual volume event: `topup <L>` -> feed-forward as if the meter had seen it */
  {
    if ((v < 1UL) || (v > 500UL)) { snprintf(out, cap, "dose topup <1..500 L> (water added by hand: dose for it)"); return 1; }
    s->vmeter_dl += (uint32_t)(v * 10UL);
    evs("ACTION", src, "%s topup %luL reported", tag, v);
  }
  else if (strcmp(t1, "batch") == 0)
  {
    if ((v < 1UL) || (v > 1000UL)) { snprintf(out, cap, "dose batch <1..1000 L> (refill volume between the low and full floats)"); return 1; }
    p->batch_dl = (uint16_t)(v * 10UL); cfg_save(s);
    evs("CONFIG", src, "%s batch %luL", tag, v);
  }
  else if (strcmp(t1, "fill") == 0)          /* `fill <max s>` */
  {
    if ((v < 10UL) || (v > 7200UL)) { snprintf(out, cap, "dose fill <10..7200 s> (refill time limit)"); return 1; }
    p->fill_max_s = (uint16_t)v; cfg_save(s);
    evs("CONFIG", src, "%s fill limit %lus", tag, v);
  }
  else if (strcmp(t1, "stir") == 0)
  {
    if (v > 3600UL) { snprintf(out, cap, "dose stir <0..3600 s> (stir after dosing)"); return 1; }
    p->stir_s = (uint16_t)v; cfg_save(s);
    evs("CONFIG", src, "%s stir %lus", tag, v);
  }
  else if (strcmp(t1, "verify") == 0)
  {
    if ((v < 1UL) || (v > 20UL)) { snprintf(out, cap, "dose verify <1..20> (mix cycles before the batch alarm)"); return 1; }
    p->verify_rounds = (uint8_t)v; cfg_save(s);
    evs("CONFIG", src, "%s verify %lu cycles", tag, v);
  }
  else if (strcmp(t1, "irr") == 0)
  {
    if (strcmp(t2, "now") == 0)              /* `irr now <s>`: manual irrigation (OFF or AUTO) */
    {
      if ((v < 1UL) || (v > 3600UL)) { snprintf(out, cap, "dose irr now <1..3600 s>"); return 1; }
      if (!pin_ok(s->b->irr)) { snprintf(out, cap, "dose: no irrigation pump bound"); return 1; }
      if (s->mode == M_RELEASE) { snprintf(out, cap, "dose: released — `dose off` first"); return 1; }
      irr_start(s, (uint32_t)v, "manual");
    }
    else if (strcmp(t2, "stop") == 0) { irr_stop(s); evs("ACTION", src, "%s irrigation stop", tag); }
    else if (strcmp(t2, "win") == 0)         /* `irr win <start_h> <end_h>` */
    {
      if ((v > 23UL) || (v2 > 24UL) || (v2 <= v)) { snprintf(out, cap, "dose irr win <start 0..23> <end 1..24>"); return 1; }
      p->irr_start_h = (uint8_t)v; p->irr_end_h = (uint8_t)v2; cfg_save(s);
      evs("CONFIG", src, "%s irr window %lu..%luh", tag, v, v2);
    }
    else if (strcmp(t2, "moist") == 0)       /* `irr moist <on x10%> <off x10%>` -> moisture closed loop; `irr sched` back */
    {
      if ((v > 1000UL) || (v2 > 1000UL) || (v2 <= v)) { snprintf(out, cap, "dose irr moist <on x10%%> <off x10%%>"); return 1; }
      p->irr_src = 1U; p->irr_on_pct = (uint16_t)v; p->irr_off_pct = (uint16_t)v2; cfg_save(s);
      evs("CONFIG", src, "%s irr by moisture %lu.%lu..%lu.%lu%%", tag, v / 10UL, v % 10UL, v2 / 10UL, v2 % 10UL);
    }
    else if (strcmp(t2, "sched") == 0) { p->irr_src = 0U; cfg_save(s); evs("CONFIG", src, "%s irr by schedule", tag); }
    else                                     /* `irr <per day> <s>` */
    {
      unsigned long n = (nt >= 3) ? strtoul(t2, 0, 10) : 0UL;
      if ((nt < 3) || (n > 24UL) || (v < 1UL) || (v > 3600UL)) { snprintf(out, cap, "dose irr <0..24 per day> <1..3600 s> | now <s> | stop | win <h> <h> | moist <on> <off> | sched"); return 1; }
      p->irr_per_day = (uint8_t)n; p->irr_s = (uint16_t)v; cfg_save(s);
      evs("CONFIG", src, "%s irr %lu/day x %lus", tag, n, v);
    }
  }
  else if (strcmp(t1, "runoff") == 0)
  {
    if ((v < 10UL) || (v > 3600UL)) { snprintf(out, cap, "dose runoff <10..3600 s> (wait after irrigation before sampling / alarming)"); return 1; }
    p->runoff_wait_s = (uint16_t)v; cfg_save(s);
    evs("CONFIG", src, "%s runoff wait %lus", tag, v);
  }
  else if (strcmp(t1, "tz") == 0)
  {
    long tz = strtol(t2, 0, 10);
    if ((tz < -720L) || (tz > 840L)) { snprintf(out, cap, "dose tz <-720..840 min east of UTC> (irrigation window clock)"); return 1; }
    p->tz_min = (int16_t)tz; cfg_save(s);
    evs("CONFIG", src, "%s tz %ldmin", tag, tz);
  }
  else if (strcmp(t1, "ack") == 0)           /* clear a tank FAULT (overflow / fill timeout / fresh water) */
  {
    for (uint8_t k = 0; k < TAL_N; k++) { if (s->tal_pend[k] == 1U) { s->tal_pend[k] = 2U; } }
    s->clog_alarmed = 0U; s->verify_alarmed = 0U;
    if (s->tstate == T_FAULT) { tank_goto(s, T_IDLE); }
    evs("CONFIG", src, "%s tank ack", tag);
  }
  else
  {
    int k = pump_by_name(s, t1, (int)strlen(t1));
    unsigned long maxml = (k == (int)P_ACID) ? MAN_MAX_ACID_ML : (k == (int)P_BASE) ? MAN_MAX_BASE_ML : MAN_MAX_ML;
    if (k < 0)
    { snprintf(out, cap, "dose: a|b|%s|ab <ml> | stop|auto|off|release | cal|shot|cap|ec|ph|phdir|mix|tank|learn|phsrc|phdiv|ecsrc|ecdiv | topup|batch|fill|stir|verify|irr|runoff|tz|ack",
               s->ph_single ? "ph" : "acid|base"); return 1; }
    if (s->mode == M_RELEASE) { snprintf(out, cap, "dose: released — `dose off` first"); return 1; }
    if ((v < 1UL) || (v > maxml)) { snprintf(out, cap, "dose %s <1..%lu ml>", pump_name(s, (uint8_t)k), maxml); return 1; }
    if (shot_queue(s, (uint8_t)k, (uint16_t)v, src, "manual") != 0)
    { snprintf(out, cap, "dose: queue full"); return 1; }
  }

  {
    int n = snprintf(out, cap,
             "%s %s%s%s%s%s ec=%u(src=%s div=%u) ph=%u(src=%s div=%u) tgt=%u±%u/%u±%u phdir=%s 24h A%lu/B%lu/ac%lu/bs%luml cap=%u/%u/%u%s shot=%u/%u/%u cal=%u/%u/%u/%u mix=%lum tank=%uL",
             (s->idx == 0U) ? "dose" : tag,
             mode_name(s->mode), s->flowhold ? "(FLOWHOLD)" : "", s->hold ? "(HOLD)" : "",
             (s->run >= 0) ? " run:" : "", (s->run >= 0) ? pump_name(s, (uint8_t)s->run) : "",
             (unsigned)s->ec, (p->ec_src == 0U) ? "avg" : ((p->ec_src == 1U) ? "1" : "2"), (unsigned)p->ec_div,
             (unsigned)s->ph,
             (p->ph_src == 0U) ? "avg" : ((p->ph_src == 1U) ? "1" : "2"), (unsigned)p->ph_div,
             (unsigned)p->ec_target, (unsigned)p->ec_db, (unsigned)p->ph_target, (unsigned)p->ph_db,
             dir_name(p->ph_dir),
             (unsigned long)ml_24h(s, P_A), (unsigned long)ml_24h(s, P_B),
             (unsigned long)ml_24h(s, P_ACID), (unsigned long)ml_24h(s, P_BASE),
             (unsigned)p->cap_ml[P_A], (unsigned)p->cap_ml[P_ACID], (unsigned)p->cap_ml[P_BASE], s->caphold ? "(HOLD)" : "",
             (unsigned)p->ec_ml, (unsigned)p->acid_ml, (unsigned)p->base_ml,
             (unsigned)p->mlmin[P_A], (unsigned)p->mlmin[P_B], (unsigned)p->mlmin[P_ACID], (unsigned)p->mlmin[P_BASE],
             (unsigned long)(p->mix_s / 60U), (unsigned)(p->tank_dl / 10U));
    if ((s->tstate != T_NONE) && (n > 0) && ((uint16_t)n < cap))
    {
      snprintf(out + n, cap - (uint16_t)n, " tank=%s%s%s di=%c%c%c%c%c batch=%u.%uL fill<%us %ums/s stir=%us verify=%u irr=%u/day x%us %u..%uh %s rate=%ums/s runoff=%us ro=%u/%u",
               s_tname[s->tstate], s->irr_on ? "+irr" : "", s->hold ? "" : "",
               (s->di_prev & FERTI_DI_LOW) ? 'L' : '-', (s->di_prev & FERTI_DI_FULL) ? 'F' : '-', (s->di_prev & FERTI_DI_OVERFLOW) ? 'O' : '-',
               (s->di_prev & FERTI_DI_FRESH_LOW) ? 'W' : '-', (s->di_prev & FERTI_DI_RUNOFF) ? 'R' : '-',
               (unsigned)(p->batch_dl / 10U), (unsigned)(p->batch_dl % 10U), (unsigned)p->fill_max_s, (unsigned)s->L.fill_mls,
               (unsigned)p->stir_s, (unsigned)p->verify_rounds, (unsigned)p->irr_per_day, (unsigned)p->irr_s,
               (unsigned)p->irr_start_h, (unsigned)p->irr_end_h, p->irr_src ? "moist" : "sched", (unsigned)s->L.irr_mls,
               (unsigned)p->runoff_wait_s, (unsigned)s->ec_runoff, (unsigned)s->ph_runoff);
    }
  }
  return 1;
}

/* ---- compatibility shell: "dose [<system>] ..." from CLI and cloud ---- */
int app_dose_cmd(const char *line, const char *src, char *out, uint16_t cap)
{
  ferti_sys_t *s;
  const char *a;
  if (strncmp(line, "dose", 4) != 0) { return 0; }
  if ((line[4] != 0) && (line[4] != ' ')) { return 0; }
  a = line + 4;
  while (*a == ' ') { a++; }
  s = ferti_first();
  if (s == NULL) { snprintf(out, cap, "dose: no system configured"); return 1; }
  {
    int len = 0;
    while (a[len] && (a[len] != ' ')) { len++; }
    if (len > 0)
    {
      ferti_sys_t *named = ferti_by_name(a, len);
      if (named != NULL) { s = named; a += len; }
    }
  }
  return ferti_cmd(s, a, src, out, cap);
}

int app_dose_cli(char *line)
{
  char out[320];
  if (!app_dose_cmd(line, "cli", out, sizeof out)) { return 0; }
  printf("%s\n\r", out);
  return 1;
}

const char *app_dose_hb(void)
{
  ferti_sys_t *s = ferti_first();
  return s ? ferti_hb(s) : "-";
}

/* ---- open / start ---- */
int ferti_open(ferti_sys_t *s, const ferti_bind_t *b)
{
  if ((s == NULL) || (b == NULL) || (s_n >= FERTI_MAX)) { return -1; }
  memset(s, 0, sizeof *s);
  s->b = b; s->idx = s_n;
  s->ph_single = (pin_ok(b->pump[P_ACID]) && (b->pump[P_ACID].pt == b->pump[P_BASE].pt) && (b->pump[P_ACID].ch == b->pump[P_BASE].ch)) ? 1U : 0U;
  if (s->idx == 0U)
  {
    snprintf(s->tag, sizeof s->tag, "dose"); snprintf(s->evsrc, sizeof s->evsrc, "dose");
    snprintf(s->cfg_file, sizeof s->cfg_file, "dose.cfg"); snprintf(s->lrn_file, sizeof s->lrn_file, "dose.lrn");
  }
  else
  {
    snprintf(s->tag, sizeof s->tag, "dose %s", b->name); snprintf(s->evsrc, sizeof s->evsrc, "dose.%s", b->name);
    snprintf(s->cfg_file, sizeof s->cfg_file, "dose_%s.cfg", b->name); snprintf(s->lrn_file, sizeof s->lrn_file, "dose_%s.lrn", b->name);
  }
  seed_param(b->kind, &s->p);
  seed_gain(b->kind, s->seed_gain);
  s->L.gain[0] = s->seed_gain[0]; s->L.gain[1] = s->seed_gain[1]; s->L.gain[2] = s->seed_gain[2];
  s->L.drift_ph = SEED_DRIFT_PH; s->L.drift_ec = SEED_DRIFT_EC;
  for (uint8_t k = 0; k < 3U; k++) { learn_seed_sums(s, k); }
  s->mode = M_OFF; s->run = -1; s->confound = 1U;
  s->ec = (uint16_t)SENS_INVAL; s->ph = (uint16_t)SENS_INVAL; s->ec_avg = (uint16_t)SENS_INVAL; s->ph_avg = (uint16_t)SENS_INVAL;
  s->tu_ec0 = (uint16_t)SENS_INVAL; s->ec_runoff = (uint16_t)SENS_INVAL; s->ph_runoff = (uint16_t)SENS_INVAL;
  s->since_irr_ms = 0xFFFFFFFFUL; s->runoff_age_s = 0xFFFFFFFFUL;
  s->tstate = pin_ok(b->fill) ? T_IDLE : (pin_ok(b->irr) ? T_READY : T_NONE);   /* no fill actuator: a float valve refills, irrigation never waits */
  s_sys[s_n++] = s;
  cfg_load(s);                       /* params from flash before anything runs (retried in the task) */
  learn_load(s);                     /* learned gains/drifts: never start from scratch after a reboot */
  return 0;
}

void ferti_start(void)
{
  if (s_started) { return; }
  s_started = 1U;
  xTaskCreate(ferti_task, "dose", 896, NULL, APP_TASK_PRIO_LOW, NULL);
}
