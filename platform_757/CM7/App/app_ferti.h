/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_ferti.h — fertigation systems: one instance per reservoir / crop line.
 *
 * Modelled on uart_drv: a compile-time BINDING (which scan-table points and channels are
 * this system's probes, pumps, floats, meter), a persisted PARAMETER block (targets,
 * calibration, caps, cycle times), and an INSTANCE that carries every byte of run-time
 * state. Every API takes the instance. Adding a system = one ferti_bind_t constant, one
 * ferti_sys_t variable and one ferti_open() in app_user.c; the core never changes.
 *
 * Every I/O is a (scan-table point, channel) pair, so a probe or a pump may live on an
 * expansion module on the backplane, on a third-party Modbus device on a front RS-485
 * port (add a scan-table row in app_user.c and point the binding at it) or, for pulse
 * meters, on a board HSDI channel (pseudo point FERTI_PT_HSDI). Hardware the system does
 * not have is bound to FERTI_PIN_NA and the matching logic simply never runs.
 *
 * The chemistry loop (EC/pH servo, self-learning, 24 h caps, top-up feed-forward) is the
 * app_dose.c control law of 2026-08-31..09-20, moved into the instance unchanged. The
 * hydraulics (float-driven batch refill, stir, verify-then-release, timed irrigation) are
 * the drip-to-waste additions of 2026-10-26 and only engage on bindings that have them.
 *
 * Command line (CLI and cloud dn/cmd, same text):  dose [<system>] <args...>
 *   The first instance opened answers plain `dose ...` (compatibility with every script
 *   and dashboard written against the single-system firmware); other instances are
 *   addressed by name, e.g. `dose coco ec 2400`. See ferti_cmd() for the verb list. */
#ifndef APP_FERTI_H
#define APP_FERTI_H
#include <stdint.h>

/* ---- binding primitives ---- */
typedef struct { uint16_t pt; uint8_t ch; } ferti_pin_t;      /* scan-table point + channel */
#define FERTI_PIN_NONE   0xFFFFU                                /* pt value: not bound */
#define FERTI_PT_HSDI    0xFFF0U                                /* pseudo point: board HSDI counter, ch = HSDI channel */
#define FERTI_PIN(p, c)  { (uint16_t)(p), (uint8_t)(c) }
#define FERTI_PIN_NA     { FERTI_PIN_NONE, 0U }

/* analog source: pin + linear scaling into the core's units (EC uS/cm, pH x100, temp x10,
 * moisture x10 %). Third-party registers come as x10 / x100 / signed / with their own
 * "invalid" code; the PH_EC module needs mul=div=1, off=0, inval=0x7FFF. */
typedef struct { ferti_pin_t pin; int16_t mul, div, off; uint16_t inval; } ferti_ai_t;
#define FERTI_AI(p, c)   { FERTI_PIN(p, c), 1, 1, 0, 0x7FFFU }
#define FERTI_AI_NA      { FERTI_PIN_NA, 1, 1, 0, 0x7FFFU }

/* ---- system kinds: fixed per binding, pick the parameter seeds and the active branches ---- */
enum {
  FERTI_NFT = 0,   /* recirculating gutters: float valve + pulse meter top-up, continuous servo */
  FERTI_RDWC,      /* recirculating buckets: small frequent shots */
  FERTI_DWC,       /* still bucket: manual top-up */
  FERTI_DTW,       /* drip-to-waste (coco/rockwool): batch refill by floats, feed-forward, verify, timed irrigation */
  FERTI_EBB,       /* ebb & flow: reserved */
  FERTI_KINDS
};

#define FERTI_PUMPS      4U       /* a, b, acid, base (acid and base may share one physical pump) */
#define FERTI_P_A        0U
#define FERTI_P_B        1U
#define FERTI_P_ACID     2U
#define FERTI_P_BASE     3U

typedef struct {
  const char *name;            /* "nft", "coco": command namespace, file suffix, heartbeat/event/alarm suffix */
  uint8_t     kind;            /* FERTI_* */

  /* chemistry: probes */
  ferti_ai_t  ec[2], ph[2];    /* [1] = FERTI_AI_NA for a single probe; the two may be on different devices */
  ferti_ai_t  t_water;         /* reserved (air pump still lives in app_aer) */
  ferti_ai_t  ec_runoff, ph_runoff;   /* drip-to-waste runoff sample (second EC/pH pair) */

  /* chemistry: dosing pumps (coils). One pH pump: bind acid AND base to the same pin and set
   * the direction with `dose <sys> phdir acid|base` (the bottle decides). */
  ferti_pin_t pump[FERTI_PUMPS];

  /* hydraulics: actuators (coils) */
  ferti_pin_t fill;            /* transfer pump / solenoid that refills the reservoir */
  ferti_pin_t stir;            /* stirring pump / air stone */
  ferti_pin_t irr;             /* irrigation pump (DTW / EBB) */
  ferti_pin_t circ;            /* circulation pump (reserved: greenhouse pumps run unswitched) */
  ferti_pin_t air;             /* air pump (reserved: app_aer owns it this release) */

  /* hydraulics: level and state inputs (discrete) */
  ferti_pin_t di_low, di_full, di_overflow, di_fresh_low, di_runoff;
  uint8_t     di_inv;          /* bit mask (FERTI_DI_* below): input is normally-closed, i.e. OPEN = asserted */

  /* irrigation trigger source (second generation, substrate moisture probes) */
  ferti_ai_t  moisture[4];

  /* volume and interlocks */
  ferti_pin_t meter;           /* pulse water meter: FERTI_PT_HSDI + channel; FERTI_PIN_NA = volume comes from floats / `topup` */
  uint16_t    meter_ppl;       /* meter pulses per litre */
  uint8_t     flow_lock;       /* 1 = hold dosing while app_flowmon reports the feed supply lost (NFT) */
} ferti_bind_t;

#define FERTI_DI_LOW       0x01U
#define FERTI_DI_FULL      0x02U
#define FERTI_DI_OVERFLOW  0x04U
#define FERTI_DI_FRESH_LOW 0x08U
#define FERTI_DI_RUNOFF    0x10U

/* ---- persisted parameters (littlefs; first instance keeps the legacy "dose.cfg" name) ---- */
typedef struct {
  /* chemistry (= the app_dose.c parameter set, LITE dose_cfg_t field names) */
  uint16_t mlmin[FERTI_PUMPS];       /* pump flow calibration, ml/min */
  uint16_t ec_target, ec_db;         /* uS/cm */
  uint16_t ph_target, ph_db;         /* pH x100 */
  uint16_t ec_ml, acid_ml, base_ml;  /* per-cycle auto shot caps */
  uint32_t mix_s;                    /* decision cycle = settling time */
  uint16_t tank_dl;                  /* reservoir volume, 0.1 L (feed-forward scale) */
  uint8_t  ph_dir;                   /* 0 both, 1 base only, 2 acid only */
  uint8_t  ph_src, ec_src;           /* 0 mean with divergence guard, 1/2 that probe */
  uint16_t ph_div, ec_div;           /* divergence hold thresholds; 0 = off */
  uint16_t cap_ml[FERTI_PUMPS];      /* rolling 24 h caps */
  /* hydraulics (ignored without the matching binding) */
  uint16_t batch_dl;                 /* refill batch volume, 0.1 L (between the low and full floats) */
  uint16_t fill_max_s;               /* refill time limit (expected x 1.5, refreshed from the learned rate) */
  uint16_t stir_s;                   /* stir after dosing */
  uint8_t  verify_rounds;            /* mix cycles to wait for EC/pH in band before giving up */
  uint8_t  irr_per_day; uint16_t irr_s;   /* timed irrigation: events per day, seconds each */
  uint8_t  irr_start_h, irr_end_h;   /* irrigation window, local hours */
  uint8_t  irr_src;                  /* 0 schedule, 1 moisture closed loop */
  uint16_t irr_on_pct, irr_off_pct;  /* moisture thresholds x10 % */
  uint16_t runoff_wait_s;            /* after irrigation: sample runoff / declare "no runoff" */
  int16_t  tz_min;                   /* local time offset for the irrigation window, minutes east of UTC */
} ferti_param_t;

/* ---- learned model (littlefs; first instance keeps "dose.lrn") ---- */
typedef struct {
  uint32_t gain[3];                  /* x1000 per ml: [0] A/B uS, [1] acid pH x100, [2] base pH x100 */
  int32_t  drift_ph, drift_ec;       /* x1000 per hour */
  int32_t  lsum_ml[3], lsum_resp[3]; /* forgetting sums behind gain[] (ml x100 / response fine) */
  uint16_t tu_typ_dl;                /* learned typical top-up volume, 0.1 L (NFT float valve) */
  uint16_t fill_mls, irr_mls;        /* DTW: transfer / irrigation pump ml per second, refreshed per batch */
  uint32_t irr_last;                 /* Unix time the last irrigation STARTED (0 = never). The irrigation record:
                                      * after a reset the day's slot bookkeeping is rebuilt from it, so a reboot
                                      * inside a slot that already ran does not irrigate again (2026-11-09) */
} ferti_lrn_t;

/* ---- instance ---- */
#define FERTI_QLEN       8U
#define FERTI_AVG_N      12U
#define FERTI_CAP_HOURS  24U
#define FERTI_SENS_INVAL 0x7FFFU

typedef struct {
  const ferti_bind_t *b;
  uint8_t  idx;                      /* position in the instance table: 0 = legacy names */
  char     tag[16];                  /* event text prefix: "dose" / "dose coco" */
  char     evsrc[16];                /* alarm src: "dose" / "dose.coco" */
  char     cfg_file[20], lrn_file[20];
  uint8_t  ph_single;                /* acid and base share one pump */
  uint32_t seed_gain[3];             /* kind seeds scaled to tank volume: learning clamp reference */
  ferti_param_t p;
  ferti_lrn_t   L;

  /* chemistry run state */
  volatile uint8_t mode;             /* FERTI_M_* */
  int8_t   run; uint32_t run_ms; uint8_t rel_done;   /* rel_done: coils dropped once on entering release */
  struct { uint8_t pump; uint16_t ml; } q[FERTI_QLEN]; uint8_t qn;
  uint16_t ml_h[FERTI_CAP_HOURS][FERTI_PUMPS]; uint8_t hi; uint32_t hour_s;
  uint8_t  caphold;
  uint8_t  al_pend[FERTI_PUMPS + 3U];
  uint8_t  phhold;                   /* pH loop held by a no-response fault: 0 none, 1 acid, 2 base (RAM only; `dose auto` re-arms) */
  uint8_t  echold;                   /* EC loop held: the A/B pair did not respond (RAM only; `dose auto` re-arms) */
  uint8_t  off_al; char off_reason[40];
  uint32_t wait_s, bad_s;
  uint8_t  faulted, flowhold;
  uint8_t  hold; char hold_why[12];  /* hydraulics hold (fill / stir): no decisions */
  uint16_t ec, ph;
  uint16_t ec_ring[FERTI_AVG_N], ph_ring[FERTI_AVG_N]; uint8_t ring_i, ring_n;
  uint16_t ec_avg, ph_avg;
  uint8_t  ph_src_bad, ec_src_bad;   /* latched: probes disagree */
  uint8_t  strikes[3], unlearned[3];
  int32_t  nr_exp[3];                /* expected movement (x1000) of the consecutive no-response shots per slot */
  int32_t  nr_mov[3];                /* what the tank really did over that run (x1000, + = the pump's own direction) */
  uint8_t  confirm[3];               /* the run reached the trip count while the tank held: the next shot is the confirmation */
  uint8_t  ph_alone;                 /* consecutive cycles the A/B pair was deferred so a pH shot could be judged alone */
  struct ferti_gate { uint16_t tr, cand, n; uint8_t oor, ok_n; } g_ec, g_ph;   /* plausibility + jump gate per channel */
  uint16_t ec_raw, ph_raw; uint8_t wq_bad_n;   /* ungated readings and how many 5 s samples in a row they were unsafe to irrigate with */
  char     use_file[20];             /* 24 h usage record on littlefs */
  uint8_t  use_loaded, use_dirty; uint16_t use_wait_s;
  uint32_t tu_last_dl; uint8_t tu_primed, tu_in;
  int32_t  tu_owed; uint32_t tu_ep_dl, tu_ep_ml, tu_quiet_s;
  uint16_t tu_pred_dl; int32_t tu_ep_dil; uint16_t tu_ec0; int32_t tu_carry;
  uint32_t vmeter_dl;                /* virtual meter for systems without a pulse meter: batch refills / `topup` add here */
  int32_t  ff_ph, ff_ec;
  struct { uint8_t kind, slot; uint16_t ml, ec0, ph0; uint32_t flow0; int32_t dil; uint8_t ph_slot; uint16_t ph_ml; uint8_t confirm; } last;
  uint8_t  confound;
  int32_t  acc_dph, acc_dec, acc_dose_ph, acc_dose_ec; uint32_t acc_t;
  uint8_t  evl;                      /* 500 ms ticks toward the 5 s evaluation */
  uint8_t  ploaded;

  /* hydraulics run state (stays FERTI_T_NONE without a fill binding) */
  uint8_t  tstate; uint32_t t_ms; uint8_t verify_n; uint8_t verify_alarmed;
  uint32_t fill_ms_batch, irr_ms_batch; uint16_t irr_n_batch;
  uint8_t  irr_on; uint32_t irr_ms_left; uint32_t irr_slot_done; uint16_t irr_day;
  uint32_t since_irr_ms;             /* ms since the last irrigation stopped (runoff timer); 0xFFFFFFFF = none yet */
  uint8_t  runoff_seen, runoff_sampled, irr_since_full, full_stuck_n, clog_alarmed, fresh_alarmed, leak_alarmed, low_alarmed;
  uint32_t low_ms;                   /* ms the low float has been asserted (float-valve systems: dwell before the alarm) */
  uint32_t stir_ms_left;             /* float-valve systems: keep the stirrer running this long after the last pump stops */
  uint16_t ec_runoff, ph_runoff; uint32_t runoff_age_s;
  uint8_t  di_prev;                  /* FERTI_DI_* bits, previous tick (edge detection) */
  uint32_t fill_dl0;                 /* inlet meter reading when the refill started (overrun guard, measured batch) */
  uint32_t m_last_dl, m_leak_dl, m_leak_ms;   /* inlet meter movement while the valve is commanded off (leak / valve stuck guard) */
  uint32_t m_base, m_raw;            /* pulse meter made monotonic: a counter that restarts (HSDI reconfigured) adds no volume */
  uint32_t noclk_ms; uint8_t noclk_alarmed;   /* scheduled irrigation waiting for a clock */
  uint8_t  irr_wq_held;              /* irrigation held: reservoir pH/EC far outside the safe range */
  uint8_t  tal_pend[9];              /* hydraulics alarms pending: 1 active, 2 clear */
  char     fault_why[16];

  char     hb[80];
  uint8_t  dfbuf[256];               /* littlefs file buffer (LFS_NO_MALLOC) */
} ferti_sys_t;

#define FERTI_M_OFF      0U
#define FERTI_M_AUTO     1U
#define FERTI_M_RELEASE  2U

#define FERTI_T_NONE     0U
#define FERTI_T_IDLE     1U
#define FERTI_T_FILL     2U
#define FERTI_T_DOSE     3U
#define FERTI_T_STIR     4U
#define FERTI_T_VERIFY   5U
#define FERTI_T_READY    6U
#define FERTI_T_FAULT    7U

#define FERTI_MAX        4U

/* ---- API ---- */
int  ferti_open(ferti_sys_t *s, const ferti_bind_t *b);  /* seeds by kind, loads cfg/lrn, coils off, registers; 0 = ok */
void ferti_start(void);                                  /* creates the (single) 500 ms service task; idempotent */
int  ferti_cmd(ferti_sys_t *s, const char *args, const char *src, char *out, uint16_t cap);   /* args = text after the system name */
const char *ferti_hb(ferti_sys_t *s);                    /* heartbeat status string */
const char *ferti_hb_more(void);                         /* ',"dose_<name>":"..."' for every instance but the first ("" when none) */
const char *ferti_desc_more(void);                       /* self-description point entries for every instance but the first */
ferti_sys_t *ferti_by_name(const char *name, int len);
ferti_sys_t *ferti_first(void);
uint8_t ferti_count(void);
int  ferti_owns_coil(uint16_t pt, uint8_t ch);           /* 1 = some instance drives this coil (dosing pump / fill / stir / irrigation): bench tools keep off */

#endif /* APP_FERTI_H */
