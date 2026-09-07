/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_hsdi_cm4.c — onboard 8-channel high-speed isolated digital inputs (CN4), CM4-owned.
 *
 * Each channel is configurable as plain DI, software (EXTI) edge counter, hardware timer
 * counter, timer-capture counter, polled counter, or quadrature encoder phase / index.
 * Results land in the ONBOARD region of the process image, so the application and any
 * external Modbus master read the same data. Contract: docs/HSDI_Configuration_and_Counting.md.
 *
 * Hardware facts that shape everything here:
 *   - one timer has ONE counter, so a hardware counter costs a whole timer (external clock mode)
 *   - external clock can only come from TI1/TI2, so HSDI2/3/7 (CH3/CH4) can never count in hardware
 *   - encoder mode needs CH1+CH2, so only TIM3 (HSDI0+1) and TIM1 (HSDI4+5) can do encoders
 *   - EXTI lines are shared by pin number across ports: HSDI2/HSDI7 both sit on EXTI0,
 *     HSDI3/HSDI4 both sit on EXTI1 — only one of each pair may use an interrupt
 *   - every HSDI pin IS a timer capture channel, so COUNT_CAP works on all eight and needs
 *     no EXTI line at all; capture coexists with the same timer counting or decoding
 *   - TIM1/3/8 are 16-bit, so the 32-bit values published to the image are extended in software
 *     from the signed 16-bit delta sampled every main-loop tick
 *
 * Dirty field signals (2026-09-03/04, four hall flow meters, 15 ms pulses): every edge-based
 * method over-counted 1.5..2x because the high phase carries microsecond-scale dips that pass
 * the timer's ~1 us input filter. The answer is the PULSE QUALIFIER below: both edges are
 * timestamped in the interrupt, and a level only becomes "the" level once it has held for
 * `min_us`. A dip inside a pulse or a spike inside a gap is a phase shorter than min_us and
 * is dropped; the count moves on qualified transitions only. The 1 ms poll merely confirms a
 * phase that has already lasted min_us (so a count is published within ~1 ms of the edge) —
 * it never has to catch an edge, so task load cannot cost a pulse.
 */
#include <string.h>
#include "stm32h7xx_hal.h"
#include "app_pimage.h"

/* ---- pin / timer map (EDA netlist) ---- */
typedef struct {
  GPIO_TypeDef *port;
  uint16_t      pin;
  TIM_TypeDef  *tim;
  uint8_t       ch;       /* timer channel 1..4 */
  uint8_t       af;
  IRQn_Type     irq;      /* EXTI interrupt for this pin number */
} hsdi_map_t;

static const hsdi_map_t s_map[PIMG_HSDI_CH] = {
  { GPIOC, GPIO_PIN_6,  TIM3, 1, GPIO_AF2_TIM3, EXTI9_5_IRQn   },  /* HSDI0 PC6  */
  { GPIOC, GPIO_PIN_7,  TIM3, 2, GPIO_AF2_TIM3, EXTI9_5_IRQn   },  /* HSDI1 PC7  */
  { GPIOB, GPIO_PIN_0,  TIM3, 3, GPIO_AF2_TIM3, EXTI0_IRQn     },  /* HSDI2 PB0  */
  { GPIOB, GPIO_PIN_1,  TIM3, 4, GPIO_AF2_TIM3, EXTI1_IRQn     },  /* HSDI3 PB1  */
  { GPIOK, GPIO_PIN_1,  TIM1, 1, GPIO_AF1_TIM1, EXTI1_IRQn     },  /* HSDI4 PK1  */
  { GPIOJ, GPIO_PIN_11, TIM1, 2, GPIO_AF1_TIM1, EXTI15_10_IRQn },  /* HSDI5 PJ11 */
  { GPIOJ, GPIO_PIN_10, TIM8, 2, GPIO_AF3_TIM8, EXTI15_10_IRQn },  /* HSDI6 PJ10 */
  { GPIOK, GPIO_PIN_0,  TIM8, 3, GPIO_AF3_TIM8, EXTI0_IRQn     },  /* HSDI7 PK0  */
};

/* encoder groups: 0 = TIM3 (A=HSDI0 B=HSDI1 Z=HSDI2), 1 = TIM1 (A=HSDI4 B=HSDI5 Z=HSDI3) */
#define ENC_TIM3_A 0U
#define ENC_TIM3_B 1U
#define ENC_TIM3_Z 2U
#define ENC_TIM1_A 4U
#define ENC_TIM1_B 5U
#define ENC_TIM1_Z 3U

static pimg_hsdi_cfg_t s_cfg[PIMG_HSDI_CH];
static uint8_t   s_loaded = 0;
static uint32_t  s_acc[PIMG_HSDI_CH];      /* 32-bit extended counter / position per channel */
static uint16_t  s_last16[PIMG_HSDI_CH];   /* last raw 16-bit timer value */
static uint8_t   s_hw_owner[3];            /* which channel owns TIM3 / TIM1 / TIM8 (0xFF = none) */
static uint8_t   s_enc_on[2];              /* encoder group active: [0]=TIM3 [1]=TIM1 */
static uint32_t  s_zrev[2], s_zlatch[2];
static volatile uint32_t s_sw[PIMG_HSDI_CH];       /* interrupt / polled counters */
static volatile uint32_t s_sw_last_ms[PIMG_HSDI_CH]; /* last accepted edge, for counter debounce */
static uint8_t   s_level[PIMG_HSDI_CH];            /* debounced level */
static uint8_t   s_praw[PIMG_HSDI_CH];             /* previous raw sample (COUNT_POLL edge detect) */
static uint8_t   s_cand[PIMG_HSDI_CH];             /* level currently being timed out */
static uint16_t  s_cand_ms[PIMG_HSDI_CH];          /* how long the candidate has been stable */
static uint32_t  s_cyc_us = 240U;                  /* DWT cycles per microsecond (SystemCoreClock / 1e6) */

static TIM_HandleTypeDef s_t3, s_t1, s_t8;

static int tim_index(const TIM_TypeDef *t) { return (t == TIM3) ? 0 : (t == TIM1) ? 1 : (t == TIM8) ? 2 : -1; }
static TIM_HandleTypeDef *tim_handle(int i) { return (i == 0) ? &s_t3 : (i == 1) ? &s_t1 : &s_t8; }
static inline uint8_t pin_level(uint8_t ch) { return (uint8_t)((s_map[ch].port->IDR & s_map[ch].pin) != 0U); }

/* ================= pulse qualifier + signal forensics ================= */
typedef struct {
  uint32_t last_cyc;   /* start of the current phase (DWT cycles) */
  uint8_t  lvl;        /* level of the current phase */
  uint8_t  qlvl;       /* qualified level: last phase that held >= min_us */
  uint8_t  valid;      /* current phase already qualified (by duration or by the poll) */
} qual_t;
static qual_t   s_q[PIMG_HSDI_CH];
static uint32_t s_min_cyc[PIMG_HSDI_CH];           /* min_us in DWT cycles; 0 = qualifier off */

/* per-channel statistics for `hsdi <ch> stat` (RPC op 0x11): interval bins between consecutive
 * interrupts of any polarity, 1 kHz shadow sampling (edges/high samples = what polling would
 * read), phases rejected by the qualifier, blips (edge pairs inside the ISR latency) */
typedef struct { uint32_t last_cyc, min_dt, bin[4], poll_edges, hi_samples, rej, blip; uint8_t praw; } capstat_t;
static capstat_t s_cs[PIMG_HSDI_CH];
static volatile uint32_t s_coinc, s_rej_cyc; static volatile uint8_t s_rej_ch = 0xFFU;   /* two channels rejecting within 200 us */

/* ---- interrupt storm breaker (2026-09-04, field recording): a hall meter emitted ~97 kHz
 * oscillation bursts (edges 5 us apart) lasting up to 8 ms — beyond any timer filter (ICF max
 * ~4 us) and enough to hold the CM4 in the capture ISR back-to-back for the whole burst. The
 * qualifier does not need those edges (a chattering line is neither level), so once a channel
 * has produced STORM_LIMIT interrupts in one millisecond its source is muted until the next
 * 1 ms poll, which re-arms it and reconciles the level. Bounds the ISR load per channel at
 * STORM_LIMIT edges per ms whatever the line does; a legitimate <=10 kHz signal never trips it. */
#define STORM_LIMIT 16U
static volatile uint8_t  s_isr_ms[PIMG_HSDI_CH];   /* interrupt edges in the current millisecond */
static volatile uint8_t  s_muted[PIMG_HSDI_CH];    /* source muted until the poll re-arms it */
static uint32_t          s_storm[PIMG_HSDI_CH];    /* storms tripped (stat) */
static uint32_t          s_isr_max[PIMG_HSDI_CH];  /* longest edge service, DWT cycles (stat) */

/* ---- edge recorder (2026-09-04): the board as its own logic analyser for the field signal ----
 * Every interrupt edge on the channels in `mask` is logged with a DWT timestamp (4.2 ns), the
 * timer capture register (a jitter-free fine delta between edges of one timer) and flags,
 * until `lim` entries are in. CM7 pulls it in 60-entry slices (RPC op 0x12) and streams it to
 * the cloud (`hsdi rec get`); tools/hsdi_rec.py turns it into phases, histograms and a plot. */
#define REC_N       2048U
#define REC_F_LOST  0x01U   /* timer over-capture: at least one edge was missed before this one */
#define REC_F_EXTI  0x02U   /* EXTI-sourced edge (no capture register) */
#define REC_F_BLIP  0x04U   /* level unchanged since the previous edge: an edge pair inside the ISR latency */
#define REC_F_SYNTH 0x08U   /* synthesised by the poll after a storm mute: the level had changed meanwhile */
typedef struct { uint32_t ts; uint16_t ccr; uint8_t chl; uint8_t flg; } rec_t;   /* chl = ch | level<<4 */
static rec_t s_rec[REC_N];
static volatile uint16_t s_rec_n, s_rec_lim;
static volatile uint8_t  s_rec_mask, s_rec_armed;

static inline void rec_log(uint8_t ch, uint8_t lvl, uint32_t now, uint16_t ccr, uint8_t flg)
{
  if (s_rec_armed && (s_rec_mask & (uint8_t)(1U << ch)))
  {
    uint16_t n = s_rec_n;
    if (n < s_rec_lim)
    {
      s_rec[n].ts = now; s_rec[n].ccr = ccr; s_rec[n].chl = (uint8_t)(ch | (lvl ? 0x10U : 0U)); s_rec[n].flg = flg;
      s_rec_n = (uint16_t)(n + 1U);
    }
    if (s_rec_n >= s_rec_lim) { s_rec_armed = 0U; }
  }
}
void hsdi_cm4_rec_arm(uint8_t mask, uint16_t lim)
{
  s_rec_armed = 0U;
  s_rec_n = 0U;
  s_rec_lim = ((lim == 0U) || (lim > REC_N)) ? (uint16_t)REC_N : lim;
  s_rec_mask = mask;
  if (mask != 0U) { s_rec_armed = 1U; }
}
uint16_t hsdi_cm4_rec_status(uint8_t *out)   /* [armed][n u16][lim u16][mask][cyc_per_us][0] */
{
  out[0] = s_rec_armed;
  out[1] = (uint8_t)(s_rec_n & 0xFFU); out[2] = (uint8_t)(s_rec_n >> 8);
  out[3] = (uint8_t)(s_rec_lim & 0xFFU); out[4] = (uint8_t)(s_rec_lim >> 8);
  out[5] = s_rec_mask; out[6] = (uint8_t)s_cyc_us; out[7] = 0U;
  return 8U;
}
uint16_t hsdi_cm4_rec_read(uint16_t idx, uint8_t n, uint8_t *out)   /* n x 8 B, n <= 60 (fits one RPMsg buffer) */
{
  uint16_t k = 0;
  if (n > 60U) { n = 60U; }
  while ((k < n) && ((uint16_t)(idx + k) < s_rec_n))
  {
    memcpy(&out[8U * k], &s_rec[idx + k], sizeof(rec_t));
    k++;
  }
  return (uint16_t)(8U * k);
}

/* a phase at `lvl` proved long enough: it becomes the qualified level; count the transition */
static void qual_settle(uint8_t ch, uint8_t lvl)
{
  qual_t *q = &s_q[ch];
  if (q->qlvl != lvl)
  {
    uint8_t e = s_cfg[ch].edge;
    q->qlvl = lvl;
    if ((e == HSDI_EDGE_BOTH) || ((e == HSDI_EDGE_RISING) && lvl) || ((e == HSDI_EDGE_FALLING) && !lvl)) { s_sw[ch]++; }
  }
}

/* every interrupt edge (capture or EXTI) of a counting channel comes through here.
 * lv = pin level read in the ISR, now = DWT cycle counter, ccr = capture register (0 for EXTI). */
static void edge_in(uint8_t ch, uint8_t lv, uint32_t now, uint16_t ccr, uint8_t flg)
{
  capstat_t *c = &s_cs[ch];
  qual_t    *q = &s_q[ch];
  if (c->last_cyc != 0U)
  {
    uint32_t dt = now - c->last_cyc, us = dt / s_cyc_us;
    if ((c->min_dt == 0U) || (dt < c->min_dt)) { c->min_dt = dt; }
    c->bin[(us < 100U) ? 0 : (us < 1000U) ? 1 : (us < 5000U) ? 2 : 3]++;
  }
  c->last_cyc = now;

  if (s_min_cyc[ch] == 0U)                        /* qualifier off: classic per-edge counting */
  {
    uint8_t e = s_cfg[ch].edge;
    rec_log(ch, lv, now, ccr, flg);
    /* level confirmation: a ring on the opposite transition is gone again by the time the ISR
     * samples the pin, so a "rising" edge that reads low is not one (field 2026-09-03) */
    if (((e == HSDI_EDGE_RISING) && !lv) || ((e == HSDI_EDGE_FALLING) && lv)) { return; }
    if (s_cfg[ch].debounce_ms != 0U)              /* contact bounce: ignore edges inside the debounce window */
    {
      uint32_t t = HAL_GetTick();
      if ((t - s_sw_last_ms[ch]) < (uint32_t)s_cfg[ch].debounce_ms) { return; }
      s_sw_last_ms[ch] = t;
    }
    s_sw[ch]++;
    return;
  }
  if (lv == q->lvl)                               /* two edges inside the ISR latency: a sub-microsecond
                                                   * blip, the phase simply continues */
  {
    c->blip++;
    rec_log(ch, lv, now, ccr, (uint8_t)(flg | REC_F_BLIP));
    return;
  }
  rec_log(ch, lv, now, ccr, flg);
  if (!q->valid)                                  /* the phase that just ended: long enough, or a glitch? */
  {
    if ((now - q->last_cyc) >= s_min_cyc[ch]) { qual_settle(ch, q->lvl); }
    else
    {
      c->rej++;
      if ((s_rej_ch != 0xFFU) && (s_rej_ch != ch) && ((now - s_rej_cyc) < (200U * s_cyc_us))) { s_coinc++; }
      s_rej_cyc = now; s_rej_ch = ch;
    }
  }
  q->lvl = lv; q->last_cyc = now; q->valid = 0U;
}

/* 1 ms poll: a phase that has already lasted min_us is qualified now, without waiting for the
 * next edge (a pulse is counted ~1 ms after its rising edge, and a line parked high or low for
 * ever still settles). Critical section: the ISR rewrites q on every edge. */
static void qual_poll(void)
{
  for (uint8_t ch = 0; ch < PIMG_HSDI_CH; ch++)
  {
    if ((s_min_cyc[ch] != 0U) && ((s_cfg[ch].mode == HSDI_COUNT_CAP) || (s_cfg[ch].mode == HSDI_COUNT_SW)))
    {
      qual_t *q = &s_q[ch];
      if (!q->valid)
      {
        uint32_t pm = __get_PRIMASK();
        __disable_irq();
        if (!q->valid && ((DWT->CYCCNT - q->last_cyc) >= s_min_cyc[ch])) { q->valid = 1U; qual_settle(ch, q->lvl); }
        __set_PRIMASK(pm);
      }
    }
  }
}

/* 1 ms poll: new millisecond for the storm breaker; a muted source is re-armed and, if the line
 * changed level while it was muted, the missed transition is fed to the qualifier now (a real
 * phase is milliseconds long, so a <=1 ms late timestamp costs nothing). */
static void storm_poll(void)
{
  for (uint8_t ch = 0; ch < PIMG_HSDI_CH; ch++)
  {
    s_isr_ms[ch] = 0U;
    if (s_muted[ch])
    {
      uint32_t pm = __get_PRIMASK();
      __disable_irq();
      s_muted[ch] = 0U;
      if (s_cfg[ch].mode == HSDI_COUNT_CAP)
      {
        TIM_TypeDef *t = s_map[ch].tim;
        uint32_t f = 1UL << s_map[ch].ch;
        t->SR = ~(f | (f << 8));                       /* drop what the storm left pending */
        t->DIER |= f;
      }
      else
      {
        __HAL_GPIO_EXTID2_CLEAR_IT(s_map[ch].pin);
        EXTI->C2IMR1 |= (uint32_t)s_map[ch].pin;
      }
      {
        uint8_t lv = pin_level(ch);
        if ((s_min_cyc[ch] != 0U) && (lv != s_q[ch].lvl)) { edge_in(ch, lv, DWT->CYCCNT, 0U, REC_F_SYNTH); }
      }
      __set_PRIMASK(pm);
    }
  }
}

uint16_t hsdi_cm4_capstat(uint8_t ch, uint8_t *out)   /* RPC 0x11 reply: 13 x u32 LE */
{
  uint32_t v[13] = {0};
  if (ch < PIMG_HSDI_CH)
  {
    capstat_t *c = &s_cs[ch];
    v[0] = s_sw[ch]; v[1] = c->poll_edges; v[2] = c->hi_samples;
    v[3] = c->min_dt / s_cyc_us;
    v[4] = c->bin[0]; v[5] = c->bin[1]; v[6] = c->bin[2]; v[7] = c->bin[3];
    v[8] = c->rej; v[9] = s_coinc; v[10] = c->blip;
    v[11] = s_storm[ch]; v[12] = (s_isr_max[ch] * 100U) / s_cyc_us;   /* 0.01 us */
  }
  memcpy(out, v, sizeof v);
  return (uint16_t)sizeof v;
}

/* ================= pin / timer setup ================= */
static int hsdi_validate(const pimg_hsdi_cfg_t *c)
{
  /* single source of truth: the shared checker in app_pimage.h — CM7 rejects the table at
   * setup with the same rules, this call is the defensive second line before touching timers */
  return pimg_hsdi_check(c);
}

static void pin_af(uint8_t ch)
{
  GPIO_InitTypeDef g = {0};
  g.Pin = s_map[ch].pin; g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate = s_map[ch].af;
  HAL_GPIO_Init(s_map[ch].port, &g);
}
static void pin_in(uint8_t ch, uint32_t mode)
{
  GPIO_InitTypeDef g = {0};
  g.Pin = s_map[ch].pin; g.Mode = mode; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(s_map[ch].port, &g);
}

static void clocks_on(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE(); __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();  __HAL_RCC_TIM3_CLK_ENABLE(); __HAL_RCC_TIM8_CLK_ENABLE();
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   /* edge timestamps */
  s_cyc_us = SystemCoreClock / 1000000U;
  if (s_cyc_us == 0U) { s_cyc_us = 1U; }
}

static void enc_start(int ti, uint8_t a_ch, uint8_t z_ch)
{
  TIM_HandleTypeDef *h = tim_handle(ti);
  TIM_Encoder_InitTypeDef e = {0};
  memset(h, 0, sizeof(*h));
  h->Instance = s_map[a_ch].tim;
  h->Init.Period = 0xFFFFU; h->Init.Prescaler = 0; h->Init.CounterMode = TIM_COUNTERMODE_UP;
  h->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  e.EncoderMode = TIM_ENCODERMODE_TI12;                 /* x4 quadrature */
  e.IC1Polarity = TIM_ICPOLARITY_RISING; e.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  e.IC1Prescaler = TIM_ICPSC_DIV1; e.IC1Filter = s_cfg[a_ch].filter & 0x0FU;
  e.IC2Polarity = TIM_ICPOLARITY_RISING; e.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  e.IC2Prescaler = TIM_ICPSC_DIV1; e.IC2Filter = s_cfg[a_ch].filter & 0x0FU;
  if (HAL_TIM_Encoder_Init(h, &e) != HAL_OK) { return; }
  __HAL_TIM_SET_COUNTER(h, 0);        /* must match s_last16 = 0, else the first delta jumps */
  (void)HAL_TIM_Encoder_Start(h, TIM_CHANNEL_ALL);
  /* TIM3 group: the Z index is captured in hardware on CH3 — it latches the encoder count */
  if ((ti == 0) && (z_ch == ENC_TIM3_Z) && (s_cfg[z_ch].mode == HSDI_ENC_Z))
  {
    TIM_IC_InitTypeDef ic = {0};
    ic.ICPolarity = TIM_ICPOLARITY_RISING; ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1; ic.ICFilter = s_cfg[z_ch].filter & 0x0FU;
    if (HAL_TIM_IC_ConfigChannel(h, &ic, TIM_CHANNEL_3) == HAL_OK)
    {
      pin_af(z_ch);
      (void)HAL_TIM_IC_Start(h, TIM_CHANNEL_3);
    }
  }
}

static void cnt_start(int ti, uint8_t ch)
{
  TIM_HandleTypeDef *h = tim_handle(ti);
  TIM_ClockConfigTypeDef sc = {0};
  memset(h, 0, sizeof(*h));
  h->Instance = s_map[ch].tim;
  h->Init.Period = 0xFFFFU; h->Init.Prescaler = 0; h->Init.CounterMode = TIM_COUNTERMODE_UP;
  h->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(h) != HAL_OK) { return; }
  sc.ClockSource = (s_map[ch].ch == 1U) ? TIM_CLOCKSOURCE_TI1 : TIM_CLOCKSOURCE_TI2;
  sc.ClockPolarity = (s_cfg[ch].edge == HSDI_EDGE_FALLING) ? TIM_CLOCKPOLARITY_FALLING
                   : (s_cfg[ch].edge == HSDI_EDGE_BOTH)    ? TIM_CLOCKPOLARITY_BOTHEDGE
                                                           : TIM_CLOCKPOLARITY_RISING;
  sc.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sc.ClockFilter = s_cfg[ch].filter & 0x0FU;
  if (HAL_TIM_ConfigClockSource(h, &sc) != HAL_OK) { return; }
  __HAL_TIM_SET_COUNTER(h, 0);        /* must match s_last16 = 0, else the first delta jumps */
  (void)HAL_TIM_Base_Start(h);
}

/* ---- capture counter (HSDI_COUNT_CAP): hardware ICF filter + both-edge CC interrupt ----
 * The timer may already be running as a hardware counter / encoder (cnt_start / enc_start);
 * if it is idle it is started free-running on the internal clock, which also makes the
 * capture register a jitter-free fine timestamp for the recorder. With min_us > 0 both edges
 * are captured (the qualifier needs phase lengths); with min_us == 0 the configured edge only. */
static void cap_start(uint8_t ch)
{
  int ti = tim_index(s_map[ch].tim);
  TIM_HandleTypeDef *h = tim_handle(ti);
  TIM_IC_InitTypeDef ic = {0};
  uint32_t chan = (uint32_t)(s_map[ch].ch - 1U) * 4U;        /* TIM_CHANNEL_1..4 = 0,4,8,12 */
  if (ti < 0) { return; }
  if (h->Instance == NULL)                                    /* timer idle: run it on the internal clock */
  {
    memset(h, 0, sizeof(*h));
    h->Instance = s_map[ch].tim;
    h->Init.Period = 0xFFFFU; h->Init.Prescaler = 0; h->Init.CounterMode = TIM_COUNTERMODE_UP;
    h->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(h) != HAL_OK) { return; }
    (void)HAL_TIM_Base_Start(h);
  }
  ic.ICPolarity  = (s_min_cyc[ch] != 0U)                  ? TIM_ICPOLARITY_BOTHEDGE
                 : (s_cfg[ch].edge == HSDI_EDGE_FALLING) ? TIM_ICPOLARITY_FALLING
                 : (s_cfg[ch].edge == HSDI_EDGE_BOTH)    ? TIM_ICPOLARITY_BOTHEDGE : TIM_ICPOLARITY_RISING;
  ic.ICSelection = TIM_ICSELECTION_DIRECTTI; ic.ICPrescaler = TIM_ICPSC_DIV1;
  ic.ICFilter    = s_cfg[ch].filter & 0x0FU;
  if (HAL_TIM_IC_ConfigChannel(h, &ic, chan) != HAL_OK) { return; }
  pin_af(ch);
  (void)HAL_TIM_IC_Start_IT(h, chan);
  {
    IRQn_Type irq = (s_map[ch].tim == TIM1) ? TIM1_CC_IRQn : (s_map[ch].tim == TIM8) ? TIM8_CC_IRQn : TIM3_IRQn;
    HAL_NVIC_SetPriority(irq, 7, 0);
    HAL_NVIC_EnableIRQ(irq);
  }
}
static void cap_isr(TIM_TypeDef *t)
{
  uint32_t sr = t->SR;
  for (uint8_t i = 0; i < PIMG_HSDI_CH; i++)
  {
    if ((s_cfg[i].mode == HSDI_COUNT_CAP) && (s_map[i].tim == t))
    {
      uint32_t f = 1UL << s_map[i].ch;                          /* CC1IF..CC4IF = SR bits 1..4; CCxOF = bits 9..12 */
      if (sr & f)
      {
        uint32_t t_in = DWT->CYCCNT, dsv;
        uint16_t ccr = (uint16_t)(&t->CCR1)[s_map[i].ch - 1U];  /* CCR1..CCR4 are contiguous */
        uint8_t  flg = (sr & (f << 8)) ? REC_F_LOST : 0U;
        t->SR = ~(f | (f << 8));
        edge_in(i, pin_level(i), t_in, ccr, flg);
        if (++s_isr_ms[i] >= STORM_LIMIT) { t->DIER &= ~f; s_muted[i] = 1U; s_storm[i]++; }   /* CCxIE = same bit as CCxIF */
        dsv = DWT->CYCCNT - t_in;
        if (dsv > s_isr_max[i]) { s_isr_max[i] = dsv; }
      }
    }
  }
}
void TIM1_CC_IRQHandler(void) { cap_isr(TIM1); }
void TIM8_CC_IRQHandler(void) { cap_isr(TIM8); }
void TIM3_IRQHandler(void)    { cap_isr(TIM3); }

static void exti_start(uint8_t ch)
{
  uint32_t mode = (s_min_cyc[ch] != 0U)                  ? GPIO_MODE_IT_RISING_FALLING   /* qualifier needs both edges */
                : (s_cfg[ch].edge == HSDI_EDGE_FALLING) ? GPIO_MODE_IT_FALLING
                : (s_cfg[ch].edge == HSDI_EDGE_BOTH)    ? GPIO_MODE_IT_RISING_FALLING
                                                        : GPIO_MODE_IT_RISING;
  pin_in(ch, mode);
  HAL_NVIC_SetPriority(s_map[ch].irq, 7, 0);
  HAL_NVIC_EnableIRQ(s_map[ch].irq);
}

static uint32_t s_hsdi_ver = 0xFFFFFFFFUL;   /* cfg_ver this channel table was applied from */

void hsdi_cm4_reload_check(void)   /* cheap: called by any bus thread that just re-snapshotted */
{
  if (s_hsdi_ver != PIMG->ctrl.cfg_ver) { extern void hsdi_cm4_reload(void); hsdi_cm4_reload(); }
}

void hsdi_cm4_reload(void)
{
  int rc;
  clocks_on();
  s_hsdi_ver = PIMG->ctrl.cfg_ver;
  for (uint8_t i = 0; i < PIMG_HSDI_CH; i++) { s_cfg[i] = PIMG->hsdi[i]; }
  rc = hsdi_validate(s_cfg);
  if (rc != 0)                                   /* illegal combination: fall back to all-DI */
  {
    for (uint8_t i = 0; i < PIMG_HSDI_CH; i++)
    {
      s_cfg[i].mode = (PIMG->hsdi[i].mode == HSDI_OFF) ? HSDI_OFF : HSDI_DI;
    }
  }
  memset(s_acc, 0, sizeof(s_acc)); memset(s_last16, 0, sizeof(s_last16));
  memset((void *)s_sw, 0, sizeof(s_sw)); memset((void *)s_sw_last_ms, 0, sizeof(s_sw_last_ms));
  memset(s_cand, 0, sizeof(s_cand)); memset(s_cand_ms, 0, sizeof(s_cand_ms)); memset(s_level, 0, sizeof(s_level));
  memset(s_zrev, 0, sizeof(s_zrev)); memset(s_zlatch, 0, sizeof(s_zlatch));
  memset(s_cs, 0, sizeof(s_cs)); memset(s_q, 0, sizeof(s_q));
  memset((void *)s_isr_ms, 0, sizeof(s_isr_ms)); memset((void *)s_muted, 0, sizeof(s_muted));
  memset(s_storm, 0, sizeof(s_storm)); memset(s_isr_max, 0, sizeof(s_isr_max));
  s_coinc = 0U; s_rej_ch = 0xFFU;
  s_hw_owner[0] = s_hw_owner[1] = s_hw_owner[2] = 0xFFU;
  s_enc_on[0] = s_enc_on[1] = 0U;
  for (uint8_t i = 0; i < PIMG_HSDI_CH; i++) { s_min_cyc[i] = (uint32_t)s_cfg[i].min_us * s_cyc_us; }

  for (uint8_t i = 0; i < PIMG_HSDI_CH; i++)
  {
    switch (s_cfg[i].mode)
    {
      case HSDI_ENC_A:
      {
        int ti = tim_index(s_map[i].tim);
        uint8_t z = (i == ENC_TIM3_A) ? ENC_TIM3_Z : ENC_TIM1_Z;
        pin_af(i);
        pin_af((i == ENC_TIM3_A) ? ENC_TIM3_B : ENC_TIM1_B);
        enc_start(ti, i, z);
        s_enc_on[(i == ENC_TIM3_A) ? 0U : 1U] = 1U;
        break;
      }
      case HSDI_COUNT_HW:
      {
        int ti = tim_index(s_map[i].tim);
        pin_af(i);
        cnt_start(ti, i);
        s_hw_owner[ti] = i;
        break;
      }
      case HSDI_COUNT_SW:   exti_start(i); break;
      case HSDI_COUNT_POLL: pin_in(i, GPIO_MODE_INPUT); s_praw[i] = 0; break;   /* counted in hsdi_cm4_poll */
      case HSDI_COUNT_CAP:  cap_start(i); break;                                 /* counted in the timer CC ISR */
      case HSDI_ENC_Z:
        /* TIM3 group's Z is armed inside enc_start (hardware capture); TIM1 group's Z uses EXTI */
        if (i == ENC_TIM1_Z) { exti_start(i); }
        break;
      case HSDI_DI:
      default:            pin_in(i, GPIO_MODE_INPUT); break;
    }
    if ((s_cfg[i].mode == HSDI_COUNT_SW) || (s_cfg[i].mode == HSDI_COUNT_CAP))
    {
      /* the qualifier starts from the level the line holds now, taken as already settled */
      uint8_t lv = pin_level(i);
      s_q[i].lvl = lv; s_q[i].qlvl = lv; s_q[i].valid = 1U; s_q[i].last_cyc = DWT->CYCCNT;
    }
  }
  s_loaded = 1U;
}

/* ---- EXTI: software counters + the TIM1-group Z index ---- */
/* ROOT CAUSE of the 2026-09-03 reset epidemic (1318 field resets, caught on SWD): on the dual-core
 * H7 the generic __HAL_GPIO_EXTI_CLEAR_IT()/GET_IT() macros address EXTI->PR1 = the CPU1 (CM7)
 * pending register. Used from the CM4 they clear the wrong core's bit, the CM4's own C2PR1 stays
 * set, the ISR re-enters forever: main loop starves -> IWDG2 (or, before the OpenAMP full-ring
 * check existed, the CM7 hard-faulted on its 4-deep RPMsg ring first). The CM4 must use the
 * __HAL_GPIO_EXTID2_* variants (C2PR1). Rule registered in the CubeMX boundary document. */
static void exti_edge(uint8_t ch)
{
  if (s_cfg[ch].mode == HSDI_COUNT_SW)
  {
    uint32_t t_in = DWT->CYCCNT, dsv;
    edge_in(ch, pin_level(ch), t_in, 0U, REC_F_EXTI);
    if (++s_isr_ms[ch] >= STORM_LIMIT) { EXTI->C2IMR1 &= ~(uint32_t)s_map[ch].pin; s_muted[ch] = 1U; s_storm[ch]++; }
    dsv = DWT->CYCCNT - t_in;
    if (dsv > s_isr_max[ch]) { s_isr_max[ch] = dsv; }
    return;
  }
  if ((ch == ENC_TIM1_Z) && (s_cfg[ch].mode == HSDI_ENC_Z) && s_enc_on[1])   /* never touch a timer enc_start did not initialise */
  {
    if (s_cfg[ch].z_action == HSDI_Z_ZERO) { __HAL_TIM_SET_COUNTER(&s_t1, 0); s_acc[ENC_TIM1_A] = 0; s_last16[ENC_TIM1_A] = 0; }
    else if (s_cfg[ch].z_action == HSDI_Z_LATCH) { s_zlatch[1] = s_acc[ENC_TIM1_A]; s_zrev[1]++; }
  }
}
static void exti_dispatch(uint16_t pin)
{
  for (uint8_t i = 0; i < PIMG_HSDI_CH; i++)
  {
    if ((s_map[i].pin == pin) && (s_cfg[i].mode != HSDI_OFF)) { exti_edge(i); }
  }
}
void EXTI0_IRQHandler(void)     { __HAL_GPIO_EXTID2_CLEAR_IT(GPIO_PIN_0); exti_dispatch(GPIO_PIN_0); }
void EXTI1_IRQHandler(void)     { __HAL_GPIO_EXTID2_CLEAR_IT(GPIO_PIN_1); exti_dispatch(GPIO_PIN_1); }
void EXTI9_5_IRQHandler(void)
{
  uint16_t p[2] = { GPIO_PIN_6, GPIO_PIN_7 };
  for (int k = 0; k < 2; k++)
  { if (__HAL_GPIO_EXTID2_GET_IT(p[k])) { __HAL_GPIO_EXTID2_CLEAR_IT(p[k]); exti_dispatch(p[k]); } }
}
void EXTI15_10_IRQHandler(void)
{
  uint16_t p[2] = { GPIO_PIN_10, GPIO_PIN_11 };
  for (int k = 0; k < 2; k++)
  { if (__HAL_GPIO_EXTID2_GET_IT(p[k])) { __HAL_GPIO_EXTID2_CLEAR_IT(p[k]); exti_dispatch(p[k]); } }
}

/* ---- main-loop tick: extend the 16-bit hardware counters, publish the image ---- */
/* ---- bench stress source (2026-09-03): fire EXTI line <n> from software every <period> ms via
 * EXTI->SWIER1. Exercises the CM4 software-counter interrupt path at up to 1 kHz with no wiring at
 * all — the tool that proved the C2PR1 fix. Off by default; RPC op 0x13 (CM7 `pulse sw <line> <hz>`). */
static uint8_t  s_swt_line = 0xFFU;
static uint16_t s_swt_period = 0, s_swt_ctr = 0;
void hsdi_cm4_swtest(uint8_t line, uint16_t period_ms)
{
  s_swt_line = (line < 16U) ? line : 0xFFU;
  s_swt_period = (s_swt_line == 0xFFU) ? 0U : period_ms;
  s_swt_ctr = 0;
}

void hsdi_cm4_poll(void)
{
  uint8_t levels = 0, status = 0, reset;
  if (!s_loaded) { return; }
  if ((s_swt_period != 0U) && (++s_swt_ctr >= s_swt_period))
  {
    s_swt_ctr = 0;
    EXTI->SWIER1 = (1UL << s_swt_line);          /* software-triggered edge on that EXTI line */
  }
  storm_poll();
  qual_poll();

  /* hardware counters / encoder positions: accumulate the signed 16-bit delta */
  for (int t = 0; t < 3; t++)
  {
    uint8_t ch = 0xFFU;
    if (s_hw_owner[t] != 0xFFU) { ch = s_hw_owner[t]; }
    else if ((t == 0) && s_enc_on[0]) { ch = ENC_TIM3_A; }
    else if ((t == 1) && s_enc_on[1]) { ch = ENC_TIM1_A; }
    if (ch == 0xFFU) { continue; }
    {
      uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(tim_handle(t));
      int16_t  d   = (int16_t)(now - s_last16[ch]);
      s_last16[ch] = now;
      s_acc[ch] = (uint32_t)((int32_t)s_acc[ch] + d);
    }
  }
  /* TIM3-group Z: the capture register holds the encoder count latched at the last index edge */
  if (s_enc_on[0] && (s_cfg[ENC_TIM3_Z].mode == HSDI_ENC_Z))
  {
    if (__HAL_TIM_GET_FLAG(&s_t3, TIM_FLAG_CC3) != RESET)
    {
      __HAL_TIM_CLEAR_FLAG(&s_t3, TIM_FLAG_CC3);
      s_zlatch[0] = (uint32_t)HAL_TIM_ReadCapturedValue(&s_t3, TIM_CHANNEL_3);
      s_zrev[0]++;
      if (s_cfg[ENC_TIM3_Z].z_action == HSDI_Z_ZERO)
      { __HAL_TIM_SET_COUNTER(&s_t3, 0); s_acc[ENC_TIM3_A] = 0; s_last16[ENC_TIM3_A] = 0; }
    }
  }
  /* interrupt / polled counters */
  for (uint8_t i = 0; i < PIMG_HSDI_CH; i++)
  {
    if ((s_cfg[i].mode == HSDI_COUNT_SW) || (s_cfg[i].mode == HSDI_COUNT_POLL) || (s_cfg[i].mode == HSDI_COUNT_CAP)) { s_acc[i] = s_sw[i]; }
  }
  /* levels (valid in every mode) + reset commands from the output image */
  reset = PIMG->out[PIMG_RGN_ONBOARD + PIMG_HSDI_RESET];
  for (uint8_t i = 0; i < PIMG_HSDI_CH; i++)
  {
    /* level with software debounce: a new level must hold for debounce_ms (this runs on the
     * 1 ms tick, so the counter is directly in milliseconds) before it is published */
    uint8_t raw = pin_level(i);
    if ((s_cfg[i].mode == HSDI_COUNT_CAP) || (s_cfg[i].mode == HSDI_COUNT_SW))   /* forensics: 1 kHz shadow sampling */
    {
      if (raw) { s_cs[i].hi_samples++; }
      if (raw && !s_cs[i].praw) { s_cs[i].poll_edges++; }
      s_cs[i].praw = raw;
    }
    if (s_cfg[i].mode == HSDI_COUNT_POLL)          /* polled counter: one sample per 1 ms tick */
    {
      uint8_t e = s_cfg[i].edge;
      if (((e == HSDI_EDGE_RISING) && raw && !s_praw[i]) ||
          ((e == HSDI_EDGE_FALLING) && !raw && s_praw[i]) ||
          ((e == HSDI_EDGE_BOTH) && (raw != s_praw[i])))
      {
        s_sw[i]++;
      }
      s_praw[i] = raw;
    }
    if (s_cfg[i].debounce_ms == 0U) { s_level[i] = raw; }
    else if (raw != s_level[i])
    {
      if (raw == s_cand[i]) { if (++s_cand_ms[i] >= s_cfg[i].debounce_ms) { s_level[i] = raw; } }
      else                  { s_cand[i] = raw; s_cand_ms[i] = 0U; }
    }
    else { s_cand[i] = raw; s_cand_ms[i] = 0U; }
    if (s_level[i]) { levels |= (uint8_t)(1U << i); }
    if (s_cfg[i].mode != HSDI_OFF) { status |= (uint8_t)(1U << i); }
    if (reset & (uint8_t)(1U << i))
    {
      s_acc[i] = 0; s_sw[i] = 0;
      if (s_hw_owner[tim_index(s_map[i].tim)] == i) { __HAL_TIM_SET_COUNTER(tim_handle(tim_index(s_map[i].tim)), 0); s_last16[i] = 0; }
    }
  }
  if (reset != 0U)   /* write-1-clears is a ONE-SHOT: acknowledge by clearing the bits we acted on.
                      * Without this a single reset request re-zeroed the channel every 1 ms tick,
                      * pinning the count at 0 forever. RMW under the output lock (CM7 sets bits
                      * under the same lock, so set and clear serialise). */
  {
    extern void app_cm4_out_lock(void); extern void app_cm4_out_unlock(void);
    app_cm4_out_lock();
    PIMG->out[PIMG_RGN_ONBOARD + PIMG_HSDI_RESET] &= (uint8_t)~reset;
    app_cm4_out_unlock();
  }

  /* publish into the ONBOARD region of the input image */
  {
    uint8_t buf[PIMG_RGN_ONBOARD_SZ];
    memset(buf, 0, sizeof(buf));
    buf[PIMG_HSDI_LEVELS] = levels;
    buf[PIMG_HSDI_STATUS] = status;
    memcpy(&buf[PIMG_HSDI_COUNT], s_acc, sizeof(s_acc));
    memcpy(&buf[PIMG_HSDI_ZREV],  s_zrev,   sizeof(s_zrev));
    memcpy(&buf[PIMG_HSDI_ZLATCH], s_zlatch, sizeof(s_zlatch));
    pimg_seq_write(&PIMG->st.onboard_seq, &PIMG->in[PIMG_RGN_ONBOARD], buf, PIMG_RGN_ONBOARD_SZ);
  }
}
