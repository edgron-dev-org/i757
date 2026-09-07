/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_pimage.h — process-image shared layout (CM7 <-> CM4, SRAM4).
 *
 * SINGLE SOURCE OF TRUTH for the dual-core process image. Both cores include
 * this header and cast PIMG_BASE to pimg_shared_t; neither hardcodes offsets.
 * Contract: docs/Process_Image_and_IO_Mapping.md.
 *
 * Data plane = this shared block in SRAM4 (0x38008440, MPU non-cacheable
 * Region1, reset-persistent). CM4 owns the scan engine; CM7 owns config + the
 * external interfaces. RPMsg (op0C) is only the reload/start/stop doorbell —
 * I/O data never travels over RPMsg.
 *
 * Consistency = seqlock (see §5 of the contract): one seq counter per entry,
 * single writer per image (CM4 writes inputs, CM7 writes outputs). Use the
 * pimg_seq_write / pimg_seq_read helpers below; they carry the barriers.
 */
#ifndef APP_PIMAGE_H
#define APP_PIMAGE_H
#include <stdint.h>

/* ---- placement (docs/Hardware_Resource_Allocation.md section 3: SRAM4 0x38008440 / <=24K) ---- */
#define PIMG_BASE        0x38008440UL
#define PIMG_MAGIC       0x50494D47UL   /* 'PIMG' */

/* ---- fixed capacities (bump here + re-check the 24K budget) ---- */
#define PIMG_MAX_ENTRIES 64U
#define PIMG_IN_BYTES    2048U
#define PIMG_OUT_BYTES   2048U
#define PIMG_NBUS        7U             /* 0..5 = front 485A..F, 6 = backplane */

/* ---- port codes (same enumeration as app_platform.h APP_PORT_* / RPMsg op0A) ---- */
#define PIMG_PORT_BACKPLANE 6U

/* ---- process-image REGION MAP (byte offsets, identical in the input and output image) ----
 * THESE OFFSETS ARE A PUBLISHED CONTRACT. Once an integrator wires an HMI/SCADA to the
 * Modbus window below, the addresses must never shift — so platform I/O that does not exist
 * yet (onboard HSDI, sub-card slots) gets its space reserved NOW, and customer scan-table
 * points are allocated only from PIMG_RGN_SCAN upward.
 * Reserved regions read as 0 until their BSP lands; that is intentional placeholding. */
#define PIMG_RGN_ONBOARD     0U    /* onboard I/O: 8x HSDI (levels, counters, encoder positions) */
#define PIMG_RGN_ONBOARD_SZ  64U
#define PIMG_RGN_SLOTC      64U    /* left sub-card slot C: DI/AI (in) or DO/AO (out) */
#define PIMG_RGN_SLOTC_SZ   64U
#define PIMG_RGN_SLOTD     128U    /* left sub-card slot D */
#define PIMG_RGN_SLOTD_SZ   64U
#define PIMG_RGN_GP        192U    /* general purpose / future platform use */
#define PIMG_RGN_GP_SZ      64U
#define PIMG_RGN_SCAN      256U    /* customer scan-table points are allocated from here up */

/* ---- onboard HSDI layout inside PIMG_RGN_ONBOARD (contract: HSDI_Configuration_and_Counting.md) ----
 * Every channel gets a fixed 32-bit slot regardless of how it is counting (hardware counter,
 * software EXTI counter, or encoder position) — uniform for an external HMI. */
#define PIMG_HSDI_LEVELS     0U    /* +0  : 1 byte, 8 channel level bits (valid in every mode) */
#define PIMG_HSDI_STATUS     1U    /* +1  : 1 byte, per-channel "configured" bits */
#define PIMG_HSDI_COUNT      4U    /* +4  : 8 x u32 counter / encoder position (signed for encoders) */
#define PIMG_HSDI_ZREV      36U    /* +36 : 2 x u32 encoder index (Z) revolution counters */
#define PIMG_HSDI_ZLATCH    44U    /* +44 : 2 x u32 encoder position latched at the last Z edge */
/* output image, same region: */
#define PIMG_HSDI_RESET      0U    /* +0  : 1 byte, write 1 = zero that channel's counter/position */

/* ---- external Modbus window (board slave register map; 485 slave ports AND Modbus TCP) ----
 * The same image bytes, viewed as Modbus registers/bits. Modbus has four independent address
 * spaces, so input and output images can share one numeric base without colliding:
 *   input  image -> FC04 input registers  / FC02 discrete inputs   (read-only outside)
 *   output image -> FC03/06/16 holding    / FC01/05/15 coils       (read + write outside)
 * register n = image bytes [2n, 2n+1];  bit n = bit n of the image byte array. */
#define PIMG_WIN_BASE      0x2000U

/* ---- access type: direction + Modbus function + data granularity ---- */
enum {
  PIMG_IN_COILS     = 0,  /* FC01, bit,  -> input image  */
  PIMG_IN_DISCRETE  = 1,  /* FC02, bit,  -> input image  */
  PIMG_IN_INPUT_REG = 2,  /* FC04, word, -> input image  */
  PIMG_IN_HOLDING   = 3,  /* FC03, word, -> input image  */
  PIMG_OUT_COILS    = 4,  /* FC15, bit,  -> output image */
  PIMG_OUT_HOLDING  = 5   /* FC16, word, -> output image */
};

/* ---- entry flags ---- */
#define PIMG_F_BYTESWAP   0x01U   /* word device is byte-swapped vs Modbus big-endian */
#define PIMG_F_HOLD_FAIL  0x02U   /* keep last value on comm failure (default: the input slice is ZEROED when the entry crosses the offline threshold) */

/* ---- one scan-table entry (16 bytes; keep POD, no bitfields) ---- */
typedef struct {
  uint8_t  port;      /* 0..5 front, 6 backplane */
  uint8_t  addr;      /* Modbus slave address */
  uint8_t  access;    /* one of the PIMG_IN_/PIMG_OUT access codes */
  uint8_t  flags;     /* PIMG_F_* */
  uint16_t start;     /* first coil / register address */
  uint16_t count;     /* qty (bits for coil/discrete, words for registers) */
  uint16_t img_off;   /* byte offset into in[] (input access) or out[] (output access) */
  uint16_t period;    /* desired poll interval in base ticks; 0 = disabled */
  uint16_t seq_idx;   /* index into st.seq[] guarding this entry's image slice */
  uint8_t  retry;     /* per-entry retry count; 0 = bus default */
  uint8_t  _pad;
} pimg_entry_t;

/* ---- control block (CM7 writes, CM4 reads) ---- */
typedef struct {
  uint32_t magic;         /* PIMG_MAGIC once CM7 has initialised the block */
  uint32_t cfg_ver;       /* CM7 bumps after writing a new cfg[] */
  uint32_t cfg_applied;   /* CM4 echoes cfg_ver once it has rebuilt its schedule */
  uint32_t m7_beat;       /* CM7 liveness counter (CM4 may watch, never clears outputs) */
  uint16_t n_entries;     /* number of valid cfg[] entries */
  uint16_t base_tick_ms;  /* scan base tick (default 1) */
  uint16_t running;       /* scanner enable (0 = stopped) */
  uint16_t _pad;
  uint32_t _rsv[6];
} pimg_ctrl_t;

/* ---- status / health (CM4 writes, CM7 reads) ---- */
typedef struct {
  uint32_t seq[PIMG_MAX_ENTRIES];   /* per-entry seqlock counters */
  uint8_t  ok[PIMG_MAX_ENTRIES];    /* last transaction succeeded */
  uint8_t  fails[PIMG_MAX_ENTRIES]; /* consecutive failures (saturating) */
  uint8_t  last_exc[PIMG_MAX_ENTRIES]; /* last Modbus exception code (0 = none) */
  uint8_t  _p[PIMG_MAX_ENTRIES];
  uint16_t online[PIMG_NBUS];       /* per-bus online bitmap (backplane: addr 1..16 -> bit0..15) */
  uint32_t polls[PIMG_NBUS];
  uint32_t bfails[PIMG_NBUS];
  uint32_t overrun[PIMG_NBUS];      /* scan cycle could not fit -> over-subscribed bus */
  uint32_t cycle_us[PIMG_NBUS];     /* measured last full scan cycle (per bus) */
  uint32_t onboard_seq;             /* seqlock for the ONBOARD region (no scan entry owns it) */
} pimg_status_t;

/* ---- onboard HSDI per-channel configuration (CM7 writes, CM4 reads on reload) ---- */
enum {
  HSDI_OFF = 0,      /* channel unused */
  HSDI_DI,           /* plain digital input: level only */
  HSDI_COUNT_SW,     /* software edge counter (EXTI) — any channel, <=10 kHz */
  HSDI_COUNT_HW,     /* hardware timer counter — HSDI0/1 (TIM3), 4/5 (TIM1), 6 (TIM8) only */
  HSDI_ENC_A,        /* quadrature A phase — HSDI0 (TIM3) or HSDI4 (TIM1) */
  HSDI_ENC_B,        /* quadrature B phase — HSDI1 (TIM3) or HSDI5 (TIM1) */
  HSDI_ENC_Z,        /* index — HSDI2 (TIM3 group, capture) or HSDI3 (TIM1 group, EXTI) */
  HSDI_COUNT_POLL,   /* polled edge counter: CM4 samples the pin on its 1 ms tick, no EXTI — any
                      * channel, <= ~200 Hz (2026-09-03: EXTI counters on HSDI5/7 tripped an
                      * unexplained CM7 software reset within seconds of the first pulse; the
                      * greenhouse flow meters run at <= 100 Hz, polling covers them) */
  HSDI_COUNT_CAP     /* timer input-capture counter (2026-09-03; any channel since 2026-09-04: every
                      * HSDI pin is a timer capture channel). The timer's digital input filter (ICF)
                      * removes ns ringing, then BOTH edges raise a CC interrupt that timestamps
                      * the phase; `min_us` decides which phases are real. No polling, no EXTI
                      * line (so no EXTI0/1 sharing conflict); coexists with a HW counter or an
                      * encoder running on the same timer. Preferred mode for field pulse sensors. */
};
#define HSDI_EDGE_RISING  0U
#define HSDI_EDGE_FALLING 1U
#define HSDI_EDGE_BOTH    2U
#define HSDI_Z_NONE       0U
#define HSDI_Z_ZERO       1U   /* Z edge zeroes the position */
#define HSDI_Z_LATCH      2U   /* Z edge latches the position and bumps the revolution counter */

typedef struct {
  uint8_t mode;        /* HSDI_* above */
  uint8_t filter;      /* timer digital input filter 0..15 (hardware, counting/encoder modes) */
  uint8_t edge;        /* HSDI_EDGE_* (counting modes) */
  uint8_t z_action;    /* HSDI_Z_* (HSDI_ENC_Z only) */
  uint8_t debounce_ms; /* software debounce for DI and software-counter modes; 0 = off.
                        * A mechanical contact needs 5..20 ms here; leave it 0 for an
                        * electronic signal, and never use it above ~50 Hz input. */
  uint8_t _rsv;
  uint16_t min_us;     /* pulse qualifier for COUNT_CAP / COUNT_SW (2026-09-04): a level must hold for
                        * at least this many microseconds before it counts as a real phase; shorter
                        * phases (dips inside a pulse, spikes inside a gap) are discarded. 0 = off,
                        * every edge counts. Edge timestamps come from the capture/EXTI interrupt,
                        * so the figure is exact and independent of task load. Field hall flow
                        * meters (15 ms pulses with microsecond dips) want ~1000 here. */
} pimg_hsdi_cfg_t;

#define PIMG_HSDI_CH  8U

/* ---- HSDI combination checker: ONE source, compiled on both cores ----
 * CM7 runs it in app_hsdi_setup so an impossible table is rejected with a real error code the
 * moment the customer writes it (the API contract promises <0). CM4 runs the very same rules
 * again defensively before touching a timer. The first cut only validated on CM4, silently
 * fell back to all-DI, and returned 0 from setup — a rejected table looked like success.
 * Rules = docs/HSDI_Configuration_and_Counting.md ("four hard constraints" + pairing). */
#define PIMG_ENC_TIM3_A 0U   /* encoder group 0 = TIM3: A=HSDI0 B=HSDI1 Z=HSDI2 (capture) */
#define PIMG_ENC_TIM3_B 1U
#define PIMG_ENC_TIM3_Z 2U
#define PIMG_ENC_TIM1_A 4U   /* encoder group 1 = TIM1: A=HSDI4 B=HSDI5 Z=HSDI3 (EXTI) */
#define PIMG_ENC_TIM1_B 5U
#define PIMG_ENC_TIM1_Z 3U

static inline int pimg_hsdi_check(const pimg_hsdi_cfg_t c[PIMG_HSDI_CH])
{
  /* board facts (same source as the CM4 pin map): pin number = EXTI line, timer group index */
  static const uint8_t pin[PIMG_HSDI_CH] = { 6U, 7U, 0U, 1U, 1U, 11U, 10U, 0U };
  static const uint8_t tim[PIMG_HSDI_CH] = { 0U, 0U, 0U, 0U, 1U, 1U,  2U,  2U };  /* 0=TIM3 1=TIM1 2=TIM8 */
  uint8_t exti_used[16] = { 0 };
  for (uint8_t i = 0; i < PIMG_HSDI_CH; i++)
  {
    uint8_t m = c[i].mode;
    if (m == HSDI_COUNT_HW)                        /* only TI1/TI2 channels can clock a timer */
    {
      if ((i != 0U) && (i != 1U) && (i != 4U) && (i != 5U) && (i != 6U)) { return -(int)(10 + i); }
    }
    if (m == HSDI_ENC_A) { if ((i != PIMG_ENC_TIM3_A) && (i != PIMG_ENC_TIM1_A)) { return -(int)(20 + i); } }
    if (m == HSDI_ENC_B) { if ((i != PIMG_ENC_TIM3_B) && (i != PIMG_ENC_TIM1_B)) { return -(int)(30 + i); } }
    if (m == HSDI_ENC_Z) { if ((i != PIMG_ENC_TIM3_Z) && (i != PIMG_ENC_TIM1_Z)) { return -(int)(40 + i); } }
    if ((m == HSDI_COUNT_SW) || ((m == HSDI_ENC_Z) && (i == PIMG_ENC_TIM1_Z)))
    {
      if (exti_used[pin[i]]) { return -(int)(50 + i); }                 /* EXTI line clash */
      exti_used[pin[i]] = 1U;
    }
  }
  for (uint8_t t = 0; t < 3U; t++)                 /* a timer: one encoder OR one HW counter */
  {
    uint8_t enc = 0, cnt = 0;
    for (uint8_t i = 0; i < PIMG_HSDI_CH; i++)
    {
      if (tim[i] != t) { continue; }
      if ((c[i].mode == HSDI_ENC_A) || (c[i].mode == HSDI_ENC_B)) { enc = 1U; }
      if (c[i].mode == HSDI_COUNT_HW) { cnt++; }
    }
    if (enc && cnt) { return -(int)(60 + t); }
    if (cnt > 1U)   { return -(int)(70 + t); }
  }
  {
    /* an encoder is a PAIR, and Z belongs to a live pair — a lone phase counts against a
     * floating input, and an orphan armed Z fires into an uninitialised timer handle */
    uint8_t a3 = (uint8_t)(c[PIMG_ENC_TIM3_A].mode == HSDI_ENC_A), b3 = (uint8_t)(c[PIMG_ENC_TIM3_B].mode == HSDI_ENC_B);
    uint8_t a1 = (uint8_t)(c[PIMG_ENC_TIM1_A].mode == HSDI_ENC_A), b1 = (uint8_t)(c[PIMG_ENC_TIM1_B].mode == HSDI_ENC_B);
    if (a3 != b3) { return -80; }
    if (a1 != b1) { return -81; }
    if ((c[PIMG_ENC_TIM3_Z].mode == HSDI_ENC_Z) && !a3) { return -82; }
    if ((c[PIMG_ENC_TIM1_Z].mode == HSDI_ENC_Z) && !a1) { return -83; }
  }
  return 0;
}

/* ---- the whole shared block ---- */
typedef struct {
  pimg_ctrl_t  ctrl;
  pimg_hsdi_cfg_t hsdi[PIMG_HSDI_CH];
  pimg_entry_t cfg[PIMG_MAX_ENTRIES];
  uint8_t      in[PIMG_IN_BYTES];    /* CM4 writes, CM7 reads */
  uint8_t      out[PIMG_OUT_BYTES];  /* CM7 writes, CM4 reads */
  pimg_status_t st;
} pimg_shared_t;

#define PIMG  ((volatile pimg_shared_t *)PIMG_BASE)

/* ---- memory barrier (self-contained; no CMSIS dependency) ---- */
#ifndef PIMG_BARRIER
#define PIMG_BARRIER() __asm volatile ("dmb 0xf" ::: "memory")
#endif

/* ---- cross-core lock for the OUTPUT image (hardware semaphore) ----
 * The output image has two writers once the external Modbus window is open: the CM7
 * application (app_io_do_set/hr_set) and CM4 answering an external master. Two writers
 * would corrupt both the data and the seqlock counter, so every WRITER takes this HSEM.
 * Readers still use the plain seqlock — with writers mutually excluded, exactly one
 * writer is ever active, which is all seqlock requires.
 * The input image keeps a single writer (the CM4 scanner) and needs no lock.
 * HSEM 0/1 belong to OpenAMP; HSEM 2 is ours (docs/Hardware_Resource_Allocation.md). Raw registers keep
 * this header free of HAL. */
#define PIMG_HSEM_BASE 0x58026400UL
#define PIMG_HSEM_ID   2U
#if defined(CORE_CM4)
#define PIMG_HSEM_COREID 1U     /* CPU2 = Cortex-M4 */
#else
#define PIMG_HSEM_COREID 3U     /* CPU1 = Cortex-M7 */
#endif

static inline void pimg_out_lock(void)
{
  volatile uint32_t *rlr = (volatile uint32_t *)(PIMG_HSEM_BASE + 0x80UL + 4UL * PIMG_HSEM_ID);
  const uint32_t mine = 0x80000000UL | ((uint32_t)PIMG_HSEM_COREID << 8);
  while (*rlr != mine) { }        /* 1-step read lock: spin until this core owns it */
  PIMG_BARRIER();
}

static inline void pimg_out_unlock(void)
{
  volatile uint32_t *r = (volatile uint32_t *)(PIMG_HSEM_BASE + 4UL * PIMG_HSEM_ID);
  PIMG_BARRIER();
  *r = ((uint32_t)PIMG_HSEM_COREID << 8);   /* LOCK=0 with matching COREID = release */
}

/* ---- seqlock write side (the single writer of this slice) ----
 * Parity self-heal (2026-07-26, board-2 reset-loop autopsy): SRAM4 survives warm resets,
 * so a reset landing inside a write leaves its seq ODD forever — every writer afterwards
 * inverts parity (even while writing) and readers spin for good; nobody re-inits, because
 * a stale-odd seq belongs to no live owner (the onboard region has no scan entry, so the
 * "CM4 clears its bookkeeping on table swap" rule never reaches it). An odd seq at write
 * ENTRY can only be that stale corpse (writers are HSEM-serialized), so the writer
 * restores even parity before opening — the lock heals on the first write after any
 * mid-write reset. */
static inline void pimg_seq_write(volatile uint32_t *seq,
                                  volatile uint8_t *dst, const uint8_t *src, uint32_t n)
{
  uint32_t s = *seq;
  if (s & 1U) { s++; }              /* stale odd from a writer killed mid-write: heal first */
  *seq = s + 1U;                    /* odd = write in progress */
  PIMG_BARRIER();
  for (uint32_t i = 0; i < n; i++) { dst[i] = src[i]; }
  PIMG_BARRIER();
  *seq = s + 2U;                    /* even = write complete */
}

/* ---- seqlock read side (lock-free, retries on collision) ----
 * Bounded spin (same autopsy): an unbounded reader starves the IWDG feed when the seq is
 * stale-odd or the writer core is dead mid-write — the board then reset-loops every 32s,
 * which is strictly worse than one torn/stale sample of telemetry. On exhaustion we copy
 * anyway and return: the writer's parity self-heal repairs the lock on its next write,
 * and a dead CM4 is caught by the RPC heartbeat, not by starving the watchdog. */
#define PIMG_SEQ_SPIN_MAX 100000U   /* ~ms-scale worst case; a real write is microseconds */
static inline void pimg_seq_read(volatile uint32_t *seq,
                                 const volatile uint8_t *src, uint8_t *dst, uint32_t n)
{
  for (uint32_t spins = 0; spins < PIMG_SEQ_SPIN_MAX; spins++)
  {
    uint32_t s1 = *seq;
    if (s1 & 1U) { continue; }      /* writer mid-update: spin until even */
    PIMG_BARRIER();
    for (uint32_t i = 0; i < n; i++) { dst[i] = src[i]; }
    PIMG_BARRIER();
    if (*seq == s1) { return; }     /* seq unchanged across the copy = consistent snapshot */
  }
  PIMG_BARRIER();                   /* give up: torn/stale copy beats an IWDG corpse */
  for (uint32_t i = 0; i < n; i++) { dst[i] = src[i]; }
}

/* ---- how many image bytes one entry occupies (single definition for all three users) ---- */
#define PIMG_SLICE_MAX 256U    /* largest single-entry slice any side buffers on the stack */

static inline uint16_t pimg_slice_bytes(uint8_t access, uint16_t count)
{
  if ((access == PIMG_IN_COILS) || (access == PIMG_IN_DISCRETE) || (access == PIMG_OUT_COILS))
  {
    return (uint16_t)((count + 7U) / 8U);      /* packed bits */
  }
  return (uint16_t)(count * 2U);               /* words */
}

/* ---- packed-bit access helpers (LSB = first coil, Modbus wire order) ---- */
static inline uint8_t pimg_bit_get(const uint8_t *buf, uint16_t ch)
{
  return (uint8_t)((buf[ch >> 3] >> (ch & 7U)) & 1U);
}
static inline void pimg_bit_set(uint8_t *buf, uint16_t ch, uint8_t on)
{
  uint8_t m = (uint8_t)(1U << (ch & 7U));
  if (on) { buf[ch >> 3] |= m; } else { buf[ch >> 3] = (uint8_t)(buf[ch >> 3] & ~m); }
}

#endif /* APP_PIMAGE_H */
