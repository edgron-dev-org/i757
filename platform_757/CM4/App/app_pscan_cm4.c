/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_pscan_cm4.c — CM4 process-image scanner (data plane).
 *
 * Reads the scan table from the SRAM4 shared block (app_pimage.h), polls each
 * configured Modbus master relationship at its own period (time-base A), and
 * maintains the process image: input entries -> input image (seqlock write),
 * output entries -> flushed to the slave from the output image (seqlock read).
 * Contract: docs/Process_Image_and_IO_Mapping.md.
 *
 * Threading: one thread per bus (app_bus_cm4.c). EACH BUS OWNS A PRIVATE
 * snapshot of its own entries, rebuilt by that thread itself at the top of its
 * loop when cfg_ver changes. That IS the contract's "safe point": a thread only
 * ever swaps tables between its own transactions, and no thread reads another
 * thread's snapshot. (The first cut kept one shared snapshot rebuilt by
 * whichever thread noticed first — every other bus thread could be mid-entry
 * when it was rewritten under them.)
 *
 * The snapshot copy itself is guarded against CM7 rewriting cfg[]: read
 * cfg_ver, copy, re-read cfg_ver; retry if it moved (CM7 bumps cfg_ver only
 * after the table is fully written).
 */
#include <string.h>
#include "stm32h7xx_hal.h"
#include "modbus_core.h"
#include "app_pimage.h"

/* backplane master transaction (app_mbport_cm4.c) */
extern uint16_t mbport_cm4_transact(uint8_t addr, const uint8_t *pdu, uint16_t pn,
                                    uint8_t *rsp_pdu, uint8_t expect_rsp);
/* front-panel master transaction (app_mbfront_cm4.c) */
extern uint16_t mbfront_transact(uint8_t port, uint8_t slave, uint8_t expect,
                                 uint16_t timeout_ms, const uint8_t *pdu, uint16_t pn,
                                 uint8_t *rsp_pdu, uint8_t *status);

#define PSCAN_FRONT_TO_MS   20U   /* front-port scan timeout. A short RTU reply is well under
                                   * this even at 9600 baud, and it bounds how long one dead
                                   * device can hold ITS OWN bus thread per attempt. */
#define PSCAN_OFFLINE_FAILS  3U   /* consecutive failures after which an entry is treated as offline */
#define PSCAN_OFFLINE_MS  2000U   /* slow probe interval for an offline entry (rejoins on success) */
#define PSCAN_RETRY_BP       2U   /* default extra attempts, backplane (bus profile, contract §6) */
#define PSCAN_RETRY_FRONT    1U   /* default extra attempts, front ports */

/* ---- per-bus private snapshot (rebuilt only by the owning thread) ---- */
typedef struct {
  pimg_entry_t ent[PIMG_MAX_ENTRIES];   /* this bus's entries only */
  uint16_t     idx[PIMG_MAX_ENTRIES];   /* their original table index (st.* and seq[] key) */
  uint32_t     next[PIMG_MAX_ENTRIES];  /* next-due tick */
  uint16_t     n;
  uint16_t     base;
  uint32_t     ver;                     /* cfg_ver this snapshot was built from */
  uint8_t      loaded;
} pscan_bus_t;

static pscan_bus_t s_bus[PIMG_NBUS];
static volatile uint32_t s_applied_ver[PIMG_NBUS];   /* for the all-buses-applied ack */

static uint16_t slice_bytes(uint8_t access, uint16_t count)
{
  switch (access)
  {
    case PIMG_IN_COILS: case PIMG_IN_DISCRETE: case PIMG_OUT_COILS:
      return (uint16_t)((count + 7U) / 8U);          /* packed bits */
    default:
      return (uint16_t)(count * 2U);                 /* words */
  }
}

static int entry_valid(const pimg_entry_t *e)
{
  uint16_t nb = slice_bytes(e->access, e->count);
  if (e->period == 0U) { return 0; }
  if (e->seq_idx >= PIMG_MAX_ENTRIES) { return 0; }
  if (e->count == 0U) { return 0; }
  if (e->access <= PIMG_IN_HOLDING)                  /* input access -> input image */
  {
    return ((uint32_t)e->img_off + nb) <= PIMG_IN_BYTES;
  }
  return ((uint32_t)e->img_off + nb) <= PIMG_OUT_BYTES;   /* output access -> output image */
}

/* Rebuild THIS bus's snapshot from the shared table. Retries until the copy was not torn by a
 * concurrent CM7 rewrite (cfg_ver stable across the copy). */
static void bus_snapshot(uint8_t bus)
{
  pscan_bus_t *b = &s_bus[bus];
  uint32_t now = HAL_GetTick();
  __HAL_RCC_HSEM_CLK_ENABLE();                        /* output-image cross-core lock (HSEM 2) */
  for (;;)
  {
    uint32_t v1 = PIMG->ctrl.cfg_ver;
    uint16_t n = PIMG->ctrl.n_entries;
    uint16_t m = 0;
    if (n > PIMG_MAX_ENTRIES) { n = PIMG_MAX_ENTRIES; }
    for (uint16_t i = 0; i < n; i++)
    {
      if (PIMG->cfg[i].port != bus) { continue; }
      b->ent[m] = PIMG->cfg[i];
      b->idx[m] = i;
      b->next[m] = now;
      m++;
    }
    b->base = PIMG->ctrl.base_tick_ms ? PIMG->ctrl.base_tick_ms : 1U;
    if (PIMG->ctrl.cfg_ver == v1) { b->n = m; b->ver = v1; break; }
  }
  b->loaded = 1U;
  /* status bookkeeping for my entries starts clean (CM4 owns the status area — CM7 must not
   * wipe it, that once inverted live seq parity) */
  for (uint16_t k = 0; k < b->n; k++)
  {
    uint16_t i = b->idx[k];
    PIMG->st.ok[i] = 0U; PIMG->st.fails[i] = 0U; PIMG->st.last_exc[i] = 0U;
  }
  s_applied_ver[bus] = b->ver;
  {
    uint8_t all = 1U;
    for (uint8_t j = 0; j < PIMG_NBUS; j++) { if (s_applied_ver[j] != b->ver) { all = 0U; break; } }
    if (all) { PIMG->ctrl.cfg_applied = b->ver; }     /* ack only when every bus has swapped */
  }
}

/* op0C subcmd 0 (reload doorbell) now has nothing left to do besides being answered: each bus
 * thread notices cfg_ver at its own safe point. Kept for contract compatibility. */
void pscan_cm4_reload(void) { }

/* v0.32: the backplane's EXPECTED address set (configured scan-table entries) — demand-driven
 * rescue only chases addresses somebody actually cares about. Called from the busBP thread,
 * which is the same thread that rebuilds this snapshot: no race. */
uint16_t pscan_cm4_expected_backplane(void)
{
  pscan_bus_t *b = &s_bus[PIMG_PORT_BACKPLANE];
  uint16_t mask = 0;
  if (!b->loaded) { return 0; }
  for (uint16_t k = 0; k < b->n; k++)
  {
    uint8_t a = b->ent[k].addr;
    if ((a >= 1U) && (a <= 16U)) { mask |= (uint16_t)(1U << (a - 1U)); }
  }
  return mask;
}

static void mark_ok(uint16_t i, uint8_t bus, const pimg_entry_t *e)
{
  PIMG->st.ok[i] = 1U;
  PIMG->st.fails[i] = 0U;
  PIMG->st.last_exc[i] = 0U;
  if (bus == PIMG_PORT_BACKPLANE && e->addr >= 1U && e->addr <= 16U)
  {
    PIMG->st.online[bus] |= (uint16_t)(1U << (e->addr - 1U));
  }
}

static void mark_fail(uint16_t i, uint8_t bus, const pimg_entry_t *e, uint8_t exc)
{
  if (PIMG->st.fails[i] < 255U) { PIMG->st.fails[i]++; }
  PIMG->st.ok[i] = 0U;
  PIMG->st.last_exc[i] = exc;
  PIMG->st.bfails[bus]++;
  if (bus == PIMG_PORT_BACKPLANE && e->addr >= 1U && e->addr <= 16U && PIMG->st.fails[i] >= PSCAN_OFFLINE_FAILS)
  {
    PIMG->st.online[bus] &= (uint16_t)~(1U << (e->addr - 1U));
  }
  /* Contract §3: unless the entry asks to HOLD, an input slice is zeroed when its device goes
   * offline — stale sensor data must not keep looking alive. Done once, on the transition. */
  if ((PIMG->st.fails[i] == PSCAN_OFFLINE_FAILS) &&
      (e->access <= PIMG_IN_HOLDING) && !(e->flags & PIMG_F_HOLD_FAIL))
  {
    uint8_t z[PIMG_SLICE_MAX];
    uint16_t nb = slice_bytes(e->access, e->count);
    if (nb <= PIMG_SLICE_MAX)
    {
      memset(z, 0, nb);
      pimg_seq_write(&PIMG->st.seq[e->seq_idx], &PIMG->in[e->img_off], z, nb);
    }
  }
}

/* one transaction on the entry's bus; returns response PDU length (0 = no/failed reply) */
static uint16_t bus_transact(const pimg_entry_t *e, const uint8_t *pdu, uint16_t pn, uint8_t *rsp)
{
  if (e->port == PIMG_PORT_BACKPLANE)
  {
    return mbport_cm4_transact(e->addr, pdu, pn, rsp, 1U);
  }
  {
    uint8_t st = 0;
    return mbfront_transact(e->port, e->addr, 1U, PSCAN_FRONT_TO_MS, pdu, pn, rsp, &st);
  }
}

/* transaction with retries (contract §3: entry.retry, 0 = bus default). Retries cover silence
 * and mangled frames only — an exception IS an answer, the device is there and objecting. */
static uint16_t bus_transact_retry(const pimg_entry_t *e, const uint8_t *pdu, uint16_t pn, uint8_t *rsp)
{
  uint8_t extra = e->retry ? e->retry
                : (e->port == PIMG_PORT_BACKPLANE) ? PSCAN_RETRY_BP : PSCAN_RETRY_FRONT;
  uint16_t rl = bus_transact(e, pdu, pn, rsp);
  while ((rl == 0U) && (extra-- > 0U)) { rl = bus_transact(e, pdu, pn, rsp); }
  return rl;
}

static void service_entry(uint8_t bus, uint16_t i, const pimg_entry_t *e)
{
  uint8_t  pdu[MB_PDU_MAX], rsp[MB_PDU_MAX];
  uint16_t nb  = slice_bytes(e->access, e->count);

  PIMG->st.polls[bus]++;

  if (e->access <= PIMG_IN_HOLDING)                   /* ---- input: read slave -> input image ---- */
  {
    uint8_t fc = (e->access == PIMG_IN_COILS)     ? MB_FC_READ_COILS
               : (e->access == PIMG_IN_DISCRETE)  ? MB_FC_READ_DISC
               : (e->access == PIMG_IN_INPUT_REG) ? MB_FC_READ_INPUT : MB_FC_READ_HOLD;
    int pn = mb_req_read(pdu, fc, e->start, e->count);
    if (pn < 0) { mark_fail(i, bus, e, 0); return; }
    uint16_t rl = bus_transact_retry(e, pdu, (uint16_t)pn, rsp);
    uint8_t exc = 0;
    if (rl == 0U) { mark_fail(i, bus, e, 0); return; }
    if (mb_rsp_is_exception(rsp, rl, &exc)) { mark_fail(i, bus, e, exc); return; }

    uint8_t tmp[MB_PDU_MAX];
    if (fc == MB_FC_READ_COILS || fc == MB_FC_READ_DISC)
    {
      if (mb_rsp_bits(rsp, rl, fc, tmp, nb) != (int)nb) { mark_fail(i, bus, e, 0); return; }
    }
    else
    {
      uint16_t regs[MB_MAX_READ_REGS];
      if (mb_rsp_regs(rsp, rl, fc, regs, e->count) != (int)e->count) { mark_fail(i, bus, e, 0); return; }
      if (e->flags & PIMG_F_BYTESWAP)
      {
        for (uint16_t r = 0; r < e->count; r++) { regs[r] = (uint16_t)((regs[r] << 8) | (regs[r] >> 8)); }
      }
      memcpy(tmp, regs, nb);                          /* store native u16 (both cores LE) */
    }
    pimg_seq_write(&PIMG->st.seq[e->seq_idx], &PIMG->in[e->img_off], tmp, nb);
    mark_ok(i, bus, e);
  }
  else                                                /* ---- output: output image -> write slave ---- */
  {
    uint8_t snap[MB_PDU_MAX];
    pimg_seq_read(&PIMG->st.seq[e->seq_idx], &PIMG->out[e->img_off], snap, nb);
    int pn;
    if (e->access == PIMG_OUT_COILS)
    {
      pn = mb_req_write_coils(pdu, e->start, e->count, snap);
    }
    else
    {
      uint16_t vals[MB_MAX_WRITE_REGS];
      memcpy(vals, snap, nb);
      if (e->flags & PIMG_F_BYTESWAP)
      {
        for (uint16_t r = 0; r < e->count; r++) { vals[r] = (uint16_t)((vals[r] << 8) | (vals[r] >> 8)); }
      }
      pn = mb_req_write_regs(pdu, e->start, e->count, vals);
    }
    if (pn < 0) { mark_fail(i, bus, e, 0); return; }
    uint16_t rl = bus_transact_retry(e, pdu, (uint16_t)pn, rsp);
    uint8_t exc = 0;
    if (rl == 0U) { mark_fail(i, bus, e, 0); return; }
    if (mb_rsp_is_exception(rsp, rl, &exc)) { mark_fail(i, bus, e, exc); return; }
    uint8_t wfc = (e->access == PIMG_OUT_COILS) ? MB_FC_WRITE_COILS : MB_FC_WRITE_REGS;
    if (mb_rsp_write_ok(rsp, rl, wfc, e->start, e->count) != 0) { mark_fail(i, bus, e, 0); return; }
    mark_ok(i, bus, e);
  }
}

/* raw microseconds for the per-bus cycle time (TIM7 is the HAL time base, 1 MHz to 1000;
 * same stable-read discipline as app_diag_cm4.c — see the monotonicity note there) */
extern TIM_HandleTypeDef htim7;
static uint32_t pscan_us(void)
{
  uint32_t t1, t2, c;
  do { t1 = HAL_GetTick(); c = (uint32_t)__HAL_TIM_GET_COUNTER(&htim7); t2 = HAL_GetTick(); }
  while (t1 != t2);
  return (t1 * 1000U) + c;
}

/* Contract §4 (v0.26): the MASTER owns event consumption. Round-robin one online backplane
 * module per tick: FC03 read EVENTS (holding 0x0010); if any bits are set, write the same
 * value back (W1C) so the module clears them and releases the attention line. Nothing on the
 * host consumes the bits yet (POWERUP -> config re-push is the future hook, e.g. EX_16DI
 * debounce restore); sweeping keeps the register and the INT line from rotting — 2026-07-24
 * field case: events sat at POWERUP|SAFE forever with the attention line held. */
#define PSCAN_EV_REG        0x0010U
#define PSCAN_EV_SWEEP_MS   2000U
static void backplane_event_sweep(void)
{
  static uint32_t s_next_ms;
  static uint8_t  s_next_addr = 1U;
  uint32_t now = HAL_GetTick();
  uint16_t online = PIMG->st.online[PIMG_PORT_BACKPLANE];
  if (((int32_t)(now - s_next_ms) < 0) || (online == 0U)) { return; }
  s_next_ms = now + PSCAN_EV_SWEEP_MS;
  for (uint8_t hop = 0; hop < 16U; hop++)             /* next online address, round-robin */
  {
    uint8_t addr = s_next_addr;
    s_next_addr = (uint8_t)((s_next_addr % 16U) + 1U);
    if (!(online & (uint16_t)(1U << (addr - 1U)))) { continue; }
    {
      uint8_t pdu[MB_PDU_MAX], rsp[MB_PDU_MAX];
      uint16_t ev[1];
      uint8_t exc;
      int pn = mb_req_read(pdu, MB_FC_READ_HOLD, PSCAN_EV_REG, 1U);
      if (pn < 0) { return; }
      uint16_t rl = mbport_cm4_transact(addr, pdu, (uint16_t)pn, rsp, 1U);
      if ((rl == 0U) || mb_rsp_is_exception(rsp, rl, &exc)) { return; }
      if (mb_rsp_regs(rsp, rl, MB_FC_READ_HOLD, ev, 1U) != 1) { return; }
      if (ev[0] != 0U)
      {
        pn = mb_req_write_regs(pdu, PSCAN_EV_REG, 1U, ev);   /* W1C: clear exactly what we saw */
        if (pn > 0) { (void)mbport_cm4_transact(addr, pdu, (uint16_t)pn, rsp, 1U); }
      }
    }
    return;                                           /* one module per sweep tick */
  }
}

/* Service the entries that are due on ONE bus. Called from that bus's own thread, so a
 * blocking transaction here delays only this bus — never the others. */
void pscan_cm4_service_bus(uint8_t bus)
{
  pscan_bus_t *b;
  uint32_t now, t0;
  uint16_t base, serviced = 0;
  if (bus >= PIMG_NBUS) { return; }
  if (PIMG->ctrl.magic != PIMG_MAGIC) { return; }     /* CM7 has not initialised the block */
  b = &s_bus[bus];
  if ((PIMG->ctrl.cfg_ver != b->ver) || !b->loaded)
  {
    bus_snapshot(bus);                                /* my own safe point: between my transactions */
    { extern void hsdi_cm4_reload_check(void); hsdi_cm4_reload_check(); }
  }
  if (!PIMG->ctrl.running || (b->n == 0U)) { return; }

  now  = HAL_GetTick();
  base = b->base ? b->base : 1U;
  t0   = pscan_us();

  for (uint16_t k = 0; k < b->n; k++)
  {
    uint32_t period_ms;
    const pimg_entry_t *e = &b->ent[k];
    uint16_t i = b->idx[k];
    if (!entry_valid(e)) { continue; }                /* period==0 = entry disabled (contract §3) */
    if ((int32_t)(now - b->next[k]) < 0) { continue; }

    period_ms = (uint32_t)e->period * base;
    /* A device that is not answering costs a full bus timeout per attempt. That only delays its
     * own bus, but still back it off to a slow probe so the healthy entries on THIS bus keep
     * their cadence; it rejoins automatically as soon as it answers. */
    if (PIMG->st.fails[i] >= PSCAN_OFFLINE_FAILS) { period_ms = PSCAN_OFFLINE_MS; }
    else if ((now - b->next[k]) > period_ms) { PIMG->st.overrun[bus]++; }
    b->next[k] = now + period_ms;
    service_entry(bus, i, e);
    serviced++;
    now = HAL_GetTick();                              /* the transaction took real time */
  }
  if (serviced > 0U)
  {
    uint32_t d = pscan_us() - t0;                     /* measured duration of this service pass */
    if (d < 0x80000000UL) { PIMG->st.cycle_us[bus] = d; }   /* raw us can dip ~1 ms (see app_diag_cm4.c) */
  }
  if (bus == PIMG_PORT_BACKPLANE) { backplane_event_sweep(); }   /* contract §4: master consumes events (W1C) */
}
