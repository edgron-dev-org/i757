/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_mbport_cm4.c — 757 backplane-bus master stack @CM4
 * Physical: UART8 (PJ8=TX/PJ9=RX, AF8) + DE=PD11 (GPIO mode, driver asserts before TX / drops on TC), 1Mbps.
 * Contract: docs/Backplane_Bus_Protocol.md v0.21 (reply timeout 5ms / offline after 3 failures / ~1s down-clocked probe / time broadcast FC16).
 * DMA assignment (resource table): DMA1 S0=UART8_RX (circular) S1=UART8_TX.
 * Difference from the nucleo version: only the master port remains (real slaves = modules on the backplane, the simulated slave port is retired); the scheduler switches to
 * HAL_GetTick pacing (the CM4 main loop has a 50ms blocking receive, so ticks can no longer be counted by call count). */
#include <string.h>
#include "stm32h7xx_hal.h"
#include "uart_drv.h"
#include "modbus_core.h"
#include "app_pimage.h"   /* PIMG_PORT_BACKPLANE for the per-bus lock */

extern void bus_cm4_lock(uint8_t bus);
extern void bus_cm4_unlock(uint8_t bus);
#define BUS_BACKPLANE 6U

#define MB_BAUD    1000000UL
#define CM4_STAGE  (*(volatile uint32_t *)0x38008400UL)   /* boot breadcrumb (SRAM4 partition table) */

static uart_drv_t s_m;                          /* backplane master port */
static uint8_t s_m_rx[512], s_m_tx[300];
static uint8_t s_inited = 0;

/* ---- Interrupt vectors: one-line wiring (generated it.c has no S0/S1/UART8 definitions, verified) ---- */
void DMA1_Stream0_IRQHandler(void) { uart_drv_dma_rx_isr(&s_m); }
void DMA1_Stream1_IRQHandler(void) { uart_drv_dma_tx_isr(&s_m); }
void UART8_IRQHandler(void)        { uart_drv_uart_isr(&s_m); }

/* ---- Pins: attach uniformly after uart_open (uart_drv header iron rule) ---- */
static void pins_attach(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Pin = GPIO_PIN_8 | GPIO_PIN_9; g.Alternate = GPIO_AF8_UART8; HAL_GPIO_Init(GPIOJ, &g);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);        /* DE receive state */
  g.Pin = GPIO_PIN_11; g.Mode = GPIO_MODE_OUTPUT_PP; g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &g);
}

/* Baud-rate code table (contract 0x0011 v0.22, extended v0.30; same source/order as slavecore) */
static const uint32_t s_baud_tbl[] = { 1000000U, 250000U, 500000U, 2000000U, 3000000U, 5000000U,
                                       4800U, 9600U, 19200U, 38400U, 57600U, 115200U };
static uint8_t s_baud_cur = 0;

void mbport_cm4_init(void)
{
  if (s_inited) { return; }
  CM4_STAGE = 0x30;
  uart_cfg_t c = {0};
  c.baud       = s_baud_tbl[s_baud_cur];
  c.parity     = UART_PARITY_NONE;
  c.stopbits   = UART_STOPBITS_1;
  c.framing    = UARTDRV_FRAME_IDLE;             /* t3.5 framing = IDLE event (contract profile) */
  c.max_frames = 8;
  c.irq_prio   = 6;
  c.instance = UART8; c.swap = 0;
  c.de_mode  = UARTDRV_DE_GPIO;                  /* PD11 has no native UART8_DE alternate function */
  c.de_port  = GPIOD; c.de_pin = GPIO_PIN_11;
  c.dma_rx = DMA1_Stream0; c.req_rx = DMA_REQUEST_UART8_RX;
  c.dma_tx = DMA1_Stream1; c.req_tx = DMA_REQUEST_UART8_TX;
  c.rxbuf = s_m_rx; c.rxlen = sizeof(s_m_rx);
  c.txbuf = s_m_tx; c.txlen = sizeof(s_m_tx);
  uart_open(&s_m, &c);
  CM4_STAGE = 0x31;
  pins_attach();                                 /* wire pins only after the port is alive (TX drives idle-high) */
  s_inited = 1;
}

/* ---- Send: write + wait for TC (request/reply synchronous semantics; longest frame at 1M ~2.5ms) ---- */
static void port_send(const uint8_t *d, uint16_t n)
{
  if (uart_write(&s_m, d, n) == 0U) { return; }  /* busy-reject: upper layer's timeout is the fallback */
  (void)uart_flush(&s_m, 10);
}

void mbport_cm4_rx_heal(void)                    /* CM4 main loop: restart in thread context after an error */
{
  uart_heal(&s_m);
}

void mbport_cm4_slave_service(void) {}           /* 757 has no simulated slave port; retired, kept as a placeholder */

int mbport_cm4_set_baud(uint8_t code)            /* op=7: hot baud-rate change (uart_set_baud, no reopen) */
{
  extern void bus_cm4_lock(uint8_t bus); extern void bus_cm4_unlock(uint8_t bus);
  int rc = 0;
  if (code >= (sizeof(s_baud_tbl) / sizeof(s_baud_tbl[0]))) { return -1; }
  mbport_cm4_init();
  bus_cm4_lock(PIMG_PORT_BACKPLANE);   /* re-timing the wire mid-transaction mangles the frame */
  if (uart_set_baud(&s_m, s_baud_tbl[code]) != 0) { rc = -1; }
  else { s_baud_cur = code; }
  bus_cm4_unlock(PIMG_PORT_BACKPLANE);
  return rc;
}

void mbport_cm4_raw_send(const uint8_t *d, uint16_t n)   /* op=2: bad-frame injection (test slave fault tolerance) */
{
  extern void bus_cm4_lock(uint8_t bus); extern void bus_cm4_unlock(uint8_t bus);
  mbport_cm4_init();
  bus_cm4_lock(PIMG_PORT_BACKPLANE);   /* contract: EVERY bus-touching leaf takes the bus mutex —
                                        * a raw frame must not interleave a live transaction */
  port_send(d, n);
  bus_cm4_unlock(PIMG_PORT_BACKPLANE);
}

static uint32_t s_last_txn_ms = 0;               /* scheduler backoff: yield during dense transactions (OTA streaming) */
static uint32_t s_last_rx_ms  = 0;               /* last time ANY frame arrived: the silent-bus tripwire's evidence */
static uint32_t s_rx_kicks    = 0;               /* tripwire firings (SWD-readable forensics) */

uint16_t mbport_cm4_transact(uint8_t addr, const uint8_t *pdu, uint16_t pn,
                             uint8_t *rsp_pdu, uint8_t expect_rsp)
{
  static uint8_t adu[MB_ADU_MAX], f[MB_ADU_MAX];
  int n;
  mbport_cm4_init();
  bus_cm4_lock(BUS_BACKPLANE);        /* the backplane thread and RPMsg pass-through share this wire */
  s_last_txn_ms = HAL_GetTick();
  n = mb_adu_build(adu, sizeof(adu), addr, pdu, pn);
  if (n < 0) { bus_cm4_unlock(BUS_BACKPLANE); return 0; }
  while (uart_read_frame(&s_m, f, sizeof(f)) != 0U) {}   /* drain leftover frames (single-frame clear lags by one frame) */
  port_send(adu, (uint16_t)n);
  uint32_t t0 = HAL_GetTick();
  for (;;)
  {
    uint16_t rn = uart_read_frame(&s_m, f, sizeof(f));
    if (rn > 0U)
    {
      uint8_t src;
      s_last_rx_ms = HAL_GetTick();              /* any frame — even a bad one — proves the RX path is alive */
      const uint8_t *rp;
      uint16_t rpn;
      if (mb_adu_check(f, rn, &src, &rp, &rpn) != 0) { bus_cm4_unlock(BUS_BACKPLANE); return 0; }
      if (src != addr) { bus_cm4_unlock(BUS_BACKPLANE); return 0; }
      memcpy(rsp_pdu, rp, rpn);
      bus_cm4_unlock(BUS_BACKPLANE);
      return rpn;
    }
    if ((HAL_GetTick() - t0) > (expect_rsp ? 5U : 3U)) { bus_cm4_unlock(BUS_BACKPLANE); return 0; }   /* contract: 5ms */
  }
}

/* ---- Master scheduler (tick-paced version): poll online devices every 100ms / probe offline ones every 1s ---- */
static uint16_t s_online = 0;
static uint8_t  s_failcnt[17];
static uint32_t s_polls = 0;
static uint16_t s_pollfails = 0;
static uint16_t s_bcasts = 0;

/* Slot registry cache (data source for the board-level slave map, contract `Universal_Modbus_Port_Config.md` §4):
 * type/fw = archived opportunistically from ident; do = real FC01 read-back (low 16 = actual slots, high 16 = reserved bits for 32-terminal modules). */
static uint16_t s_slot_type[17];
static uint16_t s_slot_fw[17];
static uint32_t s_slot_do[17];

uint16_t mbport_cm4_online_map(void)      { return s_online; }
uint16_t mbport_cm4_slot_type(uint8_t s)  { return ((s >= 1U) && (s <= 16U) && ((s_online >> (s - 1U)) & 1U)) ? s_slot_type[s] : 0U; }
uint16_t mbport_cm4_slot_fw(uint8_t s)    { return ((s >= 1U) && (s <= 16U) && ((s_online >> (s - 1U)) & 1U)) ? s_slot_fw[s] : 0U; }
uint32_t mbport_cm4_slot_do_get(uint8_t s){ return ((s >= 1U) && (s <= 16U)) ? s_slot_do[s] : 0U; }

int mbport_cm4_slot_do_write(uint8_t s, uint32_t mask)   /* write-through: FC15 pushes the low 16 channels, high 16 stored locally (reserved bits for 32-terminal modules) */
{
  uint8_t pdu[16], rsp[16];
  uint8_t bits[2] = { (uint8_t)(mask & 0xFFU), (uint8_t)((mask >> 8) & 0xFFU) };
  if ((s < 1U) || (s > 16U) || (((s_online >> (s - 1U)) & 1U) == 0U)) { return -1; }
  int n = mb_req_write_coils(pdu, 0, 16, bits);
  uint16_t rl = mbport_cm4_transact(s, pdu, (uint16_t)n, rsp, 1);
  if ((rl == 0U) || (mb_rsp_write_ok(rsp, rl, MB_FC_WRITE_COILS, 0, 16) != 0)) { return -1; }
  s_slot_do[s] = mask;
  return 0;
}

static uint16_t sched_ident(uint8_t addr)
{
  uint8_t pdu[5], rsp[16];
  uint16_t regs[4];
  pdu[0] = MB_FC_READ_INPUT; mb_put16(&pdu[1], 0); mb_put16(&pdu[3], 4);
  uint16_t rl = mbport_cm4_transact(addr, pdu, 5, rsp, 1);
  if ((rl > 0U) && (mb_rsp_regs(rsp, rl, MB_FC_READ_INPUT, regs, 4) == 4))
  {
    s_slot_type[addr] = regs[1];               /* archive registry opportunistically (0x0001=type, 0x0002=fw) */
    s_slot_fw[addr]   = regs[2];
  }
  return rl;
}

static void sched_do_readback(uint8_t addr)    /* opportunistic FC01 real read-back on the poll tick (don't trust the shadow) */
{
  uint8_t pdu[8], rsp[16], bits[2] = {0};
  int n = mb_req_read(pdu, MB_FC_READ_COILS, 0, 16);
  uint16_t rl = mbport_cm4_transact(addr, pdu, (uint16_t)n, rsp, 1);
  if ((rl > 0U) && (mb_rsp_bits(rsp, rl, MB_FC_READ_COILS, bits, 2) == 2))
  {
    s_slot_do[addr] = (s_slot_do[addr] & 0xFFFF0000UL) | (uint32_t)bits[0] | ((uint32_t)bits[1] << 8);
  }
}

void mbport_cm4_time_push(uint32_t unix_s)       /* op=3: broadcast FC16 hold 0..2 (contract §5) */
{
  uint8_t pdu[16], rsp[8];
  uint16_t t[3] = { (uint16_t)(unix_s >> 16), (uint16_t)(unix_s & 0xFFFFU), 0 };
  int n = mb_req_write_regs(pdu, 0, 3, t);
  if (n > 0) { (void)mbport_cm4_transact(0, pdu, (uint16_t)n, rsp, 0); s_bcasts++; }
}

uint16_t mbport_cm4_uart_stats(uint8_t *out)     /* op=5: keep the 2-port 40B format, port 2 = all zeros */
{
  uart_stats_t st;
  memset(out, 0, 40);
  uart_get_stats(&s_m, &st);
  uint32_t v[5] = { st.rx_bytes, st.tx_bytes, st.frames, st.overrun, st.hw_errs };
  for (int i = 0; i < 5; i++)
  {
    uint8_t *b = &out[i * 4];
    b[0] = (uint8_t)v[i]; b[1] = (uint8_t)(v[i] >> 8);
    b[2] = (uint8_t)(v[i] >> 16); b[3] = (uint8_t)(v[i] >> 24);
  }
  return 40;
}

uint16_t mbport_cm4_status(uint8_t *out)         /* op=4: format identical to nucleo */
{
  out[0] = (uint8_t)(s_online & 0xFFU);
  out[1] = (uint8_t)(s_online >> 8);
  out[2] = (uint8_t)(s_polls & 0xFFU); out[3] = (uint8_t)((s_polls >> 8) & 0xFFU);
  out[4] = (uint8_t)((s_polls >> 16) & 0xFFU); out[5] = (uint8_t)((s_polls >> 24) & 0xFFU);
  out[6] = (uint8_t)(s_pollfails & 0xFFU); out[7] = (uint8_t)(s_pollfails >> 8);
  out[8] = (uint8_t)(s_bcasts & 0xFFU); out[9] = (uint8_t)(s_bcasts >> 8);
  out[10] = 0xFFU;                               /* slave time validity is slave-side info; 757 reports n/a */
  return 11;
}

/* ---- Low-rate rescue (contract v0.31) --------------------------------------------------
 * Slaves power up at 9600 by default (stand-alone transmitters must be reachable by ordinary
 * third-party masters out of the box); on OUR backplane the HOST owns adaptation: broadcast
 * "switch to 1M" (FC06 0x0011=0) at a candidate rate — anyone parked there hops onto the
 * operating rate 20ms later and the normal roll-call/probe adopts it. A full sweep runs once
 * before the power-on roll-call; afterwards one rate every 2s in a 9600-weighted rotation
 * (fresh modules adopt within seconds, exotic rates within ~40s). Switch-send-switch is one
 * critical section under the bus mutex so the process-image scanner never fires mid-rescue.
 * Safety net: a slave that somehow stays unreachable keeps its OTA boot counter unconfirmed —
 * three power cycles swap it back to the previous firmware (which boots at its old rate). */
static void rescue_bcast(uint8_t code)
{
  extern void bus_cm4_lock(uint8_t bus); extern void bus_cm4_unlock(uint8_t bus);
  uint8_t adu[8];
  if ((code == 0U) || (code >= (uint8_t)(sizeof(s_baud_tbl) / sizeof(s_baud_tbl[0])))) { return; }
  mbport_cm4_init();
  bus_cm4_lock(PIMG_PORT_BACKPLANE);
  if (uart_set_baud(&s_m, s_baud_tbl[code]) == 0)
  {
    uint16_t crc;
    adu[0] = 0U;                                 /* broadcast: executed, never answered */
    adu[1] = 0x06U;                              /* FC06 write single register */
    adu[2] = 0x00U; adu[3] = 0x11U;              /* 0x0011 BAUD_SEL */
    adu[4] = 0x00U; adu[5] = 0x00U;              /* code 0 = the 1M operating rate */
    crc = mb_crc16(adu, 6U);
    adu[6] = (uint8_t)(crc & 0xFFU);
    adu[7] = (uint8_t)(crc >> 8);
    if (uart_write(&s_m, adu, 8U) != 0U)
    {
      (void)uart_flush(&s_m, 50);                /* 8B @4800 = 18ms — port_send's 10ms cap is short */
    }
  }
  (void)uart_set_baud(&s_m, s_baud_tbl[0]);      /* back on the operating rate before unlocking */
  bus_cm4_unlock(PIMG_PORT_BACKPLANE);
}

/* v0.32 naturalization (user ruling 2026-08-12): the backplane is a PERMANENT installation —
 * a module is rescued ONCE, then its boot-baud parameter is written to 1M and persisted, so
 * every later power-up is a plain 1M boot (old-style reliability, zero rescue chatter in
 * steady state). Modules without a config area (exception 02) or already at 1M are marked
 * done on sight. NOTE the persist (0xA55A) saves the module's WHOLE current config — a fresh
 * module carries defaults so this is safe; bench discipline: before taking a module OFF the
 * backplane for stand-alone/bench use, set its 0x0281 back (e.g. 7) and persist. */
static uint16_t s_naturalized = 0;
static void naturalize_one(uint8_t a)
{
  uint8_t pdu[8], rsp[16];
  uint16_t reg;
  int n = mb_req_read(pdu, 0x03U, 0x0281U, 1U);              /* FC03 read BAUD_BOOT */
  uint16_t rl = mbport_cm4_transact(a, pdu, (uint16_t)n, rsp, 1U);
  if (rl == 0U) { return; }                                  /* silent: retry next tick */
  if ((rsp[0] & 0x80U) != 0U)                                /* exception (02 = no config area) */
  {
    s_naturalized |= (uint16_t)(1U << (a - 1U));
    return;
  }
  if (mb_rsp_regs(rsp, rl, 0x03U, &reg, 1U) != 1U) { return; }
  if (reg != 0U)
  {
    n = mb_req_write_single(pdu, 0x06U, 0x0281U, 0U);        /* boot baud -> 1M */
    if (mbport_cm4_transact(a, pdu, (uint16_t)n, rsp, 1U) == 0U) { return; }
    if ((rsp[0] & 0x80U) != 0U) { s_naturalized |= (uint16_t)(1U << (a - 1U)); return; }
    n = mb_req_write_single(pdu, 0x06U, 0x0200U, 0xA55AU);   /* persist */
    if (mbport_cm4_transact(a, pdu, (uint16_t)n, rsp, 1U) == 0U) { return; }
  }
  s_naturalized |= (uint16_t)(1U << (a - 1U));
}

void mbport_cm4_scheduler(void)
{
  static uint32_t t_poll = 0, t_probe = 0, t_rescue = 0, t_nat = 0;
  static uint8_t scanned = 0, poll_a = 1, probe_a = 1, rescue_i = 0;
  static uint16_t s_seen = 0;                    /* every address adopted since host boot */
  static const uint8_t rescue_seq[] = { 7, 6, 8, 9, 10, 11, 1, 2, 3, 4, 5 };
  uint32_t now = HAL_GetTick();
  if (s_baud_cur != 0U) { return; }              /* non-default rate = speed-scan experiment in progress, scheduler stays silent (offline detection would false-trip) */
  /* The old "skip if a transaction happened in the last 200 ms" guard is gone: it dated from
   * the single-threaded loop, and once the process-image scanner started polling this bus
   * every ~33 ms the condition was ALWAYS true, so roll-call and offline probing silently
   * never ran again (measured: polls stuck at 0). Mutual exclusion is now the bus mutex
   * inside mbport_cm4_transact, so housekeeping can simply take its turn. */
  mbport_cm4_init();
  if (!scanned)                                  /* power-on roll-call of the whole network 1..16 */
  {
    /* v0.31: collect strays first — after a host reboot every slave has fallen back to its
     * boot rate (default 9600), the boot sweep herds them all onto 1M before the roll-call */
    static const uint8_t all_rates[] = { 7, 6, 8, 9, 10, 11, 1, 2, 3, 4, 5 };
    for (uint8_t r = 0; r < (uint8_t)sizeof(all_rates); r++) { rescue_bcast(all_rates[r]); }
    for (uint8_t a = 1; a <= 16U; a++)
    {
      if (sched_ident(a) > 0U) { s_online |= (uint16_t)(1U << (a - 1U)); }
    }
    scanned = 1;
    t_poll = t_probe = t_rescue = t_nat = now;
    s_last_rx_ms = now;                          /* tripwire baseline: silence is measured from here */
    return;
  }
  /* Silent-bus tripwire (2026-08-22 field case: TX healthy, RX stone dead, no error flag —
   * every layer below believed it was fine and the farm went blind for 2.5 h until an on-site
   * reset). If we have EVER adopted a module yet heard NOTHING for 10 s — while poll (100ms)
   * and probe (1s) keep transmitting — force the port through uart_heal's full re-init rung.
   * A genuinely empty backplane never arms it (s_seen stays 0), so steady state costs zero. */
  if ((s_seen != 0U) && ((now - s_last_rx_ms) >= 10000U))
  {
    s_last_rx_ms = now;                          /* rate-limit: one kick per silent window */
    s_rx_kicks++;
    uart_rx_kick(&s_m);
  }
  if ((now - t_poll) >= 100U)                    /* 100ms: poll the next online slave */
  {
    t_poll = now;
    for (uint8_t k = 0; k < 16U; k++)
    {
      uint8_t a = (uint8_t)(((poll_a - 1U + k) % 16U) + 1U);
      if ((s_online & (1U << (a - 1U))) != 0U)
      {
        s_polls++;
        if (sched_ident(a) == 0U)
        {
          s_pollfails++;
          if (++s_failcnt[a] >= 3U)
          {
            s_online &= (uint16_t)~(1U << (a - 1U));
            s_naturalized &= (uint16_t)~(1U << (a - 1U));   /* re-check on re-adoption (cheap) */
            s_failcnt[a] = 0;
          }
        }
        else { s_failcnt[a] = 0; sched_do_readback(a); }   /* on an online tick, opportunistically read back DO (board-level map cache ≤~1.6s stale) */
        poll_a = (uint8_t)((a % 16U) + 1U);
        break;
      }
    }
  }
  s_seen |= s_online;
  {
    extern uint16_t pscan_cm4_expected_backplane(void);
    uint16_t expected = (uint16_t)(pscan_cm4_expected_backplane() | s_seen);
    uint16_t missing  = (uint16_t)(expected & ~s_online);
    if ((now - t_probe) >= 1000U)                /* 1s: probe ONE offline address — an expected-
                                                  * missing one first (a rebooted resident re-adopts
                                                  * in ~1-3s instead of waiting out the full
                                                  * empty-slot rotation), else rotate the rest */
    {
      t_probe = now;
      uint16_t pool = missing ? missing : (uint16_t)(~s_online & 0xFFFFU);
      for (uint8_t k = 0; k < 16U; k++)
      {
        uint8_t a = (uint8_t)(((probe_a - 1U + k) % 16U) + 1U);
        if ((pool & (1U << (a - 1U))) != 0U)
        {
          if (sched_ident(a) > 0U) { s_online |= (uint16_t)(1U << (a - 1U)); }
          probe_a = (uint8_t)((a % 16U) + 1U);
          break;
        }
      }
    }
    /* v0.32 demand-driven rescue (replaces the v0.31 blind 2s rotation, user ruling: the
     * backplane is a permanent installation — steady state has ZERO rescue traffic):
     * an EXPECTED module gone missing pulls a rescue every 5s (rotating through all rates,
     * 9600 first). v0.33 (2026-08-22 user ruling, RX-wedge postmortem): the 30s steady-state
     * discovery broadcast is GONE — it hot-switched the wire's baud 2880x/day while the RX
     * DMA ran, for a scenario that cannot occur: hot-plugging is forbidden (power down the
     * rack first, and the reboot's boot sweep adopts any fresh 9600 module), and a mis-parked
     * EXPECTED module is the missing-driven rescue's job. All-online = a silent wire. */
    if ((missing != 0U) && ((now - t_rescue) >= 5000U))
    {
      t_rescue = now;
      rescue_bcast(rescue_seq[rescue_i]);
      rescue_i = (uint8_t)((rescue_i + 1U) % (uint8_t)sizeof(rescue_seq));
    }
  }
  if ((now - t_nat) >= 1000U)                    /* v0.32 naturalization: one candidate per second */
  {
    t_nat = now;
    uint16_t todo = (uint16_t)(s_online & ~s_naturalized);
    for (uint8_t a = 1; a <= 16U; a++)
    {
      if ((todo & (1U << (a - 1U))) != 0U) { naturalize_one(a); break; }
    }
  }
}
