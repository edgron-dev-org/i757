/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_mbfront_cm4.c — front-panel 6×485 configurable Modbus ports + board-level slave register map
 * Contract: docs/Universal_Modbus_Port_Config.md (port pool/role/board-level map) + docs/Inter_Core_RPMsg_Protocol.md v1.3 (op8~0B).
 * Port pool: 485A=USART1(DE=PJ15 GPIO) 485B=USART2(DE=PD4 hardware)
 *   485C=USART3(DE=PI1 GPIO) 485D=UART4(DE=PA15 hardware) 485E=USART6(DE=PA8 GPIO)
 *   485F=UART7(DE=PH15 GPIO). DMA streams follow the generated-layer allocation (it.c/usart.c same source, resource table §6).
 * Generated-layer neutralization: the six MspInit branches in usart.c return in their user regions; it.c vectors early-exit from their user regions and wire to this file
 *   (registered in CubeMX_Code_Boundary_Rules.md). uart_drv is fully self-managed (clock/DMA/NVIC/pin discipline, see its header).
 * Threading model: one thread per port (app_bus_cm4.c) services slave replies + heal; master
 * transactions come from the scanner/rpc threads and serialise on the per-bus mutex inside
 * mbfront_transact. */
#include <string.h>
#include "stm32h7xx_hal.h"
#include "uart_drv.h"
#include "modbus_core.h"
#include "cmsis_os.h"
#include "app_pimage.h"   /* process-image window exposed on this board's slave map */
/* Two-layer output lock (CM4 mutex + HSEM): several CM4 threads write the output image now
 * (this slave path and the user application), and the HSEM alone does not exclude them. */
extern void app_cm4_out_lock(void);
extern void app_cm4_out_unlock(void);

/* process-image window helpers (defined further down, used by the register-map readers above them) */
static int pimg_ir(uint16_t a, uint16_t *v);
static int pimg_disc(uint16_t a, uint8_t *bit);
static int pimg_hr_rd(uint16_t a, uint16_t *v);
static int pimg_coil_rd(uint16_t a, uint8_t *bit);
static int pimg_hr_wr(uint16_t a, uint16_t v);
static int pimg_coil_wr(uint16_t a, uint8_t on);

/* ---- Board-level slave map identity (contract §4.1; version numbers registered in Board_Type_and_Version_Registry.md) ---- */
#define MBF_MAP_VER    1U
#define MBF_BOARD_TYPE 0x0757U
#define MBF_FW_VER     0x0100U
#define MBF_HW_VER     0x0100U

/* ---- Backplane slot data source (app_mbport_cm4.c slot registry cache) ---- */
extern uint16_t mbport_cm4_online_map(void);
extern uint16_t mbport_cm4_slot_type(uint8_t s);
extern uint16_t mbport_cm4_slot_fw(uint8_t s);
extern uint32_t mbport_cm4_slot_do_get(uint8_t s);
extern int      mbport_cm4_slot_do_write(uint8_t s, uint32_t mask);

/* ---- Port-pool hardware table (same source as generated layer; for hardware-DE ports the DE pin is in the AF group too) ---- */
#define MBF_NPORT 6U
typedef struct {
  USART_TypeDef *inst;
  uartdrv_dma_t *dma_rx, *dma_tx;
  uint32_t       req_rx, req_tx;
  uint8_t        de_hw;
  GPIO_TypeDef  *de_port;
  uint16_t       de_pin;
} mbf_hw_t;

static const mbf_hw_t s_hw[MBF_NPORT] = {
  { USART1, DMA2_Stream4, DMA2_Stream5, DMA_REQUEST_USART1_RX, DMA_REQUEST_USART1_TX, 0, GPIOJ, GPIO_PIN_15 },
  { USART2, DMA1_Stream2, DMA1_Stream3, DMA_REQUEST_USART2_RX, DMA_REQUEST_USART2_TX, 1, GPIOD, GPIO_PIN_4  },
  { USART3, DMA2_Stream2, DMA2_Stream3, DMA_REQUEST_USART3_RX, DMA_REQUEST_USART3_TX, 0, GPIOI, GPIO_PIN_1  },
  { UART4,  DMA2_Stream0, DMA2_Stream1, DMA_REQUEST_UART4_RX,  DMA_REQUEST_UART4_TX,  1, GPIOA, GPIO_PIN_15 },
  { USART6, DMA1_Stream6, DMA1_Stream7, DMA_REQUEST_USART6_RX, DMA_REQUEST_USART6_TX, 0, GPIOA, GPIO_PIN_8  },
  { UART7,  DMA1_Stream4, DMA1_Stream5, DMA_REQUEST_UART7_RX,  DMA_REQUEST_UART7_TX,  0, GPIOH, GPIO_PIN_15 },
};

#define MBF_ROLE_OFF    0U
#define MBF_ROLE_SLAVE  1U
#define MBF_ROLE_MASTER 2U

typedef struct {
  uart_drv_t drv;
  uint8_t    rx[512], tx[300];
  uint8_t    opened, role, addr;
} mbf_port_t;

static mbf_port_t s_p[MBF_NPORT];
/* Per-port frame buffers: each port now runs in its own thread, so shared statics would race. */
static uint8_t s_fbuf[MBF_NPORT][MB_ADU_MAX];
static uint8_t s_abuf[MBF_NPORT][MB_ADU_MAX];
static uint8_t s_rbuf[MBF_NPORT][MB_PDU_MAX];
extern void bus_cm4_lock(uint8_t bus);
extern void bus_cm4_unlock(uint8_t bus);

/* ---- Interrupt wiring (it.c user regions early-exit into these; one group per port; unsigned params = safe across-file extern declaration) ---- */
void mbf_isr_uart(unsigned i)   { uart_drv_uart_isr(&s_p[i].drv); }
void mbf_isr_dma_rx(unsigned i) { uart_drv_dma_rx_isr(&s_p[i].drv); }
void mbf_isr_dma_tx(unsigned i) { uart_drv_dma_tx_isr(&s_p[i].drv); }

/* ---- Pin attach (after uart_open; AF values same source as generated usart.c) ---- */
static void pins_attach(uint8_t i)
{
  GPIO_InitTypeDef g = {0};
  g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_PULLUP; g.Speed = GPIO_SPEED_FREQ_HIGH;
  switch (i)
  {
    case 0:   /* 485A: PB14/PB15 AF4; DE=PJ15 GPIO */
      __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOJ_CLK_ENABLE();
      g.Pin = GPIO_PIN_14 | GPIO_PIN_15; g.Alternate = GPIO_AF4_USART1; HAL_GPIO_Init(GPIOB, &g);
      break;
    case 1:   /* 485B: PD5/PD6 AF7 + DE=PD4 AF7 (hardware DE) */
      __HAL_RCC_GPIOD_CLK_ENABLE();
      g.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_4; g.Alternate = GPIO_AF7_USART2; HAL_GPIO_Init(GPIOD, &g);
      break;
    case 2:   /* 485C: PB10/PB11 AF7; DE=PI1 GPIO */
      __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOI_CLK_ENABLE();
      g.Pin = GPIO_PIN_10 | GPIO_PIN_11; g.Alternate = GPIO_AF7_USART3; HAL_GPIO_Init(GPIOB, &g);
      break;
    case 3:   /* 485D: PA0 TX + PI9 RX AF8 + DE=PA15 AF8 (hardware DE) */
      __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOI_CLK_ENABLE();
      g.Pin = GPIO_PIN_0 | GPIO_PIN_15; g.Alternate = GPIO_AF8_UART4; HAL_GPIO_Init(GPIOA, &g);
      g.Pin = GPIO_PIN_9; HAL_GPIO_Init(GPIOI, &g);
      break;
    case 4:   /* 485E: PG14/PG9 AF7; DE=PA8 GPIO */
      __HAL_RCC_GPIOG_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE();
      g.Pin = GPIO_PIN_14 | GPIO_PIN_9; g.Alternate = GPIO_AF7_USART6; HAL_GPIO_Init(GPIOG, &g);
      break;
    default:  /* 485F: PB4/PB3 AF11; DE=PH15 GPIO */
      __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOH_CLK_ENABLE();
      g.Pin = GPIO_PIN_4 | GPIO_PIN_3; g.Alternate = GPIO_AF11_UART7; HAL_GPIO_Init(GPIOB, &g);
      break;
  }
  if (!s_hw[i].de_hw)   /* GPIO-DE: start in receive state (uart_open already drove it low, here we set direction) */
  {
    GPIO_InitTypeDef d = {0};
    HAL_GPIO_WritePin(s_hw[i].de_port, s_hw[i].de_pin, GPIO_PIN_RESET);
    d.Pin = s_hw[i].de_pin; d.Mode = GPIO_MODE_OUTPUT_PP; d.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(s_hw[i].de_port, &d);
  }
}

/* ---- Board-level slave map (contract §4; unified for all ports, TCP answers from the same source via op9) ---- */
static uint32_t s_time_unix = 0, s_time_tick = 0;   /* op3-fed baseline + tick extrapolation */
static int16_t  s_temp = 0;
static uint16_t s_flags = 0;

void mbfront_board_feed(uint32_t unix_s, int16_t temp, uint16_t flags)   /* op3 extended-payload feed */
{
  if (unix_s != 0U) { s_time_unix = unix_s; s_time_tick = HAL_GetTick(); }
  s_temp = temp; s_flags = flags;
}

static int ir_read(uint16_t a, uint16_t *v)   /* input register: 0=success -1=illegal address */
{
  if (a <= 0x0008U)
  {
    uint32_t up = HAL_GetTick() / 1000U;
    switch (a)
    {
      case 0x0000: *v = MBF_MAP_VER;    break;
      case 0x0001: *v = MBF_BOARD_TYPE; break;
      case 0x0002: *v = MBF_FW_VER;     break;
      case 0x0003: *v = 0x0003U;        break;   /* CAPS: bit0=slots bit1=time */
      case 0x0004: *v = (uint16_t)(up >> 16);  break;
      case 0x0005: *v = (uint16_t)(up & 0xFFFFU); break;
      case 0x0006: *v = (uint16_t)s_temp; break;
      case 0x0007: *v = s_flags;        break;
      default:     *v = MBF_HW_VER;     break;
    }
    return 0;
  }
  if ((a >= 0x0011U) && (a <= 0x0020U)) { *v = mbport_cm4_slot_type((uint8_t)(a - 0x0010U)); return 0; }
  if ((a >= 0x0021U) && (a <= 0x0030U)) { *v = mbport_cm4_slot_fw((uint8_t)(a - 0x0020U));   return 0; }
  if ((a >= 0x0100U) && (a <= 0x01FFU)) { *v = 0U; return 0; }   /* slot analog window: reserved in v1, not populated */
  return pimg_ir(a, v);                                          /* 0x2000+: process-image input window */
}

static void time_now_regs(uint16_t *t)   /* HR 0..2: fed baseline + tick extrapolation */
{
  uint32_t d = HAL_GetTick() - s_time_tick;
  uint32_t cur = (s_time_unix != 0U) ? (s_time_unix + d / 1000U) : 0U;
  t[0] = (uint16_t)(cur >> 16);
  t[1] = (uint16_t)(cur & 0xFFFFU);
  t[2] = (uint16_t)(d % 1000U);
}

/* ================= process-image window (contract: Process_Image_and_IO_Mapping.md) =================
 * The same SRAM4 image the CM4 scanner maintains, exposed on this board's slave register map so
 * an external master (any 485 port set to slave, or Modbus TCP — both land here) can read the
 * concentrated field data and command the outputs. Zero copy: we answer straight out of the image.
 *   FC04 / FC02  <- input image  (read-only outside)
 *   FC03/06/16 + FC01/05/15 <-> output image (read + write outside)
 * Writers of the output image (this path and CM7's app_io_*) are mutually excluded by the HSEM;
 * the owning entry's seqlock counter is bumped so the scanner never reads a torn slice. */

/* Every window entry point is gated on the image magic: before CM7 has built (or while it is
 * rebuilding) the shared block, ctrl/cfg are reset-persistent garbage, and serving a request off
 * garbage cfg[] once let an external write compute an unbounded slice. No magic -> exception 02. */
static int pimg_ready(void) { return (PIMG->ctrl.magic == PIMG_MAGIC) ? 1 : 0; }

/* Reads are zero-copy single ALIGNED 16-bit loads — atomic on Cortex-M, so one register can
 * never tear even against a concurrent pimg_seq_write. Multi-register values (32-bit counters)
 * follow standard Modbus concentrator practice: the master re-reads until two passes agree
 * (contract §12.2). The seqlock protects the in-core API readers, not this path. */
static int pimg_ir(uint16_t a, uint16_t *v)            /* FC04 <- input image word */
{
  uint32_t k;
  if ((a < PIMG_WIN_BASE) || (pimg_ready() == 0)) { return -1; }
  k = (uint32_t)(a - PIMG_WIN_BASE);
  if (k >= (PIMG_IN_BYTES / 2U)) { return -1; }
  *v = *(volatile const uint16_t *)&PIMG->in[k * 2U];   /* image is native LE */
  return 0;
}

static int pimg_disc(uint16_t a, uint8_t *bit)         /* FC02 <- input image bit */
{
  uint32_t k;
  if ((a < PIMG_WIN_BASE) || (pimg_ready() == 0)) { return -1; }
  k = (uint32_t)(a - PIMG_WIN_BASE);
  if (k >= ((uint32_t)PIMG_IN_BYTES * 8U)) { return -1; }
  *bit = (uint8_t)((PIMG->in[k >> 3] >> (k & 7U)) & 1U);
  return 0;
}

static int pimg_hr_rd(uint16_t a, uint16_t *v)         /* FC03 <- output image word (read-back) */
{
  uint32_t k;
  if ((a < PIMG_WIN_BASE) || (pimg_ready() == 0)) { return -1; }
  k = (uint32_t)(a - PIMG_WIN_BASE);
  if (k >= (PIMG_OUT_BYTES / 2U)) { return -1; }
  *v = *(volatile const uint16_t *)&PIMG->out[k * 2U];  /* aligned 16-bit load: atomic */
  return 0;
}

static int pimg_coil_rd(uint16_t a, uint8_t *bit)      /* FC01 <- output image bit (read-back) */
{
  uint32_t k;
  if ((a < PIMG_WIN_BASE) || (pimg_ready() == 0)) { return -1; }
  k = (uint32_t)(a - PIMG_WIN_BASE);
  if (k >= ((uint32_t)PIMG_OUT_BYTES * 8U)) { return -1; }
  *bit = (uint8_t)((PIMG->out[k >> 3] >> (k & 7U)) & 1U);
  return 0;
}

/* which scan entry owns this output-image byte (-1 = reserved/unallocated: nobody seqlock-watches it) */
static int out_entry_of(uint16_t off)
{
  uint16_t n = PIMG->ctrl.n_entries;
  if (n > PIMG_MAX_ENTRIES) { n = PIMG_MAX_ENTRIES; }
  for (uint16_t i = 0; i < n; i++)
  {
    volatile pimg_entry_t *e = &PIMG->cfg[i];
    if (e->access <= PIMG_IN_HOLDING) { continue; }          /* input entry */
    uint16_t nb = pimg_slice_bytes(e->access, e->count);
    if ((off >= e->img_off) && (off < (uint16_t)(e->img_off + nb))) { return (int)i; }
  }
  return -1;
}

/* Read-modify-write on the output image. Call with the HSEM held. The committed region is the
 * owning entry's whole slice (so its seqlock stays coherent), or just the touched bytes. */
static int out_rmw_begin(uint16_t off, uint16_t span, uint8_t *tmp, uint16_t *base, uint16_t *nb)
{
  int ei = out_entry_of(off);
  if (ei >= 0)
  {
    volatile pimg_entry_t *e = &PIMG->cfg[ei];
    *base = e->img_off;
    *nb   = pimg_slice_bytes(e->access, e->count);
    /* cfg[] is shared RAM: never let a torn or garbage entry size the copy. CM7 validates on
     * setup, but this side is the one an external master reaches — clamp locally too. */
    if ((*nb > PIMG_SLICE_MAX) || ((uint32_t)*base + *nb > PIMG_OUT_BYTES))
    {
      *base = off; *nb = span; ei = -1;
    }
  }
  else { *base = off; *nb = span; }
  for (uint16_t i = 0; i < *nb; i++) { tmp[i] = PIMG->out[*base + i]; }
  return ei;
}

static void out_rmw_commit(int ei, uint16_t base, const uint8_t *tmp, uint16_t nb)
{
  if (ei >= 0) { pimg_seq_write(&PIMG->st.seq[PIMG->cfg[ei].seq_idx], &PIMG->out[base], tmp, nb); }
  else { for (uint16_t i = 0; i < nb; i++) { PIMG->out[base + i] = tmp[i]; } }
}

static int pimg_hr_wr(uint16_t a, uint16_t v)          /* FC06/16 -> output image word */
{
  uint8_t tmp[PIMG_SLICE_MAX];
  uint16_t base, nb, off;
  uint32_t k;
  int ei;
  if (a < PIMG_WIN_BASE) { return -1; }
  k = (uint32_t)(a - PIMG_WIN_BASE);
  if (k >= (PIMG_OUT_BYTES / 2U)) { return -1; }
  off = (uint16_t)(k * 2U);
  app_cm4_out_lock();
  ei = out_rmw_begin(off, 2U, tmp, &base, &nb);
  tmp[off - base]      = (uint8_t)(v & 0xFFU);
  tmp[off - base + 1U] = (uint8_t)(v >> 8);
  out_rmw_commit(ei, base, tmp, nb);
  app_cm4_out_unlock();
  return 0;
}

static int pimg_coil_wr(uint16_t a, uint8_t on)        /* FC05/15 -> output image bit */
{
  uint8_t tmp[PIMG_SLICE_MAX];
  uint16_t base, nb, off;
  uint32_t k;
  int ei;
  if (a < PIMG_WIN_BASE) { return -1; }
  k = (uint32_t)(a - PIMG_WIN_BASE);
  if (k >= ((uint32_t)PIMG_OUT_BYTES * 8U)) { return -1; }
  off = (uint16_t)(k >> 3);
  app_cm4_out_lock();
  ei = out_rmw_begin(off, 1U, tmp, &base, &nb);
  {
    uint8_t m = (uint8_t)(1U << (k & 7U));
    uint16_t d = (uint16_t)(off - base);
    if (on) { tmp[d] |= m; } else { tmp[d] = (uint8_t)(tmp[d] & ~m); }
  }
  out_rmw_commit(ei, base, tmp, nb);
  app_cm4_out_unlock();
  return 0;
}

static int hr_read(uint16_t a, uint16_t *v)
{
  if (a <= 0x0002U)
  {
    uint16_t t[3];
    time_now_regs(t);
    *v = t[a];
    return 0;
  }
  if ((a >= 0x0100U) && (a <= 0x017FU))
  {
    uint8_t s = (uint8_t)((a - 0x0100U) / 8U + 1U);
    uint8_t off = (uint8_t)((a - 0x0100U) % 8U);
    uint32_t m = mbport_cm4_slot_do_get(s);
    *v = (off == 0U) ? (uint16_t)(m & 0xFFFFU) : (off == 1U) ? (uint16_t)(m >> 16) : 0U;
    return 0;
  }
  return pimg_hr_rd(a, v);   /* 0x2000+: process-image output window (read-back); 0x3000 config area still reserved */
}

static int hr_write(uint16_t a, uint16_t v)   /* 0=success -1=illegal address -2=device failure */
{
  if ((a >= 0x0100U) && (a <= 0x017FU))
  {
    uint8_t s = (uint8_t)((a - 0x0100U) / 8U + 1U);
    uint8_t off = (uint8_t)((a - 0x0100U) % 8U);
    uint32_t m = mbport_cm4_slot_do_get(s);
    if (off == 0U) { m = (m & 0xFFFF0000UL) | v; }
    else if (off == 1U) { m = (m & 0x0000FFFFUL) | ((uint32_t)v << 16); }
    else { return -1; }
    return (mbport_cm4_slot_do_write(s, m) == 0) ? 0 : -2;
  }
  return pimg_hr_wr(a, v);   /* 0x2000+: external master commands the outputs; time area stays read-only */
}

#define MBF_COIL_MAX 512U   /* 32-bit window per slot × 16 slots (contract §4.3) */

static uint8_t coil_read(uint16_t a)
{
  return (uint8_t)((mbport_cm4_slot_do_get((uint8_t)(a / 32U + 1U)) >> (a % 32U)) & 1U);
}

static int coil_write_span(uint16_t start, uint16_t qty, const uint8_t *bits, uint8_t single_val)
{
  /* aggregate bit changes per slot; each touched slot is written with a single FC15 (when bits=NULL this is FC05 single-point single_val) */
  uint8_t s0 = (uint8_t)(start / 32U + 1U), s1 = (uint8_t)((start + qty - 1U) / 32U + 1U);
  for (uint8_t s = s0; s <= s1; s++)
  {
    uint32_t m = mbport_cm4_slot_do_get(s), m0 = m;
    uint16_t base = (uint16_t)((s - 1U) * 32U);
    for (uint16_t k = 0; k < qty; k++)
    {
      uint16_t a = (uint16_t)(start + k);
      if ((a / 32U + 1U) != s) { continue; }
      uint8_t bit = (bits != (const uint8_t *)0) ? (uint8_t)((bits[k / 8U] >> (k % 8U)) & 1U) : single_val;
      uint32_t msk = 1UL << (a - base);
      m = bit ? (m | msk) : (m & ~msk);
    }
    if ((m != m0) && (mbport_cm4_slot_do_write(s, m) != 0)) { return -2; }
  }
  return 0;
}

/* PDU in -> reply PDU out (an exception counts as a reply too; returns reply length, always >0) */
static uint16_t board_handle(const uint8_t *pdu, uint16_t n, uint8_t *rsp)
{
  uint8_t fc = pdu[0];
  if ((fc == MB_FC_READ_COILS) || (fc == MB_FC_READ_DISC))
  {
    if (n != 5U) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_VAL); }
    uint16_t a = mb_get16(&pdu[1]), q = mb_get16(&pdu[3]);
    if ((q == 0U) || (q > MB_MAX_READ_BITS)) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_VAL); }
    uint8_t bits[MB_MAX_READ_BITS / 8U];
    memset(bits, 0, sizeof(bits));
    for (uint16_t k = 0; k < q; k++)
    {
      uint16_t aa = (uint16_t)(a + k);
      uint8_t b = 0;
      if (fc == MB_FC_READ_COILS)
      {
        if (aa < MBF_COIL_MAX) { b = coil_read(aa); }                    /* legacy slot-DO window */
        else if (pimg_coil_rd(aa, &b) != 0) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_ADDR); }
      }
      else                                                               /* FC02 discrete inputs */
      {
        if (aa >= MBF_COIL_MAX)                                          /* legacy area reserved, reads 0 */
        {
          if (pimg_disc(aa, &b) != 0) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_ADDR); }
        }
      }
      if (b) { bits[k / 8U] |= (uint8_t)(1U << (k % 8U)); }
    }
    return (uint16_t)mb_slv_rsp_bits(rsp, fc, bits, q);
  }
  if ((fc == MB_FC_READ_HOLD) || (fc == MB_FC_READ_INPUT))
  {
    if (n != 5U) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_VAL); }
    uint16_t a = mb_get16(&pdu[1]), q = mb_get16(&pdu[3]);
    uint16_t vals[MB_MAX_READ_REGS];
    if ((q == 0U) || (q > MB_MAX_READ_REGS)) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_VAL); }
    for (uint16_t k = 0; k < q; k++)
    {
      int rc = (fc == MB_FC_READ_HOLD) ? hr_read((uint16_t)(a + k), &vals[k])
                                       : ir_read((uint16_t)(a + k), &vals[k]);
      if (rc != 0) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_ADDR); }
    }
    return (uint16_t)mb_slv_rsp_regs(rsp, fc, vals, q);
  }
  if (fc == MB_FC_WRITE_COIL)
  {
    if (n != 5U) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_VAL); }
    uint16_t a = mb_get16(&pdu[1]), v = mb_get16(&pdu[3]);
    if ((v != 0x0000U) && (v != 0xFF00U)) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_VAL); }
    if (a < MBF_COIL_MAX)                                                /* legacy slot-DO window */
    {
      if (coil_write_span(a, 1, (const uint8_t *)0, (v == 0xFF00U) ? 1U : 0U) != 0)
      {
        return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_DEVICE_FAIL);
      }
    }
    else if (pimg_coil_wr(a, (uint8_t)((v == 0xFF00U) ? 1U : 0U)) != 0)  /* process-image output window */
    {
      return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_ADDR);
    }
    memcpy(rsp, pdu, 5);   /* normal reply = echo of the request */
    return 5;
  }
  if (fc == MB_FC_WRITE_REG)
  {
    if (n != 5U) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_VAL); }
    uint16_t a = mb_get16(&pdu[1]), v = mb_get16(&pdu[3]);
    int rc = hr_write(a, v);
    if (rc == -1) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_ADDR); }
    if (rc != 0)  { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_DEVICE_FAIL); }
    memcpy(rsp, pdu, 5);
    return 5;
  }
  if (fc == MB_FC_WRITE_COILS)
  {
    if (n < 6U) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_VAL); }
    uint16_t a = mb_get16(&pdu[1]), q = mb_get16(&pdu[3]);
    uint8_t bytes = pdu[5];
    if ((q == 0U) || (q > MB_MAX_WRITE_BITS) || (bytes != (uint8_t)((q + 7U) / 8U)) ||
        (n != (uint16_t)(6U + bytes))) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_VAL); }
    if (((uint32_t)a + q) <= MBF_COIL_MAX)                               /* legacy slot-DO window */
    {
      if (coil_write_span(a, q, &pdu[6], 0) != 0) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_DEVICE_FAIL); }
    }
    else if (a >= MBF_COIL_MAX)                                          /* process-image output window */
    {
      for (uint16_t k = 0; k < q; k++)   /* validate the whole span first — same rule as FC16, so an
                                          * invalid span never leaves earlier coils half-committed */
      {
        uint8_t probe;
        if (pimg_coil_rd((uint16_t)(a + k), &probe) != 0) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_ADDR); }
      }
      for (uint16_t k = 0; k < q; k++)
      {
        uint8_t bit = (uint8_t)((pdu[6U + (k / 8U)] >> (k % 8U)) & 1U);
        if (pimg_coil_wr((uint16_t)(a + k), bit) != 0) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_ADDR); }
      }
    }
    else { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_ADDR); }   /* span straddles both windows */
    rsp[0] = fc; mb_put16(&rsp[1], a); mb_put16(&rsp[3], q);
    return 5;
  }
  if (fc == MB_FC_WRITE_REGS)
  {
    if (n < 6U) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_VAL); }
    uint16_t a = mb_get16(&pdu[1]), q = mb_get16(&pdu[3]);
    uint8_t bytes = pdu[5];
    if ((q == 0U) || (q > MB_MAX_WRITE_REGS) || (bytes != (uint8_t)(q * 2U)) ||
        (n != (uint16_t)(6U + bytes))) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_VAL); }
    for (uint16_t k = 0; k < q; k++)   /* validate the whole span is "writable" before writing (avoids half-written spans; the read-only time area in v1 does not qualify) */
    {
      uint16_t aa = (uint16_t)(a + k);
      uint16_t probe;
      if ((aa >= 0x0100U) && (aa <= 0x017FU)) { continue; }              /* slot-DO window */
      if (pimg_hr_rd(aa, &probe) == 0) { continue; }                     /* process-image output window */
      return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_ADDR);
    }
    for (uint16_t k = 0; k < q; k++)
    {
      int rc = hr_write((uint16_t)(a + k), mb_get16(&pdu[6U + k * 2U]));
      if (rc == -1) { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_ADDR); }
      if (rc != 0)  { return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_DEVICE_FAIL); }
    }
    rsp[0] = fc; mb_put16(&rsp[1], a); mb_put16(&rsp[3], q);
    return 5;
  }
  return (uint16_t)mb_slv_exception(rsp, fc, MB_EXC_ILLEGAL_FC);
}

uint16_t mbfront_slave_pdu(const uint8_t *pdu, uint16_t n, uint8_t *rsp)   /* op9: TCP shell adjudicates against the same map */
{
  if (n == 0U) { return 0; }
  return board_handle(pdu, n, rsp);
}

/* ---- Port configuration (op8) ---- */
static const uint32_t s_par_tbl[3]  = { UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD };

int mbfront_cfg_set(uint8_t port, uint8_t role, uint32_t baud,
                    uint8_t parity, uint8_t stop, uint8_t addr)
{
  if ((port >= MBF_NPORT) || (role > MBF_ROLE_MASTER) || (parity > 2U) ||
      ((stop != 1U) && (stop != 2U)) ||
      (baud < 1200U) || (baud > 1000000U) ||
      ((role == MBF_ROLE_SLAVE) && ((addr < 1U) || (addr > 247U)))) { return 1; }
  mbf_port_t *p = &s_p[port];
  uint32_t par = s_par_tbl[parity];
  uint32_t stp = (stop == 2U) ? UART_STOPBITS_2 : UART_STOPBITS_1;
  if (!p->opened)
  {
    uart_cfg_t c = {0};
    c.instance = s_hw[port].inst;
    c.baud = baud; c.parity = par; c.stopbits = stp;
    c.framing = UARTDRV_FRAME_IDLE; c.max_frames = 8; c.irq_prio = 6;
    c.de_mode = s_hw[port].de_hw ? UARTDRV_DE_HW : UARTDRV_DE_GPIO;
    c.de_port = s_hw[port].de_port; c.de_pin = s_hw[port].de_pin;
    c.dma_rx = s_hw[port].dma_rx; c.req_rx = s_hw[port].req_rx;
    c.dma_tx = s_hw[port].dma_tx; c.req_tx = s_hw[port].req_tx;
    c.rxbuf = p->rx; c.rxlen = sizeof(p->rx);
    c.txbuf = p->tx; c.txlen = sizeof(p->tx);
    if (uart_open(&p->drv, &c) != 0) { return 2; }
    pins_attach(port);
    p->opened = 1;
  }
  else
  {
    int rc;
    bus_cm4_lock(port);            /* live re-line while this port's bus thread may be mid-frame */
    rc = uart_set_line(&p->drv, baud, par, stp);
    bus_cm4_unlock(port);
    if (rc != 0) { return 2; }
  }
  p->role = role;
  p->addr = addr;
  return 0;
}

/* ---- Send (TX duration computed from baud rate; 256B at 9600 ≈ 300ms) ---- */
static void port_send(mbf_port_t *p, const uint8_t *d, uint16_t n)
{
  uint32_t ms = ((uint32_t)n * 11000U) / p->drv.cfg.baud + 5U;
  if (uart_write(&p->drv, d, n) == 0U) { return; }
  (void)uart_flush(&p->drv, ms);
}


/* ---- Master transaction (op0A): one request/one reply on the specified port's bus ---- */
uint16_t mbfront_transact(uint8_t port, uint8_t slave, uint8_t expect,
                          uint16_t timeout_ms, const uint8_t *pdu, uint16_t pn,
                          uint8_t *rsp_pdu, uint8_t *status)
{
  mbf_port_t *p;
  uint8_t *adu, *f;
  uint32_t t0;
  uint16_t rej_n = 0, ret = 0;
  int n;
  if ((port >= MBF_NPORT) || (!s_p[port].opened) || (s_p[port].role != MBF_ROLE_MASTER))
  {
    *status = 2;
    return 0;
  }
  p = &s_p[port];
  adu = s_abuf[port]; f = s_fbuf[port];

  /* One owner of the wire at a time: this bus's thread and any aperiodic RPMsg pass-through
   * both come through here. The slave role of this same port is served under the same mutex,
   * so the old "pump the slave service inside the master's wait loop" workaround is gone —
   * with a thread per bus, a port answering another port on a shared wire just works. */
  bus_cm4_lock(port);
  n = mb_adu_build(adu, MB_ADU_MAX, slave, pdu, pn);
  if (n < 0) { *status = 2; bus_cm4_unlock(port); return 0; }
  while (uart_read_frame(&p->drv, f, MB_ADU_MAX) != 0U) {}   /* drain leftover frames (iron rule) */
  port_send(p, adu, (uint16_t)n);
  if (expect == 0U) { *status = 0; bus_cm4_unlock(port); return 0; }   /* broadcast: no reply */
  if (timeout_ms == 0U) { timeout_ms = 100U; }
  if (timeout_ms > 1000U) { timeout_ms = 1000U; }
  t0 = HAL_GetTick();
  *status = 1;
  for (;;)
  {
    uint16_t rn = uart_read_frame(&p->drv, f, MB_ADU_MAX);
    if (rn > 0U)
    {
      uint8_t src;
      const uint8_t *rp;
      uint16_t rpn;
      if ((mb_adu_check(f, rn, &src, &rp, &rpn) == 0) && (src == slave))
      {
        memcpy(rsp_pdu, rp, rpn);
        *status = 0;
        ret = rpn;
        break;
      }
      if (rej_n == 0U)               /* forensics: carry back the first rejected frame verbatim (st=3) */
      {
        rej_n = (rn > 64U) ? 64U : rn;
        memcpy(rsp_pdu, f, rej_n);
      }
    }
    if ((HAL_GetTick() - t0) > timeout_ms)
    {
      *status = (rej_n > 0U) ? 3U : 1U;
      ret = rej_n;
      break;
    }
    osDelay(1);                      /* yield: this bus waits, the others keep running */
  }
  bus_cm4_unlock(port);
  return ret;
}

/* Slave role + error recovery for ONE port, run by that port's thread. */
void mbfront_service_port(uint8_t port)
{
  mbf_port_t *p;
  uint8_t *f, *rsp, *adu;
  if (port >= MBF_NPORT) { return; }
  p = &s_p[port];
  if (!p->opened) { return; }
  f = s_fbuf[port]; rsp = s_rbuf[port]; adu = s_abuf[port];

  bus_cm4_lock(port);
  uart_heal(&p->drv);
  if (p->role == MBF_ROLE_SLAVE)
  {
    for (;;)
    {
      uint16_t rn = uart_read_frame(&p->drv, f, MB_ADU_MAX);
      uint8_t src;
      const uint8_t *pdu;
      uint16_t pn, rl;
      if (rn == 0U) { break; }
      if (mb_adu_check(f, rn, &src, &pdu, &pn) != 0) { continue; }   /* bad frame: stay silent */
      if ((src != p->addr) && (src != MB_ADDR_BCAST)) { continue; }  /* not addressed to me */
      rl = board_handle(pdu, pn, rsp);
      if ((src != MB_ADDR_BCAST) && (rl > 0U))
      {
        int an = mb_adu_build(adu, MB_ADU_MAX, p->addr, rsp, rl);
        /* fire non-blocking and move on (DMA + TC drops DE); if the previous reply is still
         * in flight, drop it — the master's timeout-retry is the fallback */
        if (an > 0) { (void)uart_write(&p->drv, adu, (uint16_t)an); }
      }
    }
  }
  bus_cm4_unlock(port);
}

uint16_t mbfront_stats(uint8_t port, uint8_t *out)   /* op0B: 10×u32 LE (last 5 = internal loopback accounting, for troubleshooting) */
{
  uart_stats_t st;
  if ((port >= MBF_NPORT) || (!s_p[port].opened)) { memset(out, 0, 40); return 40; }
  uart_drv_t *d = &s_p[port].drv;
  uart_get_stats(d, &st);
  uint32_t v[10] = { st.rx_bytes, st.tx_bytes, st.frames, st.overrun, st.hw_errs,
                     d->wr_total, d->rd_total, d->frame_start,
                     (uint32_t)d->f_wr, (uint32_t)d->f_rd };
  for (int i = 0; i < 10; i++)
  {
    out[i * 4 + 0] = (uint8_t)v[i];
    out[i * 4 + 1] = (uint8_t)(v[i] >> 8);
    out[i * 4 + 2] = (uint8_t)(v[i] >> 16);
    out[i * 4 + 3] = (uint8_t)(v[i] >> 24);
  }
  return 40;
}

/* ---- Slave-port service + heal (every CM4 main-loop tick) ---- */
