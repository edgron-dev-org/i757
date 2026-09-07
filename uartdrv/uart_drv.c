/* SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Edgron. Licensed under the Apache License, Version 2.0 (see LICENSE in this directory). */
/* uart_drv.c — generic UART low-level implementation (contract=docs/Universal_UART_Driver_Design.md; header has the three disciplines: concurrency/pins/interrupts)
 * Implementation highlights:
 *   - Handle registry: HAL callback reverse-looks-up the instance by huart (container_of); unregistered handles (e.g. BSP VCP) are always let through—
 *     this library coexists peacefully with other HAL UART users in the project.
 *   - Ring accounting = monotonic counters: bytes in ring = wr_total - rd_total, no wrap-around ambiguity; exceeding ring length = honest overrun reset.
 *   - Frame delivery = zero-copy descriptors: IDLE only records {off,len}, data stays in the DMA ring, copied to the application once on read_frame. */
#include "uart_drv.h"
#include <string.h>

/* ---- Handle registry (for callback dispatch; static limit 8 instances/core, enough for 7 ports + margin) ---- */
#define UART_DRV_MAX_INST 8
static uart_drv_t *s_reg[UART_DRV_MAX_INST];

static uart_drv_t *drv_of(UART_HandleTypeDef *hu)
{
  for (int i = 0; i < UART_DRV_MAX_INST; i++)
  {
    if ((s_reg[i] != NULL) && (&s_reg[i]->huart == hu)) { return s_reg[i]; }
  }
  return NULL;   /* not this library's handle (e.g. BSP VCP): let the callback through */
}

static int reg_add(uart_drv_t *u)
{
  for (int i = 0; i < UART_DRV_MAX_INST; i++)
  {
    if (s_reg[i] == NULL) { s_reg[i] = u; return 0; }
  }
  return -1;
}

/* ---- Chip-specific small tables: instance->clock/IRQ; DMA stream->IRQ ---- */
static int inst_clk_irq(USART_TypeDef *inst, IRQn_Type *irq)
{
  if      (inst == USART1) { __HAL_RCC_USART1_CLK_ENABLE(); *irq = USART1_IRQn; }
  else if (inst == USART2) { __HAL_RCC_USART2_CLK_ENABLE(); *irq = USART2_IRQn; }
  else if (inst == USART3) { __HAL_RCC_USART3_CLK_ENABLE(); *irq = USART3_IRQn; }
#ifdef UART4
  else if (inst == UART4)  { __HAL_RCC_UART4_CLK_ENABLE();  *irq = UART4_IRQn;  }
#endif
#ifdef UART5
  else if (inst == UART5)  { __HAL_RCC_UART5_CLK_ENABLE();  *irq = UART5_IRQn;  }
#endif
#ifdef USART6
  else if (inst == USART6) { __HAL_RCC_USART6_CLK_ENABLE(); *irq = USART6_IRQn; }
#endif
#ifdef UART7
  else if (inst == UART7)  { __HAL_RCC_UART7_CLK_ENABLE();  *irq = UART7_IRQn;  }
#endif
#ifdef UART8
  else if (inst == UART8)  { __HAL_RCC_UART8_CLK_ENABLE();  *irq = UART8_IRQn;  }
#endif
  else { return -1; }
  return 0;
}

#ifdef UARTDRV_H5
static int dma_clk_irq(uartdrv_dma_t *ch, IRQn_Type *irq)
{
  /* GPDMA channel stride 0x80 (CMSIS base address table), Channel0 at controller base+0x50 */
  static const IRQn_Type g1[8] = { GPDMA1_Channel0_IRQn, GPDMA1_Channel1_IRQn,
    GPDMA1_Channel2_IRQn, GPDMA1_Channel3_IRQn, GPDMA1_Channel4_IRQn,
    GPDMA1_Channel5_IRQn, GPDMA1_Channel6_IRQn, GPDMA1_Channel7_IRQn };
  uint32_t a = (uint32_t)ch;
  if ((a >= (uint32_t)GPDMA1_Channel0) && (a <= (uint32_t)GPDMA1_Channel7))
  {
    __HAL_RCC_GPDMA1_CLK_ENABLE();
    *irq = g1[(a - (uint32_t)GPDMA1_Channel0) / 0x80U];
    return 0;
  }
#ifdef GPDMA2
  {
    static const IRQn_Type g2[8] = { GPDMA2_Channel0_IRQn, GPDMA2_Channel1_IRQn,
      GPDMA2_Channel2_IRQn, GPDMA2_Channel3_IRQn, GPDMA2_Channel4_IRQn,
      GPDMA2_Channel5_IRQn, GPDMA2_Channel6_IRQn, GPDMA2_Channel7_IRQn };
    if ((a >= (uint32_t)GPDMA2_Channel0) && (a <= (uint32_t)GPDMA2_Channel7))
    {
      __HAL_RCC_GPDMA2_CLK_ENABLE();
      *irq = g2[(a - (uint32_t)GPDMA2_Channel0) / 0x80U];
      return 0;
    }
  }
#endif
  return -1;
}
#else
static int dma_clk_irq(uartdrv_dma_t *s, IRQn_Type *irq)
{
  static const IRQn_Type d1[8] = { DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn,
    DMA1_Stream3_IRQn, DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn };
  static const IRQn_Type d2[8] = { DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn,
    DMA2_Stream3_IRQn, DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn };
  uint32_t a = (uint32_t)s;
  if ((a >= (uint32_t)DMA1_Stream0) && (a <= (uint32_t)DMA1_Stream7))
  {
    __HAL_RCC_DMA1_CLK_ENABLE();
    *irq = d1[(a - (uint32_t)DMA1_Stream0) / 0x18U];
    return 0;
  }
  if ((a >= (uint32_t)DMA2_Stream0) && (a <= (uint32_t)DMA2_Stream7))
  {
    __HAL_RCC_DMA2_CLK_ENABLE();
    *irq = d2[(a - (uint32_t)DMA2_Stream0) / 0x18U];
    return 0;
  }
  return -1;
}
#endif

/* ---- Internal: start/restart receive ----
 * Key point of the IT degraded path: after each event (IDLE/full) ReceiveToIdle_IT auto-stops receiving and starts from the buffer head,
 * so applying ring accounting directly is bound to be wrong. Approach = segmented pseudo-ring: at each segment end, re-arm at wr_total's in-ring position, segment length = to ring end. */
static HAL_StatusTypeDef rx_arm_it(uart_drv_t *u)
{
  uint16_t off = (uint16_t)(u->wr_total % u->cfg.rxlen);
  return HAL_UARTEx_ReceiveToIdle_IT(&u->huart, u->cfg.rxbuf + off, (uint16_t)(u->cfg.rxlen - off));
}

/* 0 = reception truly armed. The HAL refuses (BUSY/ERROR) when its RX or DMA state machine is
 * not READY — e.g. an error abort still in flight. Callers MUST look at this result: ignoring
 * one refusal is how a port goes silent forever (2026-08-22 greenhouse backplane case). */
static int rx_arm(uart_drv_t *u)
{
  HAL_StatusTypeDef st;
  u->wr_total = 0;
  u->rd_total = 0;
  u->frame_start = 0;
  u->f_wr = 0;
  u->f_rd = 0;
  if (u->cfg.dma_rx != NULL)
  {
    st = HAL_UARTEx_ReceiveToIdle_DMA(&u->huart, u->cfg.rxbuf, u->cfg.rxlen);
    /* HT/TC events kept: they advance the write shadow, the heartbeat of overflow detection (design §2.2) */
  }
  else
  {
    st = rx_arm_it(u);
  }
  return (st == HAL_OK) ? 0 : -1;
}

/* ---- HAL callbacks (IRQ context; touch only this instance's fields) ---- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *hu, uint16_t Size)
{
  uart_drv_t *u = drv_of(hu);
  if (u == NULL) { return; }
  if (u->cfg.dma_rx == NULL)
  {
    /* IT segmented path: each callback = this segment's receive has ended, Size=bytes in segment; IDLE naturally aligns with segment boundary */
    uint32_t seg_base = u->wr_total;
    u->wr_total += Size;
    u->stats.rx_bytes += Size;
    if ((u->wr_total - u->rd_total) > u->cfg.rxlen)
    {
      u->stats.overrun++;
      u->rd_total = u->wr_total;
      u->frame_start = u->wr_total;
      u->f_wr = u->f_rd = 0;
    }
    else if ((u->cfg.framing == UARTDRV_FRAME_IDLE) && (Size > 0U))
    {
      uint8_t next = (uint8_t)((u->f_wr + 1U) % u->cfg.max_frames);
      if (next != u->f_rd)
      {
        u->fring[u->f_wr].off = (uint16_t)(seg_base % u->cfg.rxlen);
        u->fring[u->f_wr].len = Size;
        u->f_wr = next;
      }
      else { u->stats.overrun++; }
      u->frame_start = u->wr_total;
    }
    if (rx_arm_it(u) != HAL_OK) { u->rx_dead = 1; }   /* segment ended and receive stopped, must re-arm (allowed in callback);
                                                       * a refusal only raises the flag — uart_heal retries in thread context */
    return;
  }
  uint16_t pos = (uint16_t)(Size % u->cfg.rxlen);   /* on a full wrap HAL reports Size=ring length */
  uint16_t old = (uint16_t)(u->wr_total % u->cfg.rxlen);
  uint16_t fresh = (uint16_t)((uint16_t)(pos - old + u->cfg.rxlen) % u->cfg.rxlen);
  u->wr_total += fresh;
  u->stats.rx_bytes += fresh;
  if ((u->wr_total - u->rd_total) > u->cfg.rxlen)   /* write shadow ran over read pointer: honest reset */
  {
    u->stats.overrun++;
    u->rd_total = u->wr_total;                      /* discard all old accounting in the ring */
    u->frame_start = u->wr_total;
    u->f_wr = u->f_rd = 0;
    return;
  }
  if ((u->cfg.framing == UARTDRV_FRAME_IDLE) &&
      (HAL_UARTEx_GetRxEventType(hu) == HAL_UART_RXEVENT_IDLE))
  {
    uint32_t flen = u->wr_total - u->frame_start;
    if (flen > 0U)
    {
      uint8_t next = (uint8_t)((u->f_wr + 1U) % u->cfg.max_frames);
      if (next != u->f_rd)
      {
        u->fring[u->f_wr].off = (uint16_t)(u->frame_start % u->cfg.rxlen);
        u->fring[u->f_wr].len = (uint16_t)((flen > u->cfg.rxlen) ? u->cfg.rxlen : flen);
        u->f_wr = next;
      }
      else { u->stats.overrun++; }                  /* frame ring full: drop new frame and count (master retry is the backstop) */
      u->frame_start = u->wr_total;
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *hu)   /* UART TC: the last stop bit has left the pin */
{
  uart_drv_t *u = drv_of(hu);
  if (u == NULL) { return; }
  if (u->cfg.de_mode == UARTDRV_DE_GPIO)
  {
    HAL_GPIO_WritePin(u->cfg.de_port, u->cfg.de_pin, GPIO_PIN_RESET);   /* the only correct moment to clear DE */
  }
  u->tx_busy = 0;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *hu)
{
  uart_drv_t *u = drv_of(hu);
  if (u == NULL) { return; }
  u->stats.hw_errs++;
  u->rx_dead = 1;   /* only raise a flag; restart belongs to uart_heal (main-loop thread context) */
}

/* ---- Interrupt wiring API ---- */
void uart_drv_uart_isr  (uart_drv_t *u) { HAL_UART_IRQHandler(&u->huart); }
void uart_drv_dma_rx_isr(uart_drv_t *u) { HAL_DMA_IRQHandler(&u->hdma_rx); }
void uart_drv_dma_tx_isr(uart_drv_t *u) { HAL_DMA_IRQHandler(&u->hdma_tx); }

/* ---- Open: DMA assembly (where family differences are concentrated) ---- */
#ifdef UARTDRV_H5
/* H5 GPDMA simple-mode common Init (for TX) */
static void h5_init_fields(DMA_InitTypeDef *i, uint32_t req, uint32_t dir)
{
  i->Request              = req;
  i->BlkHWRequest         = DMA_BREQ_SINGLE_BURST;
  i->Direction            = dir;
  i->SrcInc               = (dir == DMA_MEMORY_TO_PERIPH) ? DMA_SINC_INCREMENTED : DMA_SINC_FIXED;
  i->DestInc              = (dir == DMA_MEMORY_TO_PERIPH) ? DMA_DINC_FIXED : DMA_DINC_INCREMENTED;
  i->SrcDataWidth         = DMA_SRC_DATAWIDTH_BYTE;
  i->DestDataWidth        = DMA_DEST_DATAWIDTH_BYTE;
  i->Priority             = DMA_HIGH_PRIORITY;
  i->SrcBurstLength       = 1;
  i->DestBurstLength      = 1;
  i->TransferAllocatedPort= DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  i->TransferEventMode    = DMA_TCEM_BLOCK_TRANSFER;
  i->Mode                 = DMA_NORMAL;
}

static int dma_setup_tx(uart_drv_t *u)
{
  DMA_HandleTypeDef *h = &u->hdma_tx;
  h->Instance = u->cfg.dma_tx;
  h5_init_fields(&h->Init, u->cfg.req_tx, DMA_MEMORY_TO_PERIPH);
  return (HAL_DMA_Init(h) == HAL_OK) ? 0 : -1;
}

/* H5 GPDMA has no circular bit: RX ring = single-node self-looping linked-list queue (HAL UART itself
 * rewrites the head node's size/src/dst on receive start, see stm32h5xx_hal_uart.c UART_Start_Receive_DMA) */
static int dma_setup_rx(uart_drv_t *u)
{
  DMA_HandleTypeDef *h = &u->hdma_rx;
  h->Instance                          = u->cfg.dma_rx;
  h->InitLinkedList.Priority           = DMA_HIGH_PRIORITY;
  h->InitLinkedList.LinkStepMode       = DMA_LSM_FULL_EXECUTION;
  h->InitLinkedList.LinkAllocatedPort  = DMA_LINK_ALLOCATED_PORT0;
  h->InitLinkedList.TransferEventMode  = DMA_TCEM_BLOCK_TRANSFER;
  h->InitLinkedList.LinkedListMode     = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(h) != HAL_OK) { return -1; }

  DMA_NodeConfTypeDef nc;
  memset(&nc, 0, sizeof(nc));
  nc.NodeType = DMA_GPDMA_LINEAR_NODE;
  h5_init_fields(&nc.Init, u->cfg.req_rx, DMA_PERIPH_TO_MEMORY);
  /* SrcAddress/DstAddress/DataSize left 0: HAL rewrites the head node registers on receive start */
  memset(&u->ll_queue, 0, sizeof(u->ll_queue));
  if (HAL_DMAEx_List_BuildNode(&nc, &u->ll_node)                 != HAL_OK) { return -1; }
  if (HAL_DMAEx_List_InsertNode_Tail(&u->ll_queue, &u->ll_node)  != HAL_OK) { return -1; }
  if (HAL_DMAEx_List_SetCircularMode(&u->ll_queue)               != HAL_OK) { return -1; }
  if (HAL_DMAEx_List_LinkQ(h, &u->ll_queue)                      != HAL_OK) { return -1; }
  return 0;
}
#else
static void dma_common(DMA_HandleTypeDef *h, uartdrv_dma_t *s, uint32_t req,
                       uint32_t dir, uint32_t mode)
{
  h->Instance                 = s;
  h->Init.Request             = req;
  h->Init.Direction           = dir;
  h->Init.PeriphInc           = DMA_PINC_DISABLE;
  h->Init.MemInc              = DMA_MINC_ENABLE;
  h->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  h->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  h->Init.Mode                = mode;
  h->Init.Priority            = DMA_PRIORITY_HIGH;
  h->Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(h);
}

static int dma_setup_rx(uart_drv_t *u)
{
  dma_common(&u->hdma_rx, u->cfg.dma_rx, u->cfg.req_rx, DMA_PERIPH_TO_MEMORY, DMA_CIRCULAR);
  return 0;
}

static int dma_setup_tx(uart_drv_t *u)
{
  dma_common(&u->hdma_tx, u->cfg.dma_tx, u->cfg.req_tx, DMA_MEMORY_TO_PERIPH, DMA_NORMAL);
  return 0;
}
#endif

int uart_open(uart_drv_t *u, const uart_cfg_t *cfg)
{
  IRQn_Type uirq, dirq;
  memset(u, 0, sizeof(*u));
  u->cfg = *cfg;
  if ((cfg->rxbuf == NULL) || (cfg->rxlen == 0U)) { return -1; }
  if ((cfg->framing == UARTDRV_FRAME_IDLE) &&
      ((cfg->max_frames == 0U) || (cfg->max_frames > 16U))) { return -1; }
  if (inst_clk_irq(cfg->instance, &uirq) != 0) { return -1; }
  if (reg_add(u) != 0) { return -1; }

  if (cfg->dma_rx != NULL)
  {
    if (dma_clk_irq(cfg->dma_rx, &dirq) != 0) { return -1; }
    if (dma_setup_rx(u) != 0) { return -1; }
    __HAL_LINKDMA(&u->huart, hdmarx, u->hdma_rx);
    HAL_NVIC_SetPriority(dirq, cfg->irq_prio, 0);
    HAL_NVIC_EnableIRQ(dirq);
  }
  if (cfg->dma_tx != NULL)
  {
    if (dma_clk_irq(cfg->dma_tx, &dirq) != 0) { return -1; }
    if (dma_setup_tx(u) != 0) { return -1; }
    __HAL_LINKDMA(&u->huart, hdmatx, u->hdma_tx);
    HAL_NVIC_SetPriority(dirq, cfg->irq_prio, 0);
    HAL_NVIC_EnableIRQ(dirq);
  }

  u->huart.Instance          = cfg->instance;
  u->huart.Init.BaudRate     = cfg->baud;
  u->huart.Init.WordLength   = (cfg->parity == UART_PARITY_NONE) ? UART_WORDLENGTH_8B
                                                                 : UART_WORDLENGTH_9B;
  u->huart.Init.StopBits     = cfg->stopbits;
  u->huart.Init.Parity       = cfg->parity;
  u->huart.Init.Mode         = UART_MODE_TX_RX;
  u->huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  u->huart.Init.OverSampling = UART_OVERSAMPLING_16;
  if (cfg->swap)
  {
    u->huart.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_SWAP_INIT;
    u->huart.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  }
  if (cfg->de_mode == UARTDRV_DE_HW)
  {
    /* Hardware DE: assertion timing DEAT/DEDT each 8 oversampling ticks (≈half bit), HAL dedicated init entry */
    if (HAL_RS485Ex_Init(&u->huart, UART_DE_POLARITY_HIGH, 8, 8) != HAL_OK) { return -2; }
  }
  else
  {
    if (HAL_UART_Init(&u->huart) != HAL_OK) { return -2; }
    if (cfg->de_mode == UARTDRV_DE_GPIO)
    {
      HAL_GPIO_WritePin(cfg->de_port, cfg->de_pin, GPIO_PIN_RESET);   /* power-on = receive */
    }
  }
  HAL_NVIC_SetPriority(uirq, cfg->irq_prio, 0);
  HAL_NVIC_EnableIRQ(uirq);
  return (rx_arm(u) == 0) ? 0 : -3;             /* a port that opened but cannot receive is not open */
}

/* ---- Hot-change baud rate ----
 * Does not go through uart_open (whose leading memset would trample a live DMA/ISR handle): wait for send done -> UE off ->
 * UART_SetConfig recomputes BRR (HAL private but exported in header; both H5/H7 take the core clock per their own RCC) -> UE on.
 * This function touches neither CR1's DEAT/DEDT nor CR3's DMAR/DMAT/DEM (only MODIFY of their respective fields), DMA/ring-accounting unaffected. */
int uart_set_baud(uart_drv_t *u, uint32_t baud)
{
  return uart_set_line(u, baud, u->cfg.parity, u->cfg.stopbits);
}

/* Hot-change line params (baud+parity+stop bits; front-panel configurable Modbus port, same route as uart_set_baud):
 * Word length links to parity (HAL convention: with parity=9B including the parity bit), CR1/CR2 fully field-rewritten by UART_SetConfig,
 * CR3 (DMAR/DMAT/DEM) untouched, DMA/ring-accounting/framing unaffected. Only call on an already-open instance. */
int uart_set_line(uart_drv_t *u, uint32_t baud, uint32_t parity, uint32_t stopbits)
{
  extern HAL_StatusTypeDef UART_SetConfig(UART_HandleTypeDef *huart);
  HAL_StatusTypeDef st;
  (void)uart_flush(u, 20);
  __HAL_UART_DISABLE(&u->huart);
  u->huart.Init.BaudRate   = baud;
  u->huart.Init.Parity     = parity;
  u->huart.Init.StopBits   = stopbits;
  u->huart.Init.WordLength = (parity == UART_PARITY_NONE) ? UART_WORDLENGTH_8B
                                                          : UART_WORDLENGTH_9B;
  u->cfg.baud     = baud;
  u->cfg.parity   = parity;
  u->cfg.stopbits = stopbits;
  st = UART_SetConfig(&u->huart);
  __HAL_UART_ENABLE(&u->huart);
  return (st == HAL_OK) ? 0 : -1;
}

/* ---- Transmit ---- */
uint16_t uart_write(uart_drv_t *u, const void *d, uint16_t n)
{
  if (u->tx_busy) { u->stats.tx_reject++; return 0; }
  if (n == 0U) { return 0; }
  if (u->cfg.de_mode == UARTDRV_DE_GPIO)
  {
    HAL_GPIO_WritePin(u->cfg.de_port, u->cfg.de_pin, GPIO_PIN_SET);
  }
  if (u->cfg.dma_tx != NULL)
  {
    if (n > u->cfg.txlen) { n = u->cfg.txlen; }
    memcpy(u->cfg.txbuf, d, n);                 /* source must stay resident during DMA -> port-private buffer */
    u->tx_busy = 1;
    if (HAL_UART_Transmit_DMA(&u->huart, u->cfg.txbuf, n) != HAL_OK)
    {
      if (u->cfg.de_mode == UARTDRV_DE_GPIO)    /* refused start: no TC will ever fire to drop DE —
                                                 * leaving it set jams the bus AND mutes our own receiver */
      {
        HAL_GPIO_WritePin(u->cfg.de_port, u->cfg.de_pin, GPIO_PIN_RESET);
      }
      u->tx_busy = 0;
      return 0;
    }
  }
  else
  {
    u->tx_busy = 1;                             /* no-DMA degraded path: HAL blocking transmit (low-speed port) */
    HAL_UART_Transmit(&u->huart, (const uint8_t *)d, n, 100);
    if (u->cfg.de_mode == UARTDRV_DE_GPIO)
    {
      HAL_GPIO_WritePin(u->cfg.de_port, u->cfg.de_pin, GPIO_PIN_RESET);
    }
    u->tx_busy = 0;
  }
  u->stats.tx_bytes += n;
  return n;
}

int uart_flush(uart_drv_t *u, uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();
  while (u->tx_busy)
  {
    if ((HAL_GetTick() - t0) > timeout_ms) { return -1; }
  }
  return 0;
}

int uart_tx_busy(uart_drv_t *u) { return (int)u->tx_busy; }

/* ---- Receive: STREAM ---- */
uint16_t uart_rx_count(uart_drv_t *u)
{
  return (uint16_t)(u->wr_total - u->rd_total);
}

uint16_t uart_read(uart_drv_t *u, void *d, uint16_t n)
{
  uint16_t avail = uart_rx_count(u);
  uint8_t *out = (uint8_t *)d;
  if (n > avail) { n = avail; }
  for (uint16_t i = 0; i < n; i++)
  {
    out[i] = u->cfg.rxbuf[u->rd_total % u->cfg.rxlen];
    u->rd_total++;
  }
  return n;
}

/* ---- Receive: FRAME_IDLE ---- */
uint16_t uart_read_frame(uart_drv_t *u, void *d, uint16_t cap)
{
  if (u->f_rd == u->f_wr) { return 0; }
  uart_frame_desc_t fd = u->fring[u->f_rd];
  u->f_rd = (uint8_t)((u->f_rd + 1U) % u->cfg.max_frames);
  uint16_t n = (fd.len > cap) ? cap : fd.len;
  for (uint16_t i = 0; i < n; i++)
  {
    ((uint8_t *)d)[i] = u->cfg.rxbuf[(uint16_t)((fd.off + i) % u->cfg.rxlen)];
  }
  u->rd_total += fd.len;                        /* consume the whole frame (including the tail truncated by cap) */
  u->stats.frames++;
  return n;
}

/* ---- Maintenance ----
 * Persistent heal (2026-08-22 redesign after the greenhouse backplane RX wedge): the old
 * version cleared rx_dead FIRST and ignored the re-arm result — one HAL refusal (abort still
 * in flight, state machine not READY) and the port stayed deaf forever while TX kept working.
 * Now the flag is cleared only after a re-arm the HAL accepted, and repeated refusals walk an
 * escalation ladder: plain re-arm (tries 1..7) -> HAL_UART_Abort + re-arm (try 8) -> full
 * DeInit/Init from the saved config (try 16, then the ladder restarts). Caller cadence is the
 * bus thread's ~1ms loop, so the whole ladder plays out within ~20ms. */
void uart_heal(uart_drv_t *u)
{
  if (!u->rx_dead) { return; }
  u->heal_tries++;
  if (u->heal_tries == 8U)
  {
    (void)HAL_UART_Abort(&u->huart);            /* plain re-arm keeps being refused: clear whatever HAL thinks is running */
  }
  else if (u->heal_tries >= 16U)
  {
    (void)HAL_UART_Abort(&u->huart);            /* still stuck: rebuild the peripheral from the saved config */
    (void)HAL_UART_DeInit(&u->huart);
    if (u->cfg.de_mode == UARTDRV_DE_HW)
    {
      (void)HAL_RS485Ex_Init(&u->huart, UART_DE_POLARITY_HIGH, 8, 8);
    }
    else
    {
      (void)HAL_UART_Init(&u->huart);
      if (u->cfg.de_mode == UARTDRV_DE_GPIO)
      {
        HAL_GPIO_WritePin(u->cfg.de_port, u->cfg.de_pin, GPIO_PIN_RESET);
      }
    }
    u->heal_tries = 0;
  }
  if (rx_arm(u) != 0)                           /* restart in thread context (honest full-ring reset) */
  {
    u->stats.rearm_fail++;
    return;                                     /* rx_dead stays set: retry on the next call */
  }
  u->rx_dead = 0;
  u->heal_tries = 0;
}

/* Upper-layer silent-bus watchdog hook: some wedges raise no error flag at all (every layer
 * below believes it is healthy). A master that keeps transmitting but has heard NOTHING for
 * far too long calls this; the next uart_heal goes straight to the full re-init rung.
 * Thread-context only (same owner thread that calls uart_heal), never from an ISR. */
void uart_rx_kick(uart_drv_t *u)
{
  u->heal_tries = 15U;
  u->rx_dead = 1;
}

void uart_get_stats(uart_drv_t *u, uart_stats_t *out) { *out = u->stats; }
