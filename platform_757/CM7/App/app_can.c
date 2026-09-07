/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_can.c — FDCAN1 internal-loopback self-test (user file; defaultTask context, runs once and stops)
 * Clock: FDCAN kernel clock source = HSE, 500kbps bit timing.
 * Internal loopback (LBCK+MON): TX/RX all on-chip, pins not involved -> no GPIO config, no transceiver, doesn't disturb the outside.
 * Test coverage: init / start / standard-frame TX-RX / extended-frame TX-RX / filter reject (global reject of non-matching) / byte-by-byte data compare. */
#include "app_can.h"
#include <string.h>
#include "stm32h7xx_hal.h"

static int s_fail, s_total;
#define CHECK(cond) do { s_total++; if (!(cond)) { s_fail++; } } while (0)

/* poll-wait for RX FIFO0 to reach 'want' frames, timeout ~20ms (one frame at 500k ~0.25ms, ample margin) */
static uint32_t wait_fill(FDCAN_HandleTypeDef *h, uint32_t want)
{
  uint32_t t0 = HAL_GetTick();
  uint32_t n = 0;
  while ((HAL_GetTick() - t0) < 20U)
  {
    n = HAL_FDCAN_GetRxFifoFillLevel(h, FDCAN_RX_FIFO0);
    if (n >= want) { break; }
  }
  return n;
}

int app_can_selftest(int *total)
{
  static FDCAN_HandleTypeDef h;          /* static: only runs at boot / from CLI, don't burden the task stack */
  FDCAN_TxHeaderTypeDef tx = {0};
  FDCAN_RxHeaderTypeDef rx;
  uint8_t pay[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };
  uint8_t back[8];
  s_fail = 0;
  s_total = 0;

  __HAL_RCC_FDCAN_CLK_ENABLE();
  __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_HSE);   /* explicitly select HSE 8MHz, don't bet on the reset default */

  memset(&h, 0, sizeof(h));
  h.Instance = FDCAN1;
  h.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
  h.Init.Mode                 = FDCAN_MODE_INTERNAL_LOOPBACK;
  h.Init.AutoRetransmission   = ENABLE;
  h.Init.TransmitPause        = DISABLE;
  h.Init.ProtocolException    = DISABLE;
  h.Init.NominalPrescaler     = 1;                  /* 8MHz / (1*(1+13+2)tq) = 500 kbit/s */
  h.Init.NominalSyncJumpWidth = 1;
  h.Init.NominalTimeSeg1      = 13;
  h.Init.NominalTimeSeg2      = 2;
  h.Init.DataPrescaler        = 1;                  /* unused in CLASSIC, fill a legal value */
  h.Init.DataSyncJumpWidth    = 1;
  h.Init.DataTimeSeg1         = 13;
  h.Init.DataTimeSeg2         = 2;
  h.Init.MessageRAMOffset     = 0;
  h.Init.StdFiltersNbr        = 1;
  h.Init.ExtFiltersNbr        = 1;
  h.Init.RxFifo0ElmtsNbr      = 8;
  h.Init.RxFifo0ElmtSize      = FDCAN_DATA_BYTES_8;
  h.Init.RxFifo1ElmtsNbr      = 0;
  h.Init.RxBuffersNbr         = 0;
  h.Init.TxEventsNbr          = 0;
  h.Init.TxBuffersNbr         = 0;
  h.Init.TxFifoQueueElmtsNbr  = 8;
  h.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
  h.Init.TxElmtSize           = FDCAN_DATA_BYTES_8;
  CHECK(HAL_FDCAN_Init(&h) == HAL_OK);

  /* filters: standard 0x100~0x1FF -> FIFO0; extended 0x1ABCDE00 single -> FIFO0; reject all others */
  {
    FDCAN_FilterTypeDef f = {0};
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_RANGE;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = 0x100;
    f.FilterID2    = 0x1FF;
    CHECK(HAL_FDCAN_ConfigFilter(&h, &f) == HAL_OK);
    f.IdType       = FDCAN_EXTENDED_ID;
    f.FilterType   = FDCAN_FILTER_DUAL;
    f.FilterID1    = 0x1ABCDE00;
    f.FilterID2    = 0x1ABCDE00;
    CHECK(HAL_FDCAN_ConfigFilter(&h, &f) == HAL_OK);
    CHECK(HAL_FDCAN_ConfigGlobalFilter(&h, FDCAN_REJECT, FDCAN_REJECT,
                                       FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) == HAL_OK);
  }
  CHECK(HAL_FDCAN_Start(&h) == HAL_OK);

  /* 1. standard frame 0x123 (within filter range) -> should be received, byte-by-byte data compare */
  tx.Identifier          = 0x123;
  tx.IdType              = FDCAN_STANDARD_ID;
  tx.TxFrameType         = FDCAN_DATA_FRAME;
  tx.DataLength          = FDCAN_DLC_BYTES_8;
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx.BitRateSwitch       = FDCAN_BRS_OFF;
  tx.FDFormat            = FDCAN_CLASSIC_CAN;
  tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  CHECK(HAL_FDCAN_AddMessageToTxFifoQ(&h, &tx, pay) == HAL_OK);
  CHECK(wait_fill(&h, 1) == 1U);
  memset(back, 0, sizeof(back));
  CHECK(HAL_FDCAN_GetRxMessage(&h, FDCAN_RX_FIFO0, &rx, back) == HAL_OK);
  CHECK((rx.Identifier == 0x123U) && (rx.IdType == FDCAN_STANDARD_ID));
  CHECK(memcmp(back, pay, 8) == 0);

  /* 2. standard frame 0x456 (outside filter range) -> global reject, FIFO should stay empty */
  tx.Identifier = 0x456;
  CHECK(HAL_FDCAN_AddMessageToTxFifoQ(&h, &tx, pay) == HAL_OK);
  HAL_Delay(5);
  CHECK(HAL_FDCAN_GetRxFifoFillLevel(&h, FDCAN_RX_FIFO0) == 0U);

  /* 3. extended frame 0x1ABCDE00 -> should be received */
  {
    uint8_t p2[8] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01, 0x02, 0x03 };
    tx.Identifier = 0x1ABCDE00;
    tx.IdType     = FDCAN_EXTENDED_ID;
    CHECK(HAL_FDCAN_AddMessageToTxFifoQ(&h, &tx, p2) == HAL_OK);
    CHECK(wait_fill(&h, 1) == 1U);
    CHECK(HAL_FDCAN_GetRxMessage(&h, FDCAN_RX_FIFO0, &rx, back) == HAL_OK);
    CHECK((rx.Identifier == 0x1ABCDE00U) && (rx.IdType == FDCAN_EXTENDED_ID));
    CHECK(memcmp(back, p2, 8) == 0);
  }

  /* 4. back-to-back 4-frame burst -> FIFO collects them in order */
  {
    tx.Identifier = 0x180;
    tx.IdType     = FDCAN_STANDARD_ID;
    for (uint8_t i = 0; i < 4U; i++)
    {
      pay[0] = i;
      CHECK(HAL_FDCAN_AddMessageToTxFifoQ(&h, &tx, pay) == HAL_OK);
    }
    CHECK(wait_fill(&h, 4) == 4U);
    for (uint8_t i = 0; i < 4U; i++)
    {
      CHECK(HAL_FDCAN_GetRxMessage(&h, FDCAN_RX_FIFO0, &rx, back) == HAL_OK);
      CHECK(back[0] == i);   /* order preserved */
    }
  }

  HAL_FDCAN_Stop(&h);
  HAL_FDCAN_DeInit(&h);   /* leave a clean slate for the future real driver */

  if (total != 0) { *total = s_total; }
  return s_fail;
}
