/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_qflash.c — Board1 SPI Flash (W25Q256JV 32MB) QSPI driver
 * 1-line SPI instructions throughout (for config storage, speed is irrelevant); 32MB > 16MB requires 4-byte addressing,
 * use the dedicated 4B instructions (0x13/0x12/0x21) = stateless, does not depend on the chip's 3B/4B mode bit.
 * Self-initialized (generated config = placeholder), MspInit (GPIO/clock) is brought up automatically by HAL_QSPI_Init. */
#include "main.h"
#include "app_qflash.h"

extern QSPI_HandleTypeDef hqspi;

#define QF_CMD_JEDEC   0x9FU
#define QF_CMD_WREN    0x06U
#define QF_CMD_RDSR1   0x05U
#define QF_CMD_READ4B  0x13U   /* Read Data, 4-byte addr */
#define QF_CMD_PP4B    0x12U   /* Page Program, 4-byte addr */
#define QF_CMD_SE4B    0x21U   /* Sector Erase 4KB, 4-byte addr */

static void qf_cmd_base(QSPI_CommandTypeDef *c, uint8_t ins)
{
  c->InstructionMode = QSPI_INSTRUCTION_1_LINE;
  c->Instruction     = ins;
  c->AddressMode     = QSPI_ADDRESS_NONE;
  c->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  c->DataMode        = QSPI_DATA_NONE;
  c->DdrMode         = QSPI_DDR_MODE_DISABLE;
  c->SIOOMode        = QSPI_SIOO_INST_EVERY_CMD;
}

static int qf_wren(void)
{
  QSPI_CommandTypeDef c = {0};
  qf_cmd_base(&c, QF_CMD_WREN);
  return (HAL_QSPI_Command(&hqspi, &c, 100) == HAL_OK) ? 0 : -1;
}

/* Poll status register 1 until BUSY bit clears (hardware AutoPolling, does not occupy the CPU with a busy-wait instruction stream) */
static int qf_wait_idle(uint32_t timeout_ms)
{
  QSPI_CommandTypeDef c = {0};
  QSPI_AutoPollingTypeDef p = {0};
  qf_cmd_base(&c, QF_CMD_RDSR1);
  c.DataMode        = QSPI_DATA_1_LINE;
  p.Match           = 0x00;
  p.Mask            = 0x01;            /* bit0 = BUSY */
  p.MatchMode       = QSPI_MATCH_MODE_AND;
  p.StatusBytesSize = 1;
  p.Interval        = 16;
  p.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;
  return (HAL_QSPI_AutoPolling(&hqspi, &c, &p, timeout_ms) == HAL_OK) ? 0 : -1;
}

/* Initialize (params = finalized in p5: prescaler 4 / FT1 / 32MB) and return JEDEC ID; on failure return 0xEE000x (reusing p5 error codes) */
uint32_t app_qflash_init(void)
{
  QSPI_CommandTypeDef c = {0};
  uint8_t id[3] = {0};
  HAL_QSPI_DeInit(&hqspi);
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler     = 4;
  hqspi.Init.FifoThreshold      = 1;
  hqspi.Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize          = 24;   /* 2^25 = 32MB */
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_2_CYCLE;
  hqspi.Init.ClockMode          = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID            = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash          = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK) { return 0xEE0001U; }
  qf_cmd_base(&c, QF_CMD_JEDEC);
  c.DataMode = QSPI_DATA_1_LINE;
  c.NbData   = 3;
  if (HAL_QSPI_Command(&hqspi, &c, 100) != HAL_OK) { return 0xEE0002U; }
  if (HAL_QSPI_Receive(&hqspi, id, 100) != HAL_OK)  { return 0xEE0003U; }
  return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
}

int app_qflash_read(uint32_t addr, void *buf, uint32_t len)
{
  QSPI_CommandTypeDef c = {0};
  if (len == 0U) { return 0; }
  qf_cmd_base(&c, QF_CMD_READ4B);
  c.AddressMode = QSPI_ADDRESS_1_LINE;
  c.AddressSize = QSPI_ADDRESS_32_BITS;
  c.Address     = addr;
  c.DataMode    = QSPI_DATA_1_LINE;
  c.NbData      = len;
  if (HAL_QSPI_Command(&hqspi, &c, 100) != HAL_OK)        { return -1; }
  if (HAL_QSPI_Receive(&hqspi, (uint8_t *)buf, 1000) != HAL_OK) { return -2; }
  return 0;
}

/* Page-by-page (256B) write loop: each page WREN -> PP4B -> wait BUSY (typical 0.4ms/page) */
int app_qflash_write(uint32_t addr, const void *buf, uint32_t len)
{
  const uint8_t *p = (const uint8_t *)buf;
  while (len > 0U)
  {
    uint32_t chunk = QFLASH_PAGE - (addr % QFLASH_PAGE);
    QSPI_CommandTypeDef c = {0};
    if (chunk > len) { chunk = len; }
    if (qf_wren() != 0) { return -1; }
    qf_cmd_base(&c, QF_CMD_PP4B);
    c.AddressMode = QSPI_ADDRESS_1_LINE;
    c.AddressSize = QSPI_ADDRESS_32_BITS;
    c.Address     = addr;
    c.DataMode    = QSPI_DATA_1_LINE;
    c.NbData      = chunk;
    if (HAL_QSPI_Command(&hqspi, &c, 100) != HAL_OK)             { return -2; }
    if (HAL_QSPI_Transmit(&hqspi, (uint8_t *)p, 1000) != HAL_OK) { return -3; }
    if (qf_wait_idle(100) != 0)                                  { return -4; }
    addr += chunk; p += chunk; len -= chunk;
  }
  return 0;
}

/* ---- Power-fail dying preparation (called inside the EXTI highest-priority ISR; contract = power-fail save-area iron rule) ----
 * (1) Forcibly seize the handle: may be interrupting a littlefs read/write/erase (HAL state BUSY + lock) —— do a direct hardware ABORT +
 *   state reset (cannot use HAL_QSPI_Abort: its timeout relies on HAL_GetTick, and SysTick is frozen under this ISR).
 * (2) A chip background erase in flight (SE4K worst case 400ms) would block the dying page write —— issue Erase/Program Suspend (0x75),
 *   after W25Q tSUS≈20µs page programming to [other sectors] is possible (save area is physically isolated from the littlefs area, always satisfied). */
void app_qflash_dying_prep(void)
{
  uint32_t spin;
  QUADSPI->CR |= QUADSPI_CR_ABORT;                    /* hardware-abort the in-flight transfer */
  for (spin = 0; (QUADSPI->CR & QUADSPI_CR_ABORT) && (spin < 100000U); spin++) {}
  __HAL_QSPI_CLEAR_FLAG(&hqspi, QSPI_FLAG_TO | QSPI_FLAG_SM | QSPI_FLAG_FT | QSPI_FLAG_TC | QSPI_FLAG_TE);
  hqspi.State = HAL_QSPI_STATE_READY;                 /* force unlock (one-way takeover, system never returns) */
  hqspi.Lock  = HAL_UNLOCKED;
  {
    QSPI_CommandTypeDef c = {0};
    uint8_t sr = 0;
    qf_cmd_base(&c, QF_CMD_RDSR1);
    c.DataMode = QSPI_DATA_1_LINE;
    c.NbData   = 1;
    if ((HAL_QSPI_Command(&hqspi, &c, 100) == HAL_OK) &&
        (HAL_QSPI_Receive(&hqspi, &sr, 100) == HAL_OK) && ((sr & 0x01U) != 0U))
    {
      QSPI_CommandTypeDef s = {0};
      qf_cmd_base(&s, 0x75U);                          /* Erase/Program Suspend */
      (void)HAL_QSPI_Command(&hqspi, &s, 100);
      for (spin = 0; spin < 20000U; spin++) { __NOP(); }   /* ≥tSUS (20µs) @480MHz margin */
    }
  }
}

/* 4KB sector erase (typical 45ms, max 400ms) */
int app_qflash_erase4k(uint32_t addr)
{
  QSPI_CommandTypeDef c = {0};
  if (qf_wren() != 0) { return -1; }
  qf_cmd_base(&c, QF_CMD_SE4B);
  c.AddressMode = QSPI_ADDRESS_1_LINE;
  c.AddressSize = QSPI_ADDRESS_32_BITS;
  c.Address     = addr;
  if (HAL_QSPI_Command(&hqspi, &c, 100) != HAL_OK) { return -2; }
  return (qf_wait_idle(2000) == 0) ? 0 : -3;
}
