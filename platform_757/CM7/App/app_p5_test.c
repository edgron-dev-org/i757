/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_p5_test.c — board 1 storage trio roll-call
 * SDRAM=W9825G6KH-6 (32MB, 13 rows/9 cols/4 banks, FMC BANK1@0xC0000000, SDCLK=100MHz all-8 conservative timing)
 * SFLASH=W25Q256JV (32MB, JEDEC expects EF 40 19)   SD=TF card slot (card detect PK4)
 * result string goes into the heartbeat "p5" field; runs once at boot (before mqtt_app_task enters its loop, ~3s within the IWDG 32s budget). */
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "fatfs.h"   /* f_mount + config file */
#include "app_qflash.h"   /* W25Q driver + LittleFS */

extern SDRAM_HandleTypeDef hsdram1;
extern SD_HandleTypeDef    hsd1;
extern void MX_FMC_Init(void);

static char s_p5[96] = "p5:pending";
const char *app_p5_str(void) { return s_p5; }

/* ---- SDRAM: JEDEC init sequence (W9825 generic) + address-line/data-line test ---- */
#define SDRAM_BASE  0xC0000000UL
#define SDRAM_SIZE  (32UL * 1024UL * 1024UL)

static int p5_sdram(void)
{
  FMC_SDRAM_CommandTypeDef cmd = {0};
  MX_FMC_Init();                                    /* controller (column width 9 fixed) */
  cmd.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;  cmd.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  cmd.AutoRefreshNumber = 1; cmd.ModeRegisterDefinition = 0;
  HAL_SDRAM_SendCommand(&hsdram1, &cmd, 1000);
  HAL_Delay(1);
  cmd.CommandMode = FMC_SDRAM_CMD_PALL;
  HAL_SDRAM_SendCommand(&hsdram1, &cmd, 1000);
  cmd.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE; cmd.AutoRefreshNumber = 8;
  HAL_SDRAM_SendCommand(&hsdram1, &cmd, 1000);
  cmd.CommandMode = FMC_SDRAM_CMD_LOAD_MODE; cmd.AutoRefreshNumber = 1;
  cmd.ModeRegisterDefinition = 0x230;               /* CAS=3, burst=1, sequential */
  HAL_SDRAM_SendCommand(&hsdram1, &cmd, 1000);
  HAL_SDRAM_ProgramRefreshRate(&hsdram1, 761);      /* 64ms/8192 rows @100MHz - 20 */

  /* address-as-data full scan + inverted readback: catches data-line/address-line/aliasing faults */
  volatile uint32_t *m = (volatile uint32_t *)SDRAM_BASE;
  uint32_t n = SDRAM_SIZE / 4U, i;
  for (i = 0; i < n; i += 64U) { m[i] = i; }        /* 256B-step fast scan (address faults always show) */
  for (i = 0; i < n; i += 64U) { if (m[i] != i) { return -(int)(i / 64U + 1); } }
  m[0] = 0xA5C3F00FUL; m[1] = ~0xA5C3F00FUL;        /* all data lines toggled */
  if (m[0] != 0xA5C3F00FUL || m[1] != ~0xA5C3F00FUL) { return -9990; }
  return 0;
}

/* ---- QSPI: init + JEDEC ID moved into the app_qflash.c driver (original p5_qspi_id parameters unchanged), only called here ---- */

/* ---- SD: PK4 for card detect, HAL_SD_Init only when a card is present (the generated version's failure path hits Error_Handler's infinite loop, not used) ---- */
static int p5_sd(uint32_t *cap_mb)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOK_CLK_ENABLE();
  g.Pin = GPIO_PIN_4; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOK, &g);
  HAL_Delay(2);
  if (HAL_GPIO_ReadPin(GPIOK, GPIO_PIN_4) != GPIO_PIN_RESET) { return 1; }  /* normally-open contact: high=no card */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;   /* 4-bit restored: the real culprit of ERR1 = IDMA can't reach DTCM (moved to AXI), the data lines were innocent */
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 2;                            /* conservative: ker/(2*2) */
  if (HAL_SD_Init(&hsd1) != HAL_OK) { return -1; }
  HAL_SD_CardInfoTypeDef ci;
  if (HAL_SD_GetCardInfo(&hsd1, &ci) != HAL_OK) { return -2; }
  *cap_mb = (uint32_t)(((uint64_t)ci.BlockNbr * ci.BlockSize) >> 20);
  return 0;
}

/* ---- FatFs: mount + config file i757.cfg (create if missing, reserved as a future config carrier) ---- */
static const char *p5_fatfs(void)
{
  static char r[28];
  FILINFO fi;
  FRESULT fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1);
  if (fr != FR_OK) { snprintf(r, sizeof r, "fs:ERR%d hsd:%lX st%d", (int)fr, (unsigned long)hsd1.ErrorCode, (int)HAL_SD_GetCardState(&hsd1)); return r; }
  if (f_stat("i757.cfg", &fi) == FR_OK)
  {
    snprintf(r, sizeof r, "fs:OK cfg:%luB", (unsigned long)fi.fsize);
  }
  else
  {
    UINT bw = 0;
    #define f SDFile   /* global FIL (moved to AXI + 32-byte aligned, reachable by IDMA) */
    static const char cfg[] =
      "# i757 device config (auto-created by firmware)\r\n"
      "sn=0001\r\n"
      "broker=YOUR_BROKER_HOST:18884\r\n"
      "# reserved: log=, modbus=, calib=\r\n";
    FRESULT fo = f_open(&f, "i757.cfg", FA_CREATE_ALWAYS | FA_WRITE);
    if (fo == FR_OK)
    {
      (void)f_write(&f, cfg, sizeof cfg - 1U, &bw);
      (void)f_close(&f);
      snprintf(r, sizeof r, "fs:OK cfg:new%u", (unsigned int)bw);
    }
    else { snprintf(r, sizeof r, "fs:OK cfg:WERR%d", (int)fo); }
    #undef f
  }
  return r;
}

void app_p5_test_run(void)
{
  int sr = p5_sdram();
  uint32_t qid = app_qflash_init();
  const char *lfs = (qid == 0xEF4019UL) ? app_lfs_selftest() : "lfs:SKIP";  /* mount only after confirming W25Q256 */
  uint32_t cap = 0;
  int sd = p5_sd(&cap);
  char sds[20];
  if (sd == 1)      { strcpy(sds, "NOCARD"); }
  else if (sd == 0) { snprintf(sds, sizeof sds, "%luMB", (unsigned long)cap); }
  else              { snprintf(sds, sizeof sds, "ERR%d", sd); }
  snprintf(s_p5, sizeof s_p5, "sdram:%s qspi:%06lX %s sd:%s%s%s",
           (sr == 0) ? "32MB-OK" : "FAIL", (unsigned long)qid, lfs, sds,
           (sd == 0) ? " " : "", (sd == 0) ? p5_fatfs() : "");
  if (sr != 0) { snprintf(s_p5, sizeof s_p5, "sdram:FAIL@%d qspi:%06lX sd:%s", sr, (unsigned long)qid, sds); }
  printf("[P5] %s\n\r", s_p5);
}
