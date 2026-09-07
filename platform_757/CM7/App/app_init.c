/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_init.c — CM7 board-level init / firmware identity / ETH interrupt (user file, untouched by CubeMX regeneration)
 * split from main.c / stm32h7xx_it.c */
#include "app_init.h"
#include "main.h"
#include "app_mqtt.h"   /* IPC_PING/IPC_PONG */
#include "app_ota.h"    /* fw_info_t */

/* Firmware ID tag: fixed image offset 0x400 (.fw_info section in the .ld), shared by OTA footer and heartbeat as the single version source.
 * OTA_TEST_* builds (for auto-rollback verification) force a marker in the version string -> heartbeat/dash spot it at a glance, preventing a test firmware from posing as a release build */
#if defined(OTA_TEST_HANG)
#define FW_TAG "TEST-HANG "
#elif defined(OTA_TEST_NONET)
#define FW_TAG "TEST-NONET "
#else
#define FW_TAG ""
#endif
const fw_info_t g_fw_info __attribute__((section(".fw_info"), used)) =
  { FW_INFO_MAGIC, "I757 " FW_TAG __DATE__ " " __TIME__ };

/* --- chip internal temperature: ADC3 internal temperature sensor (bare registers; this project has no ADC HAL/LL) ---
 * Factory calibration TS_CAL1@30℃ / TS_CAL2@110℃ (16bit, VDDA=3.3V). Measures chip junction temperature (ten-odd degrees above room temp). */
#define TS_CAL1_ADDR ((volatile uint16_t *)0x1FF1E820UL)
#define TS_CAL2_ADDR ((volatile uint16_t *)0x1FF1E840UL)
static uint8_t s_temp_ok = 0;
void app_temp_init(void)
{
  uint32_t g;
  RCC->AHB4ENR |= RCC_AHB4ENR_ADC3EN;   /* ADC3 clock (D3 domain AHB4) */
  (void)RCC->AHB4ENR;
  /* HCLK/4 synchronous clock + enable internal temperature sensor */
  ADC3_COMMON->CCR = (ADC3_COMMON->CCR & ~ADC_CCR_CKMODE_Msk) | (3UL << ADC_CCR_CKMODE_Pos) | ADC_CCR_TSEN;
  ADC3->CR &= ~ADC_CR_DEEPPWD;           /* exit deep power-down */
  ADC3->CR |= (3UL << 8);                /* BOOST maxed (safe on all revs) */
  ADC3->CR |= ADC_CR_ADVREGEN;           /* enable ADC regulator */
  for (g = 0; g < 30000U; g++) { __NOP(); }   /* wait for regulator to settle */
  ADC3->CR &= ~ADC_CR_ADCALDIF;          /* single-ended calibration */
  ADC3->CR |= ADC_CR_ADCAL;
  g = 0; while ((ADC3->CR & ADC_CR_ADCAL) && (++g < 2000000U)) {}
  ADC3->CFGR &= ~ADC_CFGR_RES_Msk;       /* 16-bit (RES=0) */
  ADC3->PCSEL |= (1UL << 18);            /* H7: pre-select internal channel 18 (temperature) */
  ADC3->SMPR2 |= (7UL << (3U * (18U - 10U)));  /* ch18 sampling 810.5 cycles (longest, temperature signal is slow) */
  ADC3->SQR1 = (18UL << ADC_SQR1_SQ1_Pos);     /* sequence length 1, 1st conversion = ch18 */
  ADC3->ISR = ADC_ISR_ADRDY;
  ADC3->CR |= ADC_CR_ADEN;
  g = 0; while (!(ADC3->ISR & ADC_ISR_ADRDY) && (++g < 2000000U)) {}
  s_temp_ok = (ADC3->ISR & ADC_ISR_ADRDY) ? 1U : 0U;
}
int app_temp_read(void)   /* returns chip temperature in °C; failure=-99 */
{
  uint32_t g = 0, raw;
  int32_t c1, c2;
  if (!s_temp_ok) { return -99; }
  ADC3->CR |= ADC_CR_ADSTART;
  while (!(ADC3->ISR & ADC_ISR_EOC) && (++g < 2000000U)) {}
  if (!(ADC3->ISR & ADC_ISR_EOC)) { return -99; }
  raw = ADC3->DR;   /* reading DR auto-clears EOC */
  raw = (raw * 2500UL) / 3300UL;   /* 757: VREF+=REF3025 2.5V but TS_CAL is calibrated at VDDA=3.3V, so scale back to 3.3V full-scale */
  c1 = (int32_t)(*TS_CAL1_ADDR); c2 = (int32_t)(*TS_CAL2_ADDR);
  if (c2 == c1) { return -99; }
  return (int)(((110 - 30) * ((int32_t)raw - c1)) / (c2 - c1) + 30);
}

/* MPU+Cache: ST's ETH template (SCB_InvalidateDCache_by_Addr etc.) assumes D-Cache is enabled;
 * the "cache off + no MPU" combination gives IMPRECISERR HardFault under sustained RX.
 * Region 0: SRAM2 (ETH descriptors/RX pool/LwIP heap/OTA ring) non-cacheable; Region 1: SRAM4 (dual-core IPC) non-cacheable. */
void app_mpu_cache_init(void)
{
  MPU_Region_InitTypeDef r = {0};
  HAL_MPU_Disable();
  r.Enable           = MPU_REGION_ENABLE;
  r.Number           = MPU_REGION_NUMBER2;  /* 757: the generated MPU_Config() takes Region0 (0x0/4GB anti-speculation template) and runs later than this function, so non-cacheable regions move to 2/3 to override the background region by higher-number priority */
  r.BaseAddress      = 0x30020000UL;
  r.Size             = MPU_REGION_SIZE_128KB;
  r.AccessPermission = MPU_REGION_FULL_ACCESS;
  r.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  r.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  r.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  r.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  r.TypeExtField     = MPU_TEX_LEVEL1;
  HAL_MPU_ConfigRegion(&r);
  r.Number      = MPU_REGION_NUMBER3;      /* same as above: Region1 also yields */
  r.BaseAddress = 0x38000000UL;
  r.Size        = MPU_REGION_SIZE_64KB;
  HAL_MPU_ConfigRegion(&r);
  /* 757 Region4: SDRAM window 0xC0000000/32MB — the generated anti-speculation template (Region0 subregion 6) sets it NO_ACCESS,
   * so the first SDRAM access = MemManage cascade crash. Normal memory WBWA cacheable (for CPU;
   * open a separate non-cacheable subwindow later when DMA lands in SDRAM) */
  r.Number       = MPU_REGION_NUMBER4;
  r.BaseAddress  = 0xC0000000UL;
  r.Size         = MPU_REGION_SIZE_32MB;
  r.IsCacheable  = MPU_ACCESS_CACHEABLE;
  r.IsBufferable = MPU_ACCESS_BUFFERABLE;
  r.TypeExtField = MPU_TEX_LEVEL1;         /* TEX=1+C+B = WBWA normal memory */
  HAL_MPU_ConfigRegion(&r);
  /* 757 Region5: BKPSRAM 0x38800000/4KB non-cacheable — if a power-fail dying write goes through D-Cache, it won't be
   * naturally evicted within the 39ms window and evaporates on power loss; battery-domain data must be write-through.
   * Access frequency is extremely low, so non-cacheable costs nothing */
  r.Number       = MPU_REGION_NUMBER5;
  r.BaseAddress  = 0x38800000UL;
  r.Size         = MPU_REGION_SIZE_4KB;
  r.IsCacheable  = MPU_ACCESS_NOT_CACHEABLE;
  r.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  r.TypeExtField = MPU_TEX_LEVEL1;
  HAL_MPU_ConfigRegion(&r);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  SCB_EnableICache();
  SCB_EnableDCache();
}

/* Peripheral patch: clear IPC ping-pong + ETH DMA region (D2 SRAM2) clock +
 * ETH global interrupt (CubeMX NVIC left unchecked, patched manually; priority must be > FreeRTOS syscall ceiling 5) */
void app_periph_init(void)
{
  __HAL_RCC_D2SRAM2_CLK_ENABLE();   /* IPC ping-pong retired, SRAM4 0x00/04 freed */
  __HAL_RCC_D2SRAM3_CLK_ENABLE();   /* OTA ring lives in SRAM3 since 2026-09-03 (app_ota.c RING_BASE) */
  HAL_NVIC_SetPriority(ETH_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(ETH_IRQn);
  RCC->CR |= RCC_CR_HSI48ON;                       /* HSI48 moved up to the main stage: USB init (start of defaultTask) runs before RNG (mqtt task); no clock = hang */
  while ((RCC->CR & RCC_CR_HSI48RDY) == 0U) { }
  /* CRS: HSI48 auto-trim against USB SOF (USB FS requires ±0.25%, bare HSI48 isn't enough; RNG isn't picky) */
  __HAL_RCC_CRS_CLK_ENABLE();
  CRS->CFGR = (CRS->CFGR & ~CRS_CFGR_SYNCSRC_Msk) | (2UL << CRS_CFGR_SYNCSRC_Pos);  /* SYNC source = USB SOF */
  CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
  ota_boot_guard();   /* OTA trial-period boot counter / repeated-crash rollback (before RTOS; see app_ota.c) */
  { extern void app_io_outputs_clear_boot(void); app_io_outputs_clear_boot(); }
  /* ^ EVERY reset clears the process-image OUTPUT area (PLC semantics, user ruling 2026-07-25):
   * outputs are a function of logic, never a memory. Must run pre-RTOS: on a warm reset the CM4
   * scanner can otherwise resume the retained table and re-energize relays from the stale image
   * before the application layer gets a say. Restore is the upper layer's explicit job. */
  app_iwdg_init();    /* start watchdog last (if guard decided to roll back it has already reset, no pointless start) */
}

/* IWDG1 independent watchdog: LSI(~32kHz)/256, RLR full-scale 4095 -> ~32.7s timeout.
 * Rationale for the 32s margin: defaultTask's longest block is a full-bank OTA erase ~8s, 4x headroom.
 * Once started the hardware can't be stopped; mqtt_app_task main loop feeds it every 500ms; debugger breakpoint freezes the counter (DBGMCU freeze) to prevent false bites.
 * It is the physical foundation of OTA trial-period "crash self-rescue": deadloop/HardFault hang -> 32s bite reset ->
 * ota_boot_guard counts -> auto rollback after 3 tries; also backstops any unknown hang in normal operation. */
void app_iwdg_init(void)
{
  DBGMCU->APB4FZ1 |= DBGMCU_APB4FZ1_DBG_IWDG1;   /* freeze watchdog counter while debug is paused */
  IWDG1->KR  = 0xCCCCU;   /* start */
  IWDG1->KR  = 0x5555U;   /* unlock PR/RLR */
  IWDG1->PR  = 6U;        /* LSI/256 */
  IWDG1->RLR = 0xFFFU;    /* 4095 -> ~32.7s */
  { uint32_t t = 1000000U; while ((IWDG1->SR != 0U) && (t > 0U)) { t--; } }   /* wait for register update to finish */
  IWDG1->KR  = 0xAAAAU;   /* reload */
}
void app_iwdg_feed(void) { IWDG1->KR = 0xAAAAU; }

/* FreeRTOS heap moved to AXI: with configAPPLICATION_ALLOCATED_HEAP=1 the application defines it;
 * heap content = task stacks/TCBs/queues, pure CPU access, no AXI+D-Cache coherency issue; heap_4 self-initializes on first alloc */
#include "FreeRTOS.h"
uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__((section(".axisram")));

/* ---- TLS infrastructure ---- */

/* mbedTLS-dedicated static memory pool: independent of newlib/FreeRTOS heap, single-threaded (tcpip) use is lock-free safe */
#include <stdio.h>
#include "mbedtls/platform.h"
#include "mbedtls/memory_buffer_alloc.h"
static uint8_t s_tls_pool[40 * 1024] __attribute__((section(".axisram")));   /* offload DTCM: pure CPU access, moved to AXI */   /* measured peak ≈30K (21K rx/tx buffers + certs + handshake temporaries); leave 8K for FreeRTOS heap (tcpip stack 8K) */

/* H7 hardware true random -> mbedTLS entropy source (MBEDTLS_ENTROPY_HARDWARE_ALT).
 * RNG kernel clock default source = hsi48, just enable HSI48 + RNG peripheral clock; direct register ops, don't touch generated HAL config */
static void rng_hw_init(void)
{
  RCC->CR |= RCC_CR_HSI48ON;
  while ((RCC->CR & RCC_CR_HSI48RDY) == 0U) { }
  __HAL_RCC_RNG_CLK_ENABLE();
  RNG->CR |= RNG_CR_RNGEN;
}
int mbedtls_hardware_poll(void *data, unsigned char *output, size_t len, size_t *olen)
{
  (void)data;
  size_t got = 0;
  while (got < len)
  {
    uint32_t t = 1000000U;
    while (((RNG->SR & RNG_SR_DRDY) == 0U) && (t > 0U)) { t--; }
    if (t == 0U) { break; }   /* hardware fault: return bytes obtained so far, mbedTLS flags insufficient entropy rather than using bad randomness */
    uint32_t r = RNG->DR;
    size_t n = ((len - got) < 4U) ? (len - got) : 4U;
    for (size_t i = 0; i < n; i++) { output[got + i] = (unsigned char)(r >> (8U * i)); }
    got += n;
  }
  *olen = got;
  return 0;
}
/* hardware random in mbedTLS f_rng form (for ecdsa_sign / anti-clone challenge); returns 0 on success */
int app_rng(void *ctx, unsigned char *buf, size_t len)
{
  size_t olen = 0;
  (void)ctx;
  if (mbedtls_hardware_poll(NULL, buf, len, &olen) != 0) { return -1; }
  return (olen == len) ? 0 : -1;
}

void app_tls_init(void)   /* called once after app_periph_init (start of mqtt_app_task) */
{
  rng_hw_init();
  mbedtls_memory_buffer_alloc_init(s_tls_pool, sizeof(s_tls_pool));
}

/* pool usage (bytes) for field forensics — MBEDTLS_MEMORY_DEBUG counters */
void app_tls_pool_usage(size_t *cur, size_t *maxu)
{
  mbedtls_memory_buffer_alloc_cur_get(cur, NULL);
  mbedtls_memory_buffer_alloc_max_get(maxu, NULL);
}

/* syscall stub for newlib time() (pulled in by MBEDTLS_HAVE_TIME): wired to the time service —
 * if synced (SNTP/RTC) give real unix time, if not synced fall back to boot-milliseconds conversion (monotonic-purpose backstop) */
#include <sys/time.h>
#include "app_time.h"
int _gettimeofday(struct timeval *tv, void *tz)
{
  (void)tz;
  if (tv != NULL)
  {
    uint32_t now = app_time_now();
    if (now != 0U)
    {
      tv->tv_sec  = (time_t)now;
      tv->tv_usec = 0;
    }
    else
    {
      uint32_t ms = HAL_GetTick();
      tv->tv_sec  = (time_t)(ms / 1000U);
      tv->tv_usec = (suseconds_t)((ms % 1000U) * 1000U);
    }
  }
  return 0;
}

/* ETH global interrupt service (vector linked by symbol name, legal to place in a user file; mqtt.c heartbeat consumes eth_irq_count) */
extern ETH_HandleTypeDef heth;
volatile uint32_t eth_irq_count = 0;
void ETH_IRQHandler(void)
{
  eth_irq_count++;
  HAL_ETH_IRQHandler(&heth);
}


/* ---- 757 production board: LAN8742 PHY reset (PI10, active low) ----
 * PI10 actively drives the LAN8742 reset, called before MX_LWIP_Init.
 * Timing: hold low >=100us (datasheet Tpurstd), after release wait >=2ms before accessing SMI. */
/* UID-derived unique MAC (used by ethernetif.c MACADDRESS user region): locally-administered address LAA (first byte 0x02,
 * U/L bit=1) = the legal range IEEE reserves for self-managed devices, zero-cost compliant — don't ride on ST's 00:80:E1 OUI
 * (that's their registered range, a self-made tail could collide with a real ST device). Trailing 5 bytes = 96-bit UID folded, no collision across the product line.
 * If production hits a strictly-managed network (802.1X allowlist class), enable the buy-a-range/MAC-EEPROM plan, see production checklist §three-7. */
void app_mac_from_uid(uint8_t *mac6)
{
  uint32_t w0 = HAL_GetUIDw0(), w1 = HAL_GetUIDw1(), w2 = HAL_GetUIDw2();
  uint32_t h = w0 ^ (w1 * 33U) ^ (w2 * 65599U);
  uint32_t g = w1 ^ (w2 * 2654435761U);
  mac6[0] = 0x02U;                    /* LAA | unicast */
  mac6[1] = (uint8_t)(g >> 8);
  mac6[2] = (uint8_t)g;
  mac6[3] = (uint8_t)(h >> 16);
  mac6[4] = (uint8_t)(h >> 8);
  mac6[5] = (uint8_t)h;
}

void app_phy_reset_lan8742(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOI_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_10, GPIO_PIN_RESET);
  g.Pin = GPIO_PIN_10; g.Mode = GPIO_MODE_OUTPUT_PP; g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &g);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(10);
}

/* (uxTopUsedPriority is already provided by this FreeRTOS tasks.c version, no App patch needed) */
