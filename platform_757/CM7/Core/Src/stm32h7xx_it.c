/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_i2c3_rx;
extern DMA_HandleTypeDef hdma_i2c3_tx;
extern I2C_HandleTypeDef hi2c3;
extern DMA_HandleTypeDef hdma_lpuart1_rx;
extern UART_HandleTypeDef hlpuart1;
extern MDMA_HandleTypeDef hmdma_quadspi_fifo_th;
extern QSPI_HandleTypeDef hqspi;
extern SD_HandleTypeDef hsd1;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  { volatile uint32_t *bb = (volatile uint32_t *)0x3800E440UL; uint32_t psp = __get_PSP(); uint32_t *sp = (uint32_t *)psp;
    bb[0] = 0xFA017CB7UL; bb[1] = 1U; bb[2] = *(volatile uint32_t *)0xE000ED28UL; bb[3] = *(volatile uint32_t *)0xE000ED2CUL;
    bb[4] = sp[6]; bb[5] = psp; bb[6] = *(volatile uint32_t *)0xE000ED38UL; bb[7] = *(volatile uint32_t *)0xE000ED34UL;
    { extern volatile uint32_t g_fault_bb2[8]; for (int i = 0; i < 8; i++) { g_fault_bb2[i] = bb[i]; } }   /* battery mirror (2026-09-03: SRAM4 copy got clobbered before the journal read it) */
    __DSB(); *(volatile uint32_t *)0xE000ED0CUL = 0x05FA0004UL; }
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  { volatile uint32_t *bb = (volatile uint32_t *)0x3800E440UL; uint32_t psp = __get_PSP(); uint32_t *sp = (uint32_t *)psp;
    bb[0] = 0xFA017CB7UL; bb[1] = 2U; bb[2] = *(volatile uint32_t *)0xE000ED28UL; bb[3] = *(volatile uint32_t *)0xE000ED2CUL;
    bb[4] = sp[6]; bb[5] = psp; bb[6] = *(volatile uint32_t *)0xE000ED38UL; bb[7] = *(volatile uint32_t *)0xE000ED34UL;
    { extern volatile uint32_t g_fault_bb2[8]; for (int i = 0; i < 8; i++) { g_fault_bb2[i] = bb[i]; } }   /* battery mirror (2026-09-03: SRAM4 copy got clobbered before the journal read it) */
    __DSB(); *(volatile uint32_t *)0xE000ED0CUL = 0x05FA0004UL; }
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  { volatile uint32_t *bb = (volatile uint32_t *)0x3800E440UL; uint32_t psp = __get_PSP(); uint32_t *sp = (uint32_t *)psp;
    bb[0] = 0xFA017CB7UL; bb[1] = 3U; bb[2] = *(volatile uint32_t *)0xE000ED28UL; bb[3] = *(volatile uint32_t *)0xE000ED2CUL;
    bb[4] = sp[6]; bb[5] = psp; bb[6] = *(volatile uint32_t *)0xE000ED38UL; bb[7] = *(volatile uint32_t *)0xE000ED34UL;
    { extern volatile uint32_t g_fault_bb2[8]; for (int i = 0; i < 8; i++) { g_fault_bb2[i] = bb[i]; } }   /* battery mirror (2026-09-03: SRAM4 copy got clobbered before the journal read it) */
    __DSB(); *(volatile uint32_t *)0xE000ED0CUL = 0x05FA0004UL; }
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  { volatile uint32_t *bb = (volatile uint32_t *)0x3800E440UL; uint32_t psp = __get_PSP(); uint32_t *sp = (uint32_t *)psp;
    bb[0] = 0xFA017CB7UL; bb[1] = 4U; bb[2] = *(volatile uint32_t *)0xE000ED28UL; bb[3] = *(volatile uint32_t *)0xE000ED2CUL;
    bb[4] = sp[6]; bb[5] = psp; bb[6] = 0U; bb[7] = 0U;
    { extern volatile uint32_t g_fault_bb2[8]; for (int i = 0; i < 8; i++) { g_fault_bb2[i] = bb[i]; } }   /* battery mirror */
    __DSB(); *(volatile uint32_t *)0xE000ED0CUL = 0x05FA0004UL; }
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c3_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c3_tx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles SDMMC1 global interrupt.
  */
void SDMMC1_IRQHandler(void)
{
  /* USER CODE BEGIN SDMMC1_IRQn 0 */

  /* USER CODE END SDMMC1_IRQn 0 */
  HAL_SD_IRQHandler(&hsd1);
  /* USER CODE BEGIN SDMMC1_IRQn 1 */

  /* USER CODE END SDMMC1_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1_CH1 and DAC1_CH2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles I2C3 event interrupt.
  */
void I2C3_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_EV_IRQn 0 */

  /* USER CODE END I2C3_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_EV_IRQn 1 */

  /* USER CODE END I2C3_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C3 error interrupt.
  */
void I2C3_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_ER_IRQn 0 */

  /* USER CODE END I2C3_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_ER_IRQn 1 */

  /* USER CODE END I2C3_ER_IRQn 1 */
}

/**
  * @brief This function handles QUADSPI global interrupt.
  */
void QUADSPI_IRQHandler(void)
{
  /* USER CODE BEGIN QUADSPI_IRQn 0 */

  /* USER CODE END QUADSPI_IRQn 0 */
  HAL_QSPI_IRQHandler(&hqspi);
  /* USER CODE BEGIN QUADSPI_IRQn 1 */

  /* USER CODE END QUADSPI_IRQn 1 */
}

/**
  * @brief This function handles USB OTG FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);   /* interrupt enabled while the handler idles = IRQ storm */
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles MDMA global interrupt.
  */
void MDMA_IRQHandler(void)
{
  /* USER CODE BEGIN MDMA_IRQn 0 */

  /* USER CODE END MDMA_IRQn 0 */
  HAL_MDMA_IRQHandler(&hmdma_quadspi_fifo_th);
  /* USER CODE BEGIN MDMA_IRQn 1 */

  /* USER CODE END MDMA_IRQn 1 */
}

/**
  * @brief This function handles BDMA channel0 global interrupt.
  */
void BDMA_Channel0_IRQHandler(void)
{
  /* USER CODE BEGIN BDMA_Channel0_IRQn 0 */

  /* USER CODE END BDMA_Channel0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_lpuart1_rx);
  /* USER CODE BEGIN BDMA_Channel0_IRQn 1 */

  /* USER CODE END BDMA_Channel0_IRQn 1 */
}

/**
  * @brief This function handles LPUART1 global interrupt.
  */
void LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN LPUART1_IRQn 0 */

  /* USER CODE END LPUART1_IRQn 0 */
  HAL_UART_IRQHandler(&hlpuart1);
  /* USER CODE BEGIN LPUART1_IRQn 1 */

  /* USER CODE END LPUART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
