/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_leds_beep.c — 757 on-board indicator LEDs + buzzer
 * LD1 dual-color (common cathode): PG10=green (run heartbeat, toggled every 500ms by mqtt main loop); the other color's drive awaits netlist update.
 * BUZZER1=MLT-8530 passive (PI0=TIM5_CH4 -> 1k -> transistor -> 5V), 2.7kHz resonant PWM.
 * Board 2 panel IND0~7 = HSDI channel activity LEDs (74LVC14 driven directly by field signals, not owned by the MCU——"never lights up" = no field signal connected, not a fault). */
#include "main.h"
#include "app_bsp.h"

/* ---- LD1 green = LED_GREEN (BSP shim reimplemented by this file) ---- */
static uint8_t s_inited = 0;
static void leds_init_once(void)
{
  GPIO_InitTypeDef g = {0};
  if (s_inited) { return; }
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
  g.Pin = GPIO_PIN_10; g.Mode = GPIO_MODE_OUTPUT_PP; g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &g);
  s_inited = 1;
}
int32_t BSP_LED_Init(Led_TypeDef l)   { (void)l; leds_init_once(); return 0; }
int32_t BSP_LED_DeInit(Led_TypeDef l) { (void)l; return 0; }
int32_t BSP_LED_On(Led_TypeDef l)
{ leds_init_once(); if (l == LED_GREEN) { HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_SET); } return 0; }
int32_t BSP_LED_Off(Led_TypeDef l)
{ leds_init_once(); if (l == LED_GREEN) { HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET); } return 0; }
int32_t BSP_LED_Toggle(Led_TypeDef l)
{ leds_init_once(); if (l == LED_GREEN) { HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_10); } return 0; }

/* ---- buzzer: TIM5_CH4 @PI0 (AF2), 2.7kHz/50% ---- */
static TIM_HandleTypeDef s_tim5;
static uint8_t s_beep = 0;
static uint8_t s_beep_inited = 0;

static void beep_init_once(void)
{
  GPIO_InitTypeDef g = {0};
  TIM_OC_InitTypeDef oc = {0};
  if (s_beep_inited) { return; }
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_TIM5_CLK_ENABLE();
  g.Pin = GPIO_PIN_0; g.Mode = GPIO_MODE_AF_PP; g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOI, &g);
  s_tim5.Instance               = TIM5;
  s_tim5.Init.Prescaler         = 88;      /* 240MHz/89 = 2.697MHz (in the 480M era APB1 timer=240M) */
  s_tim5.Init.Period            = 999;     /* /1000 = 2.70kHz (MLT-8530 resonance) */
  s_tim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
  s_tim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  (void)HAL_TIM_PWM_Init(&s_tim5);
  oc.OCMode     = TIM_OCMODE_PWM1;
  oc.Pulse      = 500;                     /* 50% */
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  (void)HAL_TIM_PWM_ConfigChannel(&s_tim5, &oc, TIM_CHANNEL_4);
  s_beep_inited = 1;
}

void app_beep_set(uint8_t on)
{
  beep_init_once();
  if (on) { (void)HAL_TIM_PWM_Start(&s_tim5, TIM_CHANNEL_4); }
  else    { (void)HAL_TIM_PWM_Stop(&s_tim5, TIM_CHANNEL_4); }
  s_beep = on ? 1U : 0U;
}
uint8_t app_beep_get(void) { return s_beep; }
