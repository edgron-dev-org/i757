/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_bsp.h — 757 board LED shim (minimal replacement for the ST BSP LED header; the App
 * directory is early on the include path). LD1 green = PG10 is driven by
 * app_leds_beep.c; lets the ported App layer compile with zero changes. */
#ifndef APP_BSP_H
#define APP_BSP_H

#include <stdint.h>

typedef enum { LED_GREEN = 0 } Led_TypeDef;

/* LEDs implemented in app_leds_beep.c (LD1 green = PG10 = run heartbeat) */
int32_t BSP_LED_Init(Led_TypeDef l);
int32_t BSP_LED_DeInit(Led_TypeDef l);
int32_t BSP_LED_On(Led_TypeDef l);
int32_t BSP_LED_Off(Led_TypeDef l);
int32_t BSP_LED_Toggle(Led_TypeDef l);

#endif
