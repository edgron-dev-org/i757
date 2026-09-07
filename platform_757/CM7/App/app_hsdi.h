/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_hsdi.h — A7 HSDI pulse-count validation (jumper D1->D5), user file */
#ifndef APP_HSDI_H
#define APP_HSDI_H
void app_hsdi_cli(void);   /* CLI 'hsdi': TIM4 PWM -> TIM1 external count, 1s window validation */
#endif
