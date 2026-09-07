/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_cli.h — diagnostic CLI (overall plan module 21), user file, untouched by CubeMX regeneration
 * Uses the idle RX of the ST-LINK VCP (USART3 = BSP COM1): ISR pushes chars into the ring, defaultTask fetches / echoes / executes.
 * Command set: help / stat / time / ota / mb / reboot / revert yes */
#ifndef APP_CLI_H
#define APP_CLI_H

void app_cli_init(void);   /* called once at the start of mqtt_app_task: enable USART3 RXNE interrupt + print prompt */
void app_cli_poll(void);   /* called every main-loop tick (500ms): drain the ring buffer (printf only in this context) */

#endif
