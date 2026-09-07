/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_init.h — CM7 board-level init and firmware identity (user file, untouched by CubeMX regeneration)
 * main.c only places one call each inside its USER CODE blocks; all implementation is in app_init.c */
#ifndef APP_INIT_H
#define APP_INIT_H
#include <stddef.h>
#include <stdint.h>

void app_mpu_cache_init(void);  /* call first in main() (USER CODE 1): MPU non-cacheable regions + I/D-Cache, must precede all peripherals */
void app_periph_init(void);     /* call after MX_GPIO_Init (USER CODE 2): IPC clear + SRAM2 clock + ETH NVIC manual patch
                                   + OTA trial-period boot check (ota_boot_guard) + IWDG start */
void app_iwdg_init(void);       /* IWDG1 32s watchdog (cannot be stopped once started in hardware); mqtt_app_task feeds it every 500ms */
void app_iwdg_feed(void);
void app_tls_init(void);        /* call once at the start of mqtt_app_task: hardware RNG + mbedTLS static memory pool */
int  app_rng(void *ctx, unsigned char *buf, size_t len);  /* hardware random in mbedTLS f_rng form */

void    app_temp_init(void);       /* initialize ADC3 internal temperature sensor */
int     app_temp_read(void);       /* read chip temperature in °C (failure=-99) */

#endif
