/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_time.h — time service (RTC + SNTP), user file, untouched by CubeMX regeneration
 * Architecture: RAM anchor as primary (epoch at sync moment + HAL_GetTick delta extrapolation), RTC (LSE bare registers) provides reset continuation——
 * NVIC reset/OTA swap does not lose time; re-sync via SNTP at boot.
 * Downstream: heartbeat time field / TLS timestamp (_gettimeofday) / backplane Modbus broadcast time sync (FC16 writes 0x0000~02). */
#ifndef APP_TIME_H
#define APP_TIME_H
#include <stdint.h>

void     app_time_init(void);              /* call once at the start of mqtt_app_task: bring up LSE+RTC, seed anchor if RTC has a calendar */
void     app_time_sntp_cb(unsigned long s);/* SNTP callback (tcpip thread, no printf): set anchor + write RTC + trigger report-on-change */
uint32_t app_time_now(void);               /* current unix seconds; 0=never synced */
uint8_t  app_time_source(void);            /* 0=not synced 1=RTC continuation 2=SNTP synced */

#endif
