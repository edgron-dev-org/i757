/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* mqtt.h — external interface of the network + MQTT + OTA application task (user file, untouched by CubeMX regeneration)
 * main.c's StartDefaultTask only calls mqtt_app_task(); all other details live in mqtt.c */
#ifndef APP_MQTT_H
#define APP_MQTT_H
#include <stdint.h>

/* IPC ping-pong retired: CM4 health now uses app_rpc_alive (RPMsg heartbeat) */

void mqtt_app_task(void);              /* application main loop, never returns; called by defaultTask (LwIP already init'd) */
void mqtt_broker_select(uint8_t lan);  /* force broker switch (0=public 1=LAN); ota.c command entry, tcpip thread context */
void mqtt_mark_dirty(void);            /* report-on-change: call when a key/LED/command changed state, reports immediately on the next tick (<=500ms) */
uint8_t mqtt_cloud_ok(void);           /* cloud link healthy: MQTT connected + >=3 heartbeats sent this boot (criterion for OTA trial promotion) */

#endif
