/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_rpc.h — M7-side RPMsg host interface (⚠️ not wired into the build, see Common/openamp/Integration_Guide.md) */
#ifndef APP_RPC_H
#define APP_RPC_H
#include <stdint.h>

void     app_rpc_boot_scrub(void); /* call in main() before waking CM4: scrub the stale SRAM4 resource table (boot-race root fix) */
int      app_rpc_init(void);    /* start of mqtt_app_task; blocks waiting for the CM4 endpoint to be ready */
void     app_rpc_poll(void);    /* pump messages every main-loop tick */
uint16_t app_rpc_transact(const uint8_t *req, uint16_t len,
                          uint8_t *rsp, uint16_t cap, uint32_t timeout_ms);
void     app_rpc_cli(void);     /* CLI 'rpc' echo test */
uint8_t  app_rpc_alive(void);   /* CM4 health: an RPMsg reply within 15s (replaces the retired IPC ping-pong) */
void     app_rpc_hb(void);      /* tiny heartbeat transaction (mqtt main loop every 5s) */
void     app_rpc_led_sync(uint8_t green_on); /* op=6 fire-and-forget: green toggle tells CM4, yellow locks to inverse */

#endif
