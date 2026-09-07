/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_netcfg.h — runtime-configurable static-IP fallback (littlefs /net.cfg).
 * DHCP remains the preferred path; these values are used only when DHCP times out
 * (fallback logic in app_mqtt.c). File absent = compile-time defaults (app_cfg.h),
 * so factory behavior is unchanged until somebody runs `netcfg`. */
#ifndef APP_NETCFG_H
#define APP_NETCFG_H

#include <stdint.h>

void        app_netcfg_load(void);   /* idempotent; defaultTask context (littlefs access) */
const char *app_netcfg_ip(void);
const char *app_netcfg_mask(void);
const char *app_netcfg_gw(void);
const char *app_netcfg_dns(void);

/* 1 exactly once after a successful set/default (consumer: app_mqtt main loop —
 * re-applies the address and reconnects if the board is sitting in fallback). */
uint8_t app_netcfg_take_dirty(void);

/* Shared CLI/cloud syntax: "netcfg" | "netcfg <ip> <mask> <gw> [dns]" | "netcfg default".
 * Returns 1 if the line was a netcfg command (result text in out), 0 otherwise. */
int app_netcfg_cmd(const char *line, char *out, uint16_t cap);
int app_netcfg_cli(char *line);      /* CLI passthrough wrapper: prints the result */

#endif /* APP_NETCFG_H */
