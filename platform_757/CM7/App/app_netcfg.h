/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_netcfg.h — runtime network config (littlefs /net.cfg).
 * Default = DHCP, and the addresses here are used only when DHCP times out (fallback logic in
 * app_mqtt.c). `netcfg static` turns DHCP off and uses them always. File absent = compile-time
 * defaults (app_cfg.h), so factory behavior is unchanged until somebody runs `netcfg`. */
#ifndef APP_NETCFG_H
#define APP_NETCFG_H

#include <stdint.h>

void        app_netcfg_load(void);   /* idempotent; defaultTask context (littlefs access) */
const char *app_netcfg_ip(void);
const char *app_netcfg_mask(void);
const char *app_netcfg_gw(void);
const char *app_netcfg_dns(void);
uint8_t     app_netcfg_static(void); /* 1 = static mode (DHCP not used) */
uint8_t     app_netcfg_ready(void);  /* 1 = the file system is mounted, so load() sees the stored file */

/* 1 exactly once after a successful set/default (consumer: app_mqtt main loop —
 * applies static mode / returns to DHCP / re-applies the fallback, and reconnects). */
uint8_t app_netcfg_take_dirty(void);

/* Shared CLI/cloud syntax: "netcfg" | "netcfg [static] <ip> <mask> <gw> [dns]" | "netcfg static" |
 * "netcfg dhcp" | "netcfg default".
 * Returns 1 if the line was a netcfg command (result text in out), 0 otherwise. */
int app_netcfg_cmd(const char *line, char *out, uint16_t cap);
int app_netcfg_cli(char *line);      /* CLI passthrough wrapper: prints the result */

/* Modbus TCP holding registers 0x3200..0x3209 (FC03/06/16), called from app_peek_mbtcp.
 * wr_ok = writes allowed right now. Returns the reply PDU length. */
uint16_t app_netcfg_mbtcp(const uint8_t *pdu, uint16_t len, uint8_t *rsp, uint16_t cap, int wr_ok);

#endif /* APP_NETCFG_H */
