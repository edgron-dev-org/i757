/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_mbcfg.h — generic Modbus port config (CM7 side: littlefs persistence + op8 push + shared CLI/cloud syntax)
 * Contract: docs/Universal_Modbus_Port_Config.md + docs/Inter_Core_RPMsg_Protocol.md v1.3 (op8/0A/0B) */
#ifndef APP_MBCFG_H
#define APP_MBCFG_H
#include <stdint.h>

void app_mbcfg_poll(void);                 /* defaultTask every tick: first-load push / cloud command queue / TCP service */
int  app_mbcfg_cli(char *line);            /* shared CLI/cloud syntax entry ("mbcfg ..."/"mbpoll ..."); 1=handled */
void app_mbcfg_cloud(const char *line);    /* enqueue cloud cmd (tcpip-thread safe: only copies string + raises flag) */
void app_mbcfg_hb(char *out, int cap);     /* heartbeat mb field: compact string like "a:S1@9600 tcp:on" */
uint8_t app_mbcfg_tcp_on(void);            /* TCP server toggle (queried by app_mbtcp) */

#endif
