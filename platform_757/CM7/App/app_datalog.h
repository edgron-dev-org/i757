/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_datalog.h — on-board data recorder + event journal (contract: docs/Data_Logging_and_Event_Journal.md)
 *
 * Platform standard feature: periodic aggregated sampling (avg/min/max) of module measurements
 * to daily CSV files, plus a five-type event journal (CONFIG/SYSTEM/FAULT/ACTION/NOTE).
 * Authoritative data lives on the board (SD card or QSPI littlefs, runtime-selectable);
 * the cloud only mirrors. Runs entirely on the mqtt main-loop 500 ms tick — no own task,
 * no filesystem access from any other context. */
#ifndef APP_DATALOG_H
#define APP_DATALOG_H
#include <stdint.h>

void app_datalog_poll(void);   /* mqtt main loop, every 500 ms tick (lazy init on first call) */

/* Application event API (SDK-facing, also declared in app_platform.h).
 * type: "ACTION" | "NOTE" (applications); the recorder itself emits CONFIG/SYSTEM/FAULT.
 * Formats into a small RAM queue (no filesystem I/O here) — safe from any task context. */
void app_log_event(const char *type, const char *fmt, ...);

/* Application sample fields (SDK-facing, also declared in app_platform.h).
 * Adds one recorded quantity: three CSV columns <name>_avg/_min/_max, sub-sampled every 5 s
 * like the module channels. get(arg, &value) returns 1 with a value, anything else = no
 * valid sample this time (the column stays empty for a window with none). name: 1..8
 * characters, lower-case letters and digits, and the same as the heartbeat field / desc "f"
 * so the dashboard labels and scales the curve by itself. Call from app_user_init, at most
 * 16 fields. Returns 0, or -1 (bad name, duplicate, table full). The getter runs on the
 * platform's main loop: read a cached value (app_io_ai), never block. */
#ifndef APP_DATALOG_GET_FN_DEFINED
#define APP_DATALOG_GET_FN_DEFINED
typedef int (*app_datalog_get_fn)(uint16_t arg, int32_t *value);
#endif
int app_datalog_add_field(const char *name, app_datalog_get_fn get, uint16_t arg);

/* Internal producers (CLI / cloud / recorder) with explicit source tag. */
void app_log_event_src(const char *type, const char *src, const char *fmt, ...);

/* Command surface, shared verbatim between CLI and the cloud dn/cmd channel:
 *   logcfg [dev=sd|flash|off] [period=N] [quota_mb=N]   logstat   note <text>   logget <file|date>
 * Returns 1 if the line was consumed (result rendered into out). */
int app_datalog_cmd(const char *line, const char *src, char *out, uint16_t cap);

const char *app_datalog_hb(void);   /* heartbeat "dlog" short status string */

#endif
