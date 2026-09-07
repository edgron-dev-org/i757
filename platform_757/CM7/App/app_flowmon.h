/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_flowmon.h — per-gutter feed-flow monitor: blocked-line / lost-supply alarm.
 * User application module: see app_flowmon.c for the rule and the alarm plumbing. */
#ifndef APP_FLOWMON_H
#define APP_FLOWMON_H
#include <stdint.h>

void app_flowmon_init(void);   /* call from app_user_init(), after app_hsdi_setup() */

/* "flow" command, same line format from CLI and cloud (dn/cmd), result via out/cmdr:
 *   flow | flow stat        -> status (rates, alarms, settings)
 *   flow th <L/min>         -> low-flow threshold, e.g. "flow th 1.2" (persisted)
 *   flow sus <seconds>      -> how long the flow must stay low before the alarm (persisted)
 *   flow beep on|off        -> chirp the on-board buzzer while an alarm is active (persisted)
 *   flow ack                -> silence the buzzer for the current alarms
 *   flow on|off             -> enable / disable the monitor (persisted)
 * Returns 1 if the line was a flow command (out filled), 0 otherwise. */
int app_flowmon_cmd(const char *line, const char *src, char *out, uint16_t cap);
int app_flowmon_cli(char *line);   /* CLI hook (app_cli.c pattern): prints the result */

uint8_t app_flowmon_alarms(void);  /* heartbeat "ga": bit0..3 = gutter 1..4 low, bit4 = all low (supply) */

#endif /* APP_FLOWMON_H */
