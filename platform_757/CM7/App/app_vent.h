/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_vent.h — greenhouse temperature control: three-stage roof window + two-speed fan.
 * User application module: see app_vent.c for the control law and wiring contract. */
#ifndef APP_VENT_H
#define APP_VENT_H
#include <stdint.h>

void app_vent_init(void);   /* call from app_user_init(), AFTER app_io_setup() */

/* "vent" command, same line format from CLI and cloud (dn/cmd), result via out/cmdr:
 *   vent           -> status (mode, stage/max, fan enable, air temp, slope)
 *   vent auto      -> automatic ladder (default at boot)
 *   vent 0..3      -> manual fixed stage (0 = closed); 0..5 once `vent fan on`
 *   vent fan on|off-> enable the fan stages 4/5 (O12 low / O13 high); persisted, default off
 *   vent off       -> release: logic stops driving the coils (dashboard has them)
 * Returns 1 if the line was a vent command (out filled), 0 otherwise. */
int app_vent_cmd(const char *line, const char *src, char *out, uint16_t cap);
int app_vent_cli(char *line);   /* CLI hook (app_cli.c pattern): prints the result */

uint8_t     app_vent_stage(void);  /* TRUE stage 0..5 from the relay coil read-back (heartbeat/datalog); 4/5 = fan */
const char *app_vent_mode(void);   /* "auto" | "manual" | "off" */

#endif /* APP_VENT_H */
