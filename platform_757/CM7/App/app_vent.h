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
 *   vent 0..3      -> manual fixed stage (0 = closed); 0..4 (0..5) once `vent fan 1` (`2`)
 *   vent fan off|on|1|2 [onC offC]
 *                  -> how many fan speeds are wired (O12 low / O13 high; "on" = 1) and the
 *                     fan on/off thresholds in °C (default 29.0 / 26.0); persisted (vent.cfg)
 *   vent night on|off [whiC wloC outminC airminC]
 *                  -> night cooling: water-driven floor under the window stage (default on,
 *                     22.0 / 20.0 / 12.0 / 15.0 = water start / water stop / outdoor frost
 *                     guard / air chill guard); persisted (vent.cfg line 2). Windows only.
 *   vent off       -> release: logic stops driving the coils (dashboard has them)
 * Returns 1 if the line was a vent command (out filled), 0 otherwise. */
int app_vent_cmd(const char *line, const char *src, char *out, uint16_t cap);
int app_vent_cli(char *line);   /* CLI hook (app_cli.c pattern): prints the result */

uint8_t     app_vent_stage(void);  /* TRUE stage 0..5 from the relay coil read-back (heartbeat/datalog); 4/5 = fan */
const char *app_vent_mode(void);   /* "auto" | "manual" | "off" */
const char *app_vent_night(void);  /* night cooling: "on" | "rain" | "idle-water" | "idle-cold" | "idle-outdoor" |
                                    * "arming" | "sensor" | "off" | "n/a" (not in auto) — heartbeat "ventn" */

#endif /* APP_VENT_H */
