/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_aer.h — reservoir aeration: an air pump switched on water temperature.
 * User application module: see app_aer.c for the control law and wiring contract. */
#ifndef APP_AER_H
#define APP_AER_H
#include <stdint.h>

void app_aer_init(void);   /* call from app_user_init(), AFTER app_io_setup() */

/* "aer" command, same line format from CLI and cloud (dn/cmd), result via out/cmdr:
 *   aer                -> status (mode, pump, water temp, setpoint, hysteresis)
 *   aer auto           -> temperature law (default at boot)
 *   aer on | aer off   -> force the pump (RAM only; a reboot returns to auto)
 *   aer rel            -> release: logic stops driving the coil (dashboard has it)
 *   aer set <C> [<C>]  -> setpoint [hysteresis] in degC, one decimal ("aer set 22.5 1");
 *                         persisted (littlefs "aer.cfg"); setpoint 0 = always on in auto
 * Returns 1 if the line was an aer command (out filled), 0 otherwise. */
int app_aer_cmd(const char *line, const char *src, char *out, uint16_t cap);
int app_aer_cli(char *line);    /* CLI hook (app_cli.c pattern): prints the result */

uint8_t     app_aer_state(void);   /* TRUE pump state from the relay coil read-back (heartbeat/datalog) */
const char *app_aer_mode(void);    /* "auto 22.0C hyst 1.0" | "on" | "off" | "rel" */

#endif /* APP_AER_H */
