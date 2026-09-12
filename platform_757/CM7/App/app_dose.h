/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_dose.h — nutrient/pH dosing control (four peristaltic pumps on the relay module).
 * User application module: see app_dose.c for the control law and wiring contract. */
#ifndef APP_DOSE_H
#define APP_DOSE_H
#include <stdint.h>

void app_dose_init(void);   /* call from app_user_init(), AFTER app_io_setup() */

/* "dose" command, same line format from CLI and cloud (dn/cmd), result via out/cmdr:
 *   dose                 -> status (mode, readings, targets, today's totals)
 *   dose a|b|acid|base <ml>  -> manual timed shot (flow calibration applies)
 *   dose ab <ml>         -> A then B sequentially, <ml> each (the equal-parts SOP)
 *   dose stop            -> stop pumps now, clear queue, mode -> off
 *   dose auto            -> closed loop ON (feed-forward + proportional, self-learning: see .c)
 *   dose off             -> closed loop off; pumps held OFF
 *   dose release         -> release the four coils to the dashboard
 *   dose cal a|b|acid|base <ml_per_min>   -> pump flow calibration
 *   dose ec <uS> [db] | dose ph <x100> [db] | dose mix <min>  -> targets, decision cycle
 *   dose shot acid|base <ml>              -> per-cycle auto shot caps
 *   dose tank <L>        -> reservoir volume for the top-up feed-forward (water meter -> A/B while filling)
 *   dose learn [reset]   -> learned gains/drifts (persisted in littlefs "dose.lrn")
 * Returns 1 if the line was a dose command (out filled), 0 otherwise. */
int app_dose_cmd(const char *line, const char *src, char *out, uint16_t cap);
int app_dose_cli(char *line);     /* CLI hook (app_cli.c pattern): prints the result */

const char *app_dose_hb(void);    /* short status string for the heartbeat "dose" field */

#endif /* APP_DOSE_H */
