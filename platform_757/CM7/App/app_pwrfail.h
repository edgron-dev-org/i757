/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_pwrfail.h — Three-layer power-fail retention API. Measured: warning window ~38.6ms /
 * BKPSRAM 4KB snapshot 50µs / blackbox page write ~410µs per page.
 * Contract: save area = last 2MB of SPI Flash (app_qflash.h QFLASH_PF_*), no erase, no file system, platform pre-erases slots in rotation.
 *
 * How to use the three layers (application view; the application developer fills in real content):
 *  Layer 0: Power-fail retained variables —— add the PF_RETAIN attribute to a variable to place it in the battery domain (4KB, held by CR1220 across power loss):
 *              PF_RETAIN static uint32_t total_dosed_ml;
 *          On power-up first ask app_pf_retain_valid(): 0 = first boot / battery replaced, platform has already zeroed the whole region, application must initialize.
 *  Layer 1: Dying callback —— app_pf_hook_register(fn): called inside the power-fail interrupt, copies the volatile live state into PF_RETAIN
 *          variables (4KB can be written hundreds of times within the window). Iron rule: inside the callback no RTOS/printf/peripheral restart, only write memory.
 *  Layer 2: Black box —— app_pf_blackbox_set(buf,len): register a resident buffer (≤8KB); at end-of-life the platform
 *          automatically moves it into a flash save slot (with sequence number / µs timestamp / checksum, slot rotation + background pre-erase);
 *          on power-up app_pf_blackbox_last() retrieves the most recent record.
 */
#ifndef APP_PWRFAIL_H
#define APP_PWRFAIL_H
#include <stdint.h>

#define PF_RETAIN __attribute__((section(".bkpsram")))

#define APP_PF_BLACKBOX_MAX  8192U     /* Committed spec: safe upper bound under the 20ms window floor (measured 24KB@39ms) */

void    app_pf_init(void);             /* defaultTask once (after QSPI) */
void    app_pf_poll(void);             /* defaultTask every tick: background pre-erase of next slot (at most 1 sector erased per tick) */

uint8_t app_pf_battery_ok(void);       /* Battery present (RTC BKP0R magic survives across power loss) */
uint8_t app_pf_retain_valid(void);     /* Layer 0 retention region valid this power-up; 0 = platform already zeroed it (first boot / battery replaced) */

typedef void (*app_pf_hook_t)(void);
int     app_pf_hook_register(app_pf_hook_t fn);          /* ≤8; 0=OK */

int     app_pf_blackbox_set(const void *buf, uint16_t len);   /* Register the end-of-life transfer buffer; 0=OK */
int     app_pf_blackbox_last(void *dst, uint16_t cap);        /* Returns record length; 0 = no record */
uint32_t app_pf_last_event_us(void);   /* Most recent power-fail event: detect→blackbox flush complete µs (0 = none) */

int     app_pf_cli(char *line);        /* pfstat / pftest (dry run) / pfreport (post-mortem) */

#endif
