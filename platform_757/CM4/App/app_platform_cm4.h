/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_platform_cm4.h — the platform service API for YOUR application on the CM4 core.
 *
 * Include this from CM4/App/app_user_cm4.c (and any app_*.c you add on this core).
 *
 * WHY PUT ANYTHING ON CM4: this core owns the field buses and the onboard high-speed inputs,
 * so logic that must react in microseconds — interlocks, pulse-driven counting, anything that
 * must not wait behind the network stack — belongs here. Everything else belongs on CM7, which
 * has the network, the cloud, storage and the bigger RAM.
 *
 * The function names are IDENTICAL to the CM7 API (app_platform.h) and they act on the same
 * process image, so a piece of control logic can be moved between the two cores unchanged.
 * The point indexes are the row order of the scan table registered on CM7 — keep the two in
 * sync (the example marks them with a comment).
 */
#ifndef APP_PLATFORM_CM4_H
#define APP_PLATFORM_CM4_H
#include <stdint.h>

/* ---- process image: same calls, same meaning as on CM7 ---- */
uint8_t  app_io_di    (uint16_t pt, uint16_t ch);          /* input bit (coil / discrete input) */
uint16_t app_io_ai    (uint16_t pt, uint16_t ch);          /* input word (input / holding register) */
void     app_io_do_set(uint16_t pt, uint16_t ch, uint8_t on);  /* set an output coil */
uint8_t  app_io_do_get(uint16_t pt, uint16_t ch);          /* read back a commanded output coil */
void     app_io_hr_set(uint16_t pt, uint16_t ch, uint16_t v);  /* set an output holding register */
int      app_io_ok    (uint16_t pt);                       /* 1 = last scan ok, 0 = failing */

/* ---- onboard high-speed inputs (this core owns them, so these are the freshest values) ---- */
uint8_t  app_hsdi_level(uint8_t ch);
uint32_t app_hsdi_count(uint8_t ch);

/* ---- your entry point: the platform calls this once, before the CM4 main loop starts ---- */
void app_user_cm4_init(void);

#endif /* APP_PLATFORM_CM4_H */
