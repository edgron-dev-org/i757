/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_cm4_stubs.c — 757 remaining stubs: the real mbport is now live (P2②, app_mbport_cm4.c)
 * led_sync: the 757 board has no CM4-side dual-color LED wiring (LD1 belongs to CM7), kept as a no-op. */
#include <stdint.h>

void app_cm4_led_sync(uint8_t green_on) { (void)green_on; }
