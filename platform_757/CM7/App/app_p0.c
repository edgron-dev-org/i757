/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_p0.c — P0 heartbeat leftover (from P1 on, downgraded to a tick function called each beat by the app_mqtt main loop)
 * SRAM4 address convention (boundary rules §4): 0x00/0x04=IPC ping-pong, 0x100=OTA guard, **0x180=P0 heartbeat**, 0x200=OPENAMP.
 * CM4's 485_3 CLI `stat` reads this block. IWDG1 belongs to app_init (already in nucleo), this file no longer manages it. */
#include "main.h"

typedef struct { volatile uint32_t magic, cm7, cm4; } p0_hb_t;
#define P0_HB    ((p0_hb_t *)0x38000180UL)
#define P0_MAGIC 0x50303735UL

void app_p0_hb_tick(void)
{
  P0_HB->magic = P0_MAGIC;
  P0_HB->cm7++;
}
