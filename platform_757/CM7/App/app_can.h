/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_can.h — FDCAN loopback self-test (stage A5 software half), user file, untouched by CubeMX regeneration
 * Internal loopback mode drives no pins and needs no transceiver: driver / bit timing / message RAM / filter full path validated up front,
 * so when the TJA1051 module arrives only the electrical layer remains. Production board CAN1 (non-isolated) / CAN2 (isolated) share the same driver. */
#ifndef APP_CAN_H
#define APP_CAN_H

int app_can_selftest(int *total);   /* FDCAN1 internal loopback: returns failure count (0=all pass), *total=number of cases */

#endif
