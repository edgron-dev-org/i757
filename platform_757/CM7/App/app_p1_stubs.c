/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_p1_stubs.c — 757 remaining stubs (replaced by real implementations phase by phase)
 * CLI: real implementation in place——cli stub removed
 * RPMsg: P2① real implementation (app_rpc.c), stub removed
 * mbport family: P2② real implementation (app_mbport.c + CM4 app_mbport_cm4.c), stub removed
 * CAN selftest: FDCAN belongs to P3; HSDI belongs to P4 */
#include <stdint.h>

int  app_can_selftest(int *total) { if (total != 0) { *total = 0; } return 0; }
