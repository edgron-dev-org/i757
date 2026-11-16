/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_user_points.h — YOUR I/O point names, shared by both cores.
 *
 * This is a user file. The scan table itself lives in CM7/App/app_user.c (CM7 is the core that
 * owns the configuration); this header holds only the NAMES of its rows, so that code on either
 * core addresses a point by name instead of by a number it has to keep in sync by hand.
 *
 * WHEN YOU ADD A DEVICE: add its row to s_io_table[] in CM7/App/app_user.c and a name here, in
 * the SAME ORDER — the enum value is the row index. CM7 has a compile-time check that the two
 * lists are the same length, so forgetting one side fails the build rather than silently
 * addressing the wrong point.
 */
#ifndef APP_USER_POINTS_H
#define APP_USER_POINTS_H

enum {
  PT_AIR = 0,     /* air T/RH transmitter   485E addr 1 (FC03 0x0000: RH x10, T x10 i16; 2026-11-09) */
  PT_COCO1,       /* coco substrate probe 1 485E addr 2 (FC03 0x0000: T x10 i16, moisture x10) cucumber pot A */
  PT_COCO2,       /* coco substrate probe 2 485E addr 3, eggplant pot A */
  PT_COCO3,       /* coco substrate probe 3 485E addr 4, cucumber pot B */
  PT_COCO4,       /* coco substrate probe 4 485E addr 5, eggplant pot B */
  PT_RELAY,       /* 16 relay outputs       backplane addr 2 */
  PT_RELAY_RB,    /* 16 relay read-back     backplane addr 2 */
  PT_PHEC,        /*  8 PH_EC eng values    backplane addr 4 (vent control reads t2) */
  PT_DI16,        /* 16 discrete inputs     backplane addr 1 EX_16DI (coco line floats, 2026-10-26) */
  PT_PHEC2,       /*  8 PH_EC eng values    backplane addr 5 (coco line reservoir + runoff sample) */
  PT_COUNT        /* keep last */
};

#endif /* APP_USER_POINTS_H */
