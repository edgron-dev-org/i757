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
  PT_DI = 0,      /* 12 digital inputs      485A addr 1 */
  PT_AI,          /*  8 analog inputs       485A addr 2 */
  PT_DO,          /*  8 digital outputs     485A addr 1 */
  PT_AO,          /*  8 analog outputs      485A addr 3 */
  PT_RELAY,       /* 16 relay outputs       backplane addr 2 */
  PT_RELAY_RB,    /* 16 relay read-back     backplane addr 2 */
  PT_PHEC,        /*  8 PH_EC eng values    backplane addr 4 (vent control reads t2) */
  PT_COUNT        /* keep last */
};

#endif /* APP_USER_POINTS_H */
