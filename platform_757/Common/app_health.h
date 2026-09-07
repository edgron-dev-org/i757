/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_health.h — kernel-health plumbing shared by both cores.
 *
 * Two silent failure modes are made loud here:
 *   - a task overrunning its stack, which otherwise corrupts whatever sits below it and
 *     surfaces as a hard fault nowhere near the cause;
 *   - a heap allocation returning NULL, which otherwise just means a thread quietly never
 *     starts (exactly how a per-bus thread would have vanished when the CM4 heap was too small).
 * Both hooks write a record into SRAM4, which survives the reset that follows, so the next boot
 * can name the culprit. The 'tasks' CLI prints it.
 *
 * Run-time stats need a monotonic counter per core; each core provides its own (CM7: DWT cycles
 * accumulated in 64 bits, CM4: the TIM7 HAL time base, whose DWT does not run). See the notes in
 * app_diag.c / app_diag_cm4.c — a counter that wraps corrupts the denominator, not just a sample.
 */
#ifndef APP_HEALTH_H
#define APP_HEALTH_H
#include <stdint.h>

/* SRAM4 fault records: 0x38008420/0x20, the gap between the CM4 breadcrumb and the process
 * image (docs/Hardware_Resource_Allocation.md). 16 bytes per core. */
#define HEALTH_CM7_ADDR 0x38008420UL
#define HEALTH_CM4_ADDR 0x38008430UL

#define HEALTH_MAGIC    0x484C5400UL   /* 'HLT' | type in the low byte */
#define HEALTH_NONE     0U
#define HEALTH_STACK    1U             /* stack overflow, name = offending task */
#define HEALTH_MALLOC   2U             /* heap allocation failed */

typedef struct {
  uint32_t magic;      /* HEALTH_MAGIC | type, or 0 when nothing has been recorded */
  char     name[12];   /* task that was running when it happened */
} health_rec_t;

#define HEALTH_CM7 ((volatile health_rec_t *)HEALTH_CM7_ADDR)
#define HEALTH_CM4 ((volatile health_rec_t *)HEALTH_CM4_ADDR)

/* ---- named-reset breadcrumb (2026-09-02, the silent-reset epidemic) ----
 * Every INTENTIONAL software reset must go through app_reset(reason) (CM7) or write
 * this record directly (CM4): the reason survives the reset in SRAM4 and the next
 * boot journals it. A boot that finds SFT1 set with neither this note nor the fault
 * black box (0x3800E440, stm32h7xx_it.c) is journaled UNNAMED — that in itself is
 * the clue (a reset from outside every instrumented path). One-shot: reader clears. */
#define RESETNOTE_ADDR  0x3800E460UL   /* 32 B, first free slot after the fault black box
                                        * (resource ledger: the hardware resource table in docs/) */
#define RESETNOTE_MAGIC 0x52534E42UL   /* 'RSNB' */
typedef struct {
  uint32_t magic;
  char     reason[28];   /* NUL-terminated, truncated silently */
} resetnote_t;
#define RESETNOTE ((volatile resetnote_t *)RESETNOTE_ADDR)

#endif /* APP_HEALTH_H */
