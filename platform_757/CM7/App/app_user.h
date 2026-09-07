/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_user.h — YOUR application entry point.
 *
 * The platform calls app_user_init() exactly once at startup, after every platform
 * service is up (network, cloud if enabled, RPMsg to CM4, Modbus, time, CLI) and
 * just before the platform enters its own main loop. Inside app_user_init() you
 * create your OWN FreeRTOS task(s) — see app_user.c for a worked example.
 *
 * Your tasks are real threads: they may block (vTaskDelay / queues / semaphores)
 * and run at any period. Create as many as you need.
 */
#ifndef APP_USER_H
#define APP_USER_H

/* Suggested priorities for application tasks (FreeRTOS numeric; higher = more urgent).
 * The platform runs its network/watchdog task at CMSIS osPriorityNormal, which sits
 * ABOVE all three values below — so keeping your tasks here guarantees the watchdog
 * is always fed. If you raise a task higher, make sure it calls vTaskDelay() in its
 * loop so it never starves the platform. */
#define APP_TASK_PRIO_LOW     2
#define APP_TASK_PRIO_NORMAL  8
#define APP_TASK_PRIO_HIGH    16

void app_user_init(void);   /* implemented in app_user.c; the platform calls it once at startup */

#endif /* APP_USER_H */
