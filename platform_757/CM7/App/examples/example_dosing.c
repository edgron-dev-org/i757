/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* examples/example_dosing.c — the fertilizer-dosing example from the Software Manual §4.4.
 *
 * This file IS the manual example, compiled with every firmware build so it can never rot:
 * if the manual's code stops matching the platform API, the BUILD breaks — the mismatch is
 * caught by the compiler, not by a customer. (The first version of this example was paper-only
 * and shipped with a double-counting bug for exactly that reason.)
 * Manual and file are two copies of the same code — edit them together.
 *
 * It is NOT wired into the running firmware: the two #defines below namespace its entry points
 * so they cannot collide with your real app_user.c. To actually RUN it, copy the body into your
 * app_user.c (or remove the #defines and drop the real app_user_init).
 */
#define app_user_init    example_dosing_init      /* namespacing only — the code below is verbatim */
#define app_start_dosing example_dosing_start

/* -------- manual §4.4 begins here (keep byte-identical to the manual) -------- */
/* app_user.c — dose a target volume, keep the cumulative total across power loss.
 * Reads a flow meter over Modbus (485B) and drives a pump relay on a backplane relay module. */
#include "app_user.h"
#include "app_platform.h"
#include "FreeRTOS.h"
#include "task.h"

PF_RETAIN static uint32_t g_total_dosed_ml;   /* survives power loss (battery-backed, app_pwrfail.h) */
static volatile uint32_t  s_target_ml;
static volatile uint8_t   s_running;

static void dosing_task(void *arg)
{
    (void)arg;
    if (!app_pf_retain_valid()) { g_total_dosed_ml = 0; }   /* first boot / battery replaced */
    for (;;)
    {
        if (s_running)
        {
            uint16_t flow[1];
            static uint16_t last_raw;
            static uint8_t  have_raw;
            /* read the flow meter's RUNNING TOTAL (holding reg 0, slave addr 1 on 485B) and
             * accumulate the DELTA between polls — adding the meter's total on every poll
             * would count the same volume again and again. u16 subtraction survives the
             * meter's register wrapping. */
            if (app_mb_read_holding(APP_PORT_485B, 1, 0x0000, 1, flow) == 0)
            {
                uint16_t delta = (uint16_t)(flow[0] - last_raw);
                last_raw = flow[0];
                if (!have_raw) { have_raw = 1; delta = 0; }   /* first read: baseline only */
                g_total_dosed_ml += delta;          /* cumulative total, auto-persisted */
                if (g_total_dosed_ml >= s_target_ml)
                {
                    /* stop: pump relay off — coil 0 of a relay module on the backplane at DIP addr 2 */
                    app_mb_write_coil(APP_PORT_BACKPLANE, 2, 0, 0);
                    s_running = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));             /* poll 5×/s — your choice */
    }
}

/* call this from the CLI, a cloud command, or another task */
void app_start_dosing(uint32_t target_ml)
{
    s_target_ml = target_ml;
    app_mb_write_coil(APP_PORT_BACKPLANE, 2, 0, 1);  /* start: pump relay on */
    s_running = 1;
}

void app_user_init(void)
{
    app_mbport_configure(APP_PORT_485B, APP_MB_MASTER, 9600, 'N', 1, 0);   /* flow-meter port */
    xTaskCreate(dosing_task, "dose", 1024, NULL, APP_TASK_PRIO_NORMAL, NULL);
}
/* -------- manual §4.4 ends here -------- */
