/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_p0_cm4.c — CM4 main loop (IWDG2 + heartbeat + RPMsg + backplane scheduling + front-panel Modbus ports); 1ms tick. */
#include "main.h"
#include "cmsis_os.h"

typedef struct { volatile uint32_t magic, cm7, cm4; } p0_hb_t;
#define P0_HB    ((p0_hb_t *)0x38000180UL)   /* since P1 placed at 0x180: 0x00=IPC ping-pong / 0x100=OTA guard makes way */
#define P0_MAGIC 0x50303735UL

static IWDG_HandleTypeDef s_iwdg2;

void app_p0_cm4_task(void)
{
  uint32_t t;

  P0_HB->cm4 = 0;

  s_iwdg2.Instance       = IWDG2;
  s_iwdg2.Init.Prescaler = IWDG_PRESCALER_256;
  s_iwdg2.Init.Reload    = 4095;                /* ~32.7s */
  s_iwdg2.Init.Window    = IWDG_WINDOW_DISABLE;
  (void)HAL_IWDG_Init(&s_iwdg2);

  { extern int app_rpc_cm4_init(void); (void)app_rpc_cm4_init(); }   /* P2①: RPMsg remote endpoint (guide §4) */
  { extern void bus_cm4_start(void); bus_cm4_start(); }              /* one thread per Modbus bus */
  { extern void app_user_cm4_init(void); app_user_cm4_init(); }      /* YOUR application on this core */
  t = HAL_GetTick();
  for (;;)
  {
    /* Every Modbus bus now runs in its own thread (app_bus_cm4.c): scanning, slave replies and
     * uart heal all live there, so one unreachable device can no longer stall the others. What
     * stays here is only work that never blocks. */
    { extern void hsdi_cm4_poll(void); hsdi_cm4_poll(); }                /* onboard HSDI: extend counters, publish the image */
    (void)HAL_IWDG_Refresh(&s_iwdg2);
    if ((HAL_GetTick() - t) >= 500U) { t += 500U; P0_HB->cm4++; }
    osDelay(1);                                  /* 1ms tick: slave-reply real-time responsiveness (resource table §1) */
  }
}
