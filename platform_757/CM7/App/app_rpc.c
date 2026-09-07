/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_rpc.c — M7-side RPMsg host (OpenAMP), user file. ⚠️ not yet wired into the build (see Common/openamp/Integration_Guide.md)
 * Architecture: M7 ←RPMsg→ M4 ←UART→ physical serial; payload = Modbus PDU + slave address.
 * Endpoint "mbus": CM4 announces the service, CM7 new_service_cb binds it; during bring-up do an echo round-trip first. */
#include "app_rpc.h"
#include <stdio.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#include "openamp.h"
#include "rsc_table.h"
#include "FreeRTOS.h"
#include "semphr.h"

static struct rpmsg_endpoint s_ept;
static SemaphoreHandle_t s_rpc_mtx = NULL;   /* serializes all OpenAMP access so application tasks can call Modbus APIs safely */

/* Boot-race root fix: the resource table lives in SRAM4 (0x38000200), the GCC startup code
 * doesn't init it, and a warm reset of SRAM4 (OTA bank swap / reboot) doesn't clear it — so the table keeps the
 * previous life's vring1.da and DRIVER_OK. CM4's two spin-waits (wait for table ready / wait for master ready) are
 * both let through by the stale values, the endpoint announcement goes into the old vring, then gets wiped by
 * CM7 MX_OPENAMP_Init's memset => CM4 thinks it already announced and never sends again, so the endpoint never binds
 * (all rpc times out, the trial period can never be promoted, and every reset walks one notch toward the rollback
 * threshold). First-to-arrive wins = a pure boot race, hence intermittent.
 * Fix: CM4 stops in STOP at boot waiting for a HSEM wake — clear the table **before** waking it (CM4 is asleep then,
 * zero race). This restores the ST handshake's designed semantics: CM4 must wait for CM7 to fill the table + set
 * DRIVER_OK before announcing, so binding becomes deterministic; as a bonus it fixes cold start (table = random garbage)
 * and "late-bind self-heal" (the announcement can no longer be lost). Call site = main.c Boot_Mode_Sequence_2. */
void app_rpc_boot_scrub(void)
{
  extern volatile struct shared_resource_table resource_table;   /* defined in Common/openamp/rsc_table.c */
  volatile uint32_t *p = (volatile uint32_t *)&resource_table;
  for (uint32_t i = 0; i < (sizeof(resource_table) / 4U); i++) { p[i] = 0U; }
}
static volatile uint16_t s_rx_len = 0;
static volatile uint32_t s_last_rx = 0;   /* timestamp of last reply (the alive criterion) */
static uint8_t s_rx[512];

void HSEM1_IRQHandler(void)   /* CM7-side HSEM interrupt (not taken over by CubeMX); mbox_hsem relies on it to receive kicks */
{
  HAL_HSEM_IRQHandler();
}

static int rpc_recv_cb(struct rpmsg_endpoint *ept, void *data, size_t len,
                       uint32_t src, void *priv)
{
  (void)ept; (void)src; (void)priv;
  if (len > sizeof(s_rx)) { len = sizeof(s_rx); }
  memcpy(s_rx, data, len);
  s_rx_len = (uint16_t)len;
  s_last_rx = HAL_GetTick();   /* the consumer polls on defaultTask */
  return 0;
}

static void new_service_cb(struct rpmsg_device *rdev, const char *name, uint32_t dest)
{
  OPENAMP_create_endpoint(&s_ept, name, dest, rpc_recv_cb, NULL);
}

int app_rpc_init(void)   /* called at the start of mqtt_app_task (CM4 has already announced); blocks waiting for the endpoint, CM4 dead = IWDG backstop */
{
  if (s_rpc_mtx == NULL) { s_rpc_mtx = xSemaphoreCreateMutex(); }   /* RPMsg link lock (thread-safe Modbus API) */
  HAL_NVIC_SetPriority(HSEM1_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(HSEM1_IRQn);
  rpmsg_init_ept(&s_ept, "mbus", RPMSG_ADDR_ANY, RPMSG_ADDR_ANY, NULL, NULL);
  if (MX_OPENAMP_Init(RPMSG_MASTER, new_service_cb) != 0) { return -1; }
  /* wait with a timeout: a dead CM4 mustn't drag CM7 down — on timeout run degraded, keep CLI/OTA usable,
   * otherwise every CM4 crash puts CM7 into an IWDG boot-loop and debugging just races the auto-rollback */
  { uint32_t t0 = HAL_GetTick();
    while (!is_rpmsg_ept_ready(&s_ept))
    {
      OPENAMP_check_for_message();
      if ((HAL_GetTick() - t0) > 8000U) { return -2; }   /* CM4 never announced: give up but don't block */
    } }
  return 0;
}

void app_rpc_poll(void)   /* main loop each tick; skip if a transaction owns the link (it pumps RX itself) */
{
  if (s_rpc_mtx != NULL)
  {
    if (xSemaphoreTake(s_rpc_mtx, 0) != pdTRUE) { return; }
    OPENAMP_check_for_message();
    xSemaphoreGive(s_rpc_mtx);
  }
  else { OPENAMP_check_for_message(); }
}

/* synchronous transaction: send payload, busy-wait for reply (keep pumping messages meanwhile), return reply length (0=timeout). inner = unlocked; the public wrapper takes the mutex. */
static uint16_t rpc_transact_inner(const uint8_t *req, uint16_t len,
                                   uint8_t *rsp, uint16_t cap, uint32_t timeout_ms)
{
  if (!is_rpmsg_ept_ready(&s_ept)) { return 0; }   /* CM4 not ready: refuse to send (sending on an empty endpoint = HardFault) */
  OPENAMP_check_for_message();   /* flush the previous transaction's late leftover reply, to avoid being off by one tick */
  s_rx_len = 0;
  if (OPENAMP_send(&s_ept, req, len) < 0) { return 0; }
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < timeout_ms)
  {
    OPENAMP_check_for_message();
    if (s_rx_len != 0U)
    {
      uint16_t n = (s_rx_len > cap) ? cap : s_rx_len;
      memcpy(rsp, s_rx, n);
      return n;
    }
  }
  return 0;
}

/* Public transaction: mutex-wrapped so any task (platform or application) may call it. */
uint16_t app_rpc_transact(const uint8_t *req, uint16_t len,
                          uint8_t *rsp, uint16_t cap, uint32_t timeout_ms)
{
  uint16_t r;
  if (s_rpc_mtx != NULL) { xSemaphoreTake(s_rpc_mtx, portMAX_DELAY); }
  r = rpc_transact_inner(req, len, rsp, cap, timeout_ms);
  if (s_rpc_mtx != NULL) { xSemaphoreGive(s_rpc_mtx); }
  return r;
}

uint8_t app_rpc_alive(void)
{
  return ((HAL_GetTick() - s_last_rx) < 15000U) ? 1U : 0U;
}

void app_rpc_hb(void)   /* tiny round-trip capped at 30ms, feeds the alive timestamp */
{
  uint8_t rsp[16];
  (void)app_rpc_transact((const uint8_t *)"hb", 2, rsp, sizeof(rsp), 30);
}

/* alternating LED sync: after each green-LED (CM7) toggle, send op=6 announcing the new state,
 * and CM4 locks the yellow LED to the inverse. Fire-and-forget (CM4 doesn't reply, doesn't pollute the transaction
 * channel); silently skipped while the endpoint isn't ready.
 * Diagnostic value preserved: the yellow LED is normally driven by the sync; once the sync stops (CM7 dead) CM4 falls
 * back to running on its own = still a proof that CM4 is alive. */
void app_rpc_led_sync(uint8_t green_on)
{
  uint8_t msg[2] = { 0x06U, green_on };
  if (!is_rpmsg_ept_ready(&s_ept)) { return; }
  if (s_rpc_mtx != NULL) { xSemaphoreTake(s_rpc_mtx, portMAX_DELAY); }
  (void)OPENAMP_send(&s_ept, msg, 2);
  if (s_rpc_mtx != NULL) { xSemaphoreGive(s_rpc_mtx); }
}

void app_rpc_cli(void)   /* CLI 'rpc': echo round-trip 3 times to verify the dual-core link */
{
  uint8_t rsp[64];
  for (int i = 0; i < 3; i++)
  {
    char msg[32];
    int n = snprintf(msg, sizeof(msg), "ping %d", i);
    uint16_t r = app_rpc_transact((const uint8_t *)msg, (uint16_t)n, rsp, sizeof(rsp) - 1U, 120);   /* CM4 now on a 50ms cadence, >2 ticks */
    rsp[r] = 0;
    printf("  rpc[%d]: %s\n\r", i, (r > 0U) ? (char *)rsp : "(timeout)");
  }
}
