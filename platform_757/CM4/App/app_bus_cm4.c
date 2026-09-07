/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_bus_cm4.c — one thread per Modbus bus (CM4 data plane).
 *
 * Every bus owns a thread: the six front-panel 485 ports and the backplane. That thread does
 * EVERYTHING for its bus — running the scan entries when the port is a master, answering
 * incoming frames when it is a slave, and healing its UART after an error.
 *
 * Why: a Modbus transaction blocks until the reply or the timeout. With one loop serving every
 * bus, a single unreachable device stalled all the others (and the backplane, and the onboard
 * sampling). Each UART is an independent peripheral with its own DMA, so giving each bus its
 * own thread both isolates the fault and recovers the parallelism.
 *
 * Locking: the bus mutex lives INSIDE the leaf transaction/service functions, not here, so any
 * caller is serialised — the bus thread and an aperiodic RPMsg pass-through alike — and the
 * functions never nest, so a plain mutex is enough.
 */
#include "main.h"
#include "cmsis_os.h"
#include "app_pimage.h"

#define BUS_N        PIMG_NBUS        /* 0..5 = front 485A..F, 6 = backplane */
#define BUS_STACK    (768 * 4)        /* bytes; a scan entry builds ~1.3 KB of frame buffers on the stack */

extern void pscan_cm4_service_bus(uint8_t bus);   /* app_pscan_cm4.c: run this bus's due entries */
extern void mbfront_service_port(uint8_t port);   /* app_mbfront_cm4.c: slave replies + heal, one port */
extern void mbport_cm4_scheduler(void);           /* app_mbport_cm4.c: backplane online-table housekeeping */
extern void mbport_cm4_rx_heal(void);
extern void app_rpc_cm4_poll(void);               /* app_rpc_cm4.c: pump inter-core messages */

static osMutexId_t s_lock[BUS_N];

void bus_cm4_lock(uint8_t bus)     /* called by the leaf transaction/service functions */
{
  if ((bus < BUS_N) && (s_lock[bus] != NULL)) { (void)osMutexAcquire(s_lock[bus], osWaitForever); }
}
void bus_cm4_unlock(uint8_t bus)
{
  if ((bus < BUS_N) && (s_lock[bus] != NULL)) { (void)osMutexRelease(s_lock[bus]); }
}

/* RPMsg lives in its own thread because an aperiodic pass-through (op1 / op0A) performs a real
 * bus transaction inside the receive callback and therefore blocks until the reply or the
 * timeout. On the main task that would stall the onboard sampling and the watchdog feed. */
static void rpc_thread(void *arg)
{
  (void)arg;
  for (;;) { app_rpc_cm4_poll(); osDelay(1); }
}

static void bus_thread(void *arg)
{
  uint8_t bus = (uint8_t)(uint32_t)arg;
  for (;;)
  {
    if (bus == PIMG_PORT_BACKPLANE)
    {
      mbport_cm4_scheduler();          /* roll-call / offline probe (its own tick pacing) */
      mbport_cm4_rx_heal();
    }
    else
    {
      mbfront_service_port(bus);       /* slave-role replies + this port's uart heal */
    }
    pscan_cm4_service_bus(bus);        /* master-role scan entries that are due on this bus */
    PIMG->st.cycle_us[bus]++;          /* loop counter: proves whether this thread is spinning */
    osDelay(1);
  }
}

/* Started-thread count, mirrored to the CM4 breadcrumb word so a heap shortage can never fail
 * silently: read 0x38008404 over SWD, it must equal BUS_N+1 (7 buses + rpc). */
#define BUS_BREADCRUMB (*(volatile uint32_t *)0x38008404UL)

void bus_cm4_start(void)
{
  uint32_t started = 0;
  static const char *names[BUS_N] = { "bus485A", "bus485B", "bus485C",
                                      "bus485D", "bus485E", "bus485F", "busBP" };
  for (uint8_t b = 0; b < BUS_N; b++)
  {
    s_lock[b] = osMutexNew(NULL);
  }
  for (uint8_t b = 0; b < BUS_N; b++)
  {
    osThreadAttr_t a = {0};
    a.name = names[b];
    a.stack_size = BUS_STACK;
    a.priority = (osPriority_t)osPriorityBelowNormal;   /* below the main task: never starve RPMsg/watchdog */
    if (osThreadNew(bus_thread, (void *)(uint32_t)b, &a) != NULL) { started++; }
  }
  {
    osThreadAttr_t a = {0};
    a.name = "rpcCM4";
    a.stack_size = BUS_STACK;
    a.priority = (osPriority_t)osPriorityNormal;   /* above the bus threads: keep CM7 responsive */
    if (osThreadNew(rpc_thread, NULL, &a) != NULL) { started++; }
  }
  BUS_BREADCRUMB = (0xB0000000UL | started);   /* expect 0xB0000008 (7 buses + rpc) */
}
