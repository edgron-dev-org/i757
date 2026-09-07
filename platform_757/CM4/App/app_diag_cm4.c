/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_diag_cm4.c — CM4 self-report: tasks, stack headroom, heap.
 *
 * Answers RPMsg op 0x0D so the USB console on CM7 can show BOTH cores in one view. Stack
 * headroom is the number that matters here: a task that overflows its stack corrupts whatever
 * sits below it and shows up as an unexplained hard fault far from the cause, so it has to be
 * observable before it happens.
 * Contract: docs/Inter_Core_RPMsg_Protocol.md op 0x0D.
 */
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"

#define DIAG_NAME_LEN 12U
#define DIAG_MAX_TASK 20U
#define DIAG_ENTRY    (DIAG_NAME_LEN + 5U)   /* name + state + prio + high-water u16 + cpu%% */

static void put32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

/* CPU% is computed here from the DELTA since the previous call, not since boot: what matters
 * is the load right now, and deltas are also immune to the run-time counter wrapping. */
static UBaseType_t s_prev_id[DIAG_MAX_TASK];
static uint32_t    s_prev_rt[DIAG_MAX_TASK];
static uint32_t    s_prev_total;
static uint8_t     s_prev_n;

static uint32_t prev_of(UBaseType_t id)
{
  for (uint8_t i = 0; i < s_prev_n; i++) { if (s_prev_id[i] == id) { return s_prev_rt[i]; } }
  return 0;
}

/* reply: [n][heap_free u32][heap_min u32][tick u32]
 *        then n x {name[12], state, prio, hw u16, cpu%} */
uint16_t diag_cm4_tasks(uint8_t *out)
{
  static TaskStatus_t st[DIAG_MAX_TASK];
  static uint32_t d[DIAG_MAX_TASK];
  uint32_t total = 0, dtotal = 0;
  UBaseType_t n = uxTaskGetSystemState(st, DIAG_MAX_TASK, &total);
  uint16_t off = 13U;
  if (n > DIAG_MAX_TASK) { n = DIAG_MAX_TASK; }

  /* Compute EVERY delta against the old snapshot before touching it. Updating the snapshot
   * inside the same loop was the real bug behind the nonsense percentages (shares summing to
   * 121%, then an IDLE of 222%): later lookups were finding entries this very loop had already
   * overwritten. Denominator = sum of the deltas, i.e. share of accounted CPU time — interrupt
   * time belongs to no task, so wall-clock is not a comparable base. */
  for (UBaseType_t k = 0; k < n; k++)
  {
    d[k] = st[k].ulRunTimeCounter - prev_of(st[k].xTaskNumber);
    dtotal += d[k];
  }

  out[0] = (uint8_t)n;
  put32(&out[1], (uint32_t)xPortGetFreeHeapSize());
  put32(&out[5], (uint32_t)xPortGetMinimumEverFreeHeapSize());
  put32(&out[9], (uint32_t)xTaskGetTickCount());
  for (UBaseType_t k = 0; k < n; k++)
  {
    uint8_t *e = &out[off];
    memset(e, 0, DIAG_ENTRY);
    if (st[k].pcTaskName != NULL)
    {
      size_t l = strlen(st[k].pcTaskName);
      if (l > (DIAG_NAME_LEN - 1U)) { l = DIAG_NAME_LEN - 1U; }
      memcpy(e, st[k].pcTaskName, l);
    }
    e[DIAG_NAME_LEN]      = (uint8_t)st[k].eCurrentState;
    e[DIAG_NAME_LEN + 1U] = (uint8_t)st[k].uxCurrentPriority;
    e[DIAG_NAME_LEN + 2U] = (uint8_t)(st[k].usStackHighWaterMark & 0xFFU);
    e[DIAG_NAME_LEN + 3U] = (uint8_t)(st[k].usStackHighWaterMark >> 8);
    e[DIAG_NAME_LEN + 4U] = (dtotal > 0U) ? (uint8_t)((d[k] * 100U) / dtotal) : 0U;
    off = (uint16_t)(off + DIAG_ENTRY);
  }
  for (UBaseType_t k = 0; k < n; k++)          /* snapshot updated only now */
  {
    s_prev_id[k] = st[k].xTaskNumber;
    s_prev_rt[k] = st[k].ulRunTimeCounter;
  }
  s_prev_n = (uint8_t)n;
  s_prev_total = total;
  return off;
}


/* ================= kernel health: run-time counter + the two silent-failure hooks ========= */
#include "app_health.h"

/* Run-time counter for CPU%. NOT the DWT cycle counter: on this core it stays at zero (its
 * debug block is not powered the way CM7's is), which silently produced 0% for every task.
 * TIM7 is already the HAL time base here, configured 1 MHz counting to 1000, so tick+CNT is a
 * free monotonic microsecond clock — no extra timer consumed, and known to run. */
extern TIM_HandleTypeDef htim7;

void app_rt_stats_init(void)
{
  /* nothing to do: TIM7 is started by HAL_InitTick() long before the scheduler */
}
/* The counter must be MONOTONIC or the accounting is nonsense — twice over.
 * Sampling: reading tick and CNT independently let CNT roll to 0 before HAL's tick caught up
 * (summed to 173%), and compensating with tick++ on the update flag OVER-counted a millisecond
 * (120%). Re-read until the tick is stable across the sample instead of guessing.
 * Range: the raw u32 microsecond value wraps every 71.6 min, and vTaskGetRunTimeStats uses the
 * CURRENT value as its denominator while task totals accumulate wrap-safe — exactly the
 * denominator collapse the contract forbids (CM7 shipped IDLE 423% this way). So accumulate the
 * wrap-safe deltas in 64 bits and shift on the way out: >>6 gives ~64 us resolution (15x the
 * tick rate) and ~76 h before the RETURNED value wraps. op0E percentages are trustworthy inside
 * that horizon; op0D's delta-based cpu% stays correct forever. Contract: Inter_Core op0E note. */
static uint32_t s_us_last;
static uint64_t s_us_acc;

static uint32_t rt_raw_us(void)
{
  uint32_t t1, t2, c;
  do
  {
    t1 = HAL_GetTick();
    c  = (uint32_t)__HAL_TIM_GET_COUNTER(&htim7);
    t2 = HAL_GetTick();
  } while (t1 != t2);
  return (t1 * 1000U) + c;
}
uint32_t app_rt_stats_get(void)
{
  uint32_t now = rt_raw_us();
  uint32_t d = now - s_us_last;
  /* The raw value can DIP by up to 1 ms: TIM7's counter rolls over before its update interrupt
   * has bumped uwTick, so tick*1000+CNT briefly reads ~1000 lower than the previous sample.
   * The stable-tick loop cannot see that. An unsigned dip becomes a ~2^32 us delta and credits
   * ~67M units to whatever task is running (bench-caught: rpcCM4 at 935M units = 92% while the
   * delta view said 0%). Clamp: skip the dip, the next call re-covers it (~1 ms error/event). */
  if (d < 0x80000000UL) { s_us_acc += d; }
  s_us_last = now;
  return (uint32_t)(s_us_acc >> 6);
}

static void health_record(volatile health_rec_t *r, uint32_t type, const char *name)
{
  uint32_t i;
  r->magic = HEALTH_MAGIC | type;
  for (i = 0; i < sizeof(r->name); i++)
  {
    char c = (name != NULL) ? name[i] : 0;
    r->name[i] = c;
    if (c == 0) { break; }
  }
  for (; i < sizeof(r->name); i++) { r->name[i] = 0; }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  health_record(HEALTH_CM4, HEALTH_STACK, pcTaskName);
  /* named-reset breadcrumb (SFT2 on this core; CM7 journals it next boot) */
  RESETNOTE->magic = RESETNOTE_MAGIC;
  { const char pfx[] = "cm4-stack";
    uint32_t i;
    for (i = 0; i < sizeof(pfx); i++) { RESETNOTE->reason[i] = pfx[i]; } }
  __DSB();
  NVIC_SystemReset();                 /* the stack is already corrupt: restart, but named */
}

void vApplicationMallocFailedHook(void)
{
  health_record(HEALTH_CM4, HEALTH_MALLOC, pcTaskGetName(NULL));
  /* not fatal by itself — the caller sees NULL — but now it is on the record */
}

/* FreeRTOS's own run-time table (op 0x0E). Kept alongside the hand-computed CPU% so the two can
 * be cross-checked: this one is the lifetime average straight from the kernel, no arithmetic of
 * ours in the path. */
uint16_t diag_cm4_runstats(uint8_t *out, uint16_t cap)
{
  /* vTaskGetRunTimeStats writes UNBOUNDED — sized by task count and digit width, not by the
   * caller. Worst case here (16-char names, wrapped 10-digit counters, 12+ tasks) exceeds the
   * 460 B RPMsg reply buffer, so the kernel gets its own worst-case buffer and the reply is a
   * bounded copy. 20 tasks x ~42 B < 1024. */
  static char big[1024];
  uint16_t n = 0;
  vTaskGetRunTimeStats(big);
  while ((n < (cap - 1U)) && (big[n] != 0)) { out[n] = (uint8_t)big[n]; n++; }
  return n;
}
