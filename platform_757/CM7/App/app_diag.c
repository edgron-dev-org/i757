/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_diag.c — 'tasks' CLI: both cores' task list, stack headroom and heap, over the USB console.
 *
 * Stack headroom is the point. A task that overruns its stack corrupts whatever lies below it
 * and surfaces as a hard fault nowhere near the cause, so it must be visible while there is
 * still margin. The number shown is FreeRTOS's high-water mark: the smallest free space that
 * stack has ever had, in WORDS (x4 = bytes). Treat under ~100 words as needing attention.
 *
 * CM4 reports itself over RPMsg op 0x0D (docs/Inter_Core_RPMsg_Protocol.md).
 */
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx.h"
#include "app_rpc.h"
#include "app_health.h"

#define DIAG_NAME_LEN 12U
#define DIAG_ENTRY    (DIAG_NAME_LEN + 5U)
#define DIAG_MAX_TASK 20U

static const char *state_str(uint8_t s)
{
  static const char *n[] = { "run", "ready", "block", "susp", "del" };
  return (s < 5U) ? n[s] : "?";
}

static uint32_t get32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void print_row(const char *name, uint8_t state, uint8_t prio, uint16_t hw, uint8_t cpu)
{
  printf("   %-13s %-6s pri%-3u cpu%3u%%  stack_free %4u w (%5u B)%s\n\r",
         name, state_str(state), (unsigned)prio, (unsigned)cpu,
         (unsigned)hw, (unsigned)(hw * 4U), (hw < 100U) ? "  <-- LOW" : "");
}

/* CPU% from the DELTA since the previous call: the load right now, and immune to the run-time
 * counter wrapping (a lifetime average would hide a task that only recently went busy). */
static UBaseType_t s_prev_id[DIAG_MAX_TASK];
static uint32_t    s_prev_rt[DIAG_MAX_TASK];
static uint32_t    s_prev_total;
static uint8_t     s_prev_n;

static uint32_t prev_of(UBaseType_t id)
{
  for (uint8_t i = 0; i < s_prev_n; i++) { if (s_prev_id[i] == id) { return s_prev_rt[i]; } }
  return 0;
}

static uint32_t s_rsr_raw = 0;           /* latched RCC->RSR of this boot (all flags, for forensics) */
uint32_t app_diag_rsr_raw(void) { return s_rsr_raw; }
static const char *reset_cause(void)     /* why did we come up? the field's most useful clue */
{
  static uint32_t rsr = 0;
  if (rsr == 0U) { rsr = RCC->RSR; RCC->RSR |= RCC_RSR_RMVF; s_rsr_raw = rsr; }   /* latch once, then clear */
  if (rsr & RCC_RSR_IWDG1RSTF) { return "IWDG1 watchdog (CM7)"; }
  if (rsr & RCC_RSR_IWDG2RSTF) { return "IWDG2 watchdog (CM4)"; }
  if (rsr & RCC_RSR_WWDG1RSTF) { return "window watchdog"; }
  if (rsr & RCC_RSR_SFT1RSTF)  { return "software reset"; }
  if (rsr & RCC_RSR_SFT2RSTF)  { return "software reset (CM4)"; }   /* 2026-09-02: used to fall
                                          * through to "unknown" — a CM4-initiated reset was
                                          * indistinguishable from noise */
  if (rsr & RCC_RSR_BORRSTF)   { return "brown-out"; }
  if (rsr & RCC_RSR_PORRSTF)   { return "power-on"; }
  if (rsr & RCC_RSR_PINRSTF)   { return "reset pin"; }
  return "unknown";
}

/* ---- named resets + boot forensics (2026-09-02, the silent-reset epidemic) ----
 * Three unexplained "software reset" boots in one morning, and every instrumented
 * path (cloud reboot journal, heal rung3, stack/malloc records, OTA) came back clean —
 * because the fault handlers' AIRCR write (stm32h7xx_it.c black box) and any wild
 * jump reset without leaving a JOURNALED trace. Now: intentional resets go through
 * app_reset() and name themselves; the boot journal line appends whatever the
 * breadcrumb and the fault black box retained (SRAM4 survives every reset short of
 * power loss); an SFT1 boot with neither is reported UNNAMED — itself the clue. */
void app_reset(const char *reason)
{
  volatile resetnote_t *r = RESETNOTE;
  uint32_t i;
  r->magic = RESETNOTE_MAGIC;
  for (i = 0; i < sizeof(r->reason) - 1U; i++)
  {
    char c = (reason != NULL) ? reason[i] : 0;
    r->reason[i] = c;
    if (c == 0) { break; }
  }
  r->reason[sizeof(r->reason) - 1U] = 0;
  __DSB();
  NVIC_SystemReset();
}

/* Appended to the boot journal line by app_datalog. Reads and CLEARS the one-shot
 * records. bb layout (stm32h7xx_it.c): magic,type(1=hard 2=mem 3=bus 4=usage),
 * CFSR,HFSR,stacked-PC,PSP,BFAR,MMFAR. */
#include "app_pwrfail.h"
PF_RETAIN volatile uint32_t g_fault_bb2[8];   /* battery-domain mirror of the SRAM4 fault black box, written by the
                                              * fault handlers (stm32h7xx_it.c). 2026-09-03: the SRAM4 copy was found
                                              * overwritten with 32 random bytes by boot time (writer unknown), which
                                              * turned every hard fault into an UNNAMED verdict. Read whichever survived. */
void app_diag_boot_forensics(char *out, uint16_t cap)
{
  volatile uint32_t *bb = (volatile uint32_t *)0x3800E440UL;
  if ((bb[0] != 0xFA017CB7UL) && (g_fault_bb2[0] == 0xFA017CB7UL)) { bb = g_fault_bb2; }
  volatile resetnote_t *r = RESETNOTE;
  uint16_t n = 0;
  out[0] = 0;
  if (r->magic == RESETNOTE_MAGIC)
  {
    char nm[28];
    memcpy(nm, (const void *)r->reason, sizeof nm); nm[sizeof nm - 1U] = 0;
    r->magic = 0;
    n += (uint16_t)snprintf(&out[n], (size_t)(cap - n), " cause=%s", nm);
  }
  if (bb[0] == 0xFA017CB7UL)
  {
    static const char *ft[5] = { "?", "hard", "mem", "bus", "usage" };
    uint32_t t = (bb[1] < 5U) ? bb[1] : 0U;
    n += (uint16_t)snprintf(&out[n], (size_t)(cap - n),
                            " FAULT=%s cfsr=%08lx hfsr=%08lx pc=%08lx psp=%08lx bfar=%08lx mmfar=%08lx",
                            ft[t], (unsigned long)bb[2], (unsigned long)bb[3], (unsigned long)bb[4],
                            (unsigned long)bb[5], (unsigned long)bb[6], (unsigned long)bb[7]);
    bb[0] = 0;
    g_fault_bb2[0] = 0;
  }
  if ((n == 0U) && (strcmp(reset_cause(), "software reset") == 0))
  {
    snprintf(out, cap, " cause=UNNAMED");   /* SFT1 with no note and no fault record:
                                             * a reset from outside every instrumented path */
  }
}

/* Boot-forensics ring in the battery domain (2026-09-03, greenhouse 3 s reset loop: ~1300
 * SFT1 boots and not one "boot rst=" line reached the journal — a 3 s life never drains
 * the event queue, and app_diag_boot_forensics() had already consumed the one-shot SRAM4
 * records. The four most recent boot verdicts now also land here, and every heartbeat
 * carries them as "bf", so a loop that never lives long enough to write a card is still
 * readable the moment the board comes back). */
#include "app_pwrfail.h"
#define BOOTRING_N   4U
#define BOOTRING_LEN 96U
PF_RETAIN static char     s_bootring[BOOTRING_N][BOOTRING_LEN];
PF_RETAIN static uint32_t s_bootring_n;      /* boots recorded since the ring was created */
PF_RETAIN static uint32_t s_bootring_magic;  /* own magic: the platform only zeroes the region on a
                                              * battery change, so a NEW retained variable inherits
                                              * whatever bytes its address held before (first field
                                              * push 2026-09-03: "n=1346" + binary garbage in "bf") */
#define BOOTRING_MAGIC 0xB007A1D6UL
static char s_bootring_str[BOOTRING_N * BOOTRING_LEN + 24];

void app_diag_boot_ring_note(const char *cause, const char *fx)
{
  if (!app_pf_retain_valid() || (s_bootring_magic != BOOTRING_MAGIC))
  {
    s_bootring_n = 0; memset(s_bootring, 0, sizeof s_bootring); s_bootring_magic = BOOTRING_MAGIC;
  }
  /* raw RSR alongside the verdict (2026-09-03 UNNAMED-on-EXTI case): the decoded cause hides
   * companion flags (C1/C2/D1/D2 domain resets, SFT2) that tell which core really pulled it */
  snprintf(s_bootring[s_bootring_n % BOOTRING_N], BOOTRING_LEN, "%s%s rsr=%08lx", cause, fx,
           (unsigned long)s_rsr_raw);
  s_bootring_n++;
}

const char *app_diag_boot_ring_str(void)     /* "n=<boots> | newest | ... | oldest" */
{
  uint16_t n = (uint16_t)snprintf(s_bootring_str, sizeof s_bootring_str, "n=%lu", (unsigned long)s_bootring_n);
  for (uint32_t k = 0; (k < BOOTRING_N) && (k < s_bootring_n); k++)
  {
    const char *e = s_bootring[(s_bootring_n - 1U - k) % BOOTRING_N];
    if (e[0] == 0) { continue; }
    n += (uint16_t)snprintf(&s_bootring_str[n], sizeof s_bootring_str - n, " | %s", e);
    if (n >= sizeof s_bootring_str - 1U) { break; }
  }
  if (s_bootring_magic != BOOTRING_MAGIC) { snprintf(s_bootring_str, sizeof s_bootring_str, "n=? (uninit)"); }
  for (uint16_t i = 0; s_bootring_str[i] != 0; i++)   /* JSON-safe: the ring is raw battery RAM */
  {
    char c = s_bootring_str[i];
    if ((c < 0x20) || (c > 0x7E) || (c == '"') || (c == '\\')) { s_bootring_str[i] = '?'; }
  }
  return s_bootring_str;
}

static void print_health(const char *core, volatile health_rec_t *r)
{
  uint32_t m = r->magic;
  char nm[13];
  if ((m & 0xFFFFFF00UL) != HEALTH_MAGIC) { return; }            /* nothing recorded */
  memcpy(nm, (const void *)r->name, 12); nm[12] = 0;
  printf(" !! %s FAULT ON A PREVIOUS RUN: %s in task '%s'  ('health clear' to acknowledge)\n\r",
         core, ((m & 0xFFU) == HEALTH_STACK) ? "STACK OVERFLOW" : "HEAP EXHAUSTED", nm);
}

static void diag_cm7(void)
{
  static TaskStatus_t st[DIAG_MAX_TASK];
  static uint32_t d[DIAG_MAX_TASK];
  uint32_t total = 0, dtotal = 0;
  UBaseType_t n = uxTaskGetSystemState(st, DIAG_MAX_TASK, &total);
  /* deltas first, snapshot afterwards — see the note in app_diag_cm4.c */
  for (UBaseType_t k = 0; k < n; k++)
  {
    d[k] = st[k].ulRunTimeCounter - prev_of(st[k].xTaskNumber);
    dtotal += d[k];
  }
  printf(" CM7  heap free %u / min-ever %u B   uptime %lus\n\r",
         (unsigned)xPortGetFreeHeapSize(), (unsigned)xPortGetMinimumEverFreeHeapSize(),
         (unsigned long)(xTaskGetTickCount() / configTICK_RATE_HZ));
  for (UBaseType_t k = 0; k < n; k++)
  {
    print_row(st[k].pcTaskName, (uint8_t)st[k].eCurrentState,
              (uint8_t)st[k].uxCurrentPriority, (uint16_t)st[k].usStackHighWaterMark,
              (dtotal > 0U) ? (uint8_t)((d[k] * 100U) / dtotal) : 0U);
  }
  for (UBaseType_t k = 0; k < n; k++)
  {
    s_prev_id[k] = st[k].xTaskNumber;
    s_prev_rt[k] = st[k].ulRunTimeCounter;
  }
  s_prev_n = (uint8_t)n;
  s_prev_total = total;
}


static void diag_cm4(void)
{
  uint8_t req[1] = { 0x0DU };
  static uint8_t rsp[400];
  uint16_t r = app_rpc_transact(req, 1, rsp, sizeof(rsp), 200);
  uint8_t n;
  if (r < 13U) { printf(" CM4  (no reply)\n\r"); return; }
  n = rsp[0];
  printf(" CM4  heap free %lu / min-ever %lu B   uptime %lus\n\r",
         (unsigned long)get32(&rsp[1]), (unsigned long)get32(&rsp[5]),
         (unsigned long)(get32(&rsp[9]) / 1000UL));
  for (uint8_t i = 0; i < n; i++)
  {
    const uint8_t *e = &rsp[13U + (uint16_t)i * DIAG_ENTRY];
    char name[DIAG_NAME_LEN + 1];
    if ((uint16_t)(13U + (i + 1U) * DIAG_ENTRY) > r) { break; }
    memcpy(name, e, DIAG_NAME_LEN); name[DIAG_NAME_LEN] = 0;
    print_row(name, e[DIAG_NAME_LEN], e[DIAG_NAME_LEN + 1U],
              (uint16_t)(e[DIAG_NAME_LEN + 2U] | ((uint16_t)e[DIAG_NAME_LEN + 3U] << 8)),
              e[DIAG_NAME_LEN + 4U]);
  }
}


/* FreeRTOS own table (lifetime average, no arithmetic of ours) — the cross-check for the
 * delta-based cpu%% column in tasks. */
static int diag_runstats(void)
{
  static char buf[1024];
  uint8_t req[1] = { 0x0EU };
  static uint8_t rsp[460];
  uint16_t r;
  printf("--- CM7 (FreeRTOS vTaskGetRunTimeStats, since boot) ---\n\r");
  vTaskGetRunTimeStats(buf);
  printf("%s", buf);
  printf("--- CM4 ---\n\r");
  r = app_rpc_transact(req, 1, rsp, sizeof(rsp) - 1U, 300);
  if (r == 0U) { printf("(no reply)\n\r"); return 1; }
  rsp[r] = 0;
  printf("%s", (char *)rsp);
  return 1;
}

int app_diag_cli(char *line)   /* returns 1 if it handled the line */
{
  if (strcmp(line, "health clear") == 0)
  {
    HEALTH_CM7->magic = 0; HEALTH_CM4->magic = 0;
    printf("fault records cleared\n\r");
    return 1;
  }
  if (strcmp(line, "runstats") == 0) { return diag_runstats(); }
  if ((strcmp(line, "tasks") != 0) && (strcmp(line, "ps") != 0)) { return 0; }
  printf("tasks  (cpu%% = share since the previous run; stack_free = smallest EVER)\n\r");
  printf(" last reset: %s\n\r", reset_cause());
  print_health("CM7", HEALTH_CM7);
  print_health("CM4", HEALTH_CM4);
  diag_cm7();
  diag_cm4();
  return 1;
}


/* ================= heartbeat feed (dash health, 2026-07-25) =================
 * Same data the 'tasks' CLI shows, packed for the status JSON so the dash can tell
 * "working" from "working while sick". CM7 and CM4 task tables go out as two separate
 * strings (tsk7/tsk4) so the dash renders one row per core (2026-07-26).
 * Shares the delta arrays with the CLI: whichever
 * consumer sampled last resets the window — cpu% is then "share since the previous
 * sample by anyone", which is what you want on a dashboard anyway. */

const char *app_diag_reset_cause(void) { return reset_cause(); }

void app_diag_fault_str(char *out, uint16_t cap)   /* "" = no fault recorded on a previous run */
{
  uint16_t n = 0;
  out[0] = 0;
  volatile health_rec_t *rec[2] = { HEALTH_CM7, HEALTH_CM4 };
  const char *core[2] = { "CM7", "CM4" };
  for (int i = 0; i < 2; i++)
  {
    uint32_t m = rec[i]->magic;
    char nm[13];
    if ((m & 0xFFFFFF00UL) != HEALTH_MAGIC) { continue; }
    memcpy(nm, (const void *)rec[i]->name, 12); nm[12] = 0;
    n += (uint16_t)snprintf(&out[n], (size_t)(cap - n), "%s%s-%s:%s", (n > 0U) ? "+" : "",
                            core[i], ((m & 0xFFU) == HEALTH_STACK) ? "STACK" : "HEAP", nm);
    if (n >= cap) { break; }
  }
}

/* CM4 table over RPC is throttled: refresh the cache every 5th heartbeat build */
static uint8_t  s_hb_cm4[400];
static uint16_t s_hb_cm4_len = 0;
static uint8_t  s_hb_ctr = 0;

static char state_letter(uint8_t s)   /* eRunning..eDeleted -> R r B S D */
{
  static const char l[5] = { 'R', 'r', 'B', 'S', 'D' };
  return (s < 5U) ? l[s] : '?';
}

void app_diag_hb(char *out7, uint16_t cap7, char *out4, uint16_t cap4,
                 uint8_t *cpu7, uint8_t *cpu4, uint32_t *stkmin_b)
{
  static TaskStatus_t st[DIAG_MAX_TASK];
  static uint32_t d[DIAG_MAX_TASK];
  uint32_t total = 0, dtotal = 0, stkmin = 0xFFFFFFFFUL;
  uint16_t n = 0;
  uint8_t idle7 = 100U;
  UBaseType_t cnt = uxTaskGetSystemState(st, DIAG_MAX_TASK, &total);
  for (UBaseType_t k = 0; k < cnt; k++)
  {
    d[k] = st[k].ulRunTimeCounter - prev_of(st[k].xTaskNumber);
    dtotal += d[k];
  }
  out7[0] = 0;
  for (UBaseType_t k = 0; k < cnt; k++)
  {
    uint8_t cpu = (dtotal > 0U) ? (uint8_t)((d[k] * 100U) / dtotal) : 0U;
    uint32_t freeb = (uint32_t)st[k].usStackHighWaterMark * 4U;
    if (strcmp(st[k].pcTaskName, "IDLE") == 0) { idle7 = cpu; }
    if (freeb < stkmin) { stkmin = freeb; }
    if (n < cap7) { n += (uint16_t)snprintf(&out7[n], (size_t)(cap7 - n), "%s%s:%c%u%%/%luB",
                     (n > 0U) ? " " : "", st[k].pcTaskName,
                     state_letter((uint8_t)st[k].eCurrentState), (unsigned)cpu,
                     (unsigned long)freeb); }
    s_prev_id[k] = st[k].xTaskNumber;
    s_prev_rt[k] = st[k].ulRunTimeCounter;
  }
  s_prev_n = (uint8_t)cnt;
  s_prev_total = total;
  *cpu7 = (uint8_t)(100U - idle7);

  if ((s_hb_ctr++ % 5U) == 0U)                       /* refresh the CM4 snapshot */
  {
    uint8_t req[1] = { 0x0DU };
    s_hb_cm4_len = app_rpc_transact(req, 1, s_hb_cm4, sizeof(s_hb_cm4), 120);
  }
  *cpu4 = 0;
  n = 0;
  out4[0] = 0;
  if (s_hb_cm4_len >= 13U)
  {
    uint8_t m = s_hb_cm4[0];
    uint8_t idle4 = 100U;
    for (uint8_t i = 0; i < m; i++)
    {
      const uint8_t *e = &s_hb_cm4[13U + (uint16_t)i * DIAG_ENTRY];
      char name[DIAG_NAME_LEN + 1];
      uint32_t freeb;
      if ((uint16_t)(13U + (i + 1U) * DIAG_ENTRY) > s_hb_cm4_len) { break; }
      memcpy(name, e, DIAG_NAME_LEN); name[DIAG_NAME_LEN] = 0;
      freeb = (uint32_t)(e[DIAG_NAME_LEN + 2U] | ((uint16_t)e[DIAG_NAME_LEN + 3U] << 8)) * 4U;
      if (strcmp(name, "IDLE") == 0) { idle4 = e[DIAG_NAME_LEN + 4U]; }
      if (freeb < stkmin) { stkmin = freeb; }
      if (n < cap4) { n += (uint16_t)snprintf(&out4[n], (size_t)(cap4 - n), "%s%s:%c%u%%/%luB",
                       (n > 0U) ? " " : "", name, state_letter(e[DIAG_NAME_LEN]),
                       (unsigned)e[DIAG_NAME_LEN + 4U], (unsigned long)freeb); }
    }
    *cpu4 = (uint8_t)(100U - idle4);
  }
  else { snprintf(out4, cap4, "?"); }
  *stkmin_b = (stkmin == 0xFFFFFFFFUL) ? 0U : stkmin;
}

/* ---- backplane module health -> per-module dash groups (contract v0.27 diag block) ----
 * Each online module gets its own desc group "m<addr>" with cpu/stkmin/tsk points, so the
 * dash shows one card per module instead of one lump under System (scales, distinguishes).
 * Low rate by design: snapshot refreshed every 10th call (heartbeat cadence ~2s => ~20s);
 * ident (type/fw) re-read only when the online bitmap changes, which also flags the desc
 * for a rebuild+republish. One-shot FC04 reads interleave with the CM4 scheduler exactly
 * like mota's stream does. Old slave firmware answers exception 02 on 0x0300 -> tsk "-". */
#include "modbus_core.h"
#include "app_mbport.h"

#define MODS_MAX 8U
typedef struct { uint8_t addr; uint16_t type; uint16_t fw; } mod_ident_t;
static mod_ident_t s_mods[MODS_MAX];
static uint16_t    s_nmods = 0;
static uint16_t    s_mods_bitmap = 0;
static uint8_t     s_mods_dirty = 0;      /* membership changed -> desc needs rebuild */
static char        s_mtsk[520] = "";      /* heartbeat JSON fragments: ,"m2cpu":..,"m2tsk":".." */
static uint8_t     s_mtsk_ctr = 0;

const char *app_diag_mod_typename(uint16_t type)
{
  switch (type)                            /* registry = Board_Type_and_Version_Registry.md */
  {
    case 1U: return "EX_16DO";
    case 2U: return "PH_EC";
    case 3U: return "EX_16DI";
    default: { static char b[10]; snprintf(b, sizeof(b), "TYPE%u", (unsigned)type); return b; }
  }
}

uint16_t app_diag_mod_list(const void **out)   /* -> cached ident table for desc_build */
{
  *out = s_mods;
  return s_nmods;
}
uint8_t  app_diag_mod_addr(const void *m, uint16_t i) { return ((const mod_ident_t *)m)[i].addr; }
uint16_t app_diag_mod_type(const void *m, uint16_t i) { return ((const mod_ident_t *)m)[i].type; }
uint16_t app_diag_mod_fw(const void *m, uint16_t i)   { return ((const mod_ident_t *)m)[i].fw; }

uint8_t app_diag_mods_dirty(void)          /* one-shot: 1 = membership changed since last ask */
{
  uint8_t d = s_mods_dirty;
  s_mods_dirty = 0;
  return d;
}

/* mota (app_mota.c) read the slave's ident back after a successful swap: refresh the cached
 * fw so the desc card title ("Slot 5: PH_EC v0.37") follows the upgrade without waiting for a
 * membership change or a host reboot (2026-09-05: card kept saying v0.26 after a v0.37 mota). */
void app_diag_mod_set_fw(uint8_t addr, uint16_t type, uint16_t fw)
{
  for (uint16_t i = 0; i < s_nmods; i++)
  {
    if (s_mods[i].addr != addr) { continue; }
    if ((s_mods[i].type != type) || (s_mods[i].fw != fw)) { s_mods[i].type = type; s_mods[i].fw = fw; s_mods_dirty = 1; }
    return;
  }
}

static void mods_reident(uint16_t bitmap)
{
  s_nmods = 0;
  for (uint8_t a = 1; (a <= 16U) && (s_nmods < MODS_MAX); a++)
  {
    uint8_t q[8], r[24];
    uint16_t regs[3] = { 0, 0, 0 };
    if ((bitmap & (1U << (a - 1U))) == 0U) { continue; }
    int ql = mb_req_read(q, MB_FC_READ_INPUT, 0, 3);   /* map_ver/type/fw */
    uint16_t rl = app_mb_transact(a, q, (uint16_t)ql, r, 1);
    if ((rl == 0U) || (mb_rsp_regs(r, rl, MB_FC_READ_INPUT, regs, 3) != 3)) { continue; }
    s_mods[s_nmods].addr = a;
    s_mods[s_nmods].type = regs[1];
    s_mods[s_nmods].fw   = regs[2];
    s_nmods++;
  }
  s_mods_dirty = 1;
}

static void mtsk_one(const mod_ident_t *m, char *out, uint16_t cap, uint16_t *n)
{
  uint8_t q[8], r[160];
  uint16_t hdr[3];
  int ql = mb_req_read(q, MB_FC_READ_INPUT, 0x0300, 3);
  uint16_t rl = app_mb_transact(m->addr, q, (uint16_t)ql, r, 1);
  if ((rl == 0U) || (mb_rsp_regs(r, rl, MB_FC_READ_INPUT, hdr, 3) != 3) || (hdr[0] == 0U) || (hdr[0] > 8U))
  {
    *n += (uint16_t)snprintf(&out[*n], (size_t)(cap - *n), ",\"m%utsk\":\"-\"", (unsigned)m->addr);
    return;
  }
  *n += (uint16_t)snprintf(&out[*n], (size_t)(cap - *n), ",\"m%ucpu\":%u,\"m%ustk\":%u,\"m%utsk\":\"",
                           (unsigned)m->addr, (unsigned)hdr[1], (unsigned)m->addr, (unsigned)hdr[2],
                           (unsigned)m->addr);
  {
    static uint16_t slots[64];
    ql = mb_req_read(q, MB_FC_READ_INPUT, 0x0308, (uint16_t)(hdr[0] * 8U));
    rl = app_mb_transact(m->addr, q, (uint16_t)ql, r, 1);
    if ((rl > 0U) && (mb_rsp_regs(r, rl, MB_FC_READ_INPUT, slots, (uint16_t)(hdr[0] * 8U)) == (int)(hdr[0] * 8U)))
    {
      for (uint16_t k = 0; k < hdr[0]; k++)
      {
        const uint16_t *e = &slots[k * 8U];
        char nm[9];
        for (int c = 0; c < 4; c++) { nm[c * 2] = (char)(e[c] >> 8); nm[c * 2 + 1] = (char)(e[c] & 0xFFU); }
        nm[8] = 0;
        if (*n < cap) { *n += (uint16_t)snprintf(&out[*n], (size_t)(cap - *n), "%s%s:%c%u%%/%uB",
                          (k > 0U) ? " " : "", nm, state_letter((uint8_t)(e[4] >> 8)),
                          (unsigned)e[5], (unsigned)e[6]); }
      }
    }
  }
  if (*n < cap) { *n += (uint16_t)snprintf(&out[*n], (size_t)(cap - *n), "\""); }
}

/* ---- PH_EC engineering values -> heartbeat fields (module spec §2, regs 0x0108~) ----
 * Sensor data, so read FRESH on every heartbeat (unlike the ~20 s diag snapshot): one
 * FC04 of 8 regs per PH_EC module, well under a millisecond at 1 Mbps. The module
 * computes the engineering units locally (stand-alone-transmitter principle); the host
 * only relays. 0x7FFF = channel invalid -> field omitted (dash shows a gap, not a lie).
 * Old module firmware without the region answers exception 2 -> whole block omitted. */
#define MOD_TYPE_PH_EC   2U
#define PHEC_ENG_BASE    0x0108U
#define PHEC_ENG_INVAL   0x7FFFU
/* EX_16DI process data (module spec §3, contract v0.25): FC02 = debounced levels (ON=1),
 * FC04 0x0200 = 16 free-running edge counters, u32 as 2 regs hi-word-first. */
#define MOD_TYPE_EX_16DI 3U
#define DI16_CNT_BASE    0x0200U

void app_diag_mdata(char *out, uint16_t cap)  /* heartbeat: raw JSON fragment (may be "") */
{
  /* Published temp order (user convention 2026-09-01): t1=temp-EC1 ch (outdoor probe),
   * t2=temp-EC2 (greenhouse air), t3=temp-pH1 (water), t4=temp-pH2 (water). The module's
   * engineering block stays in its contract order (pH1,pH2 temps first, EC temps last) —
   * only the published field names are re-paired here. */
  static const char *fld[8] = { "ph1", "ph2", "ec1", "ec2", "t3", "t4", "t1", "t2" };
  uint16_t n = 0;
  out[0] = 0;
  for (uint16_t i = 0; i < s_nmods; i++)
  {
    uint8_t q[8], r[24];
    uint16_t v[8];
    const mod_ident_t *m = &s_mods[i];
    if (m->type == MOD_TYPE_EX_16DI)          /* levels + counters, fresh each beat like PH_EC */
    {
      uint8_t qd[8], rd[80];                  /* rd: FC04 32-reg response = 2+64B PDU */
      uint8_t bits[2];
      uint16_t c[32];
      int ql = mb_req_read(qd, MB_FC_READ_DISC, 0, 16U);
      uint16_t rl = app_mb_transact(m->addr, qd, (uint16_t)ql, rd, 1);
      if ((rl > 0U) && (mb_rsp_bits(rd, rl, MB_FC_READ_DISC, bits, 2U) == 2))
      {
        n += (uint16_t)snprintf(&out[n], (size_t)(cap - n), ",\"m%udi\":%u",
                                (unsigned)m->addr, (unsigned)(bits[0] | ((uint16_t)bits[1] << 8)));
      }
      ql = mb_req_read(qd, MB_FC_READ_INPUT, DI16_CNT_BASE, 32U);
      rl = app_mb_transact(m->addr, qd, (uint16_t)ql, rd, 1);
      if ((rl > 0U) && (mb_rsp_regs(rd, rl, MB_FC_READ_INPUT, c, 32U) == 32))
      {
        for (uint8_t k = 0; (k < 16U) && (n < cap); k++)
        {
          uint32_t cnt = ((uint32_t)c[2U * k] << 16) | c[2U * k + 1U];
          n += (uint16_t)snprintf(&out[n], (size_t)(cap - n), ",\"m%uc%u\":%lu",
                                  (unsigned)m->addr, (unsigned)(k + 1U), (unsigned long)cnt);
        }
      }
      continue;
    }
    if (m->type != MOD_TYPE_PH_EC) { continue; }
    int ql = mb_req_read(q, MB_FC_READ_INPUT, PHEC_ENG_BASE, 8U);
    uint16_t rl = app_mb_transact(m->addr, q, (uint16_t)ql, r, 1);
    if ((rl == 0U) || (mb_rsp_regs(r, rl, MB_FC_READ_INPUT, v, 8U) != 8)) { continue; }
    for (uint8_t k = 0; (k < 8U) && (n < cap); k++)
    {
      if (v[k] == PHEC_ENG_INVAL) { continue; }
      /* ph/temp are signed x100 / x10 fixed-point, EC is unsigned uS/cm; desc carries the scale */
      n += (uint16_t)snprintf(&out[n], (size_t)(cap - n), ",\"m%u%s\":%d",
                              (unsigned)m->addr, fld[k],
                              (k == 2U || k == 3U) ? (int)v[k] : (int)(int16_t)v[k]);
    }
  }
}

/* Report-on-change source for backplane DI modules (user ruling 2026-08-13, "option 3"):
 * one FC02 read per EX_16DI module per call (~100 us at 1 Mbps); returns 1 when any
 * module's debounced bitmap moved since the previous call. Called from the mqtt loop's
 * 500 ms tick next to the onboard HSDI/relay change sampling, so a field input reaches
 * the dash in <=1 s instead of waiting out the 5 s periodic (the LED is hardware and was
 * always instant — the gap was purely the publish cadence). Keyed by ADDRESS, not slot
 * index, so a membership rebuild (mods_reident) cannot mispair the change memory.
 * Counters are deliberately not compared — same policy as the onboard block. */
uint8_t app_diag_di_changed(void)
{
  static uint16_t s_last[17];
  static uint32_t s_valid;
  uint8_t changed = 0;
  for (uint16_t i = 0; i < s_nmods; i++)
  {
    const mod_ident_t *m = &s_mods[i];
    uint8_t q[8], r[16], bits[2];
    if (m->type != MOD_TYPE_EX_16DI) { continue; }
    int ql = mb_req_read(q, MB_FC_READ_DISC, 0, 16U);
    uint16_t rl = app_mb_transact(m->addr, q, (uint16_t)ql, r, 1);
    if ((rl == 0U) || (mb_rsp_bits(r, rl, MB_FC_READ_DISC, bits, 2U) != 2)) { continue; }
    {
      uint16_t bm = (uint16_t)(bits[0] | ((uint16_t)bits[1] << 8));
      if ((s_valid & (1UL << m->addr)) && (bm != s_last[m->addr])) { changed = 1U; }
      s_last[m->addr] = bm;
      s_valid |= (1UL << m->addr);
    }
  }
  return changed;
}

void app_diag_mtsk(char *out, uint16_t cap)   /* heartbeat: raw JSON fragment (may be "") */
{
  if ((s_mtsk_ctr++ % 10U) == 0U)
  {
    uint8_t req[1] = { 0x04U };
    uint8_t rsp[16];
    uint16_t r = app_rpc_transact(req, 1, rsp, sizeof(rsp), 120);
    if (r >= 11U)
    {
      uint16_t bitmap = (uint16_t)(rsp[0] | ((uint16_t)rsp[1] << 8));
      uint16_t n = 0;
      if (bitmap != s_mods_bitmap) { mods_reident(bitmap); s_mods_bitmap = bitmap; }
      s_mtsk[0] = 0;
      for (uint16_t i = 0; (i < s_nmods) && (n < sizeof(s_mtsk) - 48U); i++)
      {
        mtsk_one(&s_mods[i], s_mtsk, sizeof(s_mtsk), &n);
      }
    }
  }
  snprintf(out, cap, "%s", s_mtsk);
}

/* ================= kernel health: run-time counter + the two silent-failure hooks ========= */

/* Run-time counter for CPU%: DWT cycle counter, so no hardware timer is consumed.
 *
 * It must be MONOTONIC over the whole measurement, because FreeRTOS uses the CURRENT value as the
 * denominator while each task's total is accumulated with wrap-safe unsigned subtraction. Reading
 * CYCCNT directly does not satisfy that: at 400 MHz a 32-bit cycle count wraps every 10.7 s, and
 * shifting the RESULT does not slow the counter down — an earlier version claimed it did, and the
 * denominator collapsing every 10.7 s is exactly what made vTaskGetRunTimeStats report IDLE 423%.
 * So accumulate the wrap-safe cycle deltas in 64 bits and shift on the way out: >>14 gives
 * ~24.4 kHz (24x the tick rate) and ~49 h before the RETURNED value wraps — the 'runstats'
 * lifetime table is trustworthy inside that horizon (contract: Inter_Core op0E note); the
 * delta-based cpu% in 'tasks' stays correct forever. */
static uint32_t s_cyc_last;
static uint64_t s_cyc_acc;

void app_rt_stats_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  s_cyc_last = 0;
  s_cyc_acc  = 0;
}
uint32_t app_rt_stats_get(void)
{
  uint32_t now = DWT->CYCCNT;
  s_cyc_acc += (uint64_t)(uint32_t)(now - s_cyc_last);   /* wrap-safe: unsigned difference */
  s_cyc_last = now;
  return (uint32_t)(s_cyc_acc >> 14);
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
  health_record(HEALTH_CM7, HEALTH_STACK, pcTaskName);
  app_reset("stack-overflow");        /* the stack is already corrupt: restart, but named */
}

void vApplicationMallocFailedHook(void)
{
  health_record(HEALTH_CM7, HEALTH_MALLOC, pcTaskGetName(NULL));
  /* not fatal by itself — the caller sees NULL — but now it is on the record */
}
