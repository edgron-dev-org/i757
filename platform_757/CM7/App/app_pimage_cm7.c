/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_pimage_cm7.c — CM7 process-image API (interface/management plane).
 *
 * Builds the scan table into the SRAM4 shared block, hands it to the CM4
 * scanner (op0C doorbell + cfg_ver bump), and exposes non-blocking accessors
 * that read/write the process image with the seqlock protocol. CM7 is the sole
 * writer of the output image and the sole reader of the input image.
 * Contract: docs/Process_Image_and_IO_Mapping.md.
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "app_platform.h"
#include "app_pimage.h"
#include "app_rpc.h"
#include "app_health.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define PIMG_SLICE_MAX 256U   /* largest single-entry image slice we buffer on the stack */

static uint16_t          s_npts = 0;
static SemaphoreHandle_t s_io_mtx = NULL;   /* serialises output RMW across CM7 tasks */

static uint16_t slice_bytes(uint8_t access, uint16_t count)
{
  if (access == APP_IO_IN_COILS || access == APP_IO_IN_DISCRETE || access == APP_IO_OUT_COILS)
  {
    return (uint16_t)((count + 7U) / 8U);
  }
  return (uint16_t)(count * 2U);
}

/* EVERY reset clears the OUTPUT image (called pre-RTOS from app_periph_init; user ruling
 * 2026-07-25, = the PLC model: outputs are recomputed by logic after any restart, never
 * remembered). Covers the SRAM4-remanence trap too: a brief mains dip can leave the magic
 * valid, and before this rule the retained commands would silently re-energize the loads —
 * exactly the "unexpected restart" industrial safety forbids. Restore is explicit and
 * upper-layer only: cloud/dash re-commands, or the application replays its own PF_RETAIN
 * state on first scan. A torn concurrent CM4 read during this memset can only produce a
 * subset of the OLD on-bits (zeros mixed in), never a new on-bit — safe. */
void app_io_outputs_clear_boot(void)
{
  if (PIMG->ctrl.magic == PIMG_MAGIC)
  {
    memset((void *)PIMG->out, 0, PIMG_OUT_BYTES);
  }
}

int app_io_setup(const app_io_point_t *pts, uint16_t n, uint16_t base_tick_ms)
{
  /* Customer points start above the reserved platform regions (onboard HSDI / slot C / slot D /
   * general) so that published Modbus addresses never shift when that I/O lands — see the
   * region map in app_pimage.h. */
  uint16_t in_off = PIMG_RGN_SCAN, out_off = PIMG_RGN_SCAN;
  if ((pts == NULL) || (n > PIMG_MAX_ENTRIES)) { return -1; }
  __HAL_RCC_HSEM_CLK_ENABLE();                          /* output-image cross-core lock */

  /* SRAM4 is reset-persistent: on a cold power-on it holds garbage. Wipe the whole
   * block once, keyed on the magic, so cfg_ver/status start clean (not garbage+1). */
  if (PIMG->ctrl.magic != PIMG_MAGIC)
  {
    memset((void *)PIMG, 0, sizeof(pimg_shared_t));
    /* Same cold boot, same reason: the fault records live in SRAM4 too and are only guarded by a
     * magic, so power-on garbage can match it by chance and greet a brand-new board with a
     * fabricated "STACK OVERFLOW". A record left by a real fault is untouched here, because that
     * reset leaves the process-image magic valid. */
    HEALTH_CM7->magic = 0;
    HEALTH_CM4->magic = 0;
  }
  /* Status area (incl. the live seq counters) belongs to CM4 — wiping it here once inverted
   * a mid-write seq's parity and left readers spinning forever. CM4 clears its own entries'
   * bookkeeping when each bus thread swaps to the new table. Residual stale-odd seqs (e.g.
   * onboard_seq, which no table entry owns) are healed by the writer itself and survived by
   * the bounded reader — see pimg_seq_write/read in app_pimage.h (2026-07-26). */

  for (uint16_t i = 0; i < n; i++)
  {
    const app_io_point_t *p = &pts[i];
    uint16_t nb = slice_bytes(p->access, p->count);
    pimg_entry_t e;
    if ((p->access > APP_IO_OUT_HOLDING) || (p->count == 0U) || (nb > PIMG_SLICE_MAX)) { return -2; }
    memset(&e, 0, sizeof(e));
    e.port = p->port; e.addr = p->addr; e.access = p->access; e.flags = p->flags;
    e.start = p->start; e.count = p->count;
    e.period = p->period;                             /* 0 = entry disabled (contract §3) */
    e.seq_idx = i;
    /* Every point starts on a 2-byte boundary so it lands on a clean Modbus register
     * boundary in the external window (register n = image bytes [2n, 2n+1]). */
    uint16_t step = (uint16_t)((nb + 1U) & ~1U);
    if (p->access <= APP_IO_IN_HOLDING)                 /* input access -> input image */
    {
      if (((uint32_t)in_off + step) > PIMG_IN_BYTES) { return -3; }
      e.img_off = in_off; in_off = (uint16_t)(in_off + step);
    }
    else                                                /* output access -> output image */
    {
      if (((uint32_t)out_off + step) > PIMG_OUT_BYTES) { return -3; }
      e.img_off = out_off; out_off = (uint16_t)(out_off + step);
    }
    PIMG->cfg[i] = e;
  }

  /* Outputs HOLD across a reconfig (the contract's reload rule — the user's explicit
   * decision). First power-on starts de-asserted via the cold-boot wipe above; a runtime
   * reconfig must not blink a single output. Note the corollary, stated in the contract:
   * if the new table lays points out differently, held bytes land under the new layout. */
  if (s_io_mtx == NULL) { s_io_mtx = xSemaphoreCreateMutex(); }
  s_npts = n;

  PIMG->ctrl.n_entries    = n;
  PIMG->ctrl.base_tick_ms = base_tick_ms ? base_tick_ms : 1U;
  PIMG->ctrl.running      = 1U;
  PIMG->ctrl.magic        = PIMG_MAGIC;                 /* gate opens after cfg[] is fully written */
  PIMG->ctrl.cfg_ver     += 1U;                         /* trigger: CM4 reloads (also auto-detected in its poll) */

  { uint8_t req[2] = { 0x0CU, 0U }, rsp[4];             /* best-effort doorbell; CM4 picks it up regardless */
    (void)app_rpc_transact(req, 2, rsp, sizeof(rsp), 50); }
  /* contract §8 step 4: wait for CM4's ack that EVERY bus thread swapped to the new table.
   * ~200 ms covers the worst safe point (one in-flight front-port transaction). */
  for (uint16_t w = 0; w < 200U; w++)
  {
    if (PIMG->ctrl.cfg_applied == PIMG->ctrl.cfg_ver) { return 0; }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return -4;                                            /* CM4 did not confirm — table stored, not live */
}

/* ---- accessors: pt = point index returned implicitly by app_io_setup order ---- */

uint8_t app_io_di(uint16_t pt, uint16_t ch)
{
  uint8_t tmp[PIMG_SLICE_MAX];
  volatile pimg_entry_t *e;
  uint16_t nb;
  if (pt >= s_npts) { return 0U; }
  e = &PIMG->cfg[pt];
  if (e->access > APP_IO_IN_DISCRETE) { return 0U; }    /* not a bit input */
  if (ch >= e->count) { return 0U; }
  nb = slice_bytes(e->access, e->count);
  pimg_seq_read(&PIMG->st.seq[e->seq_idx], &PIMG->in[e->img_off], tmp, nb);
  return pimg_bit_get(tmp, ch);
}

uint16_t app_io_ai(uint16_t pt, uint16_t ch)
{
  uint8_t tmp[PIMG_SLICE_MAX];
  volatile pimg_entry_t *e;
  uint16_t nb;
  if (pt >= s_npts) { return 0U; }
  e = &PIMG->cfg[pt];
  if ((e->access != APP_IO_IN_INPUT_REG) && (e->access != APP_IO_IN_HOLDING)) { return 0U; }
  if (ch >= e->count) { return 0U; }
  nb = slice_bytes(e->access, e->count);
  pimg_seq_read(&PIMG->st.seq[e->seq_idx], &PIMG->in[e->img_off], tmp, nb);
  return (uint16_t)(tmp[ch * 2U] | ((uint16_t)tmp[ch * 2U + 1U] << 8));   /* native LE store */
}

void app_io_do_set(uint16_t pt, uint16_t ch, uint8_t on)
{
  uint8_t tmp[PIMG_SLICE_MAX];
  volatile pimg_entry_t *e;
  uint16_t nb;
  if (pt >= s_npts) { return; }
  e = &PIMG->cfg[pt];
  if ((e->access != APP_IO_OUT_COILS) || (ch >= e->count)) { return; }
  nb = slice_bytes(e->access, e->count);
  if (s_io_mtx != NULL) { xSemaphoreTake(s_io_mtx, portMAX_DELAY); }   /* CM7 task vs CM7 task */
  pimg_out_lock();                                                     /* CM7 vs CM4 (external master) */
  for (uint16_t i = 0; i < nb; i++) { tmp[i] = PIMG->out[e->img_off + i]; }
  pimg_bit_set(tmp, ch, on);
  pimg_seq_write(&PIMG->st.seq[e->seq_idx], &PIMG->out[e->img_off], tmp, nb);
  pimg_out_unlock();
  if (s_io_mtx != NULL) { xSemaphoreGive(s_io_mtx); }
}

uint8_t app_io_do_get(uint16_t pt, uint16_t ch)
{
  volatile pimg_entry_t *e;
  if (pt >= s_npts) { return 0U; }
  e = &PIMG->cfg[pt];
  if ((e->access != APP_IO_OUT_COILS) || (ch >= e->count)) { return 0U; }
  return (uint8_t)((PIMG->out[e->img_off + (ch >> 3)] >> (ch & 7U)) & 1U);   /* read back CM7's own committed value */
}

void app_io_hr_set(uint16_t pt, uint16_t ch, uint16_t v)
{
  uint8_t tmp[PIMG_SLICE_MAX];
  volatile pimg_entry_t *e;
  uint16_t nb;
  if (pt >= s_npts) { return; }
  e = &PIMG->cfg[pt];
  if ((e->access != APP_IO_OUT_HOLDING) || (ch >= e->count)) { return; }
  nb = slice_bytes(e->access, e->count);
  if (s_io_mtx != NULL) { xSemaphoreTake(s_io_mtx, portMAX_DELAY); }   /* CM7 task vs CM7 task */
  pimg_out_lock();                                                     /* CM7 vs CM4 (external master) */
  for (uint16_t i = 0; i < nb; i++) { tmp[i] = PIMG->out[e->img_off + i]; }
  tmp[ch * 2U] = (uint8_t)(v & 0xFFU);
  tmp[ch * 2U + 1U] = (uint8_t)(v >> 8);
  pimg_seq_write(&PIMG->st.seq[e->seq_idx], &PIMG->out[e->img_off], tmp, nb);
  pimg_out_unlock();
  if (s_io_mtx != NULL) { xSemaphoreGive(s_io_mtx); }
}

int app_io_ok(uint16_t pt)
{
  if (pt >= s_npts) { return -1; }
  return (int)PIMG->st.ok[pt];
}

/* ================= onboard HSDI (8 channels, CN4) =================
 * The configuration rides the same shared block and the same reload handshake as the scan
 * table; CM4 owns the timers/EXTI and publishes results into the ONBOARD image region.
 * Contract: docs/HSDI_Configuration_and_Counting.md. */

int app_hsdi_setup(const app_hsdi_ch_t cfg[8])
{
  pimg_hsdi_cfg_t t[PIMG_HSDI_CH];
  if (cfg == NULL) { return -1; }
  for (uint8_t i = 0; i < PIMG_HSDI_CH; i++)
  {
    memset(&t[i], 0, sizeof(t[i]));                   /* _rsv really zero — this once shipped stack garbage */
    t[i].mode = cfg[i].mode; t[i].filter = cfg[i].filter;
    t[i].edge = cfg[i].edge; t[i].z_action = cfg[i].z_action;
    t[i].debounce_ms = cfg[i].debounce_ms;            /* the field the first cut forgot: every channel
                                                       * got a random 0..255 ms debounce per boot */
    t[i].min_us = cfg[i].min_us;
  }
  {
    int rc = pimg_hsdi_check(t);                      /* same rules CM4 enforces — reject NOW, loudly */
    if (rc != 0) { return rc; }
  }
  if (PIMG->ctrl.magic != PIMG_MAGIC) { memset((void *)PIMG, 0, sizeof(pimg_shared_t)); }
  for (uint8_t i = 0; i < PIMG_HSDI_CH; i++) { PIMG->hsdi[i] = t[i]; }
  PIMG->ctrl.magic = PIMG_MAGIC;
  PIMG->ctrl.cfg_ver += 1U;                          /* same doorbell as the scan table */
  { uint8_t req[2] = { 0x0CU, 0U }, rsp[4];
    (void)app_rpc_transact(req, 2, rsp, sizeof(rsp), 50); }
  for (uint16_t w = 0; w < 200U; w++)                /* same ack as the scan table */
  {
    if (PIMG->ctrl.cfg_applied == PIMG->ctrl.cfg_ver) { return 0; }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return -4;
}

/* one consistent snapshot of the ONBOARD region (CM4 writes it under onboard_seq) */
static void hsdi_snap(uint8_t *dst)
{
  pimg_seq_read(&PIMG->st.onboard_seq, &PIMG->in[PIMG_RGN_ONBOARD], dst, PIMG_RGN_ONBOARD_SZ);
}
static uint32_t rd32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint8_t app_hsdi_level(uint8_t ch)
{
  uint8_t b[PIMG_RGN_ONBOARD_SZ];
  if (ch >= PIMG_HSDI_CH) { return 0U; }
  hsdi_snap(b);
  return (uint8_t)((b[PIMG_HSDI_LEVELS] >> ch) & 1U);
}

uint32_t app_hsdi_count(uint8_t ch)
{
  uint8_t b[PIMG_RGN_ONBOARD_SZ];
  if (ch >= PIMG_HSDI_CH) { return 0U; }
  hsdi_snap(b);
  return rd32(&b[PIMG_HSDI_COUNT + 4U * ch]);
}

int32_t app_hsdi_position(uint8_t enc)
{
  uint8_t b[PIMG_RGN_ONBOARD_SZ];
  uint8_t ch = (enc == 0U) ? 0U : 4U;                /* encoder 0 = TIM3 (HSDI0), 1 = TIM1 (HSDI4) */
  if (enc > 1U) { return 0; }
  hsdi_snap(b);
  return (int32_t)rd32(&b[PIMG_HSDI_COUNT + 4U * ch]);
}

uint32_t app_hsdi_revs(uint8_t enc)
{
  uint8_t b[PIMG_RGN_ONBOARD_SZ];
  if (enc > 1U) { return 0U; }
  hsdi_snap(b);
  return rd32(&b[PIMG_HSDI_ZREV + 4U * enc]);
}

uint32_t app_hsdi_zlatch(uint8_t enc)
{
  uint8_t b[PIMG_RGN_ONBOARD_SZ];
  if (enc > 1U) { return 0U; }
  hsdi_snap(b);
  return rd32(&b[PIMG_HSDI_ZLATCH + 4U * enc]);
}

uint8_t app_hsdi_mode_of(uint8_t ch)                 /* configured mode, for the CLI view */
{
  return (ch < PIMG_HSDI_CH) ? PIMG->hsdi[ch].mode : 0U;
}

void app_hsdi_reset(uint8_t ch)                      /* reset bits live in the output image */
{
  if (ch >= PIMG_HSDI_CH) { return; }
  if (s_io_mtx != NULL) { xSemaphoreTake(s_io_mtx, portMAX_DELAY); }
  pimg_out_lock();
  PIMG->out[PIMG_RGN_ONBOARD + PIMG_HSDI_RESET] |= (uint8_t)(1U << ch);
  pimg_out_unlock();
  if (s_io_mtx != NULL) { xSemaphoreGive(s_io_mtx); }
}

/* ---- bench CLI: exercise the process image against a PC Modbus slave ---- */
static const char *acc_str(uint8_t a)
{
  static const char *n[] = { "inCOIL", "inDISC", "inIREG", "inHOLD", "outCOIL", "outHOLD" };
  return (a <= APP_IO_OUT_HOLDING) ? n[a] : "?";
}
static uint32_t next_uint(char **p, uint32_t def)
{
  char *e; uint32_t v = (uint32_t)strtoul(*p, &e, 0);
  if (e == *p) { return def; }
  *p = e; return v;
}

int app_pio_cli(char *line)   /* returns 1 if it handled the line */
{
  if (strncmp(line, "pio", 3) != 0) { return 0; }

  if ((line[3] == 0) || (strcmp(line, "pio dump") == 0))
  {
    if ((PIMG->ctrl.magic != PIMG_MAGIC) || (s_npts == 0U))
    { printf("pio: no scan table (run 'pio setup')\n\r"); return 1; }
    printf("scan: %u pts base=%ums ver=%lu applied=%lu run=%u\n\r",
           (unsigned)s_npts, (unsigned)PIMG->ctrl.base_tick_ms,
           (unsigned long)PIMG->ctrl.cfg_ver, (unsigned long)PIMG->ctrl.cfg_applied,
           (unsigned)PIMG->ctrl.running);
    for (uint16_t i = 0; i < s_npts; i++)
    {
      volatile pimg_entry_t *e = &PIMG->cfg[i];
      uint16_t nb = slice_bytes(e->access, e->count);
      const volatile uint8_t *img = (e->access <= APP_IO_IN_HOLDING)
                                    ? &PIMG->in[e->img_off] : &PIMG->out[e->img_off];
      /* external Modbus window address for this point (what an HMI/SCADA integrator needs):
       * word spaces address registers (byte/2), bit spaces address bits (byte*8). */
      unsigned wreg = (unsigned)(PIMG_WIN_BASE + (e->img_off / 2U));
      unsigned wbit = (unsigned)(PIMG_WIN_BASE + (e->img_off * 8U));
      int isbit = (e->access == APP_IO_IN_COILS) || (e->access == APP_IO_IN_DISCRETE) ||
                  (e->access == APP_IO_OUT_COILS);
      int isin  = (e->access <= APP_IO_IN_HOLDING);
      printf(" [%u] p%u a%u %s st%u n%u per%u ok%u fail%u exc%u | ext %s 0x%04X..0x%04X | ",
             (unsigned)i, (unsigned)e->port, (unsigned)e->addr, acc_str(e->access),
             (unsigned)e->start, (unsigned)e->count, (unsigned)e->period,
             (unsigned)PIMG->st.ok[i], (unsigned)PIMG->st.fails[i], (unsigned)PIMG->st.last_exc[i],
             isbit ? (isin ? "FC02" : "FC01/05/15") : (isin ? "FC04" : "FC03/06/16"),
             isbit ? wbit : wreg,
             isbit ? (wbit + e->count - 1U) : (wreg + e->count - 1U));
      for (uint16_t b = 0; (b < nb) && (b < 16U); b++) { printf("%02X", img[b]); }
      printf("\n\r");
    }
    for (uint8_t bus = 0; bus < PIMG_NBUS; bus++)
    {
      if (PIMG->st.polls[bus] || PIMG->st.online[bus])
      {
        printf(" bus%u online=%04X polls=%lu fails=%lu ovr=%lu cycle=%luus\n\r", (unsigned)bus,
               (unsigned)PIMG->st.online[bus], (unsigned long)PIMG->st.polls[bus],
               (unsigned long)PIMG->st.bfails[bus], (unsigned long)PIMG->st.overrun[bus],
               (unsigned long)PIMG->st.cycle_us[bus]);
      }
    }
    return 1;
  }

  if (strncmp(line, "pio setup", 9) == 0)
  {
    char *p = line + 9;
    uint8_t port = (uint8_t)next_uint(&p, APP_PORT_485B);
    uint8_t addr = (uint8_t)next_uint(&p, 1U);
    app_io_point_t pts[4] = {
      { port, addr, APP_IO_IN_DISCRETE,  0, 0, 16, 1 },   /* pt0: PC discrete inputs -> image */
      { port, addr, APP_IO_IN_INPUT_REG, 0, 0,  8, 2 },   /* pt1: PC input registers -> image */
      { port, addr, APP_IO_OUT_COILS,    0, 0, 16, 1 },   /* pt2: image -> PC coils */
      { port, addr, APP_IO_OUT_HOLDING,  0, 0,  8, 5 },   /* pt3: image -> PC holding registers */
    };
    int r = app_io_setup(pts, 4, 1);
    printf("pio setup port%u addr%u -> %d. First set the port master, e.g. 'mbcfg 485b master 9600 8N1'\n\r",
           (unsigned)port, (unsigned)addr, r);
    return 1;
  }

  if (strncmp(line, "pio do ", 7) == 0)
  {
    char *p = line + 7;
    uint16_t pt = (uint16_t)next_uint(&p, 2U);
    uint16_t ch = (uint16_t)next_uint(&p, 0U);
    uint8_t  v  = (uint8_t)next_uint(&p, 0U);
    app_io_do_set(pt, ch, (uint8_t)(v != 0U));
    printf("do pt%u ch%u = %u\n\r", (unsigned)pt, (unsigned)ch, (unsigned)(v != 0U));
    return 1;
  }
  if (strncmp(line, "pio hr ", 7) == 0)
  {
    char *p = line + 7;
    uint16_t pt = (uint16_t)next_uint(&p, 3U);
    uint16_t ch = (uint16_t)next_uint(&p, 0U);
    uint16_t v  = (uint16_t)next_uint(&p, 0U);
    app_io_hr_set(pt, ch, v);
    printf("hr pt%u ch%u = %u\n\r", (unsigned)pt, (unsigned)ch, (unsigned)v);
    return 1;
  }

  printf("pio: dump | setup [port addr] | do <pt> <ch> <0|1> | hr <pt> <ch> <val>\n\r");
  return 1;
}
