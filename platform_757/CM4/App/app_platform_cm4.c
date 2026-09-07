/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_platform_cm4.c — CM4 implementation of the application-facing platform API.
 *
 * Thin accessors over the process image, matching the CM7 API one for one so control logic can
 * move between cores unchanged. Contract: docs/Process_Image_and_IO_Mapping.md.
 *
 * Output-image locking on this core needs TWO layers, and both are mandatory:
 *   - a CM4 mutex, because there are now several CM4 writers (this API from a user thread, and
 *     the board slave map answering an external master from a bus thread);
 *   - the HSEM, because CM7 writes the same image.
 * The HSEM alone is not enough: its one-step read lock returns "mine" when the SAME core
 * already holds it, so a second CM4 thread would sail straight through it.
 */
#include <string.h>
#include "cmsis_os.h"
#include "app_pimage.h"
#include "app_platform_cm4.h"

#define SLICE_MAX PIMG_SLICE_MAX

static osMutexId_t s_out_mtx;

void app_cm4_out_lock(void)      /* also used by the slave-map write path (app_mbfront_cm4.c) */
{
  if (s_out_mtx == NULL) { s_out_mtx = osMutexNew(NULL); }
  if (s_out_mtx != NULL) { (void)osMutexAcquire(s_out_mtx, osWaitForever); }
  pimg_out_lock();
}
void app_cm4_out_unlock(void)
{
  pimg_out_unlock();
  if (s_out_mtx != NULL) { (void)osMutexRelease(s_out_mtx); }
}

static uint16_t slice_of(volatile pimg_entry_t *e)
{
  return pimg_slice_bytes(e->access, e->count);
}
static int pt_ok(uint16_t pt)
{
  return (PIMG->ctrl.magic == PIMG_MAGIC) && (pt < PIMG->ctrl.n_entries);
}

uint8_t app_io_di(uint16_t pt, uint16_t ch)
{
  uint8_t tmp[SLICE_MAX];
  volatile pimg_entry_t *e;
  if (!pt_ok(pt)) { return 0U; }
  e = &PIMG->cfg[pt];
  if ((e->access > PIMG_IN_DISCRETE) || (ch >= e->count)) { return 0U; }
  pimg_seq_read(&PIMG->st.seq[e->seq_idx], &PIMG->in[e->img_off], tmp, slice_of(e));
  return pimg_bit_get(tmp, ch);
}

uint16_t app_io_ai(uint16_t pt, uint16_t ch)
{
  uint8_t tmp[SLICE_MAX];
  volatile pimg_entry_t *e;
  if (!pt_ok(pt)) { return 0U; }
  e = &PIMG->cfg[pt];
  if ((e->access != PIMG_IN_INPUT_REG) && (e->access != PIMG_IN_HOLDING)) { return 0U; }
  if (ch >= e->count) { return 0U; }
  pimg_seq_read(&PIMG->st.seq[e->seq_idx], &PIMG->in[e->img_off], tmp, slice_of(e));
  return (uint16_t)(tmp[ch * 2U] | ((uint16_t)tmp[ch * 2U + 1U] << 8));
}

void app_io_do_set(uint16_t pt, uint16_t ch, uint8_t on)
{
  uint8_t tmp[SLICE_MAX];
  volatile pimg_entry_t *e;
  uint16_t nb;
  if (!pt_ok(pt)) { return; }
  e = &PIMG->cfg[pt];
  if ((e->access != PIMG_OUT_COILS) || (ch >= e->count)) { return; }
  nb = slice_of(e);
  app_cm4_out_lock();
  for (uint16_t i = 0; i < nb; i++) { tmp[i] = PIMG->out[e->img_off + i]; }
  pimg_bit_set(tmp, ch, on);
  pimg_seq_write(&PIMG->st.seq[e->seq_idx], &PIMG->out[e->img_off], tmp, nb);
  app_cm4_out_unlock();
}

uint8_t app_io_do_get(uint16_t pt, uint16_t ch)
{
  volatile pimg_entry_t *e;
  if (!pt_ok(pt)) { return 0U; }
  e = &PIMG->cfg[pt];
  if ((e->access != PIMG_OUT_COILS) || (ch >= e->count)) { return 0U; }
  return (uint8_t)((PIMG->out[e->img_off + (ch >> 3)] >> (ch & 7U)) & 1U);
}

void app_io_hr_set(uint16_t pt, uint16_t ch, uint16_t v)
{
  uint8_t tmp[SLICE_MAX];
  volatile pimg_entry_t *e;
  uint16_t nb;
  if (!pt_ok(pt)) { return; }
  e = &PIMG->cfg[pt];
  if ((e->access != PIMG_OUT_HOLDING) || (ch >= e->count)) { return; }
  nb = slice_of(e);
  app_cm4_out_lock();
  for (uint16_t i = 0; i < nb; i++) { tmp[i] = PIMG->out[e->img_off + i]; }
  tmp[ch * 2U] = (uint8_t)(v & 0xFFU);
  tmp[ch * 2U + 1U] = (uint8_t)(v >> 8);
  pimg_seq_write(&PIMG->st.seq[e->seq_idx], &PIMG->out[e->img_off], tmp, nb);
  app_cm4_out_unlock();
}

int app_io_ok(uint16_t pt)
{
  if (!pt_ok(pt)) { return -1; }
  return (int)PIMG->st.ok[pt];
}

/* ---- onboard high-speed inputs, straight out of the image this core publishes ---- */
static void hsdi_snap(uint8_t *dst)
{
  pimg_seq_read(&PIMG->st.onboard_seq, &PIMG->in[PIMG_RGN_ONBOARD], dst, PIMG_RGN_ONBOARD_SZ);
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
  const uint8_t *p;
  if (ch >= PIMG_HSDI_CH) { return 0U; }
  hsdi_snap(b);
  p = &b[PIMG_HSDI_COUNT + 4U * ch];
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
