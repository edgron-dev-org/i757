/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_lfs.c — LittleFS (v2.9.3, App/littlefs/ committed as-is) on the FIRST 30MB of the W25Q256 (the last 2MB is the raw power-fail save area, app_qflash.h)
 * Why not FatFs: NOR has no wear leveling, the FAT table region would be worn through; LittleFS = ARM official NOR standard
 * (power-fail safe + wear leveling). block = 4KB sector, all static buffers (LFS_NO_MALLOC enforced at compile time).
 * A blank chip (no littlefs superblock in blocks 0/1) is auto-formatted; resident mount, subsequent config/logs use s_lfs directly.
 * 2026-11-17 (review P7):
 *  - one lfs_t is used by defaultTask, the tcpip thread and the fertigation task, so littlefs is built
 *    with LFS_THREADSAFE and every API call takes the mutex below;
 *  - a failed mount no longer formats a chip that carries a filesystem (identity, customer signing key,
 *    network config and the module firmware store live here). It retries, then stays unmounted for this
 *    boot. `lfs-format yes` on the USB console is the deliberate way to wipe it. */
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "lfs.h"
#include "app_qflash.h"

static StaticSemaphore_t s_mtx_mem;
static SemaphoreHandle_t s_mtx = NULL;

static int lp_lock(const struct lfs_config *c)
{
  (void)c;
  if ((s_mtx == NULL) || (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)) { return 0; }   /* early boot: single context */
  return (xSemaphoreTake(s_mtx, portMAX_DELAY) == pdTRUE) ? 0 : LFS_ERR_IO;
}

static int lp_unlock(const struct lfs_config *c)
{
  (void)c;
  if ((s_mtx == NULL) || (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)) { return 0; }
  (void)xSemaphoreGive(s_mtx);
  return 0;
}

static int lp_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                   void *buffer, lfs_size_t size)
{
  (void)c;
  return (app_qflash_read(block * QFLASH_SECTOR + off, buffer, size) == 0) ? 0 : LFS_ERR_IO;
}

static int lp_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                   const void *buffer, lfs_size_t size)
{
  (void)c;
  return (app_qflash_write(block * QFLASH_SECTOR + off, buffer, size) == 0) ? 0 : LFS_ERR_IO;
}

static int lp_erase(const struct lfs_config *c, lfs_block_t block)
{
  (void)c;
  return (app_qflash_erase4k(block * QFLASH_SECTOR) == 0) ? 0 : LFS_ERR_IO;
}

static int lp_sync(const struct lfs_config *c) { (void)c; return 0; }

static uint8_t s_read_buf[256];
static uint8_t s_prog_buf[256];
static uint8_t s_look_buf[256] __attribute__((aligned(8)));
static uint8_t s_file_buf[256];

static const struct lfs_config s_cfg = {
  .read = lp_read, .prog = lp_prog, .erase = lp_erase, .sync = lp_sync,
  .lock = lp_lock, .unlock = lp_unlock,
  .read_size      = 256,
  .prog_size      = 256,             /* = W25Q page */
  .block_size     = QFLASH_SECTOR,
  .block_count    = QFLASH_LFS_SECTORS,   /* 30MB */
  .block_cycles   = 500,             /* wear-leveling relocation period */
  .cache_size     = 256,
  .lookahead_size = 256,
  .read_buffer      = s_read_buf,
  .prog_buffer      = s_prog_buf,
  .lookahead_buffer = s_look_buf,
};

static lfs_t s_lfs;
static int s_mounted = 0;

lfs_t *app_lfs(void) { return s_mounted ? &s_lfs : (lfs_t *)0; }

/* 1 = a littlefs superblock is on the chip, 0 = both superblock blocks read fine and carry none
 * (blank / foreign chip), -1 = could not read (unknown: never format on this answer). */
static int fs_present(void)
{
  uint8_t b[16];
  int bad = 0;
  for (uint32_t blk = 0; blk < 2U; blk++)
  {
    if (app_qflash_read(blk * QFLASH_SECTOR, b, sizeof b) != 0) { bad = 1; continue; }
    if (memcmp(&b[8], "littlefs", 8) == 0) { return 1; }
  }
  return bad ? -1 : 0;
}

/* Idempotent mount (formats only a blank chip). Callable early (app_identity needs the identity
 * file before TLS config creation); the P5 self-test reuses the same mount. */
int app_lfs_mount(void)
{
  int err = 0;
  if (s_mounted) { return 0; }
  if (s_mtx == NULL) { s_mtx = xSemaphoreCreateMutexStatic(&s_mtx_mem); }
  for (int tries = 0; tries < 3; tries++)       /* a single QSPI read glitch must not look like "no filesystem" */
  {
    err = lfs_mount(&s_lfs, &s_cfg);
    if (err == 0) { s_mounted = 1; return 0; }
    HAL_Delay(20);
  }
  if (fs_present() != 0)
  {
    printf("[LFS] mount err %d, filesystem present or chip unreadable -> NOT formatting (USB: lfs-format yes)\n\r", err);
    return err;
  }
  printf("[LFS] mount err %d on a blank chip -> format\n\r", err);
  if (lfs_format(&s_lfs, &s_cfg) != 0 || lfs_mount(&s_lfs, &s_cfg) != 0)
  {
    return err;
  }
  s_mounted = 1;
  return 0;
}

/* USB console `lfs-format yes`: wipe a filesystem that will not mount. Refused while mounted. */
int app_lfs_format_force(void)
{
  if (s_mounted) { return -1; }
  if (s_mtx == NULL) { s_mtx = xSemaphoreCreateMutexStatic(&s_mtx_mem); }
  if (lfs_format(&s_lfs, &s_cfg) != 0 || lfs_mount(&s_lfs, &s_cfg) != 0) { return -2; }
  s_mounted = 1;
  return 0;
}

const char *app_lfs_selftest(void)
{
  static char r[24];
  uint32_t n = 0;
  lfs_file_t f;
  static const struct lfs_file_config fc = { .buffer = s_file_buf };

  int err = app_lfs_mount();
  if (err != 0)
  {
    snprintf(r, sizeof r, "lfs:ERR%d", err);
    return r;
  }

  /* Boot-count file: +1 written back each power-up = read/write/persistence three-in-one self-proof */
  if (lfs_file_opencfg(&s_lfs, &f, "boot.cnt", LFS_O_RDWR | LFS_O_CREAT,
                       (struct lfs_file_config *)&fc) == 0)
  {
    if (lfs_file_read(&s_lfs, &f, &n, sizeof n) != (lfs_ssize_t)sizeof n) { n = 0; }
    n++;
    lfs_file_rewind(&s_lfs, &f);
    (void)lfs_file_write(&s_lfs, &f, &n, sizeof n);
    (void)lfs_file_close(&s_lfs, &f);
    snprintf(r, sizeof r, "lfs:OK boot:%lu", (unsigned long)n);
  }
  else
  {
    snprintf(r, sizeof r, "lfs:OPENERR");
  }
  return r;
}
