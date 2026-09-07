/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_lfs.c — LittleFS (v2.9.3, App/littlefs/ committed as-is) on the FIRST 30MB of the W25Q256 (the last 2MB is the raw power-fail save area, app_qflash.h)
 * Why not FatFs: NOR has no wear leveling, the FAT table region would be worn through; LittleFS = ARM official NOR standard
 * (power-fail safe + wear leveling). block = 4KB sector, all static buffers (LFS_NO_MALLOC enforced at compile time).
 * First mount failure (blank chip) auto-formats; resident mount, subsequent config/logs use s_lfs directly. */
#include <stdio.h>
#include "lfs.h"
#include "app_qflash.h"

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
  .read_size      = 256,
  .prog_size      = 256,             /* = W25Q page */
  .block_size     = QFLASH_SECTOR,
  .block_count    = QFLASH_LFS_SECTORS,   /* 30MB;
                                             old 32MB superblock size mismatches -> first mount fails, auto-format (mbport.cfg/t*.fw rebuilt) */
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

/* Idempotent mount (format-if-blank). Callable early (app_identity needs the identity
 * file before TLS config creation); the P5 self-test reuses the same mount. */
int app_lfs_mount(void)
{
  if (s_mounted) { return 0; }
  int err = lfs_mount(&s_lfs, &s_cfg);
  if (err != 0)
  {
    printf("[LFS] mount err %d -> format\n\r", err);   /* blank chip / foreign data: expected path */
    if (lfs_format(&s_lfs, &s_cfg) != 0 || lfs_mount(&s_lfs, &s_cfg) != 0)
    {
      return err;
    }
  }
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
