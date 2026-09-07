/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_qflash.h — W25Q256 QSPI driver + LittleFS mount interface */
#ifndef APP_QFLASH_H
#define APP_QFLASH_H

#include <stdint.h>

#define QFLASH_PAGE     256U
#define QFLASH_SECTOR   4096U     /* minimum erase unit = LittleFS block */
#define QFLASH_SECTORS  8192U     /* 32MB / 4KB (whole chip) */

/* Partitioning: littlefs = first 30MB, last 2MB = power-fail save area (raw page writes,
 * pre-erased in normal operation, only sequential page programming at end-of-life; no erase, no file system —— same-tier iron rule as `Universal_Modbus_Port_Config.md`) */
#define QFLASH_LFS_SECTORS 7680U          /* littlefs partition 30MB */
#define QFLASH_PF_BASE     0x01E00000U    /* power-fail save area start (30MB) */
#define QFLASH_PF_SIZE     0x00200000U    /* 2MB */

uint32_t app_qflash_init(void);   /* returns JEDEC ID (EF4019) or a 0xEE000x error code */
int app_qflash_read(uint32_t addr, void *buf, uint32_t len);
int app_qflash_write(uint32_t addr, const void *buf, uint32_t len);
int app_qflash_erase4k(uint32_t addr);
void app_qflash_dying_prep(void); /* power-fail dying preparation (ISR-usable): forcibly seize HAL handle + Erase/Program Suspend (0x75) */

/* LittleFS: mount (format if necessary) + boot-count file boot.cnt; returns a short string like "lfs:OK boot:N" */
const char *app_lfs_selftest(void);
int app_lfs_mount(void);   /* idempotent early mount (identity partition needs it before TLS init); 0=OK */

#endif
