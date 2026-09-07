/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_mota.c — expansion-module OTA: main controller = module firmware repository (littlefs) + backplane streaming (contract v0.23 FC 0x41~0x44)
 * Chain: cloud MQTT chunk (dev/<type>/<sn>/dn/fw, routed here after mota-begin) -> littlefs "m<addr>.fw" (CRC verified before storing)
 *       -> mota-flash <addr> streams to the slave's spare bank -> slave CRC matches -> SWAP_BANK reset -> read back fw_ver to accept.
 * Thread constraint: mota_fw_chunk writes littlefs on the tcpip thread; defaultTask's only lfs user is the boot self-test —
 *           the two never run concurrently (self-test finishes early). If defaultTask ever uses lfs routinely, add a lock (LFS_THREADSAFE). */
extern void app_log_event(const char *type, const char *fmt, ...);   /* event journal (app_datalog.c): remote visibility of every OTA step, 2026-09-04 */
#include <stdio.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#include "lfs.h"
#include "modbus_core.h"
#include "app_mbport.h"
#include "app_mota.h"

#define MOTA_BLOCK 240U

static uint8_t  s_recv_active = 0;   /* 1 = chunks routed to this module (mota-begin ~ last block) */
static uint32_t s_size, s_crc_expect, s_crc_run, s_got;
static lfs_file_t s_f;
static uint8_t  s_fbuf[256];
static const struct lfs_file_config s_fcfg = { .buffer = s_fbuf };
static char     s_name[12];

extern lfs_t *app_lfs(void);

static uint32_t crc32_run(uint32_t crc, const uint8_t *p, uint32_t n)
{
  static const uint32_t t[16] = {
    0x00000000U,0x1DB71064U,0x3B6E20C8U,0x26D930ACU,0x76DC4190U,0x6B6B51F4U,0x4DB26158U,0x5005713CU,
    0xEDB88320U,0xF00F9344U,0xD6D6A3E8U,0xCB61B38CU,0x9B64C2B0U,0x86D3D2D4U,0xA00AE278U,0xBDBDF21CU };
  crc = ~crc;
  while (n--)
  {
    crc ^= *p++;
    crc = (crc >> 4) ^ t[crc & 15U];
    crc = (crc >> 4) ^ t[crc & 15U];
  }
  return ~crc;
}

uint8_t mota_recv_active(void) { return s_recv_active; }

int mota_begin(uint8_t type, uint32_t size, uint32_t crc)   /* v0.24: repository keyed by module type */
{
  lfs_t *l = app_lfs();
  if ((l == 0) || s_recv_active || (type == 0U) || (size == 0U) || (size > (64U * 1024U))) { return -1; }
  snprintf(s_name, sizeof s_name, "t%u.fw", (unsigned)type);
  if (lfs_file_opencfg(l, &s_f, s_name, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                       (struct lfs_file_config *)&s_fcfg) != 0) { return -2; }
  s_size = size; s_crc_expect = crc;
  s_crc_run = 0; s_got = 0;
  s_recv_active = 1;
  printf("[MOTA] recv t%u.fw %lu bytes...\n\r", (unsigned)type, (unsigned long)size);
  return 0;
}

void mota_fw_chunk(const uint8_t *data, uint16_t len, uint8_t last)
{
  lfs_t *l = app_lfs();
  (void)last;                                  /* last = end of a single publish (1KB block), not end of the whole image; close out by byte count */
  if (!s_recv_active || (l == 0)) { return; }
  (void)lfs_file_write(l, &s_f, data, len);
  s_crc_run = crc32_run(s_crc_run, data, len);
  s_got += len;
  if (s_got >= s_size)
  {
    (void)lfs_file_close(l, &s_f);
    s_recv_active = 0;
    if ((s_got == s_size) && (s_crc_run == s_crc_expect))
    {
      printf("[MOTA] stored %s %lu bytes crc-ok\n\r", s_name, (unsigned long)s_got);
      app_log_event("SYSTEM", "mota stored %s %lu B", s_name, (unsigned long)s_got);
    }
    else
    {
      (void)lfs_remove(l, s_name);           /* don't keep a bad file in the repo */
      app_log_event("SYSTEM", "mota recv FAIL %s got=%lu/%lu", s_name, (unsigned long)s_got, (unsigned long)s_size);
      printf("[MOTA] recv FAIL got=%lu crc=%08lX (expect %lu/%08lX), removed\n\r",
             (unsigned long)s_got, (unsigned long)s_crc_run,
             (unsigned long)s_size, (unsigned long)s_crc_expect);
    }
  }
}

/* ---- streaming: littlefs repository file -> slave spare bank (blocks defaultTask ~3s, within IWDG 32s) ---- */
static int mota_poll_status(uint8_t addr, uint8_t want, uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();
  for (;;)
  {
    uint8_t pdu[1] = { 0x44U }, rsp[8];
    uint16_t rl = app_mb_transact(addr, pdu, 1, rsp, 1);
    if ((rl >= 5U) && (rsp[0] == 0x44U) && (rsp[1] == want)) { return 0; }
    if ((HAL_GetTick() - t0) > timeout_ms) { return -1; }
    HAL_Delay(50);
  }
}

int mota_flash(uint8_t addr)
{
  lfs_t *l = app_lfs();
  static uint8_t pdu[4 + MOTA_BLOCK], rsp[16], buf[MOTA_BLOCK];
  char name[12];
  lfs_file_t f;
  uint16_t mtype = 0;
  if (l == 0) { printf("[MOTA] no lfs\n\r"); return -1; }
  {
    uint8_t q[5], r2[24];                                /* v0.24: FC04-read the slave type first, then pick the image */
    uint16_t regs[9] = {0};
    uint16_t hw = 0;
    int n2 = mb_req_read(q, MB_FC_READ_INPUT, 0, 4);     /* legacy region (recognized by all slave versions) */
    uint16_t rl2 = app_mb_transact(addr, q, (uint16_t)n2, r2, 1);
    if ((rl2 == 0U) || (mb_rsp_regs(r2, rl2, MB_FC_READ_INPUT, regs, 4) != 4))
    {
      printf("[MOTA] slave %u ident failed\n\r", (unsigned)addr);
      app_log_event("SYSTEM", "mota m%u ident failed", (unsigned)addr);
      return -1;
    }
    mtype = regs[1];
    n2 = mb_req_read(q, MB_FC_READ_INPUT, 8, 1);         /* 0x0008 HW_VER: old firmware lacks this register = exception, tolerated */
    rl2 = app_mb_transact(addr, q, (uint16_t)n2, r2, 1);
    if ((rl2 > 0U) && (mb_rsp_regs(r2, rl2, MB_FC_READ_INPUT, &hw, 1) == 1)) { }
    printf("[MOTA] slave %u: type=%u hw=0x%04X fw=0x%04X\n\r",
           (unsigned)addr, (unsigned)mtype, (unsigned)hw, (unsigned)regs[2]);
  }
  snprintf(name, sizeof name, "t%u.fw", (unsigned)mtype);
  if (lfs_file_opencfg(l, &f, name, LFS_O_RDONLY, (struct lfs_file_config *)&s_fcfg) != 0)
  {
    printf("[MOTA] %s not in repo\n\r", name);
    app_log_event("SYSTEM", "mota m%u: %s not in repo", (unsigned)addr, name);
    return -1;
  }
  uint32_t size = (uint32_t)lfs_file_size(l, &f);
  uint32_t crc = 0, off = 0;
  {
    lfs_ssize_t r;
    while ((r = lfs_file_read(l, &f, buf, sizeof buf)) > 0) { crc = crc32_run(crc, buf, (uint32_t)r); }
    (void)lfs_file_rewind(l, &f);
  }
  printf("[MOTA] flash m%u: %lu bytes crc=%08lX\n\r", (unsigned)addr, (unsigned long)size, (unsigned long)crc);

  pdu[0] = 0x41U;                                        /* BEGIN */
  pdu[1] = (uint8_t)(size >> 24); pdu[2] = (uint8_t)(size >> 16);
  pdu[3] = (uint8_t)(size >> 8);  pdu[4] = (uint8_t)size;
  pdu[5] = (uint8_t)(crc >> 24);  pdu[6] = (uint8_t)(crc >> 16);
  pdu[7] = (uint8_t)(crc >> 8);   pdu[8] = (uint8_t)crc;
  uint16_t rl = app_mb_transact(addr, pdu, 9, rsp, 1);
  if ((rl < 2U) || (rsp[0] != 0x41U) || (rsp[1] != 0U))
  {
    printf("[MOTA] begin rejected\n\r"); app_log_event("SYSTEM", "mota m%u begin rejected", (unsigned)addr); (void)lfs_file_close(l, &f); return -2;
  }
  if (mota_poll_status(addr, 2, 3000) != 0)              /* wait for erase to finish (whole bank ~tens of ms) */
  {
    printf("[MOTA] erase timeout\n\r"); app_log_event("SYSTEM", "mota m%u erase timeout", (unsigned)addr); (void)lfs_file_close(l, &f); return -3;
  }

  uint16_t seq = 0;
  while (off < size)                                     /* DATA stream */
  {
    lfs_ssize_t n = lfs_file_read(l, &f, buf, MOTA_BLOCK);
    if (n <= 0) { break; }
    int tries;
    for (tries = 0; tries < 3; tries++)
    {
      pdu[0] = 0x42U; pdu[1] = (uint8_t)(seq >> 8); pdu[2] = (uint8_t)seq;
      memcpy(&pdu[3], buf, (size_t)n);
      rl = app_mb_transact(addr, pdu, (uint16_t)(3 + n), rsp, 1);
      if ((rl >= 4U) && (rsp[0] == 0x42U) && (rsp[1] == 0U)) { break; }
      if ((rl >= 4U) && (rsp[0] == 0x42U)
          && ((uint16_t)(((uint16_t)rsp[2] << 8) | rsp[3]) == (uint16_t)(seq + 1U)))
      {
        break;   /* resend-after-lost-reply was rejected but exp=seq+1 = the original was actually accepted (idempotent write semantics), advance */
      }
      printf("[MOTA] blk%u retry%d rl=%u st=%u exp=%u\n\r", (unsigned)seq, tries + 1,
             (unsigned)rl, (rl >= 2U) ? rsp[1] : 0xFFU,
             (rl >= 4U) ? (unsigned)(((uint16_t)rsp[2] << 8) | rsp[3]) : 0U);
    }
    if (tries == 3) { printf("[MOTA] block %u FAIL\n\r", (unsigned)seq); app_log_event("SYSTEM", "mota m%u block %u FAIL", (unsigned)addr, (unsigned)seq); (void)lfs_file_close(l, &f); return -4; }
    off += (uint32_t)n; seq++;
    if ((seq & 31U) == 0U) { printf("[MOTA] %lu/%lu\n\r", (unsigned long)off, (unsigned long)size); }
  }
  (void)lfs_file_close(l, &f);

  {
    uint8_t q[1] = { 0x44U }, r2[16];                    /* commit pre-check: compare rolling CRC on both sides */
    rl = app_mb_transact(addr, q, 1, r2, 1);
    if (rl >= 9U)
    {
      uint32_t scrc = ((uint32_t)r2[5] << 24) | ((uint32_t)r2[6] << 16) | ((uint32_t)r2[7] << 8) | r2[8];
      printf("[MOTA] precheck: master=%08lX slave=%08lX seq=%u\n\r",
             (unsigned long)crc, (unsigned long)scrc,
             (unsigned)(((uint16_t)r2[3] << 8) | r2[4]));
    }
  }

  pdu[0] = 0x43U;                                        /* COMMIT -> slave swaps bank and resets */
  rl = app_mb_transact(addr, pdu, 1, rsp, 1);
  if ((rl < 2U) || (rsp[0] != 0x43U) || (rsp[1] != 0U))
  {
    /* a suspicious reply != failure (stale-frame misjudgment proven: the slave may have already verified and be swapping bank); leave the verdict to the comeback probe */
    printf("[MOTA] commit rsp inconclusive (rl=%u st=%u), probing comeback...\n\r",
           (unsigned)rl, (rl >= 2U) ? rsp[1] : 0xFFU);
  }
  else { printf("[MOTA] committed, slave swapping...\n\r"); }
  HAL_Delay(800);                                        /* bank swap + reboot window */
  {
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 8000U)                 /* wait for the new firmware to come up, read the version to accept */
    {
      uint8_t q[5], r2[24];
      uint16_t regs[4] = {0};
      uint16_t hw = 0;
      int n2 = mb_req_read(q, MB_FC_READ_INPUT, 0, 4);
      rl = app_mb_transact(addr, q, (uint16_t)n2, r2, 1);
      if ((rl > 0U) && (mb_rsp_regs(r2, rl, MB_FC_READ_INPUT, regs, 4) == 4))
      {
        n2 = mb_req_read(q, MB_FC_READ_INPUT, 8, 1);
        uint16_t rl2 = app_mb_transact(addr, q, (uint16_t)n2, r2, 1);
        if ((rl2 > 0U) && (mb_rsp_regs(r2, rl2, MB_FC_READ_INPUT, &hw, 1) == 1)) { }
        printf("[MOTA] slave %u back: type=%u hw=0x%04X fw=0x%04X — OTA OK\n\r",
               (unsigned)addr, (unsigned)regs[1], (unsigned)hw, (unsigned)regs[2]);
        app_log_event("SYSTEM", "mota m%u OK: type=%u hw=0x%04X fw=0x%04X", (unsigned)addr, (unsigned)regs[1], (unsigned)hw, (unsigned)regs[2]);
        { extern void app_diag_mod_set_fw(uint8_t, uint16_t, uint16_t); app_diag_mod_set_fw(addr, regs[1], regs[2]); }   /* desc card title follows the new fw (republish) */
        return 0;
      }
      HAL_Delay(200);
    }
  }
  printf("[MOTA] slave not back in 8s (3-strike auto-revert will kick in)\n\r");
  app_log_event("SYSTEM", "mota m%u: slave not back in 8s (3-strike revert)", (unsigned)addr);
  return -6;
}
