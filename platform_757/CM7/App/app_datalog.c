/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_datalog.c — on-board data recorder + event journal (contract: docs/Data_Logging_and_Event_Journal.md)
 *
 * Authoritative history lives on the board: daily CSV files on the SD card (FatFs) or the
 * QSPI littlefs partition, selectable at runtime ("logcfg dev=..."). Sampling is aggregated
 * (avg/min/max per window, sub-sampled every 5 s over the backplane) so the files stay small
 * and the nightly minimum survives any window length. Events (config audit, faults, system,
 * application actions, human notes) go to a parallel _ev.csv with the same date.
 *
 * Threading: every filesystem touch happens in app_datalog_poll() on the mqtt main-loop
 * 500 ms tick. app_log_event() only formats into a RAM queue, so any task may call it.
 * Loss tolerance (user ruling 2026-08-21): a power cut may cost the current window and
 * whatever queue entries were not yet drained — no battery-backed staging.
 *
 * Cloud transfer: "logget <date>" streams a file in 1 KB chunks to up/log/<file>/<seq>
 * (raw CSV bytes) + up/log/<file>/end (JSON summary); one chunk per tick (~2 KB/s), each
 * chunk confirmed against the lwip publish result before advancing, so a full ring buffer
 * only slows the transfer down instead of punching holes in it. The same machinery pushes
 * yesterday's files automatically once per UTC day (mirror on the dash server). */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "fatfs.h"
#include "lfs.h"
#include "app_datalog.h"
#include "app_time.h"
#include "app_mbport.h"
#include "modbus_core.h"

/* ---- externs (providers documented at their definitions) ---- */
extern lfs_t *app_lfs(void);
extern uint16_t app_diag_mod_list(const void **out);
extern uint8_t  app_diag_mod_addr(const void *m, uint16_t i);
extern uint16_t app_diag_mod_type(const void *m, uint16_t i);
extern const char *app_diag_reset_cause(void);
extern int app_temp_read(void);
extern uint8_t app_mqtt_ready(void);                 /* broker session up */
extern int  app_mqtt_pub_updata(const char *sub, const void *data, uint16_t len);
extern uint8_t app_mqtt_pub_updata_busy(void);
extern int8_t  app_mqtt_pub_updata_result(void);

#define DL_TYPE_PHEC   2U       /* Board_Type_and_Version_Registry.md */
#define DL_TYPE_16DI   3U
#define DL_MODS        8U
#define DL_SUB_TICKS   10U      /* sub-sample every 10 x 500 ms = 5 s */
#define DL_CHUNK       1024U
#define DL_EVQ         8U
#define DL_PUSH_MAXAGE 7U       /* auto-push looks back at most this many days */
#define DL_SD_MIN_FREE_MB 100U  /* SD retention floor */

enum { DEV_OFF = 0, DEV_SD = 1, DEV_FLASH = 2 };

/* ---- config (persisted as text in littlefs /log.cfg — must work with no card) ---- */
static uint8_t  s_devcfg = DEV_SD;    /* configured medium */
static uint8_t  s_dev    = DEV_OFF;   /* effective medium after probe/fallback */
static uint16_t s_period = 60U;       /* aggregation window, s (5..3600, multiple of 5) */
static uint16_t s_quota  = 20U;       /* littlefs quota, MB */

/* ---- aggregation ---- */
typedef struct { int32_t sum; int32_t mn; int32_t mx; uint16_t cnt; } agg_t;
static struct
{
  uint8_t  addr;
  uint16_t type;
  agg_t    a[8];        /* PH_EC: ph1 ph2 ec1 ec2 t1..t4 */
  uint32_t cnt[16];     /* EX_16DI: latest counters */
  uint8_t  have_cnt;
} s_slot[DL_MODS];
static uint8_t  s_nslot = 0;
static agg_t    s_temp;               /* onboard CPU temperature, degC */
static uint32_t s_win_id = 0;         /* ts / period of the running window (0 = none) */
/* Field names in ENGINEERING-BLOCK order. Temp naming follows the 2026-09-01 user
 * convention (t1=temp-EC1/outdoor, t2=temp-EC2/air, t3=temp-pH1/water, t4=temp-pH2/water)
 * — MUST stay identical to app_diag.c's fld[] or the history CSV and the live heartbeat
 * disagree about which probe is which (split-brain shipped for a few hours on 09-01). */
static const char *FLD[8] = { "ph1", "ph2", "ec1", "ec2", "t3", "t4", "t1", "t2" };

/* ---- current file state ---- */
static char     s_date[12] = "";      /* UTC date of the open sample file */
static char     s_suffix[3] = "";     /* "", "-b".. : same-day layout rotation */
static uint32_t s_today_bytes = 0;
static uint16_t s_werr = 0;           /* consecutive write errors (3 on SD -> flash fallback) */
static uint16_t s_drops = 0;          /* event-queue overflow count */
static char     s_hbstr[48] = "init";

/* buffers that FatFs may hand to SDMMC IDMA: AXI + 32-byte aligned (DTCM unreachable) */
__attribute__((section(".axisram"), aligned(32))) static FIL  s_fil;
__attribute__((section(".axisram"), aligned(32))) static char s_hdr[1536];
__attribute__((section(".axisram"), aligned(32))) static char s_line[1024];
__attribute__((section(".axisram"), aligned(32))) static uint8_t s_chunk[DL_CHUNK];
static uint8_t s_lfs_fbuf[256];       /* littlefs file cache (LFS_NO_MALLOC; poll-context only) */

/* ---- event queue (producers: any task; consumer: poll) ---- */
typedef struct { uint32_t ts; char type[8]; char src[6]; char text[120]; } dlev_t;
static dlev_t  s_evq[DL_EVQ];
static volatile uint8_t s_ev_h = 0, s_ev_t = 0;

/* ---- transfer (logget / auto-push) ---- */
#define DL_XQ 6U          /* files per request: a day is sample segments (-b rotations) + events */
static struct
{
  uint8_t  state;        /* 0 idle, 1 data chunks, 2 end record */
  uint8_t  in_flight;    /* 1 = publish handed over, waiting for its result */
  uint16_t sent_len;     /* chunk length of the publish in flight */
  uint8_t  is_auto;
  char     q[DL_XQ][28]; /* file queue — a bare-date request enumerates EVERY file of that
                          * day (2026-08-21 field case: the fixed two-name scheme skipped the
                          * -b layout segment where all the measurement rows lived, so the
                          * mirror froze at whatever an explicit pull once caught) */
  uint8_t  qn, qi;
  char     fn[28];       /* current file = q[qi] */
  uint32_t req_off;      /* requested start offset (single-file requests only): "logget <file> <off>"
                          * = incremental tail pull, the mirror re-fetches only the new bytes */
  uint32_t base;         /* effective start offset of this file's stream (reported in the end record) */
  uint32_t size, off, seq;
} s_x;
static uint32_t s_pushed_day = 0;     /* days-since-epoch of the last fully mirrored day */

/* ================= date helpers (UTC) ================= */
static void dl_date_str(uint32_t days, char out[12])
{
  /* civil-from-days (Hinnant), valid for the unix era */
  int64_t z = (int64_t)days + 719468;
  int64_t era = z / 146097;
  int64_t doe = z - era * 146097;
  int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  int64_t y = yoe + era * 400;
  int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  int64_t mp = (5 * doy + 2) / 153;
  int64_t d = doy - (153 * mp + 2) / 5 + 1;
  int64_t m = mp + (mp < 10 ? 3 : -9);
  if (m <= 2) { y++; }
  snprintf(out, 12, "%04u-%02u-%02u", (unsigned)((uint32_t)y % 10000U),
           (unsigned)((uint32_t)m % 100U), (unsigned)((uint32_t)d % 100U));
}

/* ================= littlefs small-file helpers (config/state) ================= */
static int dl_lfs_load(const char *name, void *buf, uint16_t cap)
{
  lfs_t *l = app_lfs();
  lfs_file_t f;
  static const struct lfs_file_config fc = { .buffer = s_lfs_fbuf };
  int n;
  if (l == NULL) { return -1; }
  if (lfs_file_opencfg(l, &f, name, LFS_O_RDONLY, (struct lfs_file_config *)&fc) != 0) { return -1; }
  n = (int)lfs_file_read(l, &f, buf, cap);
  (void)lfs_file_close(l, &f);
  return n;
}

static int dl_lfs_save(const char *name, const void *buf, uint16_t len)
{
  lfs_t *l = app_lfs();
  lfs_file_t f;
  static const struct lfs_file_config fc = { .buffer = s_lfs_fbuf };
  int ok;
  if (l == NULL) { return -1; }
  if (lfs_file_opencfg(l, &f, name, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                       (struct lfs_file_config *)&fc) != 0) { return -1; }
  ok = ((uint16_t)lfs_file_write(l, &f, buf, len) == len) ? 0 : -1;
  if (lfs_file_close(l, &f) != 0) { ok = -1; }
  return ok;
}

/* ================= storage backends ================= */
static int      s_sd_fr = 0;        /* last FatFs FRESULT that failed (probe or append) — logstat diagnostics */
static int      s_lfs_err = 0;      /* last littlefs error */
static uint16_t s_sd_errs = 0, s_lfs_errs = 0;   /* lifetime failure counters */
static uint8_t  s_lfs_dir_ok = 0;   /* log/ directory known to exist on littlefs */

/* ensure the littlefs log directory exists — needed on EVERY path that lands on flash
 * (2026-08-21 field case: the runtime SD->flash fallback skipped this, so every append
 * died on LFS_ERR_NOENT with nothing recorded — files stayed at size 0, evidence lost) */
static int dl_flash_ready(void)
{
  lfs_t *l = app_lfs();
  if (l == NULL) { return -1; }
  if (!s_lfs_dir_ok)
  {
    int e = lfs_mkdir(l, "log");
    if ((e != 0) && (e != LFS_ERR_EXIST)) { s_lfs_err = e; s_lfs_errs++; return -1; }
    s_lfs_dir_ok = 1;
  }
  return 0;
}

static void dl_fallback_to_flash(int err)
{
  s_dev = DEV_FLASH;
  s_werr = 0;
  (void)dl_flash_ready();
  app_log_event_src("SYSTEM", "sys", "log sd err %d -> flash", err);
}

/* append data to log/<fname>; writes hdr first when the file is created empty */
static int dl_append(const char *fname, const char *hdr, const char *data, uint16_t dlen)
{
  if (s_dev == DEV_SD)
  {
    char path[44];
    FRESULT fr;
    UINT bw = 0;
    snprintf(path, sizeof path, "0:/log/%s", fname);
    fr = f_open(&s_fil, path, FA_OPEN_ALWAYS | FA_WRITE);
    if (fr == FR_OK)
    {
      fr = f_lseek(&s_fil, f_size(&s_fil));
      if ((fr == FR_OK) && (f_size(&s_fil) == 0U) && (hdr != NULL))
      {
        fr = f_write(&s_fil, hdr, (UINT)strlen(hdr), &bw);
      }
      if (fr == FR_OK) { fr = f_write(&s_fil, data, dlen, &bw); }
      if (f_close(&s_fil) != FR_OK) { fr = FR_DISK_ERR; }
    }
    if (fr != FR_OK)
    {
      s_sd_fr = (int)fr;
      s_sd_errs++;
      if (++s_werr >= 3U) { dl_fallback_to_flash((int)fr); }
      return -1;
    }
    s_werr = 0;
    return 0;
  }
  if (s_dev == DEV_FLASH)
  {
    lfs_t *l = app_lfs();
    lfs_file_t f;
    static const struct lfs_file_config fc = { .buffer = s_lfs_fbuf };
    char path[40];
    int e, ok = 0;
    if ((l == NULL) || (dl_flash_ready() != 0)) { return -1; }
    snprintf(path, sizeof path, "log/%s", fname);
    e = lfs_file_opencfg(l, &f, path, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND,
                         (struct lfs_file_config *)&fc);
    if (e != 0) { s_lfs_err = e; s_lfs_errs++; return -1; }
    if ((lfs_file_size(l, &f) == 0) && (hdr != NULL))
    {
      if (lfs_file_write(l, &f, hdr, (lfs_size_t)strlen(hdr)) < 0) { ok = -1; }
    }
    if ((ok == 0) && (lfs_file_write(l, &f, data, dlen) != (lfs_ssize_t)dlen)) { ok = -1; }
    if (lfs_file_close(l, &f) != 0) { ok = -1; }
    if (ok != 0) { s_lfs_errs++; }
    return ok;
  }
  return -1;
}

static int dl_fsize(const char *fname, uint32_t *size)
{
  if (s_dev == DEV_SD)
  {
    char path[44];
    FILINFO fi;
    snprintf(path, sizeof path, "0:/log/%s", fname);
    if (f_stat(path, &fi) != FR_OK) { return -1; }
    *size = (uint32_t)fi.fsize;
    return 0;
  }
  if (s_dev == DEV_FLASH)
  {
    lfs_t *l = app_lfs();
    struct lfs_info fi;
    char path[40];
    if (l == NULL) { return -1; }
    snprintf(path, sizeof path, "log/%s", fname);
    if (lfs_stat(l, path, &fi) != 0) { return -1; }
    *size = fi.size;
    return 0;
  }
  return -1;
}

/* stateless chunk read (open/seek/read/close each time: robust against a concurrent
 * same-day append, and one chunk per 500 ms tick makes the cost irrelevant) */
static int dl_fread(const char *fname, uint32_t off, uint8_t *buf, uint16_t len)
{
  if (s_dev == DEV_SD)
  {
    char path[44];
    UINT br = 0;
    snprintf(path, sizeof path, "0:/log/%s", fname);
    if (f_open(&s_fil, path, FA_READ) != FR_OK) { return -1; }
    if ((f_lseek(&s_fil, off) != FR_OK) || (f_read(&s_fil, buf, len, &br) != FR_OK)) { br = 0; }
    (void)f_close(&s_fil);
    return (br > 0U) ? (int)br : -1;
  }
  if (s_dev == DEV_FLASH)
  {
    lfs_t *l = app_lfs();
    lfs_file_t f;
    static const struct lfs_file_config fc = { .buffer = s_lfs_fbuf };
    char path[40];
    lfs_ssize_t n = -1;
    if (l == NULL) { return -1; }
    snprintf(path, sizeof path, "log/%s", fname);
    if (lfs_file_opencfg(l, &f, path, LFS_O_RDONLY, (struct lfs_file_config *)&fc) != 0) { return -1; }
    if (lfs_file_seek(l, &f, (lfs_soff_t)off, LFS_SEEK_SET) >= 0) { n = lfs_file_read(l, &f, buf, len); }
    (void)lfs_file_close(l, &f);
    return (n > 0) ? (int)n : -1;
  }
  return -1;
}

/* delete the lexicographically oldest file in the log directory (retention) */
static int dl_delete_oldest(void)
{
  char oldest[28] = "";
  if (s_dev == DEV_SD)
  {
    DIR d;
    FILINFO fi;
    if (f_opendir(&d, "0:/log") != FR_OK) { return -1; }
    while ((f_readdir(&d, &fi) == FR_OK) && (fi.fname[0] != 0))
    {
      if ((fi.fattrib & AM_DIR) != 0U) { continue; }
      if ((oldest[0] == 0) || (strcmp(fi.fname, oldest) < 0))
      {
        strncpy(oldest, fi.fname, sizeof oldest - 1U);
      }
    }
    (void)f_closedir(&d);
    if (oldest[0] == 0) { return -1; }
    { char path[44]; snprintf(path, sizeof path, "0:/log/%s", oldest); (void)f_unlink(path); }
    return 0;
  }
  if (s_dev == DEV_FLASH)
  {
    lfs_t *l = app_lfs();
    lfs_dir_t d;
    struct lfs_info fi;
    if ((l == NULL) || (lfs_dir_open(l, &d, "log") != 0)) { return -1; }
    while (lfs_dir_read(l, &d, &fi) > 0)
    {
      if (fi.type != LFS_TYPE_REG) { continue; }
      if ((oldest[0] == 0) || (strcmp(fi.name, oldest) < 0))
      {
        strncpy(oldest, fi.name, sizeof oldest - 1U);
      }
    }
    (void)lfs_dir_close(l, &d);
    if (oldest[0] == 0) { return -1; }
    { char path[40]; snprintf(path, sizeof path, "log/%s", oldest); (void)lfs_remove(l, path); }
    return 0;
  }
  return -1;
}

static void dl_retention(void)
{
  uint8_t i;
  for (i = 0; i < 4U; i++)   /* bounded work per day tick */
  {
    if (s_dev == DEV_SD)
    {
      FATFS *fs;
      DWORD fre = 0;
      if (f_getfree("0:", &fre, &fs) != FR_OK) { return; }
      /* free MB = clusters * sectors/cluster * 512 / 1M */
      if (((uint64_t)fre * fs->csize * 512ULL) >= ((uint64_t)DL_SD_MIN_FREE_MB << 20)) { return; }
    }
    else if (s_dev == DEV_FLASH)
    {
      lfs_t *l = app_lfs();
      lfs_ssize_t used;
      if (l == NULL) { return; }
      used = lfs_fs_size(l);
      if ((used < 0) || (((uint64_t)used * 4096ULL) <= ((uint64_t)s_quota << 20))) { return; }
    }
    else { return; }
    if (dl_delete_oldest() != 0) { return; }
  }
}

/* ================= event queue ================= */
static void dl_ev_enqueue(const char *type, const char *src, const char *fmt, va_list ap)
{
  dlev_t e;
  uint8_t n;
  char *p;
  e.ts = app_time_now();
  strncpy(e.type, type, sizeof e.type - 1U); e.type[sizeof e.type - 1U] = 0;
  strncpy(e.src, src, sizeof e.src - 1U);    e.src[sizeof e.src - 1U] = 0;
  vsnprintf(e.text, sizeof e.text, fmt, ap);
  for (p = e.text; *p != 0; p++)   /* keep the CSV grammar safe */
  {
    if ((*p == ',') || (*p == '\n') || (*p == '\r') || (*p == '"')) { *p = ' '; }
  }
  taskENTER_CRITICAL();
  n = (uint8_t)((s_ev_h + 1U) % DL_EVQ);
  if (n == s_ev_t) { s_drops++; }          /* full: drop newest, count it */
  else { s_evq[s_ev_h] = e; s_ev_h = n; }
  taskEXIT_CRITICAL();
}

void app_log_event(const char *type, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  dl_ev_enqueue(type, "app", fmt, ap);
  va_end(ap);
}

void app_log_event_src(const char *type, const char *src, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  dl_ev_enqueue(type, src, fmt, ap);
  va_end(ap);
}

static void dl_ev_drain(void)
{
  uint8_t budget = 2U;   /* at most 2 file appends per tick */
  while ((s_ev_t != s_ev_h) && (budget-- > 0U) && (s_dev != DEV_OFF))
  {
    dlev_t e;
    char date[12], fname[28];
    int n;
    taskENTER_CRITICAL();
    e = s_evq[s_ev_t];        /* peek — dequeue only after the append lands (2026-08-21
                               * field case: dequeue-first lost the very event that said
                               * why the storage was failing) */
    taskEXIT_CRITICAL();
    if (e.ts == 0U) { e.ts = app_time_now(); }   /* queued before first sync: stamp late */
    dl_date_str(e.ts / 86400U, date);
    snprintf(fname, sizeof fname, "%s_ev.csv", date);
    n = snprintf(s_line, sizeof s_line, "%lu,%s,%s,%s\n",
                 (unsigned long)e.ts, e.type, e.src, e.text);
    if (dl_append(fname, "ts,type,src,text\n", s_line, (uint16_t)n) != 0) { break; }
    s_today_bytes += (uint32_t)n;
    taskENTER_CRITICAL();
    s_ev_t = (uint8_t)((s_ev_t + 1U) % DL_EVQ);
    taskEXIT_CRITICAL();
  }
}

/* ================= sampling ================= */
static void dl_win_reset(void)
{
  const void *mods;
  uint16_t nm = app_diag_mod_list(&mods);
  uint16_t i;
  s_nslot = 0;
  for (i = 0; (i < nm) && (s_nslot < DL_MODS); i++)
  {
    uint16_t t = app_diag_mod_type(mods, i);
    if ((t != DL_TYPE_PHEC) && (t != DL_TYPE_16DI)) { continue; }
    memset(&s_slot[s_nslot], 0, sizeof s_slot[0]);
    s_slot[s_nslot].addr = app_diag_mod_addr(mods, i);
    s_slot[s_nslot].type = t;
    s_nslot++;
  }
  memset(&s_temp, 0, sizeof s_temp);
}

static void agg_add(agg_t *a, int32_t v)
{
  if (a->cnt == 0U) { a->mn = v; a->mx = v; }
  else { if (v < a->mn) { a->mn = v; } if (v > a->mx) { a->mx = v; } }
  a->sum += v;
  a->cnt++;
}

static void dl_subsample(void)
{
  uint8_t i;
  agg_add(&s_temp, (int32_t)app_temp_read());
  for (i = 0; i < s_nslot; i++)
  {
    uint8_t q[8], r[80];
    if (s_slot[i].type == DL_TYPE_PHEC)
    {
      uint16_t v[8];
      int ql = mb_req_read(q, MB_FC_READ_INPUT, 0x0108U, 8U);
      uint16_t rl = app_mb_transact(s_slot[i].addr, q, (uint16_t)ql, r, 1U);
      if ((rl == 0U) || (mb_rsp_regs(r, rl, MB_FC_READ_INPUT, v, 8U) != 8)) { continue; }
      { /* FAULT edge detection per channel (module already debounces its published values) */
        static uint8_t s_valid[17];        /* per backplane address: bit k = channel k valid; 0 = baseline unknown */
        static uint8_t s_seen[17];
        uint8_t vm = 0, k;
        for (k = 0; k < 8U; k++) { if (v[k] != 0x7FFFU) { vm |= (uint8_t)(1U << k); } }
        if (!s_seen[s_slot[i].addr]) { s_seen[s_slot[i].addr] = 1U; s_valid[s_slot[i].addr] = vm; }
        else if (vm != s_valid[s_slot[i].addr])
        {
          uint8_t ch = (uint8_t)(vm ^ s_valid[s_slot[i].addr]);
          for (k = 0; k < 8U; k++)
          {
            if ((ch & (1U << k)) != 0U)
            {
              app_log_event_src("FAULT", "sys", "m%u%s %s", (unsigned)s_slot[i].addr, FLD[k],
                                ((vm >> k) & 1U) ? "ok" : "fault");
            }
          }
          s_valid[s_slot[i].addr] = vm;
        }
      }
      { uint8_t k;
        for (k = 0; k < 8U; k++)
        {
          if (v[k] == 0x7FFFU) { continue; }
          /* pH/EC of exactly 0 = module-boot residue (the scan reads zeros for a beat
           * before the slave's first frame), physically impossible with a probe attached —
           * discard like the sentinel (2026-09-02 owner ruling; a single zero used to drag
           * the history chart's autoscale to a -0.7..9.5 pH axis). Temps keep their zeros:
           * 0.0 degC is a real winter reading. */
          if ((k < 4U) && (v[k] == 0U)) { continue; }
          agg_add(&s_slot[i].a[k], ((k == 2U) || (k == 3U)) ? (int32_t)v[k] : (int32_t)(int16_t)v[k]);
        } }
    }
    else   /* EX_16DI: 16 x u32 counters, keep the latest (cumulative -> min/max meaningless) */
    {
      uint16_t c[32];
      int ql = mb_req_read(q, MB_FC_READ_INPUT, 0x0200U, 32U);
      uint16_t rl = app_mb_transact(s_slot[i].addr, q, (uint16_t)ql, r, 1U);
      if ((rl == 0U) || (mb_rsp_regs(r, rl, MB_FC_READ_INPUT, c, 32U) != 32)) { continue; }
      { uint8_t k;
        for (k = 0; k < 16U; k++)
        {
          s_slot[i].cnt[k] = ((uint32_t)c[2U * k] << 16) | c[2U * k + 1U];
        } }
      s_slot[i].have_cnt = 1U;
    }
  }
}

/* ================= row flush + file rotation ================= */
static void dl_build_header(char *out, uint16_t cap)
{
  /* flow (2026-09-02, user ruling after the top-up true/false split): cumulative water
   * meter, 0.1 L units, resets to 0 on reboot — the drop itself marks the boot. Gives
   * the top-up detector's verdicts a ledger to be audited against (the 09-02 morning
   * pair: a real 2.5 L float-valve top-up and a calibration artifact looked identical
   * in EC; only the meter told them apart, and it lived nowhere but the heartbeat). */
  /* g1..g4 (2026-09-03): per-gutter feed flow, 0.1 L/min, window-end value of the 30 s
   * lazy rate (app_user.c, 1200 p/L hall meters on HSDI4/2/6/7) — a clogged line shows
   * as a gutter dropping out while the others hold. */
  uint16_t n = (uint16_t)snprintf(out, cap, "ts,temp_avg,temp_min,temp_max,vent,flow,g1,g2,g3,g4");
  uint8_t i, k;
  for (i = 0; i < s_nslot; i++)
  {
    if (s_slot[i].type == DL_TYPE_PHEC)
    {
      for (k = 0; k < 8U; k++)
      {
        n += (uint16_t)snprintf(&out[n], (size_t)(cap - n), ",m%u%s_avg,m%u%s_min,m%u%s_max",
                                (unsigned)s_slot[i].addr, FLD[k], (unsigned)s_slot[i].addr, FLD[k],
                                (unsigned)s_slot[i].addr, FLD[k]);
      }
    }
    else
    {
      for (k = 0; k < 16U; k++)
      {
        n += (uint16_t)snprintf(&out[n], (size_t)(cap - n), ",m%uc%u",
                                (unsigned)s_slot[i].addr, (unsigned)(k + 1U));
      }
    }
  }
  if (n < cap) { snprintf(&out[n], (size_t)(cap - n), "\n"); }
}

/* Does log/<fname>'s first line equal hdr (header text incl. trailing \n)?
 * 1 = match, 0 = different, -1 = unreadable (caller treats as different). */
__attribute__((section(".axisram"), aligned(32))) static char s_hdrchk[1536];
static int dl_hdr_match(const char *fname, const char *hdr)
{
  uint32_t want = (uint32_t)strlen(hdr);
  if (want > sizeof s_hdrchk) { return 0; }
  if (s_dev == DEV_SD)
  {
    char path[44];
    UINT br = 0;
    snprintf(path, sizeof path, "0:/log/%s", fname);
    if (f_open(&s_fil, path, FA_READ) != FR_OK) { return -1; }
    if (f_read(&s_fil, s_hdrchk, (UINT)want, &br) != FR_OK) { br = 0; }
    (void)f_close(&s_fil);
    return ((br == (UINT)want) && (memcmp(s_hdrchk, hdr, want) == 0)) ? 1 : 0;
  }
  if (s_dev == DEV_FLASH)
  {
    lfs_t *l = app_lfs();
    lfs_file_t f;
    static const struct lfs_file_config fc = { .buffer = s_lfs_fbuf };
    lfs_ssize_t br;
    char path[40];
    if ((l == NULL) || (dl_flash_ready() != 0)) { return -1; }
    snprintf(path, sizeof path, "log/%s", fname);
    if (lfs_file_opencfg(l, &f, path, LFS_O_RDONLY, (struct lfs_file_config *)&fc) != 0) { return -1; }
    br = lfs_file_read(l, &f, s_hdrchk, (lfs_size_t)want);
    (void)lfs_file_close(l, &f);
    return ((br == (lfs_ssize_t)want) && (memcmp(s_hdrchk, hdr, want) == 0)) ? 1 : 0;
  }
  return -1;
}

/* Choose s_suffix for (s_date, hdr): first candidate ("", -b..-z) whose file is absent/empty
 * (header will be written on create) or whose STORED header matches the current layout.
 * The files are the layout memory — RAM suffix state dies with every reset (2026-08-22 field
 * case: post-reset rotation restarted blindly at -b and appended full-width rows under the
 * old temp-only header, so the mirror parser dropped every measurement column). A matching
 * older segment is deliberately reused: layout flapping (modules dropping off and returning)
 * ping-pongs between two segments instead of burning a letter per flap. */
static void dl_seg_pick(const char *hdr)
{
  char fname[28];
  char suf[3] = "";
  for (;;)
  {
    uint32_t sz;
    snprintf(fname, sizeof fname, "%s%s.csv", s_date, suf);
    if ((dl_fsize(fname, &sz) != 0) || (sz == 0U)) { break; }
    if (dl_hdr_match(fname, hdr) == 1) { break; }
    if (suf[0] == 0) { suf[0] = '-'; suf[1] = 'b'; suf[2] = 0; }
    else if (suf[1] < 'z') { suf[1]++; }
    else { break; }                              /* 25 layouts in one day: append anyway, honesty over loss */
  }
  memcpy(s_suffix, suf, sizeof s_suffix);
}

static int32_t agg_avg(const agg_t *a)
{
  if (a->cnt == 0U) { return 0; }
  return (a->sum >= 0) ? ((a->sum + (int32_t)(a->cnt / 2U)) / (int32_t)a->cnt)
                       : -((-a->sum + (int32_t)(a->cnt / 2U)) / (int32_t)a->cnt);
}

static uint16_t row_agg(char *out, uint16_t cap, const agg_t *a)
{
  if (a->cnt == 0U) { return (uint16_t)snprintf(out, cap, ",,,"); }
  return (uint16_t)snprintf(out, cap, ",%ld,%ld,%ld",
                            (long)agg_avg(a), (long)a->mn, (long)a->mx);
}

static void dl_flush_row(uint32_t win_end_ts)
{
  static char hdr_now[1536];
  char date[12];
  uint16_t n;
  uint8_t i, k;
  if (s_dev == DEV_OFF) { return; }
  dl_date_str(win_end_ts / 86400U, date);
  dl_build_header(hdr_now, sizeof hdr_now);
  if (strcmp(date, s_date) != 0)                       /* UTC day rollover (also the post-reset first flush) */
  {
    strncpy(s_date, date, sizeof s_date - 1U);
    s_today_bytes = 0;
    dl_seg_pick(hdr_now);                              /* header-verified: never append under a foreign header */
    strncpy(s_hdr, hdr_now, sizeof s_hdr - 1U);
    dl_retention();
  }
  else if (strcmp(hdr_now, s_hdr) != 0)                /* layout changed mid-day -> header-verified segment pick */
  {
    dl_seg_pick(hdr_now);
    strncpy(s_hdr, hdr_now, sizeof s_hdr - 1U);
  }
  n = (uint16_t)snprintf(s_line, sizeof s_line, "%lu", (unsigned long)win_end_ts);
  n += row_agg(&s_line[n], (uint16_t)(sizeof s_line - n), &s_temp);
  { extern uint8_t app_vent_stage(void);   /* true window stage (relay read-back, app_vent.c) */
    n += (uint16_t)snprintf(&s_line[n], (size_t)(sizeof s_line - n), ",%u", (unsigned)app_vent_stage()); }
  { extern uint32_t app_flow_dl(void);     /* water meter total, 0.1 L (app_user.c, HSDI0) */
    n += (uint16_t)snprintf(&s_line[n], (size_t)(sizeof s_line - n), ",%lu", (unsigned long)app_flow_dl()); }
  { extern uint16_t app_gutter_rate_dlmin(uint8_t);   /* per-gutter feed flow, 0.1 L/min (app_user.c) */
    for (uint8_t g = 0; g < 4U; g++)
    { n += (uint16_t)snprintf(&s_line[n], (size_t)(sizeof s_line - n), ",%u", (unsigned)app_gutter_rate_dlmin(g)); } }
  for (i = 0; i < s_nslot; i++)
  {
    if (s_slot[i].type == DL_TYPE_PHEC)
    {
      for (k = 0; k < 8U; k++)
      {
        n += row_agg(&s_line[n], (uint16_t)(sizeof s_line - n), &s_slot[i].a[k]);
      }
    }
    else
    {
      for (k = 0; k < 16U; k++)
      {
        if (s_slot[i].have_cnt)
        {
          n += (uint16_t)snprintf(&s_line[n], sizeof s_line - n, ",%lu",
                                  (unsigned long)s_slot[i].cnt[k]);
        }
        else { n += (uint16_t)snprintf(&s_line[n], sizeof s_line - n, ","); }
      }
    }
  }
  if (n < sizeof s_line - 1U) { s_line[n++] = '\n'; s_line[n] = 0; }
  { char fname[28];
    snprintf(fname, sizeof fname, "%s%s.csv", s_date, s_suffix);
    if (dl_append(fname, s_hdr, s_line, n) == 0) { s_today_bytes += n; } }
  snprintf(s_hbstr, sizeof s_hbstr, "%s %us %s%s %luKB",
           (s_dev == DEV_SD) ? "sd" : "flash", (unsigned)s_period, s_date, s_suffix,
           (unsigned long)(s_today_bytes >> 10));
}

/* ================= transfer (logget + daily mirror) ================= */
static void dl_xfer_begin_file(void)
{
  s_x.off = 0; s_x.seq = 0; s_x.size = 0; s_x.base = 0;
  if (dl_fsize(s_x.fn, &s_x.size) != 0)
  {
    s_x.size = 0;
    s_x.state = 2U;   /* missing: still announce {"size":0} so the requester isn't left hanging */
    return;
  }
  if (s_x.qn == 1U)                    /* offset applies to explicit single-file requests only */
  {
    s_x.base = (s_x.req_off <= s_x.size) ? s_x.req_off : 0U;   /* shrunk/rotated: restream fully */
    s_x.off = s_x.base;
  }
  s_x.state = (s_x.off >= s_x.size) ? 2U : 1U;   /* nothing new: straight to the end record */
}

static void dl_xfer_reset(uint8_t is_auto)
{
  memset(&s_x, 0, sizeof s_x);
  s_x.is_auto = is_auto;
}

static void dl_xfer_add(const char *fn)
{
  if (s_x.qn < DL_XQ)
  {
    strncpy(s_x.q[s_x.qn], fn, sizeof s_x.q[0] - 1U);
    s_x.qn++;
  }
}

/* enumerate the log directory on the active medium for every file of one day */
static void dl_scan_date(const char *date)
{
  if (s_dev == DEV_SD)
  {
    DIR d;
    FILINFO fi;
    if (f_opendir(&d, "0:/log") != FR_OK) { return; }
    while ((f_readdir(&d, &fi) == FR_OK) && (fi.fname[0] != 0))
    {
      if (((fi.fattrib & AM_DIR) == 0U) && (strncmp(fi.fname, date, 10) == 0)) { dl_xfer_add(fi.fname); }
    }
    (void)f_closedir(&d);
  }
  else if (s_dev == DEV_FLASH)
  {
    lfs_t *l = app_lfs();
    lfs_dir_t d;
    struct lfs_info fi;
    if ((l == NULL) || (lfs_dir_open(l, &d, "log") != 0)) { return; }
    while (lfs_dir_read(l, &d, &fi) > 0)
    {
      if ((fi.type == LFS_TYPE_REG) && (strncmp(fi.name, date, 10) == 0)) { dl_xfer_add(fi.name); }
    }
    (void)lfs_dir_close(l, &d);
  }
}

static void dl_xfer_go(void)
{
  s_x.qi = 0;
  strncpy(s_x.fn, s_x.q[0], sizeof s_x.fn - 1U);
  dl_xfer_begin_file();
}

static void dl_push_mark_done(void);

static void dl_xfer_step(void)
{
  char sub[64];
  if (s_x.state == 0U) { return; }
  if (!app_mqtt_ready()) { return; }             /* wait for the link; resumes where it left */
  if (s_x.in_flight)
  {
    if (app_mqtt_pub_updata_busy()) { return; }
    s_x.in_flight = 0U;
    if (app_mqtt_pub_updata_result() == 0)
    {
      if (s_x.state == 1U)
      {
        s_x.off += s_x.sent_len;
        s_x.seq++;
        if (s_x.off >= s_x.size) { s_x.state = 2U; }
      }
      else                                        /* end record confirmed -> next file / idle */
      {
        s_x.qi++;
        if (s_x.qi < s_x.qn)
        {
          strncpy(s_x.fn, s_x.q[s_x.qi], sizeof s_x.fn - 1U);
          dl_xfer_begin_file();
        }
        else
        {
          if (s_x.is_auto) { dl_push_mark_done(); }
          s_x.state = 0U;
        }
        return;
      }
    }
    /* nonzero result: ring full / transient — resend the same payload next tick */
  }
  if (s_x.state == 1U)
  {
    uint16_t len = (uint16_t)(((s_x.size - s_x.off) > DL_CHUNK) ? DL_CHUNK : (s_x.size - s_x.off));
    int n = dl_fread(s_x.fn, s_x.off, s_chunk, len);
    if (n <= 0)
    {
      app_log_event_src("SYSTEM", "sys", "logget %s read err", s_x.fn);
      s_x.state = 0U;
      return;
    }
    snprintf(sub, sizeof sub, "log/%s/%lu", s_x.fn, (unsigned long)s_x.seq);
    if (app_mqtt_pub_updata(sub, s_chunk, (uint16_t)n) == 0)
    {
      s_x.sent_len = (uint16_t)n;
      s_x.in_flight = 1U;
    }
  }
  else if (s_x.state == 2U)
  {
    char body[64];
    int bl = snprintf(body, sizeof body, "{\"size\":%lu,\"chunks\":%lu,\"off\":%lu}",
                      (unsigned long)s_x.size, (unsigned long)s_x.seq, (unsigned long)s_x.base);
    snprintf(sub, sizeof sub, "log/%s/end", s_x.fn);
    if (app_mqtt_pub_updata(sub, body, (uint16_t)bl) == 0)
    {
      s_x.sent_len = 0U;
      s_x.in_flight = 1U;
    }
  }
}

/* daily mirror: push the oldest un-mirrored day (up to DL_PUSH_MAXAGE back), one day at a time */
static uint32_t s_push_day_active = 0;

static void dl_push_mark_done(void)
{
  if (s_push_day_active != 0U)
  {
    s_pushed_day = s_push_day_active;
    s_push_day_active = 0U;
    (void)dl_lfs_save("log.pushed", &s_pushed_day, sizeof s_pushed_day);
  }
}

static void dl_push_poll(void)
{
  uint32_t now = app_time_now();
  uint32_t today, yesterday, day;
  char date[12];
  if ((now == 0U) || !app_mqtt_ready() || (s_x.state != 0U) || (s_dev == DEV_OFF)) { return; }
  today = now / 86400U;
  yesterday = today - 1U;
  if (s_pushed_day == 0U)                       /* first run: start mirroring from tomorrow on */
  {
    s_pushed_day = yesterday;
    (void)dl_lfs_save("log.pushed", &s_pushed_day, sizeof s_pushed_day);
    return;
  }
  if (s_pushed_day >= yesterday) { return; }
  day = s_pushed_day + 1U;
  if (day < (today - DL_PUSH_MAXAGE)) { day = today - DL_PUSH_MAXAGE; }
  dl_date_str(day, date);
  dl_xfer_reset(1U);
  dl_scan_date(date);                           /* every segment of that day, events included */
  s_push_day_active = day;
  if (s_x.qn == 0U) { dl_push_mark_done(); return; }   /* nothing recorded that day */
  dl_xfer_go();
}

/* ================= config ================= */
static void dl_cfg_save(void)
{
  char buf[64];
  int n = snprintf(buf, sizeof buf, "dev=%s\nperiod=%u\nquota_mb=%u\n",
                   (s_devcfg == DEV_SD) ? "sd" : (s_devcfg == DEV_FLASH) ? "flash" : "off",
                   (unsigned)s_period, (unsigned)s_quota);
  (void)dl_lfs_save("log.cfg", buf, (uint16_t)n);
}

static void dl_cfg_load(void)
{
  char buf[80];
  int n = dl_lfs_load("log.cfg", buf, sizeof buf - 1U);
  char *p;
  if (n <= 0) { return; }
  buf[n] = 0;
  if ((p = strstr(buf, "dev=")) != NULL)
  {
    if (strncmp(p + 4, "off", 3) == 0) { s_devcfg = DEV_OFF; }
    else if (strncmp(p + 4, "flash", 5) == 0) { s_devcfg = DEV_FLASH; }
    else { s_devcfg = DEV_SD; }
  }
  if ((p = strstr(buf, "period=")) != NULL)
  {
    long v = strtol(p + 7, NULL, 10);
    if ((v >= 5) && (v <= 3600) && ((v % 5) == 0)) { s_period = (uint16_t)v; }
  }
  if ((p = strstr(buf, "quota_mb=")) != NULL)
  {
    long v = strtol(p + 9, NULL, 10);
    if ((v >= 1) && (v <= 28)) { s_quota = (uint16_t)v; }
  }
}

static void dl_dev_probe(void)
{
  s_dev = DEV_OFF;
  if (s_devcfg == DEV_OFF) { snprintf(s_hbstr, sizeof s_hbstr, "off"); return; }
  if (s_devcfg == DEV_SD)
  {
    FRESULT fr = f_mkdir("0:/log");
    if ((fr == FR_OK) || (fr == FR_EXIST)) { s_dev = DEV_SD; }
    else
    {
      s_sd_fr = (int)fr;
      s_sd_errs++;
      s_dev = DEV_FLASH;
      app_log_event_src("SYSTEM", "sys", "log no sd (%d) -> flash", (int)fr);
    }
  }
  else { s_dev = DEV_FLASH; }
  if (s_dev == DEV_FLASH)
  {
    if (dl_flash_ready() != 0) { s_dev = DEV_OFF; return; }
  }
  snprintf(s_hbstr, sizeof s_hbstr, "%s %us", (s_dev == DEV_SD) ? "sd" : "flash",
           (unsigned)s_period);
}

/* ================= command surface (CLI + cloud, one implementation) ================= */
static int dl_name_ok(const char *s)
{
  uint16_t i;
  for (i = 0; s[i] != 0; i++)
  {
    char c = s[i];
    if (i >= 26U) { return 0; }
    if (((c >= '0') && (c <= '9')) || ((c >= 'a') && (c <= 'z')) ||
        (c == '-') || (c == '_') || (c == '.')) { continue; }
    return 0;
  }
  return (s[0] != 0);
}

int app_datalog_cmd(const char *line, const char *src, char *out, uint16_t cap)
{
  if (strncmp(line, "note ", 5) == 0)
  {
    if (line[5] == 0) { snprintf(out, cap, "note: empty"); return 1; }
    app_log_event_src("NOTE", src, "%s", line + 5);
    snprintf(out, cap, "note ok");
    return 1;
  }
  if (strcmp(line, "logstat") == 0)
  {
    snprintf(out, cap,
             "dlog %s cfg=%s period=%us quota=%uMB file=%s%s today=%luB drops=%u xfer=%s"
             " sdfr=%d sderrs=%u lfse=%d lfserrs=%u",
             (s_dev == DEV_SD) ? "sd" : (s_dev == DEV_FLASH) ? "flash" : "off",
             (s_devcfg == DEV_SD) ? "sd" : (s_devcfg == DEV_FLASH) ? "flash" : "off",
             (unsigned)s_period, (unsigned)s_quota,
             (s_date[0] != 0) ? s_date : "-", s_suffix,
             (unsigned long)s_today_bytes, (unsigned)s_drops,
             (s_x.state != 0U) ? s_x.fn : "idle",
             s_sd_fr, (unsigned)s_sd_errs, s_lfs_err, (unsigned)s_lfs_errs);
    return 1;
  }
  if (strncmp(line, "logcfg", 6) == 0)
  {
    const char *p;
    uint8_t changed = 0;
    if ((p = strstr(line, "dev=")) != NULL)
    {
      if (strncmp(p + 4, "off", 3) == 0) { s_devcfg = DEV_OFF; changed = 1U; }
      else if (strncmp(p + 4, "sd", 2) == 0) { s_devcfg = DEV_SD; changed = 1U; }
      else if (strncmp(p + 4, "flash", 5) == 0) { s_devcfg = DEV_FLASH; changed = 1U; }
      else { snprintf(out, cap, "logcfg: dev=sd|flash|off"); return 1; }
    }
    if ((p = strstr(line, "period=")) != NULL)
    {
      long v = strtol(p + 7, NULL, 10);
      if ((v < 5) || (v > 3600) || ((v % 5) != 0))
      {
        snprintf(out, cap, "logcfg: period=5..3600, multiple of 5");
        return 1;
      }
      s_period = (uint16_t)v; changed = 1U;
    }
    if ((p = strstr(line, "quota_mb=")) != NULL)
    {
      long v = strtol(p + 9, NULL, 10);
      if ((v < 1) || (v > 28)) { snprintf(out, cap, "logcfg: quota_mb=1..28"); return 1; }
      s_quota = (uint16_t)v; changed = 1U;
    }
    if (changed)
    {
      dl_cfg_save();
      dl_dev_probe();
      s_hdr[0] = 0;             /* force segment rotation under the new layout/period */
      s_win_id = 0;
      dl_win_reset();
      app_log_event_src("CONFIG", src, "%s", line);
    }
    snprintf(out, cap, "logcfg %s period=%us quota=%uMB%s",
             (s_devcfg == DEV_SD) ? "sd" : (s_devcfg == DEV_FLASH) ? "flash" : "off",
             (unsigned)s_period, (unsigned)s_quota, changed ? " saved" : "");
    return 1;
  }
  if (strncmp(line, "logget ", 7) == 0)
  {
    const char *a = line + 7;
    char name[28];
    uint32_t off = 0;
    uint16_t i = 0;
    while ((a[i] != 0) && (a[i] != ' ') && (i < sizeof name - 1U)) { name[i] = a[i]; i++; }
    name[i] = 0;
    if (a[i] == ' ') { off = (uint32_t)strtoul(&a[i + 1U], NULL, 10); }   /* tail pull from offset */
    if (!dl_name_ok(name)) { snprintf(out, cap, "logget: bad name"); return 1; }
    if (s_x.state != 0U) { snprintf(out, cap, "logget busy (%s)", s_x.fn); return 1; }
    dl_xfer_reset(0U);
    if ((strlen(name) == 10U) && (name[4] == '-') && (name[7] == '-'))   /* bare date -> every file of that day */
    {
      dl_scan_date(name);
      if (s_x.qn == 0U)                          /* nothing that day: still close the loop with {"size":0} */
      {
        char f1[28];
        snprintf(f1, sizeof f1, "%s.csv", name);
        dl_xfer_add(f1);
      }
    }
    else
    {
      dl_xfer_add(name);
      s_x.req_off = off;
    }
    dl_xfer_go();
    snprintf(out, cap, "logget %s start (%u files, off=%lu)", name, (unsigned)s_x.qn, (unsigned long)off);
    return 1;
  }
  return 0;
}

int app_datalog_cli(char *line)   /* CLI hook (app_cli.c): note/logcfg/logstat/logget */
{
  char out[224];
  if (!app_datalog_cmd(line, "cli", out, sizeof out)) { return 0; }
  printf("%s\n\r", out);
  return 1;
}

const char *app_datalog_hb(void) { return s_hbstr; }

/* ================= poll ================= */
void app_datalog_poll(void)
{
  static uint8_t  s_inited = 0;
  static uint8_t  s_subtick = 0;
  static uint8_t  s_prev_link = 0;
  static uint8_t  s_prev_tsrc = 0;
  uint32_t now;
  if (!s_inited)
  {
    s_inited = 1U;
    dl_cfg_load();
    dl_dev_probe();
    (void)dl_lfs_load("log.pushed", &s_pushed_day, sizeof s_pushed_day);
    dl_win_reset();
    /* boot forensics (2026-09-02): the reset names itself — app_reset() breadcrumb,
     * fault black-box dump (type/CFSR/PC/...), or the loud "UNNAMED" that means a
     * reset from outside every instrumented path. Records are one-shot (cleared). */
    { extern void app_diag_boot_forensics(char *out, uint16_t cap);
      char fx[160];
      app_diag_boot_forensics(fx, sizeof fx);
      { extern void app_diag_boot_ring_note(const char *, const char *);
        app_diag_boot_ring_note(app_diag_reset_cause(), fx); }   /* battery ring: survives loops the journal misses */
      app_log_event_src("SYSTEM", "sys", "boot rst=%s%s", app_diag_reset_cause(), fx); }
  }
  /* system-event edges */
  { uint8_t up = app_mqtt_ready();
    if (up != s_prev_link)
    {
      app_log_event_src("SYSTEM", "sys", "mqtt %s", up ? "up" : "down");
      s_prev_link = up;
    } }
  { uint8_t ts = app_time_source();
    if ((ts == 2U) && (s_prev_tsrc != 2U)) { app_log_event_src("SYSTEM", "sys", "time sync"); }
    s_prev_tsrc = ts; }

  /* configured for SD but running on flash (boot race / card pulled): re-probe every 10 min —
   * a working card takes over seamlessly; history written to flash meanwhile stays there */
  { static uint16_t s_sd_retry = 0;
    if ((s_devcfg == DEV_SD) && (s_dev == DEV_FLASH))
    {
      if (++s_sd_retry >= 1200U)
      {
        FRESULT fr;
        s_sd_retry = 0;
        fr = f_mkdir("0:/log");
        if ((fr == FR_OK) || (fr == FR_EXIST))
        {
          s_dev = DEV_SD;
          s_werr = 0;
          s_hdr[0] = 0;   /* new medium: rotate segment so the header lands in the new file */
          app_log_event_src("SYSTEM", "sys", "log sd back");
        }
      }
    }
    else { s_sd_retry = 0; } }

  now = app_time_now();
  if ((now != 0U) && (s_dev != DEV_OFF))
  {
    /* sub-sample every 5 s; flush when the window id (ts/period) moves */
    if (++s_subtick >= DL_SUB_TICKS)
    {
      s_subtick = 0;
      { uint32_t win = now / s_period;
        if (s_win_id == 0U) { s_win_id = win; dl_win_reset(); }
        else if (win != s_win_id)
        {
          dl_flush_row(win * s_period);   /* row stamped at the window boundary */
          s_win_id = win;
          dl_win_reset();
        } }
      dl_subsample();
    }
    dl_ev_drain();
    dl_push_poll();
  }
  dl_xfer_step();
}
