/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_flowmon.c — per-gutter feed-flow monitor: blocked-line / lost-supply alarm (2026-09-04).
 *
 * Input: the four hall flow meters on HSDI4..7 (1200 pulses per litre, one per NFT gutter feed
 * line), read as pulse totals from the process image. A 60 s sliding window (six 10 s
 * buckets) gives each gutter's rate independently of the heartbeat's lazy 30 s figure.
 *
 * Rule (user request 2026-09-04: "flow below 1.2 L/min means the line is probably blocked"):
 *   low   : rate < threshold (default 1.2 L/min) sustained `sus` seconds (default 120 s)
 *           -> alarm ACTIVE for that gutter
 *   all   : all four low at the same time is not four blockages, it is the pump or the
 *           supply -> ONE "feed supply" alarm instead of four gutter alarms
 *   clear : rate >= threshold + 0.1 L/min sustained 60 s -> alarm CLEAR
 *   warm  : nothing is judged until the window has 60 s of data after boot
 *
 * Where an alarm goes:
 *   1. the on-board event journal (FAULT, so it lands on the history charts as a red marker)
 *   2. the cloud, live: dev/<type>/<sn>/up/event, JSON per Device_Cloud_Protocol.md §9
 *      (id "al.gutterN.flow" / "al.feed.supply", state active|clear) — the dashboard relays
 *      it to a phone (ntfy / Telegram) the moment it arrives, no 2-minute mirror delay
 *   3. the heartbeat field "ga" (bitmask) -> dashboard point "Gutter Alarm"
 *   4. optionally the on-board buzzer: a short chirp every 10 s while an alarm is active,
 *      `flow beep on|off` to enable, `flow ack` to silence the current alarms
 *
 * Settings (`flow th/sus/beep/on/off`) persist in littlefs "flow.cfg" (dose.cfg pattern).
 * Task: one 10 s tick, tiny stack; no I/O other than reading counters and the buzzer. */
#include "app_user.h"
#include "app_platform.h"
#include "app_flowmon.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lfs.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern void app_log_event_src(const char *type, const char *src, const char *fmt, ...);
extern int  app_mqtt_pub_updata(const char *sub, const void *data, uint16_t len);
extern uint8_t app_mqtt_pub_updata_busy(void);
extern uint8_t app_mqtt_ready(void);

#define FM_GUTTERS     4U
#define FM_PPL         1200UL        /* meter pulses per litre */
#define FM_TICK_S      10U           /* control period */
#define FM_BUCKETS     6U            /* 6 x 10 s = 60 s rate window */
#define FM_CLEAR_HYS   1U            /* clear at threshold + 0.1 L/min ... (0.2 left a gutter running at 1.3..1.4 stuck in alarm against a 1.2 limit, field test 2026-09-04) */
#define FM_CLEAR_SUS   60U           /* ... sustained this long */
#define FM_CHIRP_MS    300U

static const uint8_t s_ch[FM_GUTTERS] = { 4, 5, 6, 7 };   /* HSDI channel per gutter (app_user.c s_gutter_ch) */

/* settings (persisted) */
static uint8_t  s_en = 1;            /* monitor enabled */
static uint16_t s_th_dl = 12;        /* threshold, 0.1 L/min */
static uint16_t s_sus_s = 120;       /* low must last this long */
static uint8_t  s_beep_en = 1;       /* chirp while an alarm is active */

/* state */
static uint32_t s_last_cnt[FM_GUTTERS];
static uint16_t s_bucket[FM_GUTTERS][FM_BUCKETS];
static uint8_t  s_bidx = 0, s_filled = 0;
static uint16_t s_rate_dl[FM_GUTTERS];        /* last computed rate, 0.1 L/min */
static uint16_t s_low_s[FM_GUTTERS], s_ok_s[FM_GUTTERS];
static uint8_t  s_alarm[FM_GUTTERS];          /* per-gutter alarm active */
static uint8_t  s_supply;                     /* all-low alarm active */
static uint16_t s_all_low_s, s_all_ok_s;
static uint8_t  s_acked;                      /* buzzer silenced until every alarm clears */
static uint8_t  s_primed;

/* cloud event queue: one pending state per alarm id (4 gutters + supply); retried each tick
 * while the shared uplink slot is busy or the link is down */
#define FM_IDS (FM_GUTTERS + 1U)
static uint8_t  s_pend[FM_IDS];               /* 0 = nothing, 1 = active, 2 = clear */
static uint16_t s_pend_val[FM_IDS];

/* ---- persistence: littlefs "flow.cfg" = "en th sus beep\n" ---- */
#define FM_FILE "flow.cfg"
extern lfs_t *app_lfs(void);
static uint8_t s_fbuf[256];
static const struct lfs_file_config s_fcfg = { .buffer = s_fbuf };
static uint8_t s_loaded;

static void fm_save(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[48];
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, FM_FILE, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                       (struct lfs_file_config *)&s_fcfg) != 0) { return; }
  snprintf(buf, sizeof buf, "%u %u %u %u\n", (unsigned)s_en, (unsigned)s_th_dl, (unsigned)s_sus_s, (unsigned)s_beep_en);
  (void)lfs_file_write(fs, &f, buf, strlen(buf));
  (void)lfs_file_close(fs, &f);
}
static void fm_load(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[48];
  unsigned en, th, sus, bp;
  if (s_loaded || (fs == NULL)) { return; }
  s_loaded = 1;
  if (lfs_file_opencfg(fs, &f, FM_FILE, LFS_O_RDONLY, (struct lfs_file_config *)&s_fcfg) != 0) { return; }
  lfs_ssize_t n = lfs_file_read(fs, &f, buf, sizeof(buf) - 1U);
  (void)lfs_file_close(fs, &f);
  if (n <= 0) { return; }
  buf[n] = 0;
  if (sscanf(buf, "%u %u %u %u", &en, &th, &sus, &bp) == 4)
  {
    s_en = (uint8_t)(en != 0U); s_th_dl = (uint16_t)th; s_sus_s = (uint16_t)sus; s_beep_en = (uint8_t)(bp != 0U);
  }
}

/* ---- alarm raise / clear: journal + cloud event + buzzer arming ---- */
static void fm_event(uint8_t id, uint8_t active, uint16_t val_dl)
{
  if (id < FM_GUTTERS)
  {
    if (active)
    {
      app_log_event("FAULT", "gutter %u flow low %u.%u L/min (<%u.%u, blocked?)", (unsigned)(id + 1U),
                    (unsigned)(val_dl / 10U), (unsigned)(val_dl % 10U), (unsigned)(s_th_dl / 10U), (unsigned)(s_th_dl % 10U));
    }
    else
    {
      app_log_event("FAULT", "gutter %u flow ok %u.%u L/min", (unsigned)(id + 1U), (unsigned)(val_dl / 10U), (unsigned)(val_dl % 10U));
    }
  }
  else
  {
    if (active) { app_log_event("FAULT", "feed flow low on all gutters (pump/supply?)"); }
    else        { app_log_event("FAULT", "feed flow ok on all gutters"); }
  }
  s_pend[id] = active ? 1U : 2U;
  s_pend_val[id] = val_dl;
  if (active) { s_acked = 0U; }              /* a new alarm re-arms the buzzer even after an ack */
}

/* one attempt per tick per pending id: the uplink slot is shared with the datalog mirror */
static void fm_flush_events(void)
{
  for (uint8_t id = 0; id < FM_IDS; id++)
  {
    char js[220];
    int n;
    if (s_pend[id] == 0U) { continue; }
    if (!app_mqtt_ready() || app_mqtt_pub_updata_busy()) { return; }
    if (id < FM_GUTTERS)
    {
      n = snprintf(js, sizeof js,
                   "{\"pv\":1,\"ts\":%lu,\"id\":\"al.gutter%u.flow\",\"src\":\"io.g%u\",\"sev\":\"alarm\",\"state\":\"%s\","
                   "\"msg\":\"Gutter %u feed flow %s: %u.%u L/min (limit %u.%u)\",\"val\":%u.%u}",
                   (unsigned long)app_time_now(), (unsigned)(id + 1U), (unsigned)(id + 1U),
                   (s_pend[id] == 1U) ? "active" : "clear", (unsigned)(id + 1U),
                   (s_pend[id] == 1U) ? "LOW - line blocked?" : "back to normal",
                   (unsigned)(s_pend_val[id] / 10U), (unsigned)(s_pend_val[id] % 10U),
                   (unsigned)(s_th_dl / 10U), (unsigned)(s_th_dl % 10U),
                   (unsigned)(s_pend_val[id] / 10U), (unsigned)(s_pend_val[id] % 10U));
    }
    else
    {
      n = snprintf(js, sizeof js,
                   "{\"pv\":1,\"ts\":%lu,\"id\":\"al.feed.supply\",\"src\":\"io.g1\",\"sev\":\"alarm\",\"state\":\"%s\","
                   "\"msg\":\"Feed flow %s on ALL gutters - pump or supply?\",\"val\":0}",
                   (unsigned long)app_time_now(), (s_pend[id] == 1U) ? "active" : "clear",
                   (s_pend[id] == 1U) ? "LOW" : "back to normal");
    }
    if ((n > 0) && (app_mqtt_pub_updata("event", js, (uint16_t)n) == 0)) { s_pend[id] = 0U; }
    else { return; }
  }
}

/* ---- the 10 s tick ---- */
static void fm_tick(void)
{
  uint32_t sum;
  uint8_t  low[FM_GUTTERS], nlow = 0, any = 0;

  fm_load();
  /* rate window */
  for (uint8_t g = 0; g < FM_GUTTERS; g++)
  {
    uint32_t c = app_hsdi_count(s_ch[g]);
    uint32_t d = c - s_last_cnt[g];
    s_last_cnt[g] = c;
    if (!s_primed) { d = 0; }
    s_bucket[g][s_bidx] = (uint16_t)((d > 65535UL) ? 65535UL : d);
  }
  s_primed = 1U;
  s_bidx = (uint8_t)((s_bidx + 1U) % FM_BUCKETS);
  if (s_bidx == 0U) { s_filled = 1U; }
  if (!s_filled) { return; }                               /* boot warm-up: no verdict on partial data */
  for (uint8_t g = 0; g < FM_GUTTERS; g++)
  {
    sum = 0;
    for (uint8_t k = 0; k < FM_BUCKETS; k++) { sum += s_bucket[g][k]; }
    s_rate_dl[g] = (uint16_t)((sum * 10UL) / FM_PPL);     /* pulses per 60 s -> 0.1 L/min */
    low[g] = (uint8_t)(s_rate_dl[g] < s_th_dl);
    if (low[g]) { nlow++; }
  }
  if (!s_en) { return; }

  /* all four low = supply problem (one alarm), else per-gutter blockage alarms */
  if (nlow == FM_GUTTERS) { s_all_low_s = (uint16_t)(s_all_low_s + FM_TICK_S); s_all_ok_s = 0; }
  else                    { s_all_low_s = 0; s_all_ok_s = (uint16_t)(s_all_ok_s + FM_TICK_S); }
  if (!s_supply && (s_all_low_s >= s_sus_s))
  {
    s_supply = 1U; fm_event(FM_GUTTERS, 1U, 0);
    for (uint8_t g = 0; g < FM_GUTTERS; g++) { s_low_s[g] = 0; }   /* gutter timers restart when supply is back */
  }
  else if (s_supply && (s_all_ok_s >= FM_CLEAR_SUS)) { s_supply = 0U; fm_event(FM_GUTTERS, 0U, 0); }

  for (uint8_t g = 0; g < FM_GUTTERS; g++)
  {
    if (low[g]) { s_low_s[g] = (uint16_t)(s_low_s[g] + FM_TICK_S); }
    else        { s_low_s[g] = 0; }
    if (s_rate_dl[g] >= (uint16_t)(s_th_dl + FM_CLEAR_HYS)) { s_ok_s[g] = (uint16_t)(s_ok_s[g] + FM_TICK_S); }
    else { s_ok_s[g] = 0; }
    if (!s_alarm[g] && !s_supply && (s_low_s[g] >= s_sus_s)) { s_alarm[g] = 1U; fm_event(g, 1U, s_rate_dl[g]); }
    else if (s_alarm[g] && (s_ok_s[g] >= FM_CLEAR_SUS))       { s_alarm[g] = 0U; fm_event(g, 0U, s_rate_dl[g]); }
    if (s_alarm[g]) { any = 1U; }
  }
  if (s_supply) { any = 1U; }
  if (!any) { s_acked = 0U; }

  fm_flush_events();

  if (any && s_beep_en && !s_acked)                         /* short chirp per tick, never a continuous tone */
  {
    app_beep_set(1); vTaskDelay(pdMS_TO_TICKS(FM_CHIRP_MS)); app_beep_set(0);
  }
}

static void flowmon_task(void *arg)
{
  (void)arg;
  vTaskDelay(pdMS_TO_TICKS(20000));   /* counters and the process image must be alive first */
  for (;;)
  {
    fm_tick();
    vTaskDelay(pdMS_TO_TICKS(FM_TICK_S * 1000U));
  }
}

uint8_t app_flowmon_alarms(void)
{
  uint8_t m = 0;
  for (uint8_t g = 0; g < FM_GUTTERS; g++) { if (s_alarm[g]) { m |= (uint8_t)(1U << g); } }
  if (s_supply) { m |= 0x10U; }
  return m;
}

/* "1.2" / "12" / "1" -> tenths */
static int parse_dl(const char *s)
{
  int v = atoi(s), frac = 0;
  const char *dot = strchr(s, '.');
  if (dot != NULL) { if ((dot[1] >= '0') && (dot[1] <= '9')) { frac = dot[1] - '0'; } return v * 10 + frac; }
  return v;                                                  /* no dot: the caller scales whole litres */
}

int app_flowmon_cmd(const char *line, const char *src, char *out, uint16_t cap)
{
  if ((strncmp(line, "flow", 4) != 0) || ((line[4] != 0) && (line[4] != ' '))) { return 0; }
  const char *a = line + 4;
  while (*a == ' ') { a++; }
  fm_load();
  if ((*a == 0) || (strcmp(a, "stat") == 0)) { /* status only */ }
  else if (strncmp(a, "th ", 3) == 0)
  {
    int v = parse_dl(a + 3);
    if (strchr(a + 3, '.') == NULL) { v *= 10; }             /* "flow th 1" = 1.0 L/min */
    if ((v < 1) || (v > 300)) { snprintf(out, cap, "flow th: 0.1..30.0 L/min"); return 1; }
    s_th_dl = (uint16_t)v; fm_save();
    app_log_event_src("CONFIG", src, "flow th %u.%u L/min", (unsigned)(s_th_dl / 10U), (unsigned)(s_th_dl % 10U));
  }
  else if (strncmp(a, "sus ", 4) == 0)
  {
    int v = atoi(a + 4);
    if ((v < 10) || (v > 3600)) { snprintf(out, cap, "flow sus: 10..3600 s"); return 1; }
    s_sus_s = (uint16_t)v; fm_save();
    app_log_event_src("CONFIG", src, "flow sus %us", (unsigned)s_sus_s);
  }
  else if (strncmp(a, "beep ", 5) == 0)
  {
    s_beep_en = (uint8_t)(strncmp(a + 5, "on", 2) == 0); fm_save();
    if (!s_beep_en) { app_beep_set(0); }
    app_log_event_src("CONFIG", src, "flow beep %s", s_beep_en ? "on" : "off");
  }
  else if (strcmp(a, "ack") == 0) { s_acked = 1U; app_beep_set(0); app_log_event_src("ACTION", src, "flow alarm ack"); }
  else if (strcmp(a, "on") == 0)  { s_en = 1U; fm_save(); app_log_event_src("CONFIG", src, "flow monitor on"); }
  else if (strcmp(a, "off") == 0)
  {
    s_en = 0U; fm_save(); app_beep_set(0);
    for (uint8_t g = 0; g < FM_GUTTERS; g++) { s_alarm[g] = 0; s_low_s[g] = 0; s_ok_s[g] = 0; }
    s_supply = 0; s_all_low_s = 0; s_all_ok_s = 0;
    app_log_event_src("CONFIG", src, "flow monitor off");
  }
  else { snprintf(out, cap, "flow: stat|th <L/min>|sus <s>|beep on|off|ack|on|off"); return 1; }

  snprintf(out, cap, "flow mon=%s th=%u.%u sus=%us beep=%s alm=0x%02x%s g=%u.%u/%u.%u/%u.%u/%u.%u%s",
           s_en ? "on" : "off", (unsigned)(s_th_dl / 10U), (unsigned)(s_th_dl % 10U), (unsigned)s_sus_s,
           s_beep_en ? "on" : "off", (unsigned)app_flowmon_alarms(), s_acked ? "(ack)" : "",
           (unsigned)(s_rate_dl[0] / 10U), (unsigned)(s_rate_dl[0] % 10U), (unsigned)(s_rate_dl[1] / 10U), (unsigned)(s_rate_dl[1] % 10U),
           (unsigned)(s_rate_dl[2] / 10U), (unsigned)(s_rate_dl[2] % 10U), (unsigned)(s_rate_dl[3] / 10U), (unsigned)(s_rate_dl[3] % 10U),
           s_filled ? "" : " (warming up)");
  return 1;
}

int app_flowmon_cli(char *line)
{
  char out[160];
  if (!app_flowmon_cmd(line, "cli", out, sizeof out)) { return 0; }
  printf("%s\n\r", out);
  return 1;
}

void app_flowmon_init(void)
{
  fm_load();
  xTaskCreate(flowmon_task, "flowmon", 768, NULL, APP_TASK_PRIO_LOW, NULL);
}
