/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_mbcfg.c — generic Modbus port config (CM7 side; contract docs/Universal_Modbus_Port_Config.md)
 * Duties: (1) config model + littlefs /mbport.cfg persistence (line-based = CLI syntax) (2) op8 push to CM4
 * (3) shared CLI/cloud syntax mbcfg/mbpoll (4) defaultTask tick: first load / cloud command queue / TCP service.
 * Thread model: all on defaultTask except app_mbcfg_cloud (tcpip thread, only copies string + raises flag). */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "app_mbcfg.h"
#include "app_mbtcp.h"
#include "app_rpc.h"
#include "modbus_core.h"
#include "lfs.h"
#include "app_platform.h"   /* APP_PORT_BACKPLANE + coil/reg helper prototypes */
#include "app_mbport.h"     /* app_mb_transact() — raw backplane transaction on CM4 UART8 */

extern lfs_t *app_lfs(void);

#define NPORT 6
static const char *s_names[NPORT] = { "485a", "485b", "485c", "485d", "485e", "485f" };
static const char s_role_ch[3] = { '-', 'S', 'M' };

typedef struct {
  uint8_t  role;      /* 0=off 1=slave 2=master */
  uint32_t baud;
  uint8_t  parity;    /* 0=N 1=E 2=O */
  uint8_t  stop;      /* 1|2 */
  uint8_t  addr;      /* slave address */
} mbcfg_t;

static mbcfg_t s_cfg[NPORT];
static uint8_t s_tcp_on = 1;
static uint8_t s_applied = 0;
static uint8_t s_app_cfg = 0;   /* set by app_mbport_configure(): app owns port config -> skip the boot-time file load */

static char s_cloudq[96];
static volatile uint8_t s_cloud_pend = 0;

uint8_t app_mbcfg_tcp_on(void) { return s_tcp_on; }

static void cfg_defaults(void)
{
  for (int i = 0; i < NPORT; i++)
  {
    s_cfg[i].role = 0; s_cfg[i].baud = 9600; s_cfg[i].parity = 0;
    s_cfg[i].stop = 1; s_cfg[i].addr = 1;
  }
  s_tcp_on = 1;
}

/* ---- littlefs persistence (line-based; LFS_NO_MALLOC = bring your own static file buffer, precedent app_lfs.c) ---- */
static uint8_t s_fbuf[256];
static const struct lfs_file_config s_fcfg = { .buffer = s_fbuf };

static void cfg_line(int i, char *out, int cap)
{
  if (s_cfg[i].role == 0U) { snprintf(out, (size_t)cap, "%s off", s_names[i]); return; }
  snprintf(out, (size_t)cap, "%s %s %lu 8%c%u%s%u",
           s_names[i], (s_cfg[i].role == 1U) ? "slave" : "master",
           (unsigned long)s_cfg[i].baud, "NEO"[s_cfg[i].parity], s_cfg[i].stop,
           (s_cfg[i].role == 1U) ? " addr=" : " addr=", s_cfg[i].addr);
}

static void cfg_save(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[64];
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, "mbport.cfg", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                       (struct lfs_file_config *)&s_fcfg) != 0) { return; }
  for (int i = 0; i < NPORT; i++)
  {
    cfg_line(i, buf, sizeof(buf) - 2);
    strcat(buf, "\n");
    (void)lfs_file_write(fs, &f, buf, strlen(buf));
  }
  snprintf(buf, sizeof(buf), "tcp server=%s\n", s_tcp_on ? "on" : "off");
  (void)lfs_file_write(fs, &f, buf, strlen(buf));
  (void)lfs_file_close(fs, &f);
}

static int port_by_name(const char *s, int len)
{
  for (int i = 0; i < NPORT; i++)
  {
    if ((strncmp(s, s_names[i], (size_t)len) == 0) && ((int)strlen(s_names[i]) == len)) { return i; }
  }
  return -1;
}

/* Parse one config line (CLI syntax is also the file syntax): "485a slave 9600 8N1 addr=1" / "485a off" / "tcp server=on"
 * Returns: port index 0..5 / 6=tcp / -1=unrecognized; on success already written into s_cfg/s_tcp_on (not pushed, not saved) */
static int cfg_parse_line(const char *line)
{
  char tok[6][16];
  int nt = 0;
  const char *p = line;
  while ((*p != 0) && (nt < 6))
  {
    while (*p == ' ') { p++; }
    if (*p == 0) { break; }
    int k = 0;
    while ((*p != 0) && (*p != ' ') && (k < 15)) { tok[nt][k++] = *p++; }
    tok[nt][k] = 0;
    nt++;
  }
  if (nt < 2) { return -1; }
  if (strcmp(tok[0], "tcp") == 0)
  {
    if (strcmp(tok[1], "server=on") == 0 || strcmp(tok[1], "on") == 0)  { s_tcp_on = 1; return 6; }
    if (strcmp(tok[1], "server=off") == 0 || strcmp(tok[1], "off") == 0) { s_tcp_on = 0; return 6; }
    return -1;
  }
  int i = port_by_name(tok[0], (int)strlen(tok[0]));
  if (i < 0) { return -1; }
  if (strcmp(tok[1], "off") == 0) { s_cfg[i].role = 0; return i; }
  uint8_t role;
  if      (strcmp(tok[1], "slave") == 0)  { role = 1; }
  else if (strcmp(tok[1], "master") == 0) { role = 2; }
  else { return -1; }
  uint32_t baud = 9600;
  uint8_t parity = 0, stop = 1, addr = s_cfg[i].addr;
  for (int t = 2; t < nt; t++)
  {
    if (strncmp(tok[t], "addr=", 5) == 0) { addr = (uint8_t)strtoul(&tok[t][5], 0, 10); }
    else if ((tok[t][0] == '8') && (tok[t][1] != 0) && (tok[t][2] != 0))   /* 8N1/8E1/8O2 */
    {
      parity = (tok[t][1] == 'E' || tok[t][1] == 'e') ? 1U : (tok[t][1] == 'O' || tok[t][1] == 'o') ? 2U : 0U;
      stop = (tok[t][2] == '2') ? 2U : 1U;
    }
    else if ((tok[t][0] >= '0') && (tok[t][0] <= '9')) { baud = strtoul(tok[t], 0, 10); }
  }
  s_cfg[i].role = role; s_cfg[i].baud = baud; s_cfg[i].parity = parity;
  s_cfg[i].stop = stop; s_cfg[i].addr = addr;
  return i;
}

static void cfg_load(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  static char buf[512];
  cfg_defaults();
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, "mbport.cfg", LFS_O_RDONLY,
                       (struct lfs_file_config *)&s_fcfg) != 0) { return; }
  lfs_ssize_t n = lfs_file_read(fs, &f, buf, sizeof(buf) - 1);
  (void)lfs_file_close(fs, &f);
  if (n <= 0) { return; }
  buf[n] = 0;
  char *line = buf;
  while (line != NULL)
  {
    char *nl = strchr(line, '\n');
    if (nl != NULL) { *nl = 0; }
    if (line[0] != 0) { (void)cfg_parse_line(line); }
    line = (nl != NULL) ? (nl + 1) : NULL;
  }
}

/* ---- op8 push (defaultTask) ---- */
static int port_apply(int i)
{
  uint8_t req[10], ack[4];
  if (s_cfg[i].role == 0U) { return 0; }   /* off = don't push (CM4 side defaults to off; a runtime change to off is pushed via port_apply_off) */
  req[0] = 0x08U; req[1] = (uint8_t)i; req[2] = s_cfg[i].role;
  req[3] = (uint8_t)(s_cfg[i].baud & 0xFFU);
  req[4] = (uint8_t)((s_cfg[i].baud >> 8) & 0xFFU);
  req[5] = (uint8_t)((s_cfg[i].baud >> 16) & 0xFFU);
  req[6] = (uint8_t)((s_cfg[i].baud >> 24) & 0xFFU);
  req[7] = s_cfg[i].parity; req[8] = s_cfg[i].stop; req[9] = s_cfg[i].addr;
  uint16_t r = app_rpc_transact(req, 10, ack, sizeof(ack), 150);
  return ((r >= 1U) && (ack[0] == 0U)) ? 0 : -1;
}

static int port_apply_off(int i)   /* runtime switch to off: send role=0 anyway (CM4 stops serving that port) */
{
  uint8_t req[10] = { 0x08U, (uint8_t)i, 0U, 0x80U, 0x25U, 0U, 0U, 0U, 1U, 1U };   /* baud=9600 placeholder */
  uint8_t ack[4];
  uint16_t r = app_rpc_transact(req, 10, ack, sizeof(ack), 150);
  return ((r >= 1U) && (ack[0] == 0U)) ? 0 : -1;
}

/* ---- heartbeat mb field: compact string like "a:S1@9600 c:M@19200 tcp:2" ---- */
void app_mbcfg_hb(char *out, int cap)
{
  int w = 0;
  out[0] = 0;
  for (int i = 0; i < NPORT; i++)
  {
    if (s_cfg[i].role == 0U) { continue; }
    w += snprintf(out + w, (size_t)(cap - w), "%s%c:%c%u@%lu",
                  (w > 0) ? " " : "", s_names[i][3], s_role_ch[s_cfg[i].role],
                  (s_cfg[i].role == 1U) ? s_cfg[i].addr : 0U,
                  (unsigned long)s_cfg[i].baud);
    if (w >= cap - 12) { break; }
  }
  snprintf(out + w, (size_t)(cap - w), "%stcp:%s", (w > 0) ? " " : "", s_tcp_on ? "on" : "off");
}

/* ---- CLI: mbcfg ---- */
static void cli_show(void)
{
  char buf[64];
  printf("port  role  line        addr   rx/tx/frm/ovr/err\n\r");
  for (int i = 0; i < NPORT; i++)
  {
    if (s_cfg[i].role == 0U) { printf("  %s  off\n\r", s_names[i]); continue; }
    printf("  %s  %-6s %lu 8%c%u  %u", s_names[i],
           (s_cfg[i].role == 1U) ? "slave" : "master",
           (unsigned long)s_cfg[i].baud, "NEO"[s_cfg[i].parity], s_cfg[i].stop,
           s_cfg[i].addr);
    {
      uint8_t req[2] = { 0x0BU, (uint8_t)i };
      uint8_t st[48];
      uint16_t r = app_rpc_transact(req, 2, st, sizeof(st), 150);
      if (r >= 20U)
      {
        uint32_t v[10] = {0};
        for (int k = 0; (k < 10) && ((unsigned)(k * 4 + 4) <= r); k++)
        {
          const uint8_t *b = &st[k * 4];
          v[k] = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
        }
        printf("   %lu/%lu/%lu/%lu/%lu  ring[wr=%lu rd=%lu fs=%lu fw=%lu fr=%lu]",
               (unsigned long)v[0], (unsigned long)v[1], (unsigned long)v[2],
               (unsigned long)v[3], (unsigned long)v[4], (unsigned long)v[5],
               (unsigned long)v[6], (unsigned long)v[7], (unsigned long)v[8], (unsigned long)v[9]);
      }
    }
    printf("\n\r");
  }
  printf("  tcp   server=%s (port 502, max 2 conn)\n\r", s_tcp_on ? "on" : "off");
  app_mbcfg_hb(buf, sizeof(buf));
  printf("  hb: %s\n\r", buf);
}

static void cli_mbcfg(char *line)   /* line = the part after "mbcfg" (may be empty) */
{
  while (*line == ' ') { line++; }
  if (*line == 0) { cli_show(); return; }
  uint8_t was_on[NPORT];
  for (int i = 0; i < NPORT; i++) { was_on[i] = (s_cfg[i].role != 0U) ? 1U : 0U; }
  int r = cfg_parse_line(line);
  if (r < 0) { printf("mbcfg: bad syntax. ex: mbcfg 485a slave 9600 8N1 addr=1 | mbcfg 485b off | mbcfg tcp on\n\r"); return; }
  if (r == 6)   /* tcp toggle */
  {
    if (s_tcp_on) { app_mbtcp_init(); }
    cfg_save();
    printf("tcp server=%s (saved)\n\r", s_tcp_on ? "on" : "off");
    return;
  }
  int rc = (s_cfg[r].role != 0U) ? port_apply(r) : (was_on[r] ? port_apply_off(r) : 0);
  cfg_save();
  printf("%s -> %s (%s, saved)\n\r", s_names[r],
         (s_cfg[r].role == 0U) ? "off" : (s_cfg[r].role == 1U) ? "slave" : "master",
         (rc == 0) ? "applied" : "APPLY FAIL");
}

/* ---- CLI: mbpoll <port|tcp> ... one transaction ---- */
static void poll_print_rsp(uint8_t fc, const uint8_t *rsp, uint16_t rl)
{
  uint8_t exc;
  if (rl == 0U) { printf("no reply (timeout)\n\r"); return; }
  if (mb_rsp_is_exception(rsp, rl, &exc) == 1) { printf("exception %u\n\r", exc); return; }
  if ((fc == MB_FC_READ_HOLD) || (fc == MB_FC_READ_INPUT))
  {
    uint16_t regs[32];
    int n = mb_rsp_regs(rsp, rl, fc, regs, 32);
    if (n < 0) { printf("bad response\n\r"); return; }
    printf("regs:");
    for (int i = 0; i < n; i++) { printf(" %u(0x%04X)", regs[i], regs[i]); }
    printf("\n\r");
    return;
  }
  if ((fc == MB_FC_READ_COILS) || (fc == MB_FC_READ_DISC))
  {
    uint8_t bits[8];
    int n = mb_rsp_bits(rsp, rl, fc, bits, 8);
    if (n < 0) { printf("bad response\n\r"); return; }
    printf("bits:");
    for (int i = 0; i < n; i++) { printf(" %02X", bits[i]); }
    printf("\n\r");
    return;
  }
  printf("write ok\n\r");
}

static void cli_mbpoll(char *line)   /* "mbpoll 485b 1 4 0 4" | "mbpoll tcp 192.168.137.1[:1502] 1 3 0 4" | write: +val */
{
  char tgt[24];
  unsigned addr, fc, reg, n;
  unsigned long val = 0;
  int is_tcp = 0, nf;
  while (*line == ' ') { line++; }
  if (strncmp(line, "tcp ", 4) == 0)   /* tcp target: next token = ip[:port] */
  {
    is_tcp = 1;
    nf = sscanf(line + 4, "%23s %u %u %u %u %lu", tgt, &addr, &fc, &reg, &n, &val);
  }
  else
  {
    nf = sscanf(line, "%23s %u %u %u %u %lu", tgt, &addr, &fc, &reg, &n, &val);
  }
  if (nf < 5) { printf("mbpoll <485x|tcp ip[:port]> <addr|unit> <fc> <reg> <n> [val]\n\r"); return; }
  uint8_t pdu[64], rsp[MB_PDU_MAX];
  int pn;
  switch (fc)
  {
    case 1: case 2: case 3: case 4:
      pn = mb_req_read(pdu, (uint8_t)fc, (uint16_t)reg, (uint16_t)n);
      break;
    case 5:
      pn = mb_req_write_single(pdu, MB_FC_WRITE_COIL, (uint16_t)reg, (val != 0UL) ? 0xFF00U : 0x0000U);
      break;
    case 6:
      pn = mb_req_write_single(pdu, MB_FC_WRITE_REG, (uint16_t)reg, (uint16_t)val);
      break;
    case 15:
    {
      uint8_t bits[4] = { (uint8_t)val, (uint8_t)(val >> 8), (uint8_t)(val >> 16), (uint8_t)(val >> 24) };
      if ((n == 0U) || (n > 32U)) { printf("fc15: n<=32 (val=mask)\n\r"); return; }
      pn = mb_req_write_coils(pdu, (uint16_t)reg, (uint16_t)n, bits);
      break;
    }
    case 16:
    {
      uint16_t vals[8];
      if ((n == 0U) || (n > 8U)) { printf("fc16: n<=8 (same val)\n\r"); return; }
      for (unsigned k = 0; k < n; k++) { vals[k] = (uint16_t)val; }
      pn = mb_req_write_regs(pdu, (uint16_t)reg, (uint16_t)n, vals);
      break;
    }
    default: printf("fc 1..6/15/16 only\n\r"); return;
  }
  if (pn <= 0) { printf("bad params\n\r"); return; }
  if (is_tcp)
  {
    uint16_t port = 502;
    char *colon = strchr(tgt, ':');
    if (colon != NULL) { *colon = 0; port = (uint16_t)strtoul(colon + 1, 0, 10); }
    int r = app_mbtcp_query(tgt, port, (uint8_t)addr, pdu, (uint16_t)pn, rsp, sizeof(rsp), 3000);
    if (r <= 0) { printf("tcp query fail (%d)\n\r", r); return; }
    poll_print_rsp((uint8_t)fc, rsp, (uint16_t)r);
    return;
  }
  int i = port_by_name(tgt, (int)strlen(tgt));
  if (i < 0) { printf("unknown port '%s'\n\r", tgt); return; }
  if (s_cfg[i].role != 2U) { printf("%s not master (mbcfg %s master ...)\n\r", tgt, tgt); return; }
  static uint8_t req[300], rrsp[300];
  req[0] = 0x0AU; req[1] = (uint8_t)i; req[2] = (uint8_t)addr; req[3] = 1U;
  req[4] = 0xF4U; req[5] = 0x01U;   /* timeout 500ms LE */
  memcpy(&req[6], pdu, (size_t)pn);
  uint16_t r = app_rpc_transact(req, (uint16_t)(6 + pn), rrsp, sizeof(rrsp), 2200);  /* > CM4 worst case: bus-mutex wait <=1 s + wire <=1 s (contract L13) */
  if ((r >= 2U) && (rrsp[0] == 3U))   /* st=3: a reply frame arrived but was rejected (CRC/address), dump it verbatim as evidence */
  {
    printf("REJECTED frame (%u B):", (unsigned)(r - 1U));
    for (uint16_t k = 1; k < r; k++) { printf(" %02X", rrsp[k]); }
    printf("\n\r");
    return;
  }
  if ((r < 1U) || (rrsp[0] != 0U)) { printf("no reply (st=%u)\n\r", (r >= 1U) ? rrsp[0] : 255U); return; }
  poll_print_rsp((uint8_t)fc, &rrsp[1], (uint16_t)(r - 1U));
}

int app_mbcfg_cli(char *line)
{
  if (strncmp(line, "mbcfg", 5) == 0 && (line[5] == 0 || line[5] == ' '))
  {
    cli_mbcfg(line + 5);
    return 1;
  }
  if (strncmp(line, "mbpoll ", 7) == 0)
  {
    cli_mbpoll(line + 7);
    return 1;
  }
  return 0;
}

void app_mbcfg_cloud(const char *line)   /* tcpip thread: only copies string + raises flag, defaultTask executes */
{
  if (s_cloud_pend) { return; }
  strncpy(s_cloudq, line, sizeof(s_cloudq) - 1U);
  s_cloudq[sizeof(s_cloudq) - 1U] = 0;
  s_cloud_pend = 1;
}

void app_mbcfg_poll(void)   /* defaultTask every tick (500ms) */
{
  if (!s_applied && app_rpc_alive() && (app_lfs() != NULL))
  {
    if (!s_app_cfg) { cfg_load(); }   /* app configured ports in code -> keep them, don't load the persisted file over them */
    for (int i = 0; i < NPORT; i++)
    {
      if (s_cfg[i].role != 0U)
      {
        printf("[MBCFG] %s -> %s@%lu %s\n\r", s_names[i],
               (s_cfg[i].role == 1U) ? "slave" : "master", (unsigned long)s_cfg[i].baud,
               (port_apply(i) == 0) ? "ok" : "FAIL");
      }
    }
    if (s_tcp_on) { app_mbtcp_init(); }
    s_applied = 1;
  }
  if (s_cloud_pend)
  {
    static char q[96];
    strncpy(q, s_cloudq, sizeof(q) - 1U);
    q[sizeof(q) - 1U] = 0;
    s_cloud_pend = 0;
    (void)app_mbcfg_cli(q);   /* printf goes to the CDC console = visible to operators */
  }
  app_mbtcp_service();
}

/* ============================================================================
 *  Public application API — declared in app_platform.h
 *  Front-panel 485 port configuration + Modbus master transactions.
 *  Thread-safe: the underlying RPMsg link is mutex-guarded (app_rpc.c), so the
 *  read/write helpers may be called from your own FreeRTOS tasks. Buffers are on
 *  the stack (reentrant) — give an application task >= 4 KB of stack.
 *  app_mbport_configure() also writes littlefs; call it from app_user_init()
 *  (single-threaded setup), not concurrently from several running tasks.
 * ==========================================================================*/

int app_mbport_configure(uint8_t port, uint8_t role, uint32_t baud,
                         char parity, uint8_t stop, uint8_t slave_addr)
{
  if ((port >= NPORT) || (role > 2U)) { return -1; }
  uint8_t par = ((parity == 'E') || (parity == 'e')) ? 1U
              : ((parity == 'O') || (parity == 'o')) ? 2U : 0U;
  s_cfg[port].role   = role;
  s_cfg[port].baud   = baud;
  s_cfg[port].parity = par;
  s_cfg[port].stop   = (stop == 2U) ? 2U : 1U;
  s_cfg[port].addr   = slave_addr;
  s_app_cfg = 1;                  /* app owns port config: the boot-time file load is skipped (see app_mbcfg_poll) */
  if (!s_applied) { return 0; }   /* platform not up yet (called from app_user_init): the first poll applies it */
  cfg_save();                     /* runtime reconfigure: persist + push to CM4 now */
  return (role == 0U) ? port_apply_off((int)port) : port_apply((int)port);
}

int app_mbport_master(uint8_t port, uint8_t slave_addr,
                      const uint8_t *pdu, uint16_t pdu_len,
                      uint8_t *rsp, uint16_t rsp_cap)
{
  if ((pdu == NULL) || (pdu_len == 0U) || (pdu_len > 250U)) { return -1; }
  if (port == APP_PORT_BACKPLANE)              /* backplane expansion bus = generic Modbus RTU on CM4 UART8 (always master) */
  {
    uint8_t brr[MB_PDU_MAX];
    uint16_t br = app_mb_transact(slave_addr, pdu, pdu_len, brr, 1U);
    if (br == 0U) { return -3; }
    uint16_t bn = (br > rsp_cap) ? rsp_cap : br;
    if (bn > 0U) { memcpy(rsp, brr, bn); }
    return (int)bn;
  }
  if (port >= NPORT) { return -1; }
  if (s_cfg[port].role != 2U) { return -2; }   /* front port not configured as master */
  uint8_t req[262];
  uint8_t rr[262];
  req[0] = 0x0AU; req[1] = port; req[2] = slave_addr; req[3] = 1U;
  req[4] = 0xF4U; req[5] = 0x01U;   /* 500 ms bus timeout (LE) */
  memcpy(&req[6], pdu, pdu_len);
  uint16_t r = app_rpc_transact(req, (uint16_t)(6U + pdu_len), rr, sizeof(rr), 2200);      /* same budget as above */
  if ((r < 1U) || (rr[0] != 0U)) { return -3; }   /* st!=0: no reply / rejected frame */
  uint16_t n = (uint16_t)(r - 1U);
  if (n > rsp_cap) { n = rsp_cap; }
  if (n > 0U) { memcpy(rsp, &rr[1], n); }
  return (int)n;
}

/* helper: run one master request PDU, map exceptions to -(100+code) */
static int mb_master_do(uint8_t port, uint8_t addr, const uint8_t *pdu, int pn,
                        uint8_t *rsp, uint16_t cap)
{
  if (pn <= 0) { return -1; }
  int r = app_mbport_master(port, addr, pdu, (uint16_t)pn, rsp, cap);
  if (r <= 0) { return -2; }
  uint8_t exc = 0;
  if (mb_rsp_is_exception(rsp, (uint16_t)r, &exc)) { return -(100 + (int)exc); }
  return r;
}

int app_mb_read_holding(uint8_t port, uint8_t addr, uint16_t reg, uint16_t count, uint16_t *out)
{
  uint8_t pdu[8], rsp[MB_PDU_MAX];
  int r = mb_master_do(port, addr, pdu, mb_req_read(pdu, MB_FC_READ_HOLD, reg, count), rsp, sizeof(rsp));
  if (r < 0) { return r; }
  return (mb_rsp_regs(rsp, (uint16_t)r, MB_FC_READ_HOLD, out, count) == (int)count) ? 0 : -3;
}

int app_mb_read_input(uint8_t port, uint8_t addr, uint16_t reg, uint16_t count, uint16_t *out)
{
  uint8_t pdu[8], rsp[MB_PDU_MAX];
  int r = mb_master_do(port, addr, pdu, mb_req_read(pdu, MB_FC_READ_INPUT, reg, count), rsp, sizeof(rsp));
  if (r < 0) { return r; }
  return (mb_rsp_regs(rsp, (uint16_t)r, MB_FC_READ_INPUT, out, count) == (int)count) ? 0 : -3;
}

int app_mb_write_single(uint8_t port, uint8_t addr, uint16_t reg, uint16_t value)
{
  uint8_t pdu[8], rsp[MB_PDU_MAX];
  int r = mb_master_do(port, addr, pdu, mb_req_write_single(pdu, MB_FC_WRITE_REG, reg, value), rsp, sizeof(rsp));
  if (r < 0) { return r; }
  return mb_rsp_write_ok(rsp, (uint16_t)r, MB_FC_WRITE_REG, reg, value) ? 0 : -3;
}

int app_mb_write_multi(uint8_t port, uint8_t addr, uint16_t reg, uint16_t count, const uint16_t *vals)
{
  uint8_t pdu[MB_PDU_MAX], rsp[MB_PDU_MAX];
  int r = mb_master_do(port, addr, pdu, mb_req_write_regs(pdu, reg, count, vals), rsp, sizeof(rsp));
  if (r < 0) { return r; }
  return mb_rsp_write_ok(rsp, (uint16_t)r, MB_FC_WRITE_REGS, reg, count) ? 0 : -3;
}

/* ---- coil (digital output/input) helpers — relay modules live here. bits = packed little-endian ---- */
int app_mb_read_coils(uint8_t port, uint8_t addr, uint16_t coil, uint16_t count, uint8_t *out)   /* FC01 */
{
  uint8_t pdu[8], rsp[MB_PDU_MAX];
  int r = mb_master_do(port, addr, pdu, mb_req_read(pdu, MB_FC_READ_COILS, coil, count), rsp, sizeof(rsp));
  if (r < 0) { return r; }
  uint16_t nb = (uint16_t)((count + 7U) / 8U);
  return (mb_rsp_bits(rsp, (uint16_t)r, MB_FC_READ_COILS, out, nb) == (int)nb) ? 0 : -3;
}

int app_mb_write_coil(uint8_t port, uint8_t addr, uint16_t coil, uint8_t on)                     /* FC05 */
{
  uint8_t pdu[8], rsp[MB_PDU_MAX];
  uint16_t v = on ? 0xFF00U : 0x0000U;
  int r = mb_master_do(port, addr, pdu, mb_req_write_single(pdu, MB_FC_WRITE_COIL, coil, v), rsp, sizeof(rsp));
  if (r < 0) { return r; }
  return mb_rsp_write_ok(rsp, (uint16_t)r, MB_FC_WRITE_COIL, coil, v) ? 0 : -3;
}

int app_mb_write_coils(uint8_t port, uint8_t addr, uint16_t coil, uint16_t count, const uint8_t *bits) /* FC15 */
{
  uint8_t pdu[MB_PDU_MAX], rsp[MB_PDU_MAX];
  int r = mb_master_do(port, addr, pdu, mb_req_write_coils(pdu, coil, count, bits), rsp, sizeof(rsp));
  if (r < 0) { return r; }
  return mb_rsp_write_ok(rsp, (uint16_t)r, MB_FC_WRITE_COILS, coil, count) ? 0 : -3;
}
