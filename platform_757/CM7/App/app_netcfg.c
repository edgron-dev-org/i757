/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_netcfg.c — runtime network config: static-fallback addresses (2026-08-31) and static mode
 * plus a Modbus TCP register block to set them from the LAN (2026-11-27).
 *
 * Born from a field case: a rebooted board missed its DHCP window, fell back to the
 * compiled-in address of the day (192.168.137.2, an artifact of the old PC-ICS bench
 * network; the default is 192.168.1.250 since 2026-09-29) on a 192.168.1.x site, and
 * sat unreachable — zero SYNs — until someone walked over.
 * DHCP stays the default; this module replaces the compile-time CFG_STATIC_* literals with a
 * littlefs-backed value the field can set over the USB console or a cloud command, so one
 * universal image fits every site.
 *
 * Second field case (greenhouse, 2026-11-26): a second DHCP server on the site handed the board a
 * lease whose gateway led nowhere. The board was alive on the LAN and unreachable from the cloud,
 * so the cloud `netcfg` command could not be delivered - and a fallback address would not have
 * helped anyway, because DHCP had "succeeded". Hence:
 *   - static mode (`netcfg static ...`): DHCP is not used at all;
 *   - holding registers 0x3200.. on Modbus TCP: the same settings from a PC on the LAN
 *     (contract = docs/Universal_Modbus_Port_Config.md §4.6; who may write = app_peek.c).
 *
 * Pattern precedent: app_mbcfg.c — line-based file (= CLI syntax), static file
 * buffer (LFS_NO_MALLOC), first-use lazy load on defaultTask. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "lfs.h"
#include "app_cfg.h"
#include "app_netcfg.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"

extern lfs_t *app_lfs(void);

#define NETCFG_FILE "net.cfg"
#define NC_BASE     0x3200U     /* Modbus TCP holding registers */
#define NC_WORDS    10U
#define NC_COMMIT   9U
#define NC_KEY      0xA55AU
#define MB_EX_FUNC  0x01U
#define MB_EX_ADDR  0x02U
#define MB_EX_VALUE 0x03U

static char s_ip[16], s_mask[16], s_gw[16], s_dns[16];
static uint8_t s_loaded = 0, s_init = 0, s_from_file = 0;
static uint8_t s_static = 0;                 /* 1 = static mode: DHCP is not used */
static volatile uint8_t s_dirty = 0;
static uint8_t s_st_ok = 0;                 /* Modbus staging copy mirrors the stored config (0 = refill on next access) */

static uint8_t s_fbuf[256];                  /* LFS_NO_MALLOC: bring your own buffer */
static const struct lfs_file_config s_fcfg = { .buffer = s_fbuf };

static void set_builtin(void)
{
  strncpy(s_ip,   CFG_STATIC_IP,   sizeof(s_ip) - 1U);
  strncpy(s_mask, CFG_STATIC_MASK, sizeof(s_mask) - 1U);
  strncpy(s_gw,   CFG_STATIC_GW,   sizeof(s_gw) - 1U);
  strncpy(s_dns,  CFG_DNS_SERVER,  sizeof(s_dns) - 1U);
  s_from_file = 0;
  s_static = 0;
  s_st_ok = 0;
}

static int valid_ip(const char *s)
{
  ip4_addr_t a;
  return ip4addr_aton(s, &a);
}

static uint32_t to_u32(const char *s)        /* dotted quad -> host order, 0 if malformed */
{
  ip4_addr_t a;
  if (!ip4addr_aton(s, &a)) { return 0U; }
  return lwip_ntohl(ip4_addr_get_u32(&a));
}

static void to_str(uint32_t v, char *out)    /* host order -> dotted quad, out[16] */
{
  snprintf(out, 16, "%u.%u.%u.%u", (unsigned)(v >> 24), (unsigned)((v >> 16) & 255U),
           (unsigned)((v >> 8) & 255U), (unsigned)(v & 255U));
}

/* NULL = usable; else what is wrong with it. A setting that fails here would put the board out of
 * reach of both the LAN and the cloud, and the only way back is the USB console. */
static const char *check(uint32_t ip, uint32_t mask, uint32_t gw)
{
  uint32_t inv = ~mask;
  if ((mask == 0U) || (mask == 0xFFFFFFFFUL) || ((inv & (inv + 1U)) != 0U)) { return "mask is not a netmask"; }
  if (((ip & inv) == 0U) || ((ip & inv) == inv))  { return "ip is the network or broadcast address"; }
  if (((ip >> 24) == 0U) || ((ip >> 24) == 127U) || ((ip >> 24) >= 224U)) { return "ip is not a host address"; }
  if ((gw & mask) != (ip & mask))                 { return "gateway is outside the ip's subnet"; }
  if (((gw & inv) == 0U) || ((gw & inv) == inv) || (gw == ip)) { return "gateway is not a host address"; }
  return NULL;
}

void app_netcfg_load(void)
{
  if (!s_init) { s_init = 1; set_builtin(); }
  if (s_loaded) { return; }
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[96];
  char ip[16], mk[16], gw[16], dns[16], mode[8];
  if (fs == NULL) { return; }                /* not mounted yet: builtins for now, look again next call */
  s_loaded = 1;
  if (lfs_file_opencfg(fs, &f, NETCFG_FILE, LFS_O_RDONLY,
                       (struct lfs_file_config *)&s_fcfg) != 0) { return; }
  lfs_ssize_t n = lfs_file_read(fs, &f, buf, sizeof(buf) - 1U);
  (void)lfs_file_close(fs, &f);
  if (n <= 0) { return; }
  buf[n] = 0;
  mode[0] = 0;
  int nt = sscanf(buf, "%15s %15s %15s %15s %7s", ip, mk, gw, dns, mode);
  if ((nt < 3) || !valid_ip(ip) || !valid_ip(mk) || !valid_ip(gw)) { return; }  /* corrupt -> builtin */
  strcpy(s_ip, ip);
  strcpy(s_mask, mk);
  strcpy(s_gw, gw);
  strcpy(s_dns, ((nt >= 4) && valid_ip(dns)) ? dns : gw);   /* dns defaults to the gateway */
  s_static = (uint8_t)((nt >= 5) && (strcmp(mode, "static") == 0));   /* 5th field appended 2026-11-27; older firmware ignores it */
  s_from_file = 1;
  printf("[NET] %s config from " NETCFG_FILE ": %s %s gw=%s dns=%s\n\r",
         s_static ? "STATIC" : "fallback", s_ip, s_mask, s_gw, s_dns);
}

const char *app_netcfg_ip(void)   { return s_ip; }
const char *app_netcfg_mask(void) { return s_mask; }
const char *app_netcfg_gw(void)   { return s_gw; }
const char *app_netcfg_dns(void)  { return s_dns; }
uint8_t     app_netcfg_static(void) { return s_static; }
uint8_t     app_netcfg_ready(void)  { return (uint8_t)(app_lfs() != NULL); }

uint8_t app_netcfg_take_dirty(void)
{
  if (!s_dirty) { return 0; }
  s_dirty = 0;
  return 1;
}

static void save_file(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[96];
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, NETCFG_FILE, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                       (struct lfs_file_config *)&s_fcfg) != 0) { return; }
  snprintf(buf, sizeof(buf), "%s %s %s %s%s\n", s_ip, s_mask, s_gw, s_dns, s_static ? " static" : "");
  (void)lfs_file_write(fs, &f, buf, strlen(buf));
  (void)lfs_file_close(fs, &f);
}

/* One door for every way in (console, cloud, Modbus TCP). Addresses in host order, dns 0 = use the
 * gateway. NULL = stored and flagged for app_mqtt to apply; else the reason it was refused. */
static const char *set_cfg(uint8_t is_static, uint32_t ip, uint32_t mask, uint32_t gw, uint32_t dns)
{
  const char *why = check(ip, mask, gw);
  if (why != NULL) { return why; }
  to_str(ip, s_ip);
  to_str(mask, s_mask);
  to_str(gw, s_gw);
  to_str((dns != 0U) ? dns : gw, s_dns);
  s_static = (uint8_t)(is_static != 0U);
  s_from_file = 1;
  s_st_ok = 0;
  save_file();
  s_dirty = 1;                               /* app_mqtt applies it on its next tick */
  return NULL;
}

int app_netcfg_cmd(const char *line, char *out, uint16_t cap)
{
  if ((strncmp(line, "netcfg", 6) != 0) || ((line[6] != 0) && (line[6] != ' '))) { return 0; }
  app_netcfg_load();
  const char *p = line + 6;
  uint8_t mode = 0;
  while (*p == ' ') { p++; }
  if (*p == 0)                               /* show */
  {
    extern struct netif gnetif;
    snprintf(out, cap, "%s %s %s gw=%s dns=%s (%s); current ip=%s",
             s_static ? "STATIC" : "dhcp, fallback", s_ip, s_mask, s_gw, s_dns,
             s_from_file ? NETCFG_FILE : "builtin", ip4addr_ntoa(netif_ip4_addr(&gnetif)));
    return 1;
  }
  if (strcmp(p, "default") == 0)             /* back to compile-time defaults (DHCP) */
  {
    lfs_t *fs = app_lfs();
    if (fs != NULL) { (void)lfs_remove(fs, NETCFG_FILE); }
    set_builtin();
    s_dirty = 1;
    snprintf(out, cap, "netcfg saved: builtin defaults restored (dhcp, fallback %s)", s_ip);
    return 1;
  }
  if (strcmp(p, "dhcp") == 0)                /* leave static mode; the stored addresses stay as the fallback */
  {
    const char *why = set_cfg(0, to_u32(s_ip), to_u32(s_mask), to_u32(s_gw), to_u32(s_dns));
    if (why != NULL) { snprintf(out, cap, "netcfg: stored addresses unusable (%s); use netcfg default", why); return 1; }
    snprintf(out, cap, "netcfg saved: dhcp (fallback %s on DHCP timeout)", s_ip);
    return 1;
  }
  if ((strncmp(p, "static", 6) == 0) && ((p[6] == 0) || (p[6] == ' ')))
  {
    mode = 1;
    p += 6;
    while (*p == ' ') { p++; }
  }
  {                                          /* [static] <ip> <mask> <gw> [dns]; bare `static` = the stored addresses */
    char ip[16], mk[16], gw[16], dns[16];
    const char *why;
    int nt = 0;
    if (mode && (*p == 0)) { why = set_cfg(1, to_u32(s_ip), to_u32(s_mask), to_u32(s_gw), to_u32(s_dns)); }
    else
    {
      nt = sscanf(p, "%15s %15s %15s %15s", ip, mk, gw, dns);
      if ((nt < 3) || !valid_ip(ip) || !valid_ip(mk) || !valid_ip(gw) ||
          ((nt >= 4) && !valid_ip(dns)))
      {
        snprintf(out, cap, "usage: netcfg [[static] <ip> <mask> <gw> [dns] | static | dhcp | default]");
        return 1;
      }
      why = set_cfg(mode, to_u32(ip), to_u32(mk), to_u32(gw), (nt >= 4) ? to_u32(dns) : 0U);
    }
    if (why != NULL) { snprintf(out, cap, "netcfg refused: %s", why); return 1; }
    snprintf(out, cap, "netcfg saved: %s %s %s gw=%s dns=%s (%s)", mode ? "STATIC" : "dhcp, fallback",
             s_ip, s_mask, s_gw, s_dns, mode ? "DHCP not used" : "used on DHCP timeout");
    return 1;
  }
}

int app_netcfg_cli(char *line)
{
  char out[160];
  if (!app_netcfg_cmd(line, out, sizeof(out))) { return 0; }
  printf("%s\n\r", out);
  return 1;
}

/* ---- Modbus TCP holding registers 0x3200..0x3209 ----
 *   +0 MODE 0 = DHCP (the addresses are the fallback on DHCP timeout), 1 = static
 *   +1/+2 IP   +3/+4 MASK   +5/+6 GATEWAY   +7/+8 DNS (0 = the gateway)      high word first
 *   +9 COMMIT: write 0xA55A = check, store and apply +0..+8; reads 1 while they differ from what is stored
 * Writes land in a staging copy, so a half-written address is never used. */
static uint16_t s_st[NC_WORDS - 1U];

static void stage_from_stored(void)
{
  const uint32_t v[4] = { to_u32(s_ip), to_u32(s_mask), to_u32(s_gw), to_u32(s_dns) };
  s_st[0] = s_static;
  for (unsigned i = 0; i < 4U; i++) { s_st[1U + 2U * i] = (uint16_t)(v[i] >> 16); s_st[2U + 2U * i] = (uint16_t)v[i]; }
  s_st_ok = 1;
}

static uint8_t stage_pending(void)
{
  uint16_t now[NC_WORDS - 1U];
  memcpy(now, s_st, sizeof now);
  stage_from_stored();
  if (memcmp(now, s_st, sizeof now) == 0) { return 0; }
  memcpy(s_st, now, sizeof now);
  return 1;
}

static uint16_t ex(uint8_t fc, uint8_t code, uint8_t *rsp) { rsp[0] = (uint8_t)(fc | 0x80U); rsp[1] = code; return 2U; }

static void lan_committed(void)
{
  extern void app_log_event_src(const char *type, const char *src, const char *fmt, ...);
  printf("[NET] netcfg committed over Modbus TCP\n\r");
  app_log_event_src("CONFIG", "lan", "netcfg %s %s %s gw=%s dns=%s (Modbus TCP)", s_static ? "static" : "dhcp", s_ip, s_mask, s_gw, s_dns);
}

static const char *commit(void)
{
  if (s_st[0] > 1U) { return "mode"; }
  return set_cfg((uint8_t)s_st[0], ((uint32_t)s_st[1] << 16) | s_st[2], ((uint32_t)s_st[3] << 16) | s_st[4],
                 ((uint32_t)s_st[5] << 16) | s_st[6], ((uint32_t)s_st[7] << 16) | s_st[8]);
}

/* pdu = FC03/06/16 aimed at 0x3200..0x32FF; wr_ok = the caller's verdict on whether this LAN may
 * change the settings right now. Returns the reply PDU length. */
uint16_t app_netcfg_mbtcp(const uint8_t *pdu, uint16_t len, uint8_t *rsp, uint16_t cap, int wr_ok)
{
  uint8_t  fc = pdu[0];
  uint16_t addr = (uint16_t)(((uint16_t)pdu[1] << 8) | pdu[2]);
  uint16_t n = (uint16_t)(((uint16_t)pdu[3] << 8) | pdu[4]);    /* quantity (FC03/16) or value (FC06) */
  uint16_t off = (uint16_t)(addr - NC_BASE);
  app_netcfg_load();
  if (!s_st_ok) { stage_from_stored(); }

  if (fc == 0x03U)
  {
    if ((n == 0U) || ((uint32_t)off + n > NC_WORDS) || ((uint16_t)(2U + 2U * n) > cap)) { return ex(fc, MB_EX_ADDR, rsp); }
    rsp[0] = fc; rsp[1] = (uint8_t)(2U * n);
    for (uint16_t k = 0; k < n; k++)
    {
      uint16_t v = ((off + k) == NC_COMMIT) ? stage_pending() : s_st[off + k];
      rsp[2U + 2U * k] = (uint8_t)(v >> 8); rsp[3U + 2U * k] = (uint8_t)v;
    }
    return (uint16_t)(2U + 2U * n);
  }
  if ((fc != 0x06U) && (fc != 0x10U)) { return ex(fc, MB_EX_FUNC, rsp); }
  if (!wr_ok) { return ex(fc, MB_EX_FUNC, rsp); }               /* not from this LAN, not now: use the cloud or the console */
  if (fc == 0x06U)
  {
    if (off >= NC_WORDS) { return ex(fc, MB_EX_ADDR, rsp); }
    if (off == NC_COMMIT)
    {
      const char *why = (n == NC_KEY) ? commit() : "key";
      if (why != NULL) { return ex(fc, MB_EX_VALUE, rsp); }
      lan_committed();
    }
    else { s_st[off] = n; }
    memcpy(rsp, pdu, 5);
    return 5U;
  }
  /* FC16: any run inside the block; a run that ends on COMMIT stages and applies in one request */
  if ((n == 0U) || ((uint32_t)off + n > NC_WORDS) || (len < (uint16_t)(6U + 2U * n)) || (pdu[5] != (uint8_t)(2U * n)))
  {
    return ex(fc, MB_EX_ADDR, rsp);
  }
  {
    uint8_t do_commit = 0;
    for (uint16_t k = 0; k < n; k++)
    {
      uint16_t v = (uint16_t)(((uint16_t)pdu[6U + 2U * k] << 8) | pdu[7U + 2U * k]);
      if ((off + k) == NC_COMMIT) { if (v != NC_KEY) { return ex(fc, MB_EX_VALUE, rsp); } do_commit = 1; }
    }
    for (uint16_t k = 0; k < n; k++)
    {
      if ((off + k) != NC_COMMIT) { s_st[off + k] = (uint16_t)(((uint16_t)pdu[6U + 2U * k] << 8) | pdu[7U + 2U * k]); }
    }
    if (do_commit)
    {
      if (commit() != NULL) { return ex(fc, MB_EX_VALUE, rsp); }
      lan_committed();
    }
  }
  memcpy(rsp, pdu, 5);
  return 5U;
}
