/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_netcfg.c — runtime static-fallback network config (2026-08-31).
 *
 * Born from a field case: a rebooted board missed its DHCP window, fell back to the
 * COMPILED-IN 192.168.137.2 (an artifact of the old PC-ICS bench network) on a
 * 192.168.1.x site, and sat unreachable — zero SYNs — until someone walked over.
 * DHCP stays the preferred path; this module only replaces the compile-time
 * CFG_STATIC_* literals with a littlefs-backed value the field can set over the
 * USB console or a cloud command, so one universal image fits every site.
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

static char s_ip[16], s_mask[16], s_gw[16], s_dns[16];
static uint8_t s_loaded = 0, s_from_file = 0;
static volatile uint8_t s_dirty = 0;

static uint8_t s_fbuf[256];                  /* LFS_NO_MALLOC: bring your own buffer */
static const struct lfs_file_config s_fcfg = { .buffer = s_fbuf };

static void set_builtin(void)
{
  strncpy(s_ip,   CFG_STATIC_IP,   sizeof(s_ip) - 1U);
  strncpy(s_mask, CFG_STATIC_MASK, sizeof(s_mask) - 1U);
  strncpy(s_gw,   CFG_STATIC_GW,   sizeof(s_gw) - 1U);
  strncpy(s_dns,  CFG_DNS_SERVER,  sizeof(s_dns) - 1U);
  s_from_file = 0;
}

static int valid_ip(const char *s)
{
  ip4_addr_t a;
  return ip4addr_aton(s, &a);
}

void app_netcfg_load(void)
{
  if (s_loaded) { return; }
  s_loaded = 1;
  set_builtin();
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[80];
  char ip[16], mk[16], gw[16], dns[16];
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, NETCFG_FILE, LFS_O_RDONLY,
                       (struct lfs_file_config *)&s_fcfg) != 0) { return; }
  lfs_ssize_t n = lfs_file_read(fs, &f, buf, sizeof(buf) - 1U);
  (void)lfs_file_close(fs, &f);
  if (n <= 0) { return; }
  buf[n] = 0;
  int nt = sscanf(buf, "%15s %15s %15s %15s", ip, mk, gw, dns);
  if ((nt < 3) || !valid_ip(ip) || !valid_ip(mk) || !valid_ip(gw)) { return; }  /* corrupt -> builtin */
  strcpy(s_ip, ip);
  strcpy(s_mask, mk);
  strcpy(s_gw, gw);
  strcpy(s_dns, ((nt >= 4) && valid_ip(dns)) ? dns : gw);   /* dns defaults to the gateway */
  s_from_file = 1;
  printf("[NET] fallback config from " NETCFG_FILE ": %s %s gw=%s dns=%s\n\r",
         s_ip, s_mask, s_gw, s_dns);
}

const char *app_netcfg_ip(void)   { return s_ip; }
const char *app_netcfg_mask(void) { return s_mask; }
const char *app_netcfg_gw(void)   { return s_gw; }
const char *app_netcfg_dns(void)  { return s_dns; }

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
  char buf[80];
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, NETCFG_FILE, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                       (struct lfs_file_config *)&s_fcfg) != 0) { return; }
  snprintf(buf, sizeof(buf), "%s %s %s %s\n", s_ip, s_mask, s_gw, s_dns);
  (void)lfs_file_write(fs, &f, buf, strlen(buf));
  (void)lfs_file_close(fs, &f);
}

int app_netcfg_cmd(const char *line, char *out, uint16_t cap)
{
  if (strncmp(line, "netcfg", 6) != 0) { return 0; }
  app_netcfg_load();
  const char *p = line + 6;
  while (*p == ' ') { p++; }
  if (*p == 0)                               /* show */
  {
    extern struct netif gnetif;
    snprintf(out, cap, "fallback %s %s gw=%s dns=%s (%s); current ip=%s",
             s_ip, s_mask, s_gw, s_dns, s_from_file ? NETCFG_FILE : "builtin",
             ip4addr_ntoa(netif_ip4_addr(&gnetif)));
    return 1;
  }
  if (strcmp(p, "default") == 0)             /* back to compile-time defaults */
  {
    lfs_t *fs = app_lfs();
    if (fs != NULL) { (void)lfs_remove(fs, NETCFG_FILE); }
    set_builtin();
    s_dirty = 1;
    snprintf(out, cap, "netcfg: builtin defaults restored (%s)", s_ip);
    return 1;
  }
  {                                          /* set: <ip> <mask> <gw> [dns] */
    char ip[16], mk[16], gw[16], dns[16];
    int nt = sscanf(p, "%15s %15s %15s %15s", ip, mk, gw, dns);
    if ((nt < 3) || !valid_ip(ip) || !valid_ip(mk) || !valid_ip(gw) ||
        ((nt >= 4) && !valid_ip(dns)))
    {
      snprintf(out, cap, "usage: netcfg [<ip> <mask> <gw> [dns] | default]");
      return 1;
    }
    strcpy(s_ip, ip);
    strcpy(s_mask, mk);
    strcpy(s_gw, gw);
    strcpy(s_dns, (nt >= 4) ? dns : gw);
    s_from_file = 1;
    save_file();
    s_dirty = 1;                             /* app_mqtt re-applies if currently in fallback */
    snprintf(out, cap, "netcfg saved: %s %s gw=%s dns=%s (used on DHCP timeout)",
             s_ip, s_mask, s_gw, s_dns);
    return 1;
  }
}

int app_netcfg_cli(char *line)
{
  char out[128];
  if (!app_netcfg_cmd(line, out, sizeof(out))) { return 0; }
  printf("%s\n\r", out);
  return 1;
}
