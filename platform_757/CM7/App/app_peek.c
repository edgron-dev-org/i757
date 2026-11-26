/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_peek.c — field diagnostics that do not depend on the cloud link (2026-11-27).
 *
 * Born from a field case (greenhouse, 2026-11-26): after a reboot the board ran its control loops
 * normally but never reached the broker. A second DHCP server on the site had handed it a lease
 * whose gateway led nowhere. The board was alive on the LAN (Modbus TCP answered) yet there was no
 * way to ask it for its own IP, gateway or MQTT state - the cloud command channel was the thing
 * that was down, and the console needs someone on site. Finding the cause took an hour of packet
 * captures; one look at the gateway address would have settled it.
 *
 * Three read-only views, contract = docs/Universal_Modbus_Port_Config.md §4.5 and
 * docs/Device_Cloud_Protocol.md (command `peek`):
 *   1. Network diagnostic block: Modbus TCP input registers 0x3000.. (always on, nothing secret):
 *      addresses the board is using, who gave them, link / DHCP / broker state, last connect error.
 *   2. `peek <hex addr> [len]`: cloud command and console - a hex dump of RAM (or the Cortex-M
 *      fault registers). The cloud channel is mutually authenticated, the console is physical.
 *   3. LAN memory window: Modbus TCP holding registers 0x3100/0x3101 = address, input registers
 *      0x3100.. = the bytes there. Modbus TCP has NO authentication, so this view is OFF until
 *      `peek lan on` is given over the cloud or the console (stored in peek.cfg), and it never
 *      shows the TLS memory pool (session keys).
 *
 * Only RAM can be read: the flash image is not served (it carries the device credentials on boards
 * without a secure element), and peripheral registers are not served either - several clear on
 * read and an unmapped address is a bus fault, i.e. a reset.
 *
 * Thread model: every entry point runs on defaultTask (cloud deferred commands, the console and
 * app_mbtcp_service all live there), so the file access needs no extra lock. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "lfs.h"
#include "lwip/netif.h"
#include "lwip/dhcp.h"
#include "lwip/dns.h"
#include "lwip/ip4_addr.h"
#include "app_cfg.h"
#include "app_netcfg.h"
#include "app_peek.h"

extern lfs_t *app_lfs(void);
extern void app_tls_pool_region(uintptr_t *base, size_t *len);
extern void app_mqtt_netdiag(app_netdiag_t *d);

#define PEEK_FILE      "peek.cfg"
#define PEEK_CMD_MAX   64U      /* bytes per `peek` command (the answer rides the heartbeat "cmdr" field) */
#define DIAG_BASE      0x3000U
#define DIAG_WORDS     32U
#define WIN_BASE       0x3100U
#define WIN_WORDS      120U     /* 240 bytes: one FC04 read of up to 120 registers covers the window */
#define MB_EX_FUNC     0x01U
#define MB_EX_ADDR     0x02U
#define MB_EX_VALUE    0x03U

static uint8_t  s_loaded = 0, s_lan = 0;
static uint32_t s_win_addr = 0;
static uint8_t  s_fbuf[256];                 /* LFS_NO_MALLOC: bring your own buffer */
static const struct lfs_file_config s_fcfg = { .buffer = s_fbuf };

/* readable RAM (STM32H757, CM7 view) + the Cortex-M system control block (CPUID..fault status/address) */
static const struct { uint32_t base, len; } k_ram[] = {
  { 0x20000000UL, 0x00020000UL },   /* DTCM 128K */
  { 0x24000000UL, 0x00080000UL },   /* AXI SRAM 512K */
  { 0x30000000UL, 0x00048000UL },   /* SRAM1..3 288K (shared with CM4, Ethernet buffers) */
  { 0x38000000UL, 0x00010000UL },   /* SRAM4 64K (process image, RPMsg) */
  { 0x38800000UL, 0x00001000UL },   /* backup SRAM 4K (PF_RETAIN, black box) */
  { 0xE000ED00UL, 0x00000040UL },   /* SCB: CFSR / HFSR / MMFAR / BFAR ... */
};

static void load(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  char buf[8];
  if (s_loaded) { return; }
  s_loaded = 1;
  if (fs == NULL) { return; }
  if (lfs_file_opencfg(fs, &f, PEEK_FILE, LFS_O_RDONLY, (struct lfs_file_config *)&s_fcfg) != 0) { return; }
  if (lfs_file_read(fs, &f, buf, 1) == 1) { s_lan = (uint8_t)(buf[0] == '1'); }
  (void)lfs_file_close(fs, &f);
}

static void save(void)
{
  lfs_t *fs = app_lfs();
  lfs_file_t f;
  if (fs == NULL) { return; }
  if (!s_lan) { (void)lfs_remove(fs, PEEK_FILE); return; }   /* default = no file */
  if (lfs_file_opencfg(fs, &f, PEEK_FILE, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                       (struct lfs_file_config *)&s_fcfg) != 0) { return; }
  (void)lfs_file_write(fs, &f, "1\n", 2);
  (void)lfs_file_close(fs, &f);
}

/* 1 = [addr, addr+len) lies inside one readable region; lan = also keep off the TLS pool */
int app_peek_ok(uint32_t addr, uint32_t len, int lan)
{
  uint8_t ok = 0;
  if ((len == 0U) || (len > 0x1000UL)) { return 0; }
  for (unsigned i = 0; i < (sizeof k_ram / sizeof k_ram[0]); i++)
  {
    if ((addr >= k_ram[i].base) && ((addr - k_ram[i].base) <= (k_ram[i].len - len))) { ok = 1; break; }
  }
  if (!ok) { return 0; }
  if (lan)
  {
    uintptr_t pb = 0; size_t pl = 0;
    app_tls_pool_region(&pb, &pl);
    if ((pl != 0U) && (addr < (uint32_t)pb + (uint32_t)pl) && ((uint32_t)pb < addr + len)) { return 0; }
  }
  return 1;
}

/* `peek` | `peek lan on|off` | `peek <hex addr> [len 1..64]`. 1 = handled. */
int app_peek_cmd(const char *line, char *out, uint16_t cap)
{
  const char *p;
  if ((strncmp(line, "peek", 4) != 0) || ((line[4] != 0) && (line[4] != ' '))) { return 0; }
  load();
  p = line + 4;
  while (*p == ' ') { p++; }
  if (*p == 0)
  {
    snprintf(out, cap, "peek: lan window %s; usage: peek <hex addr> [len 1..%u] | peek lan on|off", s_lan ? "ON" : "off", (unsigned)PEEK_CMD_MAX);
    return 1;
  }
  if (strncmp(p, "lan", 3) == 0)
  {
    p += 3;
    while (*p == ' ') { p++; }
    if (strcmp(p, "on") == 0)       { s_lan = 1; save(); }
    else if (strcmp(p, "off") == 0) { s_lan = 0; save(); }
    else if (*p != 0) { snprintf(out, cap, "usage: peek lan on|off"); return 1; }
    snprintf(out, cap, "peek lan %s (Modbus TCP memory window 0x3100; no authentication on that port)", s_lan ? "ON" : "off");
    return 1;
  }
  {
    char *end = NULL;
    uint32_t addr = (uint32_t)strtoul(p, &end, 16);
    uint32_t len = 16U;
    int n;
    if (end == p) { snprintf(out, cap, "usage: peek <hex addr> [len 1..%u]", (unsigned)PEEK_CMD_MAX); return 1; }
    while (*end == ' ') { end++; }
    if (*end != 0) { len = (uint32_t)strtoul(end, NULL, 0); }
    if ((len == 0U) || (len > PEEK_CMD_MAX)) { snprintf(out, cap, "peek: len 1..%u", (unsigned)PEEK_CMD_MAX); return 1; }
    if (!app_peek_ok(addr, len, 0)) { snprintf(out, cap, "peek %08lx: not readable (RAM and SCB only)", (unsigned long)addr); return 1; }
    n = snprintf(out, cap, "peek %08lx:", (unsigned long)addr);
    for (uint32_t i = 0; (i < len) && (n > 0) && ((size_t)n + 4U < cap); i++)
    {
      n += snprintf(out + n, (size_t)cap - (size_t)n, "%s%02x", ((i % 4U) == 0U) ? " " : "", (unsigned)*(const volatile uint8_t *)(addr + i));
    }
    return 1;
  }
}

int app_peek_cli(char *line)
{
  static char out[224];
  if (!app_peek_cmd(line, out, sizeof(out))) { return 0; }
  printf("%s\n\r", out);
  return 1;
}

/* ---- Modbus TCP side ---- */
static app_netdiag_t s_d;                    /* one snapshot per request, so a block read is consistent */
static uint32_t s_ip, s_gw, s_mask, s_dns, s_srv;
static void diag_snapshot(void)
{
  extern struct netif gnetif;
  app_netdiag_t d;
  uint32_t ip, gw, mask, dnsa, srv;
  {
    const struct dhcp *dh = netif_dhcp_data(&gnetif);
    const ip_addr_t *ds = dns_getserver(0);
    ip   = lwip_ntohl(ip4_addr_get_u32(netif_ip4_addr(&gnetif)));
    gw   = lwip_ntohl(ip4_addr_get_u32(netif_ip4_gw(&gnetif)));
    mask = lwip_ntohl(ip4_addr_get_u32(netif_ip4_netmask(&gnetif)));
    dnsa = (ds != NULL) ? lwip_ntohl(ip4_addr_get_u32(ip_2_ip4(ds))) : 0U;
    srv  = (dh != NULL) ? lwip_ntohl(ip4_addr_get_u32(ip_2_ip4(&dh->server_ip_addr))) : 0U;
    memset(&d, 0, sizeof d);
    app_mqtt_netdiag(&d);
    d.flags = (uint16_t)((d.flags & 0x0018U) |
              (netif_is_link_up(&gnetif) ? 1U : 0U) |
              (((dh != NULL) && dhcp_supplied_address(&gnetif)) ? 2U : 0U) |
              (d.fallback ? 4U : 0U) | (s_lan ? 0x20U : 0U) | (app_netcfg_static() ? 0x40U : 0U));
  }
  s_d = d; s_ip = ip; s_gw = gw; s_mask = mask; s_dns = dnsa; s_srv = srv;
}
static uint16_t diag_word(uint16_t i)        /* i = offset inside the diagnostic block */
{
  const app_netdiag_t d = s_d;
  const uint32_t ip = s_ip, gw = s_gw, mask = s_mask, dnsa = s_dns, srv = s_srv;
  switch (i)
  {
    case 0:  return 1U;                              /* DIAG_VER */
    case 1:  return d.flags;                         /* bit0 link, bit1 DHCP lease, bit2 static fallback, bit3 broker connected, bit4 broker reached this boot, bit5 LAN memory window on, bit6 static mode */
    case 2:  return (uint16_t)(ip >> 16);    case 3:  return (uint16_t)ip;
    case 4:  return (uint16_t)(mask >> 16);  case 5:  return (uint16_t)mask;
    case 6:  return (uint16_t)(gw >> 16);    case 7:  return (uint16_t)gw;
    case 8:  return (uint16_t)(dnsa >> 16);  case 9:  return (uint16_t)dnsa;
    case 10: return (uint16_t)(srv >> 16);   case 11: return (uint16_t)srv;       /* DHCP server that gave the lease */
    case 12: return (uint16_t)(d.broker_ip >> 16); case 13: return (uint16_t)d.broker_ip;
    case 14: return d.broker_port;
    case 15: return d.broker_id;                     /* which configured broker (app_cfg.h numbering) */
    case 16: return d.conn_state;                    /* lwIP MQTT client state: 0 idle, 1 TCP connecting, 2 MQTT connecting, 3 connected, 4 disconnecting */
    case 17: return (uint16_t)(int16_t)d.conn_err;   /* last immediate connect error (lwIP err_t, signed) */
    case 18: return d.conn_fails;
    case 19: return (uint16_t)(d.offline_s >> 16);   case 20: return (uint16_t)d.offline_s;   /* seconds without the broker */
    case 21: return d.mute_resets;                   /* self resets this offline episode */
    case 22: return (uint16_t)(d.pub_ok >> 16);      case 23: return (uint16_t)d.pub_ok;      /* publishes acknowledged since boot */
    case 24: return d.rx_age_s;                      /* seconds since the last Ethernet frame was received */
    default: return 0U;                              /* 25..31 reserved */
  }
}

static uint16_t ex(uint8_t fc, uint8_t code, uint8_t *rsp) { rsp[0] = (uint8_t)(fc | 0x80U); rsp[1] = code; return 2U; }

/* May a PC on the LAN change the network settings right now? Modbus TCP has no authentication, so:
 * only while the authenticated channel is out of reach (no broker for a minute - the case this
 * exists for), or when the owner has declared the LAN trusted (`peek lan on`), or on a build with
 * no cloud at all (then Modbus is the only remote door and the site already trusts it). */
#define NC_OFFLINE_S   60U
static int netcfg_wr_ok(void)
{
#if APP_ENABLE_CLOUD
  app_netdiag_t d;
  if (s_lan) { return 1; }
  memset(&d, 0, sizeof d);
  app_mqtt_netdiag(&d);
  return (((d.flags & 0x08U) == 0U) && (d.offline_s >= NC_OFFLINE_S)) ? 1 : 0;
#else
  return 1;
#endif
}

/* A Modbus TCP PDU aimed at 0x3000..0x32FF is answered here (CM7 owns this state; the rest of the
 * map lives on CM4). Returns the reply PDU length, 0 = not ours, pass it on. */
uint16_t app_peek_mbtcp(const uint8_t *pdu, uint16_t len, uint8_t *rsp, uint16_t cap)
{
  uint8_t  fc;
  uint16_t addr, n;
  if (len < 5U) { return 0; }
  fc = pdu[0];
  addr = (uint16_t)(((uint16_t)pdu[1] << 8) | pdu[2]);
  if ((addr < DIAG_BASE) || (addr > 0x32FFU)) { return 0; }
  if ((fc != 0x03U) && (fc != 0x04U) && (fc != 0x06U) && (fc != 0x10U)) { return 0; }
  if (addr >= 0x3200U)                                       /* network settings block (app_netcfg.c) */
  {
    load();
    if (fc == 0x04U) { return ex(fc, MB_EX_ADDR, rsp); }
    return app_netcfg_mbtcp(pdu, len, rsp, cap, netcfg_wr_ok());
  }
  if ((fc != 0x04U) && (addr < WIN_BASE)) { return 0; }      /* holding 0x3000..0x30FF = reserved port-config area on the CM4 map */
  load();
  n = (uint16_t)(((uint16_t)pdu[3] << 8) | pdu[4]);          /* quantity (FC03/04/16) or value (FC06) */

  if (fc == 0x04U)
  {
    if ((n == 0U) || (n > 125U) || ((uint16_t)(3U + 2U * n) > cap)) { return ex(fc, MB_EX_VALUE, rsp); }
    if ((addr >= DIAG_BASE) && ((uint32_t)addr + n <= DIAG_BASE + DIAG_WORDS))
    {
      diag_snapshot();
      rsp[0] = fc; rsp[1] = (uint8_t)(2U * n);
      for (uint16_t k = 0; k < n; k++)
      {
        uint16_t v = diag_word((uint16_t)(addr - DIAG_BASE + k));
        rsp[2U + 2U * k] = (uint8_t)(v >> 8); rsp[3U + 2U * k] = (uint8_t)v;
      }
      return (uint16_t)(2U + 2U * n);
    }
    if ((addr >= WIN_BASE) && ((uint32_t)addr + n <= WIN_BASE + WIN_WORDS))
    {
      uint32_t a = s_win_addr + 2UL * (uint32_t)(addr - WIN_BASE);
      if (!s_lan || !app_peek_ok(a, 2UL * n, 1)) { return ex(fc, MB_EX_ADDR, rsp); }
      rsp[0] = fc; rsp[1] = (uint8_t)(2U * n);
      for (uint16_t k = 0; k < 2U * n; k++) { rsp[2U + k] = *(const volatile uint8_t *)(a + k); }   /* memory order: byte 0 first */
      return (uint16_t)(2U + 2U * n);
    }
    return ex(fc, MB_EX_ADDR, rsp);
  }

  /* holding registers 0x3100 / 0x3101 = window address high / low word */
  if (!s_lan) { return ex(fc, MB_EX_ADDR, rsp); }
  if (fc == 0x03U)
  {
    if ((n == 0U) || (addr < WIN_BASE) || ((uint32_t)addr + n > WIN_BASE + 2U)) { return ex(fc, MB_EX_ADDR, rsp); }
    rsp[0] = fc; rsp[1] = (uint8_t)(2U * n);
    for (uint16_t k = 0; k < n; k++)
    {
      uint16_t v = ((addr - WIN_BASE + k) == 0U) ? (uint16_t)(s_win_addr >> 16) : (uint16_t)s_win_addr;
      rsp[2U + 2U * k] = (uint8_t)(v >> 8); rsp[3U + 2U * k] = (uint8_t)v;
    }
    return (uint16_t)(2U + 2U * n);
  }
  if (fc == 0x06U)
  {
    if ((addr != WIN_BASE) && (addr != WIN_BASE + 1U)) { return ex(fc, MB_EX_ADDR, rsp); }
    if (addr == WIN_BASE) { s_win_addr = (s_win_addr & 0x0000FFFFUL) | ((uint32_t)n << 16); }
    else                  { s_win_addr = (s_win_addr & 0xFFFF0000UL) | n; }
    memcpy(rsp, pdu, 5);
    return 5U;
  }
  /* FC16: both words in one write */
  if ((addr != WIN_BASE) || (n != 2U) || (len < 10U) || (pdu[5] != 4U)) { return ex(fc, MB_EX_ADDR, rsp); }
  s_win_addr = ((uint32_t)pdu[6] << 24) | ((uint32_t)pdu[7] << 16) | ((uint32_t)pdu[8] << 8) | pdu[9];
  memcpy(rsp, pdu, 5);
  return 5U;
}
