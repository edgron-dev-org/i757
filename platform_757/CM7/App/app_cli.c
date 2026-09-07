/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_cli.c — diagnostic CLI implementation (user file)
 * Thread model: usbd_cdc_if (USB interrupt) -> app_cdc_rx_feed pushes bytes into a ring;
 *           app_cli_poll (defaultTask) drains the ring, echoes, and executes whole lines
 *           (the legal printf zone). Input source = USB-C CDC (the COM port on the PC).
 * The CLI consumes no UART, so all six front-panel RS-485 ports stay free for the application. */
#include "app_cli.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/netif.h"
#include "app_time.h"
#include "app_ota.h"
#include "app_init.h"
#include "app_mqtt.h"
#include "modbus_core.h"
#include "app_can.h"
#include "app_mbport.h"
#include "app_rpc.h"
#include "app_hsdi.h"
#include "app_cfg.h"
#define APP_STR2(x) #x
#define APP_STR(x)  APP_STR2(x)
/* 757: input source = USB CDC ring (app_cdc_console.c), the uart_drv console path is retired */
extern int app_cdc_getchar(void);

/* 757: USART3 = front-panel port 485C, owned by CM4 as a customer-configurable Modbus port — not touched here. */

/* ---- command implementations (all in defaultTask context) ---- */
static void cmd_help(void)
{
  printf("commands:\n\r"
         "  stat        network/ota/heap/time one-screen\n\r"
         "  time        unix + UTC calendar + source\n\r"
         "  ota         bank/state/event/trial\n\r"
         "  mb          re-run modbus core selftest\n\r"
         "  can         re-run fdcan internal loopback\n\r"
         "  wires       jumper continuity check (PD5-PC7, PC6-PD6)\n\r"
         "  mbx         modbus master-slave acceptance over real wires\n\r"
         "  mbus        M4 scheduler status; ustat = uart port stats\n\r"
         "  mbr/mbw     backplane slave holding regs: mbr <addr> <reg> [n] | mbw <addr> <reg> <val>\n\r"
         "  mbcfg       show/set modbus ports (mbcfg 485a slave 9600 8N1 addr=1 | 485a off | tcp on)\n\r"
         "  mbpoll      one-shot master txn (mbpoll 485b 1 4 0 4 | mbpoll tcp <ip> 1 3 0 4)\n\r"
         "  netcfg      DHCP-timeout fallback IP: netcfg [<ip> <mask> <gw> [dns] | default]\n\r"
         "  dose        dosing pumps: dose a|b|acid|ab <ml> | stop|auto|off | cal|ec|ph|mix|phsrc|phdiv\n\r"
         "  tasks       both cores: task list, stack headroom, heap (alias: ps)\n\r"
         "  pio         process-image scanner: pio setup [port addr] | dump | do <pt> <ch> <0|1> | hr <pt> <ch> <v>\n\r"
         "  pfstat      power-fail layers status (battery/retain/blackbox slot)\n\r"
         "  pftest      power-fail dry-run; pfreport = last blackbox record\n\r"
         "  sec         crypto chips test (I2C scan + 608A wake + SE050 ENA)\n\r"
         "  id          factory identity: id | id-pub | id-cert <hexDER> | id-cert-show | id-clear yes\n\r"
         "  id2         customer identity (608A slot " APP_STR(CFG_SE_CUSTOMER_SLOT) ", your own CA/cloud): id2 | id2-gen yes |\n\r"
         "              id2-pub | id2-cert <hexDER> | id2-cert-show | id2-clear yes (guide: Connect_Your_Own_AWS_IoT)\n\r"
         "  fwkey2      customer OTA signing key (2nd verify key, USB only): fwkey2 | fwkey2 set <hex> | fwkey2 clear yes\n\r"
         "  beep 1|0    buzzer on/off\n\r"
         "  reboot      NVIC system reset\n\r"
         "  revert yes  swap firmware bank (same gate as cloud)\n\r"
         "  fwchunk <n> USB firmware ingest (tools/usb_fw_push.ps1; no-network OTA)\n\r"
         "  ota-*/mota-* full cloud command set works here verbatim (incl. 'ota-confirm yes')\n\r");
}

static void print_time(void)
{
  uint32_t now = app_time_now();
  if (now == 0U)
  {
    printf("time: not synced yet\n\r");
    return;
  }
  time_t t = (time_t)now;
  struct tm g;
  gmtime_r(&t, &g);
  printf("time: %lu = %04d-%02d-%02d %02d:%02d:%02d UTC (src=%s)\n\r",
         (unsigned long)now, g.tm_year + 1900, g.tm_mon + 1, g.tm_mday,
         g.tm_hour, g.tm_min, g.tm_sec,
         (app_time_source() == 2U) ? "sntp" : "rtc");
}

static void cmd_stat(void)
{
  extern struct netif gnetif;
  printf("uptime %lus  heap %u  temp %dC\n\r",
         (unsigned long)(HAL_GetTick() / 1000U),
         (unsigned int)xPortGetFreeHeapSize(), app_temp_read());
  { extern uint16_t mqtt_rebuilds(void); extern const char *mqtt_last_autopsy(void);
    if (mqtt_rebuilds() > 0U)
    { printf("mqtt: autopsies=%u last-autopsy: %s\n\r", (unsigned)mqtt_rebuilds(), mqtt_last_autopsy()); } }
  printf("net: ip=%s link=%s cloud=%s\n\r",
         ip4addr_ntoa(netif_ip4_addr(&gnetif)),
         netif_is_link_up(&gnetif) ? "up" : "DOWN",
         mqtt_cloud_ok() ? "ok" : "---");
  printf("ota: bank=%lu state=%s evt=\"%s\"%s\n\r",
         (unsigned long)(ota_swap_active() + 1U), ota_state_str(), ota_last_evt(),
         ota_in_trial() ? " [TRIAL]" : "");
  print_time();
}

static void cmd_ota(void)
{
  printf("bank=%lu state=%s evt=\"%s\" trial=%d attempts=%lu\n\r",
         (unsigned long)(ota_swap_active() + 1U), ota_state_str(), ota_last_evt(),
         (int)ota_in_trial(), (unsigned long)ota_boot_attempts());
}

static void cmd_mb(void)
{
  int t = 0;
  int f = mb_selftest(&t);
  printf("modbus selftest %s: %d/%d\n\r", (f == 0) ? "ok" : "FAIL", t - f, t);
}

static void cmd_can(void)
{
  int t = 0;
  int f = app_can_selftest(&t);
  printf("fdcan loopback %s: %d/%d\n\r", (f == 0) ? "ok" : "FAIL", t - f, t);
}

static void print_ustat(const char *name, const uint32_t *v)
{
  printf("  %-8s rx=%lu tx=%lu frm=%lu ovr=%lu err=%lu\n\r", name,
         (unsigned long)v[0], (unsigned long)v[1], (unsigned long)v[2],
         (unsigned long)v[3], (unsigned long)v[4]);
}

static void cmd_ustat(void)   /* three-port UART stats: local console + CM4's two ports (RPMsg op=5) */
{
  uint32_t v[5] = {0, 0, 0, 0, 0};   /* 757: console = CDC, no uart_drv stats */
  printf("uart stats:\n\r");
  print_ustat("console(cdc)", v);
  {
    uint8_t req[1] = { 0x05U };
    uint8_t rsp[48];
    uint16_t r = app_rpc_transact(req, 1, rsp, sizeof(rsp), 150);
    if (r >= 40U)
    {
      const char *names[2] = { "mb-mstr", "mb-slave" };
      for (int k = 0; k < 2; k++)
      {
        for (int i = 0; i < 5; i++)
        {
          const uint8_t *b = &rsp[(k * 5 + i) * 4];
          v[i] = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
        }
        print_ustat(names[k], v);
      }
    }
    else { printf("  (CM4 ports: no reply)\n\r"); }
  }
}

static void cli_exec(char *line)
{
  if (line[0] == 0)               { return; }
  if (strcmp(line, "help") == 0)  { cmd_help(); return; }
  if (strcmp(line, "stat") == 0)  { cmd_stat(); return; }
  if (strcmp(line, "time") == 0)  { print_time(); return; }
  if (strcmp(line, "ota") == 0)   { cmd_ota(); return; }
  if (strcmp(line, "mb") == 0)    { cmd_mb(); return; }
  if (strcmp(line, "can") == 0)   { cmd_can(); return; }
  if (strcmp(line, "wires") == 0)
  {
    char rep[96];
    app_mb_wires_test(rep, sizeof(rep));
    printf("%s\n\r", rep);
    return;
  }
  if (strncmp(line, "beep ", 5) == 0) { extern void app_beep_set(uint8_t on); app_beep_set((uint8_t)(line[5] == '1')); printf("beep %s\n\r", (line[5] == '1') ? "on" : "off"); return; }
  { extern int app_usbfw_cli(char *l); if (app_usbfw_cli(line)) { return; } }   /* fwchunk + ota-/mota-/broker- passthrough (USB no-network OTA, app_usb_fw.c) */
  { extern int app_mbcfg_cli(char *l); if (app_mbcfg_cli(line)) { return; } }   /* mbcfg/mbpoll (Universal_Modbus_Port_Config.md) */
  { extern int app_netcfg_cli(char *l); if (app_netcfg_cli(line)) { return; } }   /* netcfg: static-fallback IP (littlefs net.cfg) */
  { extern int app_diag_cli(char *l); if (app_diag_cli(line)) { return; } }     /* tasks / ps: both cores' stacks + heap */
  { extern int app_pio_cli(char *l); if (app_pio_cli(line)) { return; } }       /* pio: process-image scanner bench (Process_Image_and_IO_Mapping.md) */
  { extern int app_pf_cli(char *l); if (app_pf_cli(line)) { return; } }         /* pfstat/pftest/pfreport (power-fail three-layer API) */
  { extern int app_datalog_cli(char *l); if (app_datalog_cli(line)) { return; } } /* note/logcfg/logstat/logget (Data_Logging_and_Event_Journal.md) */
  { extern int app_vent_cli(char *l); if (app_vent_cli(line)) { return; } }       /* vent: greenhouse roof-vent control (app_vent.c) */
  { extern int app_dose_cli(char *l); if (app_dose_cli(line)) { return; } }       /* dose: nutrient/pH pumps (app_dose.c) */
  { extern int app_flowmon_cli(char *l); if (app_flowmon_cli(line)) { return; } } /* flow: gutter feed-flow monitor / alarm (app_flowmon.c) */
  { extern int app_pulse_cli(char *l); if (app_pulse_cli(line)) { return; } }     /* pulse/hsdi: bench reset-reproduction (app_user.c) */
  { extern int app_sec_cli(char *l); if (app_sec_cli(line)) { return; } }       /* sec (first test of the two security chips) */
  { extern int app_identity_cli(char *l); if (app_identity_cli(line)) { return; } } /* id/id-pub/id-cert (identity partition, production jig) */
  if (strcmp(line, "mbx") == 0)   { app_mbx_run(); return; }
  if (strcmp(line, "mbspeed") == 0) { extern void app_mb_speed_sweep(void); app_mb_speed_sweep(); return; }
  if (strcmp(line, "wscan") == 0) { app_mb_wire_scan(); return; }
  if (strcmp(line, "rpc") == 0)   { app_rpc_cli(); return; }
  if (strcmp(line, "mbus") == 0)  { app_mb_bus_status(); return; }
  if (strncmp(line, "mbr ", 4) == 0) { app_mb_reg_cli('r', line + 4); return; }
  if (strncmp(line, "mbw ", 4) == 0) { app_mb_reg_cli('w', line + 4); return; }
  if (strcmp(line, "hsdi") == 0)  { app_hsdi_cli(); return; }
  if (strcmp(line, "ustat") == 0) { cmd_ustat(); return; }
  if (strcmp(line, "reboot") == 0)
  {
    printf("rebooting...\n\r");
    HAL_Delay(50);                /* let the serial port finish talking */
    { extern void app_reset(const char *reason); app_reset("cli reboot"); }
  }
  if (strcmp(line, "revert yes") == 0)
  {
    /* same path / same gate as the cloud (the footnote check is in ota_poll's APPLY_PEND); only sets a state variable, safe in this context */
    printf("revert requested (gate applies)...\n\r");
    ota_cmd("ota-revert", 10);
    return;
  }
  if (strncmp(line, "revert", 6) == 0)
  {
    printf("dangerous: type exactly 'revert yes'\n\r");
    return;
  }
  printf("unknown '%s' (try: help)\n\r", line);
}

/* ---- public API ---- */
void app_cli_init(void)
{
  /* 757: RX is fed into the ring by usbd_cdc_if -> app_cdc_rx_feed, no UART init */
  printf("[CLI] ready (USB CDC), type 'help'\n\ri757> ");
}

void app_cli_poll(void)
{
  static char s_line[1400];   /* sized for `id-cert <hex-DER>` (device cert ~600B DER = ~1200 hex chars) */
  static uint32_t s_fill = 0;
  char c;
  int ch;
  { /* binary firmware transfer in progress (app_usb_fw.c): the ring carries raw image
     * bytes, not console input — pump them to the OTA sinks and skip line parsing */
    extern uint8_t app_usbfw_active(void); extern void app_usbfw_pump(void);
    if (app_usbfw_active()) { app_usbfw_pump(); if (app_usbfw_active()) { return; } }
  }
  while ((ch = app_cdc_getchar()) >= 0)
  {
    c = (char)ch;
    if ((c == '\r') || (c == '\n'))
    {
      printf("\n\r");
      s_line[s_fill] = 0;
      cli_exec(s_line);
      s_fill = 0;
      printf("i757> ");
    }
    else if ((c == 0x08) || (c == 0x7F))       /* backspace */
    {
      if (s_fill > 0U) { s_fill--; printf("\b \b"); }
    }
    else if ((c >= 0x20) && (c < 0x7F) && (s_fill < (sizeof(s_line) - 1U)))
    {
      s_line[s_fill++] = c;
      printf("%c", c);                          /* echo */
    }
  }
}
