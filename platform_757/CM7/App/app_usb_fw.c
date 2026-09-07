/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_usb_fw.c — firmware ingest over the USB CDC console (no-network service path)
 *
 * Why: OTA normally travels cloud MQTT -> littlefs repo -> backplane (modules) or
 * dual-bank engine (self). Some sites have neither internet nor LAN; the USB-C service
 * port is then the only way in. This layer adds NO new update machinery — it feeds the
 * exact sinks the MQTT path feeds, so every downstream guarantee (CRC precheck, bank
 * swap, 3-strike guard, trial rollback) is identical.
 *
 * Protocol (PC side = tools/usb_fw_push.ps1; all replies ride the normal console printf):
 *   > mota-begin <type> <size> <crc32hex>      passthrough to ota_cmd (module -> repo)
 *   > ota-begin <offhex> <len> <crc32hex>      passthrough to ota_cmd (self, per segment)
 *   > fwchunk <n>                              device answers "go", then expects n raw bytes
 *   <n raw bytes>                              device answers "ok" (or "err ..." -> restart transfer)
 *   ... repeat begin/fwchunk as the flow requires, then mota-flash <addr> / ota-sign / ota-apply.
 * Routing per chunk = same rule as app_mqtt's dn/fw callback: mota_recv_active() decides.
 *
 * Thread model: everything here runs in defaultTask (app_cli_poll), which is also where
 * ota_poll consumes — no new cross-thread traffic. Do NOT run a cloud OTA and a USB OTA
 * concurrently; the engines are single-session (documented in OTA与MQTT使用手册). */
#include "app_usb_fw.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "app_ota.h"

extern int     app_cdc_getchar(void);            /* app_cdc_console.c */
extern uint8_t mota_recv_active(void);           /* app_mota.c */
extern void    mota_fw_chunk(const uint8_t *d, uint16_t l, uint8_t last);

#define USBFW_CHUNK_MAX    1536U   /* must leave headroom in the 2048B CDC ring */
#define USBFW_TIMEOUT_MS   3000U   /* stalled transfer -> abort back to text mode */

static uint8_t  s_active = 0;
static uint8_t  s_route_mota = 0;
static uint32_t s_expect = 0;
static uint32_t s_last_ms = 0;

int app_usbfw_cli(char *line)
{
  if (strncmp(line, "fwchunk ", 8) == 0)
  {
    uint32_t n = (uint32_t)strtoul(&line[8], 0, 10);
    if ((n == 0U) || (n > USBFW_CHUNK_MAX)) { printf("err size (1..%u)\n\r", (unsigned)USBFW_CHUNK_MAX); return 1; }
    s_route_mota = mota_recv_active();           /* same routing rule as the MQTT dn/fw callback */
    if (!s_route_mota) { ota_fw_msg_start(n); }  /* self path: one fwchunk == one dn/fw message */
    s_expect  = n;
    s_active  = 1;
    s_last_ms = HAL_GetTick();
    printf("go\n\r");                            /* PC waits for this before blasting bytes */
    return 1;
  }
  /* Text passthrough: the whole cloud command set works verbatim from the console
   * (ota-begin/ota-sign/ota-apply/ota-confirm/mota-begin/mota-flash/broker-*). */
  if ((strncmp(line, "ota-", 4) == 0) || (strncmp(line, "mota-", 5) == 0) ||
      (strncmp(line, "broker-", 7) == 0))
  {
    ota_cmd(line, (uint16_t)strlen(line));
    return 1;
  }
  return 0;
}

uint8_t app_usbfw_active(void) { return s_active; }

void app_usbfw_pump(void)
{
  uint8_t buf[128];
  uint16_t n = 0;
  int ch;
  while (s_active && ((ch = app_cdc_getchar()) >= 0))
  {
    buf[n++] = (uint8_t)ch;
    s_expect--;
    if ((s_expect == 0U) || (n == (uint16_t)sizeof(buf)))
    {
      uint8_t last = (s_expect == 0U) ? 1U : 0U;
      if (s_route_mota) { mota_fw_chunk(buf, n, last); }
      else              { ota_fw_chunk(buf, n, last); }
      n = 0;
      s_last_ms = HAL_GetTick();
      if (last)
      {
        s_active = 0;
        printf("ok\n\ri757> ");
        return;
      }
    }
  }
  if (n > 0U)                                    /* ring drained mid-chunk: flush partial fragment */
  {
    if (s_route_mota) { mota_fw_chunk(buf, n, 0); }
    else              { ota_fw_chunk(buf, n, 0); }
    s_last_ms = HAL_GetTick();
  }
  if (s_active && ((HAL_GetTick() - s_last_ms) > USBFW_TIMEOUT_MS))
  {
    s_active = 0;                                /* sink is now mid-message: restart the whole transfer */
    s_expect = 0;
    printf("err timeout\n\ri757> ");
  }
}
