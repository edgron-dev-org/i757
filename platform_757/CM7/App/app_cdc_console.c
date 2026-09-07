/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_cdc_console.c — USB CDC service port = CM7 local console
 * RX: CDC_Receive_FS in usbd_cdc_if (USB interrupt context) -> app_cdc_rx_feed only pushes into the ring ->
 *     app_cli_poll (defaultTask) consumes via app_cdc_getchar (printf-legal zone), thread model same as nucleo CLI.
 * TX: strong symbol _write overrides the weak syscalls implementation -> printf goes over CDC (not enumerated/busy = discard, never blocks the task). */
#include <stdint.h>
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

/* ---- RX ring (single producer = USB ISR, single consumer = defaultTask, lock-free) ----
 * 2048B: must absorb a whole `id-cert <hexDER>` line (~920B) between two app_cli_poll drains
 * (500ms apart) — the provisioning jig bursts a full certificate in one write, which is not
 * human typing (the original 256B ring silently dropped the tail, 2026-07-24 board-2 case). */
static volatile uint8_t  s_ring[2048];
static volatile uint16_t s_put = 0, s_get = 0;
#define CDC_RING_MASK 0x7FFU

void app_cdc_rx_feed(const uint8_t *buf, uint32_t len)
{
  uint32_t i;
  for (i = 0; i < len; i++)
  {
    uint16_t next = (uint16_t)((s_put + 1U) & CDC_RING_MASK);
    if (next == s_get) { break; }        /* drop if full */
    s_ring[s_put] = buf[i];
    s_put = next;
  }
}

int app_cdc_getchar(void)                /* -1=empty */
{
  uint8_t c;
  if (s_get == s_put) { return -1; }
  c = s_ring[s_get];
  s_get = (uint16_t)((s_get + 1U) & CDC_RING_MASK);
  return (int)c;
}

uint8_t app_cdc_ready(void)
{
  return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1U : 0U;
}

/* ---- printf redirection: strong _write overrides the weak syscalls implementation ----
 * when CDC_Transmit_FS is busy, spin briefly for one beat (64B packet @FS ~64µs), discard on timeout——losing console characters is acceptable, stalling the task is not */
int _write(int fd, char *ptr, int len)
{
  int done = 0;
  (void)fd;
  if (!app_cdc_ready()) { return len; }  /* no terminal attached: pretend the write completed, printf is zero-cost */
  while (done < len)
  {
    uint16_t chunk = (uint16_t)((len - done) > 64 ? 64 : (len - done));
    uint32_t spin = 40000U;              /* ~a few hundred µs @400MHz */
    while ((CDC_Transmit_FS((uint8_t *)&ptr[done], chunk) == USBD_BUSY) && (--spin != 0U)) { }
    if (spin == 0U) { break; }           /* terminal not receiving (suspended/disconnected): discard the rest */
    done += chunk;
  }
  return len;
}
