/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * app_ethdiag.h - Ethernet receive- and transmit-path diagnostics (observation only). See app_ethdiag.c. */
#ifndef APP_ETHDIAG_H
#define APP_ETHDIAG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Install the netif->input wrapper. Call once after MX_LWIP_Init (any thread). */
void app_ethdiag_init(void);

/* Link-thread hook: call every iteration of ethernet_link_thread (100 ms) with the PHY address
 * and the LAN8742_GetLinkState() result. Counts link flaps; every 10 s samples PHY registers and
 * the clear-on-read MTL/DMA drop counters. Must run on the link thread: it is the only MDIO user. */
void app_ethdiag_link_poll(uint32_t phy_addr, int32_t link_state);

/* 'stat' CLI: five lines (eth / tx / mac / phy / lwip). defaultTask context (printf). */
void app_ethdiag_print(void);

/* Compact one-line summary for the cloud status "eth" field (static buffer, defaultTask only). */
const char *app_ethdiag_hb(void);

/* Heartbeat / autopsy accessors. */
uint32_t app_ethdiag_rx_frames(void);   /* frames handed to lwIP since boot */
uint32_t app_ethdiag_rx_age_s(void);    /* seconds since the last frame reached lwIP (uptime if none yet) */
uint32_t app_ethdiag_in_err(void);      /* tcpip_input refusals (mailbox full) */

/* Transmit side (2026-10-10, publish-stall case): frames lwIP handed to the driver vs frames the MAC
 * put on the wire. app_ethdiag_init() also wraps netif->linkoutput. */
uint32_t app_ethdiag_tx_frames(void);   /* frames handed to linkoutput since boot (attempts, errors included) */
uint32_t app_ethdiag_tx_err(void);      /* linkoutput error returns (ERR_IF: HAL refused / pbuf chain too long) */
uint32_t app_ethdiag_tx_age_s(void);    /* seconds since the last frame was handed to the driver */
/* Stall/kick autopsy fragment (any thread, no MDIO): "tx=<lwip>/<mac-good> own=<dma-owned desc>/<in-use>
 * txe=<errors> uf=<fifo underflow> tps=<DMADSR TPS0>". Returns chars written. */
int app_ethdiag_tx_snap(char *o, uint32_t cap);

#ifdef __cplusplus
}
#endif
#endif /* APP_ETHDIAG_H */
