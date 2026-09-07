/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_mbtcp.h — Modbus TCP (CM7/LwIP): server on 502 (<=2 connections, PDU dispatched via op9 to CM4's map for the answer)
 * + client single transaction. Contract: docs/Universal_Modbus_Port_Config.md §5. */
#ifndef APP_MBTCP_H
#define APP_MBTCP_H
#include <stdint.h>

void app_mbtcp_init(void);       /* once: set up listener (handled internally via tcpip_callback) */
void app_mbtcp_service(void);    /* defaultTask every tick: process pending requests (op9) + send replies */

/* client single transaction (defaultTask context, blocks up to timeout_ms): returns reply PDU length, <=0=failure */
int app_mbtcp_query(const char *ip_str, uint16_t port, uint8_t unit,
                    const uint8_t *pdu, uint16_t pn,
                    uint8_t *rsp_pdu, uint16_t cap, uint32_t timeout_ms);

#endif
