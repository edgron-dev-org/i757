/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_mbport.h — M7-side Modbus client (757 P2②): UART lives on CM4 (UART8 backplane), transactions over RPMsg
 * Contract: docs/Backplane_Bus_Protocol.md + docs/Inter_Core_RPMsg_Protocol.md (op table) + docs/Device_Cloud_Protocol.md (m2.* module points) */
#ifndef APP_MBPORT_H
#define APP_MBPORT_H
#include <stdint.h>

int  app_mb_wires_test(char *report, int cap);   /* 757: n/a (nucleo jumper tool retired) */
void app_mb_wire_scan(void);                     /* 757: n/a */
void app_mbx_run(void);         /* CLI 'mbx': backplane acceptance sequence against a bench relay module (addr 2) */
void app_mb_speed_sweep(void);  /* CLI 'mbspeed': baud-rate sweep (contract 0x0011+op7), wraps up back at 1M automatically */
void app_mb_bus_status(void);   /* CLI 'mbus': query M4 scheduler online table / counters (op=4) */
void app_mb_reg_cli(char rw, const char *args);  /* CLI 'mbr'/'mbw': generic slave holding-register access (config/cal 0x0200~, contract v0.28) */
void app_mb_time_push(void);    /* mqtt main loop every 10s: feed time to M4 -> broadcast onto bus (op=3) */

/* public Modbus transaction (handed to CM4 over RPMsg; used by app_mota streaming) */
uint16_t app_mb_transact(uint8_t addr, const uint8_t *pdu, uint16_t pn,
                         uint8_t *rsp_pdu, uint8_t expect_rsp);

#endif
