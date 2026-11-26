/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_peek.h — field diagnostics without the cloud link: network diagnostic block and memory
 * window on Modbus TCP, `peek` command on the cloud channel and the console. See app_peek.c. */
#ifndef APP_PEEK_H
#define APP_PEEK_H
#include <stdint.h>

/* what app_mqtt.c knows about the broker link (filled by app_mqtt_netdiag) */
typedef struct {
  uint16_t flags;        /* bit3 broker connected, bit4 broker reached this boot (bits 0..2 and 5 are filled by app_peek.c) */
  uint8_t  fallback;     /* 1 = static fallback address in use (DHCP never answered) */
  uint16_t broker_id;
  uint32_t broker_ip;    /* host order, 0 = not resolved */
  uint16_t broker_port;
  uint16_t conn_state;
  int16_t  conn_err;
  uint16_t conn_fails;
  uint32_t offline_s;
  uint16_t mute_resets;
  uint32_t pub_ok;
  uint16_t rx_age_s;
} app_netdiag_t;

int      app_peek_ok(uint32_t addr, uint32_t len, int lan);
int      app_peek_cmd(const char *line, char *out, uint16_t cap);     /* 1 = handled */
int      app_peek_cli(char *line);
uint16_t app_peek_mbtcp(const uint8_t *pdu, uint16_t len, uint8_t *rsp, uint16_t cap);   /* reply PDU length, 0 = not ours */

#endif
