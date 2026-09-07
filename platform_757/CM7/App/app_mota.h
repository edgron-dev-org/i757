/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_mota.h — expansion-module OTA (contract = Backplane_Bus_Protocol.md v0.23) */
#ifndef APP_MOTA_H
#define APP_MOTA_H
#include <stdint.h>

uint8_t mota_recv_active(void);   /* 1 = cloud fw chunks should be routed to mota (not to self OTA) */
int     mota_begin(uint8_t type, uint32_t size, uint32_t crc);   /* "mota-begin <type> <size> <crc>": store into repo as t<type>.fw */
void    mota_fw_chunk(const uint8_t *data, uint16_t len, uint8_t last);
int     mota_flash(uint8_t addr);                                /* "mota-flash <addr>": read type -> pick image -> stream */

#endif
