/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_usb_fw.h — firmware ingest over the USB CDC console (no-network service path).
 * PC tool = tools/usb_fw_push.ps1. Feeds the SAME sinks as the cloud path (app_mqtt dn/fw):
 * `fwchunk <n>` is the console equivalent of one dn/fw MQTT message. */
#ifndef APP_USB_FW_H
#define APP_USB_FW_H
#include <stdint.h>

int     app_usbfw_cli(char *line);   /* CLI hook: fwchunk + ota-/mota-/broker- passthrough; 1=consumed */
uint8_t app_usbfw_active(void);      /* 1 = binary transfer in progress (CLI must not parse the ring) */
void    app_usbfw_pump(void);        /* called from app_cli_poll while active: drain ring -> chunk sink */

#endif
