/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* ota.h — dual-Bank OTA (H757)
 * Layout: each bank = [CM7 @+0, 512K][CM4 @+0x80000, 512K]; the 0x08100000 window always maps the "non-running bank"
 * Thread model: ota_cmd/ota_fw_chunk are called on the tcpip thread (mqtt callback); ota_poll is called on defaultTask
 */
#ifndef OTA_H
#define OTA_H
#include <stdint.h>

/* Firmware ID tag: linked at a fixed offset FW_INFO_OFF in every CM7 image (.fw_info section in the .ld, after the vector table);
 * when OTA writes the standby bank footer it reads this from the pushed image to get the version and build date/time */
#define FW_INFO_OFF    0x400UL
#define FW_INFO_MAGIC  0x49373537UL   /* 'I757' */
typedef struct { uint32_t magic; char ver[40]; } fw_info_t;
extern const fw_info_t g_fw_info;

void        ota_cmd(const char *cmd, uint16_t len);
void        ota_fw_msg_start(uint32_t tot_len);   /* call once at the start of each chunked MQTT message */
void        ota_fw_chunk(const uint8_t *data, uint16_t len, uint8_t last);
void        ota_poll(void);
const char *ota_state_str(void);
uint32_t    ota_swap_active(void);   /* 0=bank1 running (not swapped), 1=bank2 running */
uint32_t    ota_recv(void);          /* bytes of this segment already written to flash */

/* ---- automatic rollback (trial period) ---- */
void        ota_boot_guard(void);    /* called in app_periph_init (before RTOS): trial-period boot counter, auto rollback on repeated crashes */
uint8_t     ota_in_trial(void);      /* 1 = this bank was just OTA'd in and not yet cloud-confirmed (trial period) */
uint32_t    ota_boot_attempts(void); /* number of trial-period boots so far (diagnostic) */
const char *ota_signer_str(void);    /* who signed the RUNNING image, from its bank footer: "edgron" / "customer" / "legacy" (pre-09-07 footer) / "-" (no footer: SWD image); heartbeat "fws" */
const char *ota_last_evt(void);      /* most recent OTA event (confirmed, rollback:xx, refused:xx); persisted in SRAM4, still reportable after a rollback reset */

#endif
