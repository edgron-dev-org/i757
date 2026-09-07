/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_cfg.h — Application-level configuration (broker / topics / static IP),
 * decoupled from CubeMX-generated code.
 *
 * ▶ EDIT THIS FILE to point at YOUR MQTT broker before building.
 *
 * Dual broker (public first, auto-fallback to LAN; see app_mqtt.c):
 *   public = your cloud broker over mTLS (device certificate = identity, see app_certs)
 *   LAN    = local mosquitto, plaintext (development only)
 * Trust anchor (CA) follows the deployment: to use your own cloud / AWS / Azure,
 *   swap ca_pem in app_certs (can be pushed via OTA).
 */
#ifndef APP_CFG_H
#define APP_CFG_H

/* ==== Feature switch: cloud connectivity ====
 * Set APP_ENABLE_CLOUD to 0 for a STANDALONE controller with no cloud at all:
 * no MQTT, no TLS connection, no OTA-over-MQTT, no SNTP time sync. Everything else
 * keeps working — Modbus master/slave on the 6 front ports, Modbus TCP, backplane
 * expansion modules, USB CLI, RTC time, power-fail retention, and your own app tasks
 * (app_user.c). In this mode firmware is flashed over SWD only, and the board needs
 * no broker, no certificates, and no keys/ folder to build (~54 KB smaller image).
 * (Can also be overridden from the build with -DAPP_ENABLE_CLOUD=0.) */
#ifndef APP_ENABLE_CLOUD
#define APP_ENABLE_CLOUD 1
#endif

/* -- Public broker: set to your cloud broker host/IP -- */
#define CFG_MQTT_PUB_HOST   "YOUR_BROKER_HOST"   /* e.g. "203.0.113.10" or "mqtt.example.com" */
#define CFG_MQTT_PUB_PORT   18884                /* mTLS port: device certificate = identity (app_certs) */
#define CFG_MQTT_PUB_USER   NULL                 /* mTLS: username/password retired (cert CN = username) */
#define CFG_MQTT_PUB_PASS   NULL

/* -- LAN broker: local mosquitto for development -- */
#define CFG_MQTT_LAN_HOST   "192.168.137.1"
#define CFG_MQTT_LAN_PORT   1883

/* -- (Optional) AWS IoT Core broker: domain access, needs DNS+SNI, trust anchor = Amazon Root CA 1 -- */
#define CFG_MQTT_AWS_HOST   "YOUR_AWS_IOT_ENDPOINT"  /* e.g. "xxxxxxxx-ats.iot.<region>.amazonaws.com" */
#define CFG_MQTT_AWS_PORT   8883
#define CFG_DNS_SERVER      "192.168.137.1"          /* DNS proxy; used to resolve the AWS domain */

/* -- Preferred (home) broker: selected at boot, and the side the auto-fallback
 * returns to after another broker fails repeatedly (see broker_fail, app_mqtt.c).
 * 0 = public broker (CFG_MQTT_PUB_*, mTLS)   2 = AWS IoT Core (CFG_MQTT_AWS_*)
 * Set to 2 when AWS is your only cloud (guide: docs/Connect_Your_Own_AWS_IoT.md);
 * 1 (LAN, plaintext) is deliberately not allowed as home — debug only. */
#define CFG_MQTT_PREFERRED  0

/* -- Customer identity slot in the ATECC608 secure element --
 * Slot 0 holds the FACTORY identity (key generated on-chip at production, slot-locked,
 * certificate signed by the vendor CA — used for anti-clone and for the vendor-hosted
 * broker). Slots 1..7 are provisioned open for the customer (P-256, GenKey enabled).
 * This slot hosts YOUR OWN identity for YOUR OWN cloud: generate the key with the
 * `id2-gen yes` console command, sign the public key with your own CA, store the
 * certificate with `id2-cert` — see docs/Connect_Your_Own_AWS_IoT.md. Valid: 1..7. */
#define CFG_SE_CUSTOMER_SLOT 2

/* -- Static IP fallback on DHCP timeout (match your network) -- */
#define CFG_STATIC_IP       "192.168.137.2"
#define CFG_STATIC_MASK     "255.255.255.0"
#define CFG_STATIC_GW       "192.168.137.1"

/* -- MQTT topics: per-device namespace (Device Cloud Protocol §3) --
 * Topics are built at runtime as dev/<type>/<sn>/... where <sn> = the CN of the
 * embedded device certificate (certificate = identity; CN doubles as the MQTT
 * client-id and the mosquitto username). One firmware image per board is already
 * the rule (the certificate is compiled in), so no extra per-board config here. */
#define CFG_DEV_TYPE        "I757-M"

#endif /* APP_CFG_H */
