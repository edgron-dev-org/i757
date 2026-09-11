/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* mqtt.c — network + MQTT + OTA application main loop (user file, untouched by CubeMX regeneration)
 * split from main.c.
 *
 * Thread model:
 *  - all *_cb callbacks in this file run on the tcpip thread — no printf (newlib stdio is not thread-safe, concurrent use = deadlock);
 *  - mqtt_app_task runs on defaultTask, printf belongs to it alone;
 *  - LwIP core APIs (dhcp_stop/netif_set_addr/mqtt_* etc.) must never be called directly across threads, always dispatch via tcpip_callback.
 *
 * broker strategy: public first, auto switch sides after 2 consecutive failures; a connect attempt stuck for 20s (SYN black hole) is force-aborted and counts as one failure;
 * downlink broker-pub / broker-lan force a switch. Does not depend on DHCP result (under an ICS topology a static fallback still has a NAT public path). */
#include "app_mqtt.h"
#include "app_bsp.h"       /* 757: App-directory LED shim */
#include "app_identity.h"  /* app_fwkey2_fp: heartbeat "fwk2" */
extern const char *app_p5_str(void);   /* 757: storage self-test result (app_p5_test.c) */
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "lwip/netif.h"
#include "lwip/dhcp.h"
#include "lwip/tcpip.h"
#include "lwip/dns.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"   /* offline-watchdog autopsy peeks at client internals (conn_state/ring/pcb) */
#include "lwip/priv/tcp_priv.h"    /* kick autopsy peeks at the inner tcp_pcb (unsent/unacked/rtime) — diagnostics only */
#include "lwip/altcp_tls.h"
#include "lwip/stats.h"            /* offline-watchdog autopsy: lwip mem/memp usage (ERR_MEM source disambiguation) */
#include "lwip/etharp.h"           /* heal-ladder rung 2: gratuitous ARP re-announce (WiFi-bridge translation tables) */
#include "app_pwrfail.h"           /* PF_RETAIN: rung-3 self-reset budget survives the reset it causes */
#include "app_cfg.h"
#include "app_netcfg.h"
#include "app_ota.h"
#include "app_init.h"
#include "app_certs.h"
#include "app_anticlone.h"
#include "modbus_core.h"   /* Modbus RTU frame core (software/modbus): boot self-test report */
#include "app_can.h"       /* FDCAN loopback self-test */
#include "app_time.h"      /* time service: RTC+SNTP */
#include "app_cli.h"       /* diagnostic CLI (VCP RX) */
#include "app_rpc.h"       /* RPMsg host (dual-core) */
#include "app_mbport.h"    /* bus time feed (op=3) */
#include "lwip/apps/sntp.h"

#define FW_VERSION       (g_fw_info.ver)   /* single version source = image 0x400 ID tag (defined in main.c) */

/* ---- device identity + per-device topics (Device Cloud Protocol §2/§3) ----
 * sn = CN of the embedded device certificate (parsed once at startup). It is used as
 * the MQTT client-id (each board unique -> no client-id takeover between boards),
 * as the <sn> segment of every topic, and as the desc "sn" field. Topics:
 *   up:   dev/<type>/<sn>/up/{state,desc,status}
 *   down: dev/<type>/<sn>/dn/{cmd,fw}   (single wildcard subscription dn/#) */
#define DEV_SN_MAX  32
static char s_dev_sn[DEV_SN_MAX] = "no-cn";   /* fallback if the cert has no parsable CN (stub certs) */
static char s_top_state[64], s_top_desc[64], s_top_status[64], s_top_dn[64], s_top_ota[64];
#define LWT_ONLINE_0 "{\"online\":false}"     /* LWT payload (broker publishes on ungraceful loss) */
#define LWT_ONLINE_1 "{\"online\":true}"

/* broker one-of-three */
#define BRK_PUB  0U   /* public VPS (default, our mosquitto, mTLS) */
#define BRK_LAN  1U   /* LAN PC (plaintext, debug) */
#define BRK_AWS  2U   /* AWS IoT Core (mTLS+SNI+DNS, demo trust anchor follows the deployment) */

static mqtt_client_t *s_mqtt = NULL;
static volatile uint8_t s_mqtt_up = 0;
static volatile uint8_t s_mqtt_connecting = 0;
#if CFG_MQTT_PREFERRED == 1
#error "CFG_MQTT_PREFERRED=1 (LAN, plaintext) is debug-only and cannot be the home broker"
#endif
static volatile uint8_t s_broker = (uint8_t)CFG_MQTT_PREFERRED;
static volatile uint8_t s_conn_fails = 0;   /* 2 consecutive failures trigger fallback: non-preferred -> back to VPS, preferred -> retry in place */
static volatile uint32_t s_mqtt_pub_ok = 0;
static volatile uint8_t  s_status_dirty = 0;   /* report-on-change: set to 1 and the main loop sends status immediately on the next tick */
/* offline diagnostics: the wedge (2026-07-24 field case) is silent, so every path that can eat
 * a reconnect attempt gets a counter, and a 120s-offline autopsy photographs the client object +
 * both allocators (observation only — the recovery watchdog was withdrawn, see main loop). Last
 * autopsy is kept for the CLI ('stat') so a wedge that fires unattended can still be read back. */
static volatile uint16_t s_diag_skip_conn = 0;   /* attempts skipped: client claimed MQTT_CONNECTED while s_mqtt_up==0 (wedge hypothesis A) */
static volatile int8_t   s_diag_conn_err = 0;    /* last immediate mqtt_client_connect() error code */
static volatile uint16_t s_mq_autopsies = 0;     /* 120s-offline autopsy events since boot (heartbeat "mqfix") */
static char s_autopsy[208] = "none";             /* last autopsy line (CLI stat + heartbeat "mqa"); sized for worst-case field widths */
static char s_wda[304] = "none";                 /* publish-watchdog wedge autopsy: TX-halt state photographed right
                                                  * before the forced reconnect; rides the post-reconnect heartbeat
                                                  * as "wda" (2026-08-23, pcap-verified 15s TX halts) */
static volatile uint16_t s_wda_cnt = 0;          /* wedge autopsies since boot */
static volatile int8_t   s_diag_pub_last = 0;    /* last mqtt_publish() retcode from the heartbeat path (ERR_MEM vs
                                                  * ERR_CONN vs ERR_OK discriminates where a TX wedge lives) */
static volatile uint16_t s_mq_kicks = 0;         /* TX-stall gentle kicks fired (watchdog stage 1, heartbeat "mqk") */
static volatile uint16_t s_mq_kick_cures = 0;    /* stalls that resumed after a kick, reconnect avoided ("mqkc") */
static uint8_t           s_kick_armed = 0;       /* a kick is outstanding for the current stall */
static volatile uint16_t s_mq_rexmits = 0;       /* forced RTO retransmits fired by the kick (folded into "wda") */
static char s_kda[112] = "";                     /* kick autopsy: inner tcp_pcb state at kick time (folded into "wda"
                                                  * on escalation) — discriminates unsent-stranded (kick cures) vs
                                                  * transmitted-and-lost / timer-wedged subspecies (kick can't) */
/* ETH MAC hardware TX forensics (2026-08-25 four-hour reconnect mute, WiFi-bridge topology):
 * MMCTPCGR is the MAC's own good-frames-transmitted counter, incremented by hardware with zero
 * firmware involvement. Its delta discriminates the two remaining suspects lwip-level evidence
 * cannot separate: frames left the MAC and vanished on the wire/bridge (counter advances) vs
 * TX path wedged inside MAC/DMA (counter frozen). Sampled in both autopsies below. */
static uint32_t s_mmc_tx_base = 0;               /* MMCTPCGR at the previous offline autopsy (or last online tick) */
static uint32_t s_mmc_tx_stall = 0;              /* MMCTPCGR when the current publish stall was first detected */
static volatile uint8_t s_cloud_was_up = 0;      /* broker reached at least once this boot (rung-3 guard: a board
                                                  * that never had cloud must not reboot-loop on a dead network) */
PF_RETAIN static uint8_t s_pf_mute_resets;       /* rung-3 self-resets this offline episode; battery domain so it
                                                  * survives the reset it causes; cleared on every successful connect */
/* values published on behalf of the application (defined in app_user.c) */
extern uint8_t  app_demo_hsdi_bits(void);
extern uint32_t app_demo_count(uint8_t idx);
extern uint16_t app_demo_relays(void);

static char s_pub_buf[3968];  /* heartbeat JSON buffer (640->1280 health block; ->1536 module health mtsk, 2026-07-25;
                               * ->3648 publish-watchdog wedge autopsy "wda" field, 2026-08-23.
                               * ->2560 PH_EC engineering mdata; ->2944 mbr/mbw "cmdr" result field, 2026-08-09;
                               * ->3392 EX_16DI level bitmap + 16 counters in mdata, 2026-08-13;
                               * ->3456 datalog "dlog" status field, 2026-08-21;
                               * ->3904 offline-autopsy "mqa" field, 2026-08-25;
                               * ->3968 firmware signer "fws" + customer key fingerprint "fwk2", 2026-09-07.
                               * Sized for the worst case of every fragment at its cap — a truncated heartbeat is
                               * malformed JSON, the panel drops it silently) */
volatile int8_t s_desc_pub_err = 1;   /* desc publish result (0=ERR_OK; nonzero=ring-full etc. failure, main loop resends). 07-18 dash showing old sn = desc silently failed to send */
static void mqtt_desc_republish_cb(void *arg);   /* defined below */
static uint8_t s_rx_route = 0;              /* 1=cmd 2=ota firmware */
static char    s_cmd_buf[256];              /* command cross-fragment accumulation buffer */
static u16_t   s_cmd_fill = 0;
static int     s_clone = APP_AC_ABSENT;     /* anti-clone verification result (once at startup) */
static struct altcp_tls_config *s_tls_conf = NULL;      /* our CA trust anchor (for VPS) */
static struct altcp_tls_config *s_tls_conf_aws = NULL;  /* Amazon Root CA trust anchor (for AWS) */
static uint8_t s_aws_ident2 = 0;   /* AWS config presents the customer identity (identity2 + customer slot) */
static ip_addr_t s_aws_ip;                  /* AWS domain-name resolution result */
static volatile uint8_t s_aws_resolved = 0;

/* SNI hostname: read by the altcp_tls patch (if non-empty it sets SNI + verifies hostname). Set to the domain only when connecting to AWS.
 * Note: sending the SNI extension also requires mbedtls config to enable MBEDTLS_SSL_SERVER_NAME_INDICATION (see app_mbedtls_config.h) */
const char *g_altcp_sni_hostname = NULL;

/* fallback strategy (prevents the board getting stuck on the LAN plaintext broker after a home-broker outage):
 * home = CFG_MQTT_PREFERRED (app_cfg.h; mTLS side only). Any non-home broker failing consecutively -> back home; home itself failing -> retry in place, never auto switch sides.
 * Prevents two things: (1) sticking to a "reachable backup port" (especially plaintext LAN) and never coming home; (2) auto-downgrading from mTLS to plaintext.
 * Other brokers can only be entered explicitly via broker-pub/lan/aws (operator aware); after explicit entry, consecutive failures also auto-return home. */
static void broker_fail(void)
{
  if (++s_conn_fails >= 2U)
  {
    s_conn_fails = 0;
    if (s_broker != CFG_MQTT_PREFERRED)   /* non-home broker failing consecutively -> come home */
    {
      s_broker = CFG_MQTT_PREFERRED;
      g_altcp_sni_hostname = NULL;   /* the connect path re-derives SNI per broker */
    }
    /* s_broker == home: keep it, keep retrying (no auto-drop to LAN; power/reconnect eventually reaches a recovered broker) */
  }
}

/* ---- network: DHCP-timeout static fallback (addresses in app_cfg.h) + persistent periodic retry after fallback ----
 * When swapping routers (WISP mode doesn't issue a lease until WAN is ready) the 8s DHCP timeout can fail; fallback is only a
 * temporary state, resends
 * DISCOVER every 30s until it gets a lease (dhcp_start doesn't clear the current address; once obtained lwip auto-switches to the new address + DHCP-provided DNS). */
static volatile uint8_t s_ip_fallback = 0;   /* 1 = currently using the static fallback address (DHCP never succeeded) */
static void mqtt_force_reconnect_cb(void *arg);   /* defined below */
static void set_static_ip_cb(void *arg)  /* tcpip thread */
{
  extern struct netif gnetif;
  ip4_addr_t s_ip, s_mask, s_gw;
  (void)arg;
  ip4addr_aton(app_netcfg_ip(), &s_ip);     /* littlefs net.cfg, else app_cfg.h builtins */
  ip4addr_aton(app_netcfg_mask(), &s_mask);
  ip4addr_aton(app_netcfg_gw(), &s_gw);
  dhcp_stop(&gnetif);
  netif_set_addr(&gnetif, &s_ip, &s_mask, &s_gw);
  s_ip_fallback = 1;
  /* DNS server (for resolving AWS domain / NTP pool; ICS proxy, follows the PC's DNS, region-agnostic) */
  { ip_addr_t dns; if (ipaddr_aton(app_netcfg_dns(), &dns)) { dns_setserver(0, &dns); } }
}
static void dhcp_retry_cb(void *arg)  /* tcpip thread: periodic retry while in fallback state */
{
  extern struct netif gnetif;
  (void)arg;
  dhcp_start(&gnetif);
}
static void mqtt_dhcp_bound_check(void)     /* defaultTask each tick: leave fallback state as soon as the lease arrives */
{
  extern struct netif gnetif;
  if (s_ip_fallback && (dhcp_supplied_address(&gnetif) != 0))
  {
    s_ip_fallback = 0;
    printf("[CM7] DHCP bound (late): %s -> leaving fallback\n\r",
           ip4addr_ntoa(netif_ip4_addr(&gnetif)));
    tcpip_callback(mqtt_force_reconnect_cb, NULL);   /* source address changed, rebuild the MQTT connection */
  }
}
/* AWS domain-name resolution callback (tcpip thread) */
static void aws_dns_found(const char *name, const ip_addr_t *ipaddr, void *arg)
{
  (void)name; (void)arg;
  if (ipaddr != NULL) { s_aws_ip = *ipaddr; s_aws_resolved = 1; }
}

/* ---- MQTT callbacks (all on tcpip thread) ---- */
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
  (void)arg;
  if (strcmp(topic, s_top_ota) == 0)
  {
    s_rx_route = 2;
    ota_fw_msg_start(tot_len);
  }
  else
  {
    s_rx_route = 1;
    s_cmd_fill = 0;   /* command new session: clear the accumulation buffer (a command may be delivered across fragments, e.g. ota-sign's long hex) */
  }
}
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
  (void)arg;
  uint8_t last = ((flags & MQTT_DATA_FLAG_LAST) != 0U) ? 1U : 0U;
  if (s_rx_route == 2)
  {
    { extern uint8_t mota_recv_active(void);
      extern void mota_fw_chunk(const uint8_t *d, uint16_t l, uint8_t la);
      if (mota_recv_active()) { mota_fw_chunk(data, len, last); }   /* module image -> littlefs repository */
      else { ota_fw_chunk(data, len, last); } }
  }
  else
  {
    /* accumulate the command by fragment (ota-sign's ~150B hex spans MQTT fragments), hand to ota_cmd once complete (last) */
    u16_t room = (u16_t)(sizeof(s_cmd_buf) - 1U - s_cmd_fill);
    u16_t n = (len < room) ? len : room;
    memcpy(&s_cmd_buf[s_cmd_fill], data, n);
    s_cmd_fill = (u16_t)(s_cmd_fill + n);
    if (last)
    {
      s_cmd_buf[s_cmd_fill] = 0;
      /* mbr/mbw (backplane register access, contract v0.28) and the datalog command set
       * (note/logcfg/logstat/logget, Data_Logging_and_Event_Journal.md): blocking work —
       * tcpip thread only stashes it, the mqtt main loop executes and reports via "cmdr" */
      if ((strncmp(s_cmd_buf, "mbr ", 4) == 0) || (strncmp(s_cmd_buf, "mbw ", 4) == 0) ||
          (strncmp(s_cmd_buf, "note ", 5) == 0) || (strncmp(s_cmd_buf, "logcfg", 6) == 0) ||
          (strncmp(s_cmd_buf, "logstat", 7) == 0) || (strncmp(s_cmd_buf, "logget ", 7) == 0) ||
          (strncmp(s_cmd_buf, "vent", 4) == 0) || (strncmp(s_cmd_buf, "netcfg", 6) == 0) ||
          (strncmp(s_cmd_buf, "dose", 4) == 0) || (strncmp(s_cmd_buf, "pulse", 5) == 0) ||
          (strncmp(s_cmd_buf, "hsdi ", 5) == 0) || (strncmp(s_cmd_buf, "flow", 4) == 0) ||
          (strncmp(s_cmd_buf, "aer", 3) == 0))
      {
        extern volatile uint8_t s_mbreg_pend;
        extern char s_mbreg_cmd[160];
        if (!s_mbreg_pend)
        {
          strncpy(s_mbreg_cmd, s_cmd_buf, sizeof(s_mbreg_cmd) - 1U);
          s_mbreg_cmd[sizeof(s_mbreg_cmd) - 1U] = 0;
          s_mbreg_pend = 1;
        }
      }
      else if (strcmp(s_cmd_buf, "reboot") == 0)
      {
        /* Cloud reboot (2026-08-22 field lesson: a backplane RX wedge needed an on-site reset).
         * MUST be deferred: dn/cmd is QoS1 — resetting before the PUBACK leaves the command
         * queued on the broker, which redelivers it on reconnect = infinite reboot loop. */
        extern volatile uint8_t s_reboot_cnt;
        extern const char *s_reboot_why;
        s_reboot_why = "cloud reboot";
        s_reboot_cnt = 5;                        /* mqtt main loop 500ms beats: reset in ~2.5s */
      }
      else { ota_cmd(s_cmd_buf, s_cmd_fill); }
      s_cmd_fill = 0;
    }
  }
}
/* deferred mbr/mbw + datalog commands (see above): executed in the mqtt task, result lands in the heartbeat */
volatile uint8_t s_mbreg_pend = 0;
volatile uint8_t s_reboot_cnt = 0;   /* cloud "reboot": armed by the downlink callback, counted down and fired by the mqtt main loop */
const char *s_reboot_why = "deferred reboot";   /* breadcrumb for the reset that s_reboot_cnt fires */
char s_mbreg_cmd[160];   /* sized for "note <text<=120B>" */
static char s_mbreg_result[300] = "";
static void mqtt_pub_done_cb(void *arg, err_t result)
{
  (void)arg;
  if (result == ERR_OK) { s_mqtt_pub_ok++; }
}
static void mqtt_sub_result_cb(void *arg, err_t result)
{
  (void)arg;
  (void)result;
}
/* device self-description: after connecting, publish to dev/<type>/<sn>/up/desc (retain). Each point carries "f" = the matching flat-heartbeat field name,
 * the panel takes values generically from it — adding/changing a point only changes this desc + heartbeat, the panel needs no change. Flat heartbeat kept (the burn-in tool relies on it). */
/* desc all-English: panel UI already English, point/group/command names follow */
/* template: sn/name filled from the certificate CN at startup (see desc_build) */
/* desc group order IS the dash card order, grouped by NATURE, not by slot (user ruling
 * 2026-08-09): first every process card — Onboard IO, then each slot's process card in
 * address order — then every health card — System, then each slot's cpu/stack card —
 * Network last. desc_build() splices the module groups at the two seams (HEAD ends after
 * the process cards, G_SYS/G_NET bracket the health cards); MID ends inside the points
 * array for the module points. Membership change (app_diag_mods_dirty) -> rebuild +
 * republish (retained). */
static const char DESC_HEAD[] =
"{\"pv\":1,\"type\":\"" CFG_DEV_TYPE "\",\"sn\":\"%s\",\"name\":\"I757 %s\","
"\"groups\":[{\"id\":\"io\",\"name\":\"Onboard IO\"},{\"id\":\"rly\",\"name\":\"Relay Module (slot 2)\"}";
static const char DESC_G_SYS[] = ",{\"id\":\"sys\",\"name\":\"System\"}";
static const char DESC_G_NET[] = ",{\"id\":\"net\",\"name\":\"Network\"}";
static const char DESC_MID[] =
"],"
"\"points\":["
"{\"id\":\"io.hsdi\",\"name\":\"HSDI Inputs\",\"kind\":\"bits\",\"width\":8,\"rw\":\"ro\",\"group\":\"io\",\"f\":\"hsdi\"},"
"{\"id\":\"io.cnt0\",\"name\":\"Counter HSDI0\",\"kind\":\"num\",\"rw\":\"ro\",\"group\":\"io\",\"f\":\"c0\"},"
"{\"id\":\"io.g1\",\"name\":\"Gutter 1 Flow\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"L/min\",\"scale\":0.1,\"group\":\"io\",\"f\":\"g1\"},"
"{\"id\":\"io.g2\",\"name\":\"Gutter 2 Flow\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"L/min\",\"scale\":0.1,\"group\":\"io\",\"f\":\"g2\"},"
"{\"id\":\"io.g3\",\"name\":\"Gutter 3 Flow\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"L/min\",\"scale\":0.1,\"group\":\"io\",\"f\":\"g3\"},"
"{\"id\":\"io.g4\",\"name\":\"Gutter 4 Flow\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"L/min\",\"scale\":0.1,\"group\":\"io\",\"f\":\"g4\"},"
"{\"id\":\"io.galm\",\"name\":\"Gutter Alarm\",\"kind\":\"num\",\"rw\":\"ro\",\"group\":\"io\",\"f\":\"ga\"},"
"{\"id\":\"io.flowr\",\"name\":\"Water Flow Rate\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"L/min\",\"scale\":0.1,\"group\":\"io\",\"f\":\"flowr\"},"
"{\"id\":\"rly.do\",\"name\":\"Relay Outputs\",\"kind\":\"bits\",\"width\":16,\"rw\":\"rw\",\"group\":\"rly\",\"f\":\"rly\"},"
"{\"id\":\"vent.stage\",\"name\":\"Roof Vent Stage\",\"kind\":\"num\",\"rw\":\"ro\",\"group\":\"io\",\"f\":\"vent\"},"
"{\"id\":\"vent.mode\",\"name\":\"Roof Vent Mode\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"io\",\"f\":\"ventm\"},"
"{\"id\":\"aer.state\",\"name\":\"Air Pump\",\"kind\":\"bool\",\"rw\":\"ro\",\"group\":\"io\",\"f\":\"aer\"},"
"{\"id\":\"aer.mode\",\"name\":\"Air Pump Mode\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"io\",\"f\":\"aerm\"},"
"{\"id\":\"dose.state\",\"name\":\"Dosing Pumps\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"io\",\"f\":\"dose\"},"
"{\"id\":\"do.beep\",\"name\":\"Buzzer\",\"kind\":\"bool\",\"rw\":\"rw\",\"group\":\"io\",\"f\":\"beep\"},"
"{\"id\":\"sys.ver\",\"name\":\"Firmware Version\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"sys\",\"f\":\"ver\"},"
"{\"id\":\"sys.bank\",\"name\":\"Active Bank\",\"kind\":\"num\",\"rw\":\"ro\",\"group\":\"sys\",\"f\":\"bank\"},"
"{\"id\":\"sys.ota\",\"name\":\"OTA Status\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"sys\",\"f\":\"ota\"},"
"{\"id\":\"sys.otaevt\",\"name\":\"OTA Last Event\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"sys\",\"f\":\"evt\"},"
"{\"id\":\"sys.fws\",\"name\":\"Firmware Signer\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"sys\",\"f\":\"fws\"},"
"{\"id\":\"sys.fwk2\",\"name\":\"Customer Sign Key\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"sys\",\"f\":\"fwk2\"},"
"{\"id\":\"sys.time\",\"name\":\"Unix Time\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"s\",\"group\":\"sys\",\"f\":\"time\"},"
"{\"id\":\"sys.uptime\",\"name\":\"Uptime\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"s\",\"group\":\"sys\",\"f\":\"tick\",\"scale\":0.5},"
"{\"id\":\"sys.heap\",\"name\":\"Free Heap\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"B\",\"group\":\"sys\",\"f\":\"heap\"},"
"{\"id\":\"sys.temp\",\"name\":\"Chip Temperature\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"℃\",\"group\":\"sys\",\"f\":\"temp\"},"
"{\"id\":\"net.broker\",\"name\":\"Current Broker\",\"kind\":\"enum\",\"rw\":\"ro\",\"group\":\"net\",\"f\":\"bkr\","
  "\"enum\":[{\"v\":\"pub\",\"label\":\"VPS\"},{\"v\":\"aws\",\"label\":\"AWS\"},{\"v\":\"lan\",\"label\":\"LAN\"}]},"
"{\"id\":\"sys.p5\",\"name\":\"Storage Self-Test\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"sys\",\"f\":\"p5\"},"
"{\"id\":\"sys.mb\",\"name\":\"Modbus Ports\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"sys\",\"f\":\"mb\"},"
"{\"id\":\"sys.cpu\",\"name\":\"CPU Load CM7\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"%%\",\"group\":\"sys\",\"f\":\"cpu\"},"
"{\"id\":\"sys.cpu4\",\"name\":\"CPU Load CM4\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"%%\",\"group\":\"sys\",\"f\":\"cpu4\"},"
"{\"id\":\"sys.stkmin\",\"name\":\"Min Stack Headroom\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"B\",\"group\":\"sys\",\"f\":\"stkmin\"},"
"{\"id\":\"sys.tsk7\",\"name\":\"CM7 Tasks (state cpu%% stackfree)\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"sys\",\"f\":\"tsk7\"},"
"{\"id\":\"sys.tsk4\",\"name\":\"CM4 Tasks (state cpu%% stackfree)\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"sys\",\"f\":\"tsk4\"},"
"{\"id\":\"sys.rst\",\"name\":\"Last Reset Cause\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"sys\",\"f\":\"rst\"},"
"{\"id\":\"sys.flt\",\"name\":\"Prev-Run Fault\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"sys\",\"f\":\"flt\"},"
"{\"id\":\"sec.hs608\",\"name\":\"608A TLS Handshake Signs\",\"kind\":\"num\",\"rw\":\"ro\",\"group\":\"sys\",\"f\":\"hs608\"},"
"{\"id\":\"net.pubok\",\"name\":\"Publish Count\",\"kind\":\"num\",\"rw\":\"ro\",\"group\":\"net\",\"f\":\"pub_ok\"}";
static const char DESC_TAIL[] =
"],"
"\"commands\":["
/* broker.set removed from desc: after the web switches broker the panel loses contact (panel only watches VPS).
 * Roaming capability retained; burn-in uses the old text commands broker-aws/broker-pub (serial or MQTT downlink) explicitly. */
"{\"id\":\"ota.revert\",\"name\":\"Revert Firmware (swap bank)\",\"danger\":true,\"confirm\":\"strong\",\"args\":[]}"
"]}";

/* head+mid+tail + sn twice + up to 8 module blocks (group ~80B + three diag points ~330B
 * + eight PH_EC engineering points ~1000B or EX_16DI bitmap+16 counters ~1800B for the
 * modules that have them; per-module budget = the largest block) */
static char s_desc_buf[sizeof(DESC_HEAD) + sizeof(DESC_G_SYS) + sizeof(DESC_G_NET) + sizeof(DESC_MID) + sizeof(DESC_TAIL) + 2U * DEV_SN_MAX + 8U * 2240U];
static u16_t s_desc_len = 0;

static void desc_build(void)
{
  extern uint16_t app_diag_mod_list(const void **out);
  extern uint8_t  app_diag_mod_addr(const void *m, uint16_t i);
  extern uint16_t app_diag_mod_type(const void *m, uint16_t i);
  extern uint16_t app_diag_mod_fw(const void *m, uint16_t i);
  extern const char *app_diag_mod_typename(uint16_t type);
  const void *mods = 0;
  uint16_t nm = app_diag_mod_list(&mods);
  u16_t n = (u16_t)snprintf(s_desc_buf, sizeof(s_desc_buf), DESC_HEAD, s_dev_sn, s_dev_sn);
  for (uint16_t i = 0; i < nm; i++)         /* process cards, address order */
  {
    unsigned a = app_diag_mod_addr(mods, i);
    if (app_diag_mod_type(mods, i) == 2U)   /* PH_EC: measurements card ("m<addr>" = health) */
    {
      n += (u16_t)snprintf(&s_desc_buf[n], sizeof(s_desc_buf) - n,
             ",{\"id\":\"m%up\",\"name\":\"Slot %u: PH_EC Measurements\"}", a, a);
    }
    if (app_diag_mod_type(mods, i) == 3U)   /* EX_16DI: inputs/counters card */
    {
      n += (u16_t)snprintf(&s_desc_buf[n], sizeof(s_desc_buf) - n,
             ",{\"id\":\"m%up\",\"name\":\"Slot %u: DI Inputs\"}", a, a);
    }
  }
  n += (u16_t)snprintf(&s_desc_buf[n], sizeof(s_desc_buf) - n, DESC_G_SYS);
  for (uint16_t i = 0; i < nm; i++)         /* health cards, address order, after System */
  {
    unsigned a = app_diag_mod_addr(mods, i), fw = app_diag_mod_fw(mods, i);
    n += (u16_t)snprintf(&s_desc_buf[n], sizeof(s_desc_buf) - n,
           ",{\"id\":\"m%u\",\"name\":\"Slot %u: %s v%u.%u\"}",
           a, a, app_diag_mod_typename(app_diag_mod_type(mods, i)), fw >> 8, fw & 0xFFU);
  }
  n += (u16_t)snprintf(&s_desc_buf[n], sizeof(s_desc_buf) - n, DESC_G_NET);
  n += (u16_t)snprintf(&s_desc_buf[n], sizeof(s_desc_buf) - n, DESC_MID);
  for (uint16_t i = 0; i < nm; i++)
  {
    unsigned a = app_diag_mod_addr(mods, i);
    n += (u16_t)snprintf(&s_desc_buf[n], sizeof(s_desc_buf) - n,
           ",{\"id\":\"m%u.cpu\",\"name\":\"CPU\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"%%\",\"group\":\"m%u\",\"f\":\"m%ucpu\"}"
           ",{\"id\":\"m%u.stkmin\",\"name\":\"Min Stack\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"B\",\"group\":\"m%u\",\"f\":\"m%ustk\"}"
           ",{\"id\":\"m%u.tsk\",\"name\":\"Tasks\",\"kind\":\"str\",\"rw\":\"ro\",\"group\":\"m%u\",\"f\":\"m%utsk\"}",
           a, a, a, a, a, a, a, a, a);
    if (app_diag_mod_type(mods, i) == 2U)   /* PH_EC: engineering-value points (module spec §2),
                                             * grouped on the process card m<addr>p */
    {
      n += (u16_t)snprintf(&s_desc_buf[n], sizeof(s_desc_buf) - n,
           ",{\"id\":\"m%u.ph1\",\"name\":\"pH 1\",\"kind\":\"num\",\"rw\":\"ro\",\"group\":\"m%up\",\"f\":\"m%uph1\",\"scale\":0.01}"
           ",{\"id\":\"m%u.ph2\",\"name\":\"pH 2\",\"kind\":\"num\",\"rw\":\"ro\",\"group\":\"m%up\",\"f\":\"m%uph2\",\"scale\":0.01}"
           ",{\"id\":\"m%u.ec1\",\"name\":\"EC 1\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"µS/cm\",\"group\":\"m%up\",\"f\":\"m%uec1\"}"
           ",{\"id\":\"m%u.ec2\",\"name\":\"EC 2\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"µS/cm\",\"group\":\"m%up\",\"f\":\"m%uec2\"}"
           ",{\"id\":\"m%u.t1\",\"name\":\"Temp EC1\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"℃\",\"group\":\"m%up\",\"f\":\"m%ut1\",\"scale\":0.1}"
           ",{\"id\":\"m%u.t2\",\"name\":\"Temp EC2\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"℃\",\"group\":\"m%up\",\"f\":\"m%ut2\",\"scale\":0.1}"
           ",{\"id\":\"m%u.t3\",\"name\":\"Temp pH1\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"℃\",\"group\":\"m%up\",\"f\":\"m%ut3\",\"scale\":0.1}"
           ",{\"id\":\"m%u.t4\",\"name\":\"Temp pH2\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"℃\",\"group\":\"m%up\",\"f\":\"m%ut4\",\"scale\":0.1}",
           a, a, a, a, a, a, a, a, a, a, a, a, a, a, a, a, a, a, a, a, a, a, a, a);
    }
    if (app_diag_mod_type(mods, i) == 3U)   /* EX_16DI: level bitmap + 16 counters (spec §3,
                                             * contract v0.25), grouped on process card m<addr>p */
    {
      n += (u16_t)snprintf(&s_desc_buf[n], sizeof(s_desc_buf) - n,
           ",{\"id\":\"m%u.di\",\"name\":\"DI Inputs\",\"kind\":\"bits\",\"width\":16,\"rw\":\"ro\",\"group\":\"m%up\",\"f\":\"m%udi\"}",
           a, a, a);
      for (unsigned k = 1; k <= 16U; k++)
      {
        n += (u16_t)snprintf(&s_desc_buf[n], sizeof(s_desc_buf) - n,
             ",{\"id\":\"m%u.c%u\",\"name\":\"Counter DI%u\",\"kind\":\"num\",\"rw\":\"ro\",\"group\":\"m%up\",\"f\":\"m%uc%u\"}",
             a, k, k, a, a, k);
      }
    }
  }
  n += (u16_t)snprintf(&s_desc_buf[n], sizeof(s_desc_buf) - n, DESC_TAIL);
  s_desc_len = n;
}

#if APP_ENABLE_CLOUD
#include "app_identity.h"
/* pull the subject CN of the ACTIVE device cert (identity partition > embedded) -> s_dev_sn.
 * Called after app_identity_init (mbedTLS pool + littlefs ready), before any connect. */
static void dev_identity_init(void)
{
  (void)app_identity_cn(s_dev_sn, sizeof(s_dev_sn));   /* keeps the "no-cn" fallback on failure */
  snprintf(s_top_state,  sizeof(s_top_state),  "dev/%s/%s/up/state",  CFG_DEV_TYPE, s_dev_sn);
  snprintf(s_top_desc,   sizeof(s_top_desc),   "dev/%s/%s/up/desc",   CFG_DEV_TYPE, s_dev_sn);
  snprintf(s_top_status, sizeof(s_top_status), "dev/%s/%s/up/status", CFG_DEV_TYPE, s_dev_sn);
  snprintf(s_top_dn,     sizeof(s_top_dn),     "dev/%s/%s/dn/#",      CFG_DEV_TYPE, s_dev_sn);
  snprintf(s_top_ota,    sizeof(s_top_ota),    "dev/%s/%s/dn/fw",     CFG_DEV_TYPE, s_dev_sn);
  desc_build();                            /* modules unknown yet: base desc; rebuilt on discovery */
}
#endif

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
  (void)arg;
  s_mqtt_connecting = 0;
  if (status == MQTT_CONNECT_ACCEPTED)
  {
    s_mqtt_up = 1;
    s_conn_fails = 0;
    s_cloud_was_up = 1;
    s_pf_mute_resets = 0;   /* offline episode over: refill the rung-3 self-reset budget */
    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);
    err_t sub_err = mqtt_subscribe(client, s_top_dn, 1, mqtt_sub_result_cb, NULL);
    (void)sub_err;   /* single downlink subscription (with two concurrent subscribes the second fails silently) */
    /* birth: online=true (retain) counters the LWT; panel gets accurate per-device presence (protocol §13) */
    (void)mqtt_publish(client, s_top_state, LWT_ONLINE_1, (u16_t)(sizeof(LWT_ONLINE_1) - 1U), 1, 1, NULL, NULL);
    /* self-description: publish desc (retain, panel gets it on subscribe); capture result, failure (ring full) resent by main loop */
    { extern volatile int8_t s_desc_pub_err;
      s_desc_pub_err = (int8_t)mqtt_publish(client, s_top_desc, s_desc_buf, s_desc_len, 1, 1, NULL, NULL); }
  }
  else
  {
    s_mqtt_up = 0;
    broker_fail();
  }
}
static void mqtt_connect_cb(void *arg)  /* tcpip thread */
{
  /* three profiles: public first; switch sides / fall back on consecutive failures. AWS needs DNS resolution of the domain first + set SNI */
  static struct mqtt_connect_client_info_t ci = {
    .keep_alive = 30,
  };
  ci.client_id   = s_dev_sn;        /* client-id = certificate CN: unique per board, no takeover between boards */
  ci.will_topic  = s_top_state;     /* LWT: broker announces {"online":false} (retain) on ungraceful loss (protocol §13) */
  ci.will_msg    = LWT_ONLINE_0;
  ci.will_qos    = 1;
  ci.will_retain = 1;
  ip_addr_t broker;
  u16_t port;
  (void)arg;
  if (s_broker == BRK_LAN)
  {
    if (!ipaddr_aton(CFG_MQTT_LAN_HOST, &broker)) { s_mqtt_connecting = 0; return; }
    port = CFG_MQTT_LAN_PORT;
    ci.client_user = NULL; ci.client_pass = NULL; ci.tls_config = NULL;   /* plaintext */
    g_altcp_sni_hostname = NULL;
  }
  else if (s_broker == BRK_AWS)
  {
    /* domain access: if not resolved, send a DNS query first, next round retries with the cached IP once resolved */
    if (!s_aws_resolved)
    {
      err_t d = dns_gethostbyname(CFG_MQTT_AWS_HOST, &s_aws_ip, aws_dns_found, NULL);
      if (d == ERR_OK) { s_aws_resolved = 1; }   /* already cached */
      else { s_mqtt_connecting = 0; return; }     /* ERR_INPROGRESS: wait for callback, back out this round */
    }
    broker = s_aws_ip;
    port = CFG_MQTT_AWS_PORT;
    ci.client_user = NULL; ci.client_pass = NULL;   /* AWS authenticates by certificate, no username/password */
    ci.tls_config = s_tls_conf_aws;                 /* trust anchor = Amazon Root CA 1 */
    g_altcp_sni_hostname = CFG_MQTT_AWS_HOST;        /* AWS forces SNI + hostname verification */
    { extern void app_se_sign_slot_set(uint16_t);   /* sign with the slot the presented cert certifies */
      app_se_sign_slot_set(s_aws_ident2 ? (uint16_t)CFG_SE_CUSTOMER_SLOT : 0U); }
  }
  else /* BRK_PUB */
  {
    if (!ipaddr_aton(CFG_MQTT_PUB_HOST, &broker)) { s_mqtt_connecting = 0; return; }
    port = CFG_MQTT_PUB_PORT;
    ci.client_user = CFG_MQTT_PUB_USER;
    ci.client_pass = CFG_MQTT_PUB_PASS;
    ci.tls_config = s_tls_conf;   /* our VPS: mTLS, our CA */
    g_altcp_sni_hostname = NULL;   /* connect by IP, only verify the CA chain */
    { extern void app_se_sign_slot_set(uint16_t);
      app_se_sign_slot_set(0U); }                   /* factory identity signs on the vendor broker */
  }
  if (s_mqtt == NULL) { s_mqtt = mqtt_client_new(); }
  if (s_mqtt != NULL && !mqtt_client_is_connected(s_mqtt))
  {
    err_t ce = mqtt_client_connect(s_mqtt, &broker, port, mqtt_connection_cb, NULL, &ci);
    if (ce != ERR_OK)
    {
      s_diag_conn_err = (int8_t)ce;   /* evidence: which error eats the silent retries */
      s_mqtt_connecting = 0;          /* immediate failure has no callback, must clear the flag manually or it never retries */
    }
    /* NAGLE — WANTED BUT WITHDRAWN (2026-08-23). Motive: pcap showed the stage-1 kick's forced
     * retransmit of a lost sub-MSS record being PARKED by Nagle ("small segment while data is
     * unacked") until the close's FIN flushed it 15 s too late; TCP_NODELAY is the textbook
     * MQTT answer. BUT: calling altcp_nagle_disable(s_mqtt->conn) right here bricked the
     * whole network side — the v4 field push produced ZERO packets (not even a SYN) for the
     * full 10 min until the OTA no-cloud gate rolled it back (first live save of that gate on
     * this board). App loop alive, no fault recorded, mechanism NOT understood. Do not
     * re-attempt on the field board until reproduced on the bench under SWD. Candidate safer
     * variant for the bench: set tcp_nagle_disable() lazily on the inner tcp_pcb inside
     * mqtt_kick_cb (post-established, plain flag write, no altcp dispatch). */
  }
  else
  {
    /* client refuses to reconnect because it claims it IS connected — while the app-level
     * flag says down. This inconsistent pair is wedge hypothesis A; count every hit. */
    if ((s_mqtt != NULL) && !s_mqtt_up) { s_diag_skip_conn++; }
    s_mqtt_connecting = 0;
  }
}
static void mqtt_pub_cb(void *arg)  /* tcpip thread */
{
  (void)arg;
  if (s_mqtt != NULL && s_mqtt_up)
  {
    s_diag_pub_last = (int8_t)mqtt_publish(s_mqtt, s_top_status, s_pub_buf, (u16_t)strlen(s_pub_buf), 0, 0, mqtt_pub_done_cb, NULL);
  }
}
static void mqtt_desc_republish_cb(void *arg)  /* tcpip thread: resend desc after first publish failed (ring full), until ERR_OK */
{
  (void)arg;
  if (s_mqtt != NULL && s_mqtt_up && (s_desc_pub_err != 0))
  {
    s_desc_pub_err = (int8_t)mqtt_publish(s_mqtt, s_top_desc, s_desc_buf, s_desc_len, 1, 1, NULL, NULL);
  }
}
/* ---- generic uplink publish slot (app_datalog file chunks etc.): one payload in flight,
 * result readable so the caller can confirm-or-resend. Payload is copied into the slot, so
 * the caller's buffer is free as soon as the call accepts. QoS0 — TCP already orders and
 * retransmits; only a broker reconnect can lose a chunk, and the seq numbers expose that. */
static char    s_up_topic[96];
static uint8_t s_up_payload[1100];
static u16_t   s_up_len = 0;
static volatile uint8_t s_up_busy = 0;
static volatile int8_t  s_up_result = 0;
static void mqtt_pub_updata_cb(void *arg)  /* tcpip thread */
{
  (void)arg;
  if ((s_mqtt != NULL) && s_mqtt_up)
  {
    s_up_result = (int8_t)mqtt_publish(s_mqtt, s_up_topic, s_up_payload, s_up_len, 0, 0, NULL, NULL);
  }
  else { s_up_result = -1; }
  s_up_busy = 0;
}
uint8_t app_mqtt_ready(void) { return s_mqtt_up; }
uint8_t app_mqtt_pub_updata_busy(void) { return s_up_busy; }
int8_t  app_mqtt_pub_updata_result(void) { return s_up_result; }
int app_mqtt_pub_updata(const char *sub, const void *data, uint16_t len)
{
#if APP_ENABLE_CLOUD
  if (!s_mqtt_up || s_up_busy || (len > sizeof(s_up_payload))) { return -1; }
  snprintf(s_up_topic, sizeof(s_up_topic), "dev/%s/%s/up/%s", CFG_DEV_TYPE, s_dev_sn, sub);
  memcpy(s_up_payload, data, len);
  s_up_len = len;
  s_up_busy = 1;
  if (tcpip_callback(mqtt_pub_updata_cb, NULL) != ERR_OK) { s_up_busy = 0; return -1; }
  return 0;
#else
  (void)sub; (void)data; (void)len;
  return -1;
#endif
}
static void mqtt_kick_cb(void *arg)  /* tcpip thread: transmit TCP segments stranded on the unsent queue.
                                      * Root cause (wda #1..#4, 2026-08-23): this broker link is ACK-only
                                      * downstream — when the last in-flight byte gets ACKed and the next
                                      * TLS record is queued from inside that ACK's own callback chain,
                                      * no later rx traffic ever calls tcp_output again; the segment sits
                                      * on the unsent queue forever (never transmitted -> no RTO timer
                                      * either), sndbuf pins at 0 and every publish backs up. One
                                      * altcp_output() from a clean context flushes it. */
{
  (void)arg;
  if ((s_mqtt != NULL) && s_mqtt_up && (s_mqtt->conn != NULL))
  {
    /* photograph the inner tcp_pcb BEFORE the kick: whether the stranded record sits on the
     * unsent queue (kick cures) or on unacked (transmitted-and-lost on the 4G uplink — only
     * RTO can help; rtime/nrtx show whether the retransmit machinery is alive) */
    struct altcp_pcb *inner = s_mqtt->conn->inner_conn;
    struct tcp_pcb *t = (inner != NULL) ? (struct tcp_pcb *)inner->state : NULL;
    if (t != NULL)
    {
      snprintf(s_kda, sizeof(s_kda), "k uns=%d una=%d rt=%d nrtx=%u cwnd=%u wnd=%u fl=%02x rx=%u",
               (int)(t->unsent != NULL), (int)(t->unacked != NULL),
               (int)t->rtime, (unsigned)t->nrtx,
               (unsigned)t->cwnd, (unsigned)t->snd_wnd, (unsigned)t->flags,
               (unsigned)s_mq_rexmits);
      /* subspecies B (wda #2 kick autopsy, 2026-08-23): the record was transmitted and lost on
       * the 4G uplink, and this connection's RTO has inflated past the 15 s watchdog budget
       * (rtime counting, nrtx==0, retransmit never fires before the reconnect). Waiting out a
       * 15-30 s RTO is slower than a reconnect — force the RTO retransmit NOW instead; it
       * cures in one RTT. Only when unsent is empty: otherwise the plain output kick below
       * is the right medicine (subspecies A, stranded unsent). */
      if (t->unacked != NULL)
      {
        /* fire on every kick, unsent empty or not: tcp_rexmit_rto() prepends unacked to
         * unsent and outputs the lot — the 15 s watchdog budget has no room for a polite
         * first-kick-only attempt (wda #1 on kick-v2: guard was too strict, rexmit never
         * fired and the stall still ended in a reconnect) */
        s_mq_rexmits++;
        tcp_rexmit_rto(t);
      }
    }
    (void)altcp_output(s_mqtt->conn);   /* altcp_default_output -> inner TCP tcp_output */
  }
}
static void mqtt_force_reconnect_cb(void *arg)  /* tcpip thread: half-open TCP self-heal */
{
  (void)arg;
  if (s_mqtt != NULL) { mqtt_disconnect(s_mqtt); }
  s_mqtt_up = 0;
  s_mqtt_connecting = 0;
}
#if APP_ENABLE_CLOUD
/* build the two altcp_tls client configs from the active identity cert. Called once at boot.
 * The config STRUCT lives on the lwip heap (altcp_mbedtls_alloc_config -> mem_calloc); only the
 * parsed cert chains live in the mbedTLS pool. Freeing any previous configs first is mandatory:
 * the 2026-07-24 tier-2 watchdog NULLed these pointers without altcp_tls_free_config and leaked
 * ~3K of the 16K lwip heap per fire, until mqtt_client_new (~3.3K contiguous) could never
 * succeed again — the actual "connect stuck at ERR_MEM forever" death. */
static void tls_configs_build(void)
{
  size_t certlen = 0, cert2len = 0;
  const u8_t *cert  = app_identity_cert(&certlen);
  const u8_t *cert2 = app_identity2_cert(&cert2len);   /* customer identity, optional */
  if (s_tls_conf != NULL)     { altcp_tls_free_config(s_tls_conf);     s_tls_conf = NULL; }
  if (s_tls_conf_aws != NULL) { altcp_tls_free_config(s_tls_conf_aws); s_tls_conf_aws = NULL; }
  if (cert == NULL) { cert = (const u8_t *)""; certlen = 1; }   /* unprovisioned: stub -> configs fail gracefully, cloud stays down */
  s_tls_conf = altcp_tls_create_config_client_2wayauth(
      (const u8_t *)app_tls_ca_pem,      app_tls_ca_len,
      (const u8_t *)app_tls_devkey_pem,  app_tls_devkey_len,
      NULL, 0, cert, certlen);
  /* AWS profile: trust anchor = Amazon Root CA 1; client identity = the CUSTOMER identity
   * (identity2.der certifying the CFG_SE_CUSTOMER_SLOT key) when provisioned, else the
   * factory identity. The private-key handle passed here is the placeholder either way —
   * real signing goes to the 608A, slot chosen per connect (app_se_sign_slot_set). */
  s_aws_ident2 = (cert2 != NULL) ? 1U : 0U;
  s_tls_conf_aws = altcp_tls_create_config_client_2wayauth(
      (const u8_t *)app_aws_ca_pem,      app_aws_ca_len,
      (const u8_t *)app_tls_devkey_pem,  app_tls_devkey_len,
      NULL, 0, s_aws_ident2 ? cert2 : cert, s_aws_ident2 ? cert2len : certlen);
}
#endif
static void sntp_start_cb(void *arg)  /* tcpip thread: start SNTP once IP is ready (poll mode, default 1h re-sync) */
{
  (void)arg;
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_init();
}
static void mqtt_conn_abort_cb(void *arg)  /* tcpip thread: force-abort a stuck connect attempt (SYN unanswered slow timeout), count one failure */
{
  (void)arg;
  if (!s_mqtt_up && s_mqtt_connecting)
  {
    if (s_mqtt != NULL) { mqtt_disconnect(s_mqtt); }
    s_mqtt_connecting = 0;
    broker_fail();
  }
}
static void net_refresh_cb(void *arg)  /* tcpip thread: heal-ladder rung 2 — re-announce this station.
                                        * A WiFi bridge in client mode masquerades wired MACs behind its
                                        * own; when its translation entry for us dies we go silently
                                        * TX-deaf to the WAN side (2026-08-25 four-hour mute). A
                                        * gratuitous ARP plus a DHCP renew re-feeds every table between
                                        * here and the router; both are harmless no-ops on a healthy net. */
{
  extern struct netif gnetif;
  (void)arg;
  etharp_gratuitous(&gnetif);
  if (dhcp_supplied_address(&gnetif) != 0) { dhcp_renew(&gnetif); }
}
void mqtt_mark_dirty(void) { s_status_dirty = 1; }   /* call this when state changed (LED/command) to trigger an immediate report */

/* cloud link healthy (OTA trial-promotion criterion): broker connected + at least 3 heartbeats actually sent this boot.
 * pub_ok is the TCP-layer send-success count (QoS0), the connection itself is mTLS mutual auth -> enough to prove link+identity are both real */
uint8_t mqtt_cloud_ok(void) { return (s_mqtt_up && (s_mqtt_pub_ok >= 3U)) ? 1U : 0U; }

/* offline forensics for the CLI ('stat'): autopsy count + last autopsy line */
uint16_t mqtt_rebuilds(void) { return s_mq_autopsies; }
const char *mqtt_last_autopsy(void) { return s_autopsy; }

void mqtt_broker_select(uint8_t idx)  /* ota_cmd (tcpip thread): broker-pub/lan/aws force switch */
{
  s_broker = (idx <= BRK_AWS) ? idx : CFG_MQTT_PREFERRED;
  s_conn_fails = 0;
  mqtt_force_reconnect_cb(NULL);   /* already on tcpip thread, direct call safe */
}

/* ---- application main loop (defaultTask context; LwIP already init'd by main.c generated code) ----
 * green LED 1Hz + dual-core ping-pong + DHCP static fallback + MQTT heartbeat/watchdog + OTA consume + console diagnostics */
void mqtt_app_task(void)
{
  extern struct netif gnetif;
  /* TLS base: hardware RNG + mbedTLS memory pool, then parse the cert triple to build the client config (mutual auth) */
  app_tls_init();   /* hardware RNG + mbedTLS pool (also used by anti-clone) — kept in both modes */
  /* device identity: littlefs identity partition first, embedded cert as fallback
   * (one universal image for every board; the jig writes identity.der in production).
   * Outside the cloud switch: anti-clone verifies against the identity cert in both modes. */
  app_identity_init();
#if APP_ENABLE_CLOUD
  tls_configs_build();
  printf("[TLS] configs: vps=%s aws=%s\n\r",
         (s_tls_conf != NULL) ? "OK" : "FAIL", (s_tls_conf_aws != NULL) ? "OK" : "FAIL");
  dev_identity_init();   /* sn = device-cert CN -> client-id + dev/<type>/<sn>/... topics + desc */
  printf("[MQTT] identity: sn=%s topics=dev/%s/%s/* desc=%uB\n\r",
         s_dev_sn, CFG_DEV_TYPE, s_dev_sn, (unsigned)s_desc_len);
#else
  printf("[TLS] cloud disabled (APP_ENABLE_CLOUD=0) -> standalone mode\n\r");
#endif
  /* anti-clone: challenge-response verify the hardware holds the genuine SE private key (software emulation in dev, hardware 608B in production).
   * ⚠️ here it only prints + reports for demo; the production board must weave the s_clone result into real functionality (see comments in app_anticlone.c) */
  s_clone = app_anticlone_verify();
  printf("[SEC] anti-clone: %s\n\r",
         (s_clone == APP_AC_GENUINE) ? "GENUINE" : (s_clone == APP_AC_ABSENT) ? "no-SE(dev)" : "CLONE SUSPECT");
  app_temp_init();    /* initialize ADC3 internal temperature sensor */
  /* OTA trial-period banner: first boot after swap = TRIAL (awaiting cloud confirm), normal = normal (never rolls back) */
  printf("[OTA] boot: bank=%lu %s attempts=%lu evt=\"%s\"\n\r",
         (unsigned long)(ota_swap_active() + 1U), ota_in_trial() ? "TRIAL" : "normal",
         (unsigned long)ota_boot_attempts(), ota_last_evt());
  /* Modbus frame core boot self-test (no x86 compiler on this host, unit tests run on-board = permanent regression) */
  {
    int t = 0;
    int f = mb_selftest(&t);
    printf("[MB] selftest %s: %d/%d\n\r", (f == 0) ? "ok" : "FAIL", t - f, t);
  }
  /* FDCAN1 internal loopback self-test (A5 software half, no transceiver so no pins used) */
  {
    int t = 0;
    int f = app_can_selftest(&t);
    printf("[CAN] loopback %s: %d/%d\n\r", (f == 0) ? "ok" : "FAIL", t - f, t);
  }
  app_time_init();   /* LSE+RTC; on reset-survival time is available immediately, cold start waits for SNTP */
  printf("[TIME] init: src=%s now=%lu\n\r",
         (app_time_source() == 1U) ? "rtc" : "none", (unsigned long)app_time_now());
  app_cli_init();    /* diagnostic CLI: VCP RX interrupt + prompt */
  printf("[RPC] waiting CM4 endpoint...\n\r");
  { int rpc_rc = app_rpc_init();   /* -1=OpenAMP init failed  -2=no CM4 announce in 8s (run degraded, late-bind backstop) */
    printf("[RPC] init %s (rc=%d)\n\r", (rpc_rc == 0) ? "ok" : "FAIL", rpc_rc); }
  uint32_t tick = 0;
  uint8_t ip_shown = 0;
  uint32_t mqtt_retry_at = 0;
  uint32_t conn_start = 0;
  uint32_t wd_last_pub = 0, wd_stale = 0;
  { extern void app_user_init(void); app_user_init(); }   /* YOUR application: create your FreeRTOS tasks here (app_user.c) */
  for(;;)
  {
    { static uint8_t p5done = 0;                              /* 757: storage triple boot self-test once */
      if (!p5done) { extern void app_p5_test_run(void); app_p5_test_run(); p5done = 1U;
                     extern void app_pf_init(void); app_pf_init(); } }   /* power-fail detection armed (after QSPI) */
    { extern void app_p0_hb_tick(void); app_p0_hb_tick(); }  /* 757: SRAM4 0x180 heartbeat (read by CM4 CLI stat) */
    app_iwdg_feed();   /* 32s watchdog (started in app_init.c): a hang/HardFault in this loop -> reset -> trial-period count bites a rollback */
#ifdef OTA_TEST_HANG   /* verification matrix #4: deliberately deadloop 10s after boot (no feed) -> IWDG bites at 32s -> reset loop -> 4th boot auto-rolls back */
    if (tick >= 20U) { printf("[TEST] deliberate hang NOW (IWDG bites in ~32s)\n\r"); for (;;) { } }
#endif
    { static uint8_t s_green = 0;   /* green LED = CM7 heartbeat; explicitly set to hold state, after toggle sync CM4 latches yellow LED inverted */
      s_green ^= 1U;
      if (s_green) { BSP_LED_On(LED_GREEN); } else { BSP_LED_Off(LED_GREEN); }
      app_rpc_led_sync(s_green); }
    app_cli_poll();   /* consume CLI input (echo/execute both in this task = legal printf zone) */
    app_rpc_poll();   /* pump RPMsg messages */
    { extern void app_mbcfg_poll(void); app_mbcfg_poll(); }   /* Modbus ports: first load / cloud queue / TCP service (contract §2/§5) */
    { extern void app_pf_poll(void); app_pf_poll(); }         /* power-fail black box: background pre-erase of the next slot (app_pwrfail.h) */
    { extern void app_datalog_poll(void); app_datalog_poll(); }   /* data recorder + event journal (Data_Logging_and_Event_Journal.md) */
#if APP_ENABLE_CLOUD
    if (s_mqtt_up && (s_desc_pub_err != 0)) { tcpip_callback(mqtt_desc_republish_cb, NULL); }   /* desc first publish ring-full failure = resend until success (dash gets new sn/name) */
#endif
    if ((tick % 20U) == 6U) { app_mb_time_push(); }   /* every 10s feed time to M4 -> broadcast onto the bus */
    if ((tick % 10U) == 4U) { app_rpc_hb(); }         /* every 5s RPMsg heartbeat (CM4 alive criterion) */
    if (!ip_shown && (ip4_addr_get_u32(netif_ip4_addr(&gnetif)) != 0U))
    {
      printf("[CM7] IP READY: %s (broker=%s)\n\r", ip4addr_ntoa(netif_ip4_addr(&gnetif)),
             (s_broker == BRK_AWS) ? "aws" : (s_broker == BRK_LAN) ? "lan" : "pub");
      ip_shown = 1;
#if APP_ENABLE_CLOUD
      tcpip_callback(sntp_start_cb, NULL);   /* once there's an IP, start time sync */
#endif
    }
#if APP_ENABLE_CLOUD
    /* SNTP first sync (callback on tcpip thread forbids printf, print moved here) */
    {
      static uint8_t time_printed = 0;
      if (!time_printed && (app_time_source() == 2U))
      {
        printf("[TIME] SNTP synced: %lu\n\r", (unsigned long)app_time_now());
        time_printed = 1;
      }
    }
#endif
#if APP_ENABLE_CLOUD
    /* MQTT: has IP and not connected -> try connecting to broker every 10s; connected -> send one status every 5s */
#ifndef OTA_TEST_NONET   /* verification matrix #5: once defined, never initiate a connection, simulating "new firmware broke the network stack" -> trial-period timeout rollback */
    if (ip_shown && !s_mqtt_up && !s_mqtt_connecting && (tick >= mqtt_retry_at))
    {
      s_mqtt_connecting = 1;
      conn_start = tick;
      mqtt_retry_at = tick + 20U;
      tcpip_callback(mqtt_connect_cb, NULL);
    }
#endif
    /* connect attempt undecided after 20s (SYN black hole must wait for TCP slow timeout) -> abort, count a failure to allow switching sides */
    if (!s_mqtt_up && s_mqtt_connecting && ((tick - conn_start) >= 40U))
    {
      tcpip_callback(mqtt_conn_abort_cb, NULL);
    }
#endif
    ota_poll();
    if (s_mbreg_pend)                          /* deferred mbr/mbw/datalog cmd from the MQTT downlink */
    {
      if (s_mbreg_cmd[0] == 'm')
      {
        extern void app_mb_reg_str(char rw, const char *args, char *out, uint16_t cap);
        app_mb_reg_str((s_mbreg_cmd[2] == 'w') ? 'w' : 'r', s_mbreg_cmd + 4,
                       s_mbreg_result, sizeof(s_mbreg_result));
        if ((s_mbreg_cmd[2] == 'w') && (strncmp(s_mbreg_result, "mbw ok", 6) == 0))
        {
          extern void app_log_event_src(const char *type, const char *src, const char *fmt, ...);
          app_log_event_src("CONFIG", "cloud", "%s -> %s", s_mbreg_cmd, s_mbreg_result);
        }
      }
      else if (strncmp(s_mbreg_cmd, "vent", 4) == 0)
      {
        extern int app_vent_cmd(const char *line, const char *src, char *out, uint16_t cap);
        (void)app_vent_cmd(s_mbreg_cmd, "cloud", s_mbreg_result, sizeof(s_mbreg_result));
      }
      else if (strncmp(s_mbreg_cmd, "aer", 3) == 0)
      {
        extern int app_aer_cmd(const char *line, const char *src, char *out, uint16_t cap);
        (void)app_aer_cmd(s_mbreg_cmd, "cloud", s_mbreg_result, sizeof(s_mbreg_result));
      }
      else if (strncmp(s_mbreg_cmd, "dose", 4) == 0)
      {
        extern int app_dose_cmd(const char *line, const char *src, char *out, uint16_t cap);
        (void)app_dose_cmd(s_mbreg_cmd, "cloud", s_mbreg_result, sizeof(s_mbreg_result));
      }
      else if (strncmp(s_mbreg_cmd, "flow", 4) == 0)
      {
        extern int app_flowmon_cmd(const char *line, const char *src, char *out, uint16_t cap);
        (void)app_flowmon_cmd(s_mbreg_cmd, "cloud", s_mbreg_result, sizeof(s_mbreg_result));
      }
      else if ((strncmp(s_mbreg_cmd, "pulse", 5) == 0) || (strncmp(s_mbreg_cmd, "hsdi ", 5) == 0))
      {
        extern int app_pulse_cmd(const char *line, const char *src, char *out, uint16_t cap);
        extern int app_hsdi_cmd(const char *line, const char *src, char *out, uint16_t cap);
        if (!app_pulse_cmd(s_mbreg_cmd, "cloud", s_mbreg_result, sizeof(s_mbreg_result)))
        { (void)app_hsdi_cmd(s_mbreg_cmd, "cloud", s_mbreg_result, sizeof(s_mbreg_result)); }
      }
      else if (strncmp(s_mbreg_cmd, "netcfg", 6) == 0)
      {
        (void)app_netcfg_cmd(s_mbreg_cmd, s_mbreg_result, sizeof(s_mbreg_result));
        if (strncmp(s_mbreg_result, "netcfg saved", 12) == 0)
        {
          extern void app_log_event_src(const char *type, const char *src, const char *fmt, ...);
          app_log_event_src("CONFIG", "cloud", "%s -> %s", s_mbreg_cmd, s_mbreg_result);
        }
      }
      else
      {
        extern int app_datalog_cmd(const char *line, const char *src, char *out, uint16_t cap);
        (void)app_datalog_cmd(s_mbreg_cmd, "cloud", s_mbreg_result, sizeof(s_mbreg_result));
      }
      s_mbreg_pend = 0;
      s_status_dirty = 1;                      /* result rides the next heartbeat ("cmdr") */
    }
    if (s_reboot_cnt != 0U)                    /* cloud "reboot", deferred past the QoS1 PUBACK (see downlink handler) */
    {
      if (s_reboot_cnt == 5U)                  /* first beat: journal while later beats can still flush it to disk */
      {
        extern void app_log_event_src(const char *type, const char *src, const char *fmt, ...);
        app_log_event_src("SYSTEM", "cloud", "reboot cmd");
      }
      if (--s_reboot_cnt == 0U)
      { extern void app_reset(const char *reason); app_reset(s_reboot_why); }
    }
#if APP_ENABLE_CLOUD
    /* Report-on-change for the application's I/O: a physical input moving or the logic driving
     * an output must not wait for the 5 s periodic. Sampled here so it publishes in this very
     * tick. Counters are deliberately not compared — they move continuously once pulses arrive
     * and the 500 ms tick already bounds how often anything is sent. */
    {
      static uint8_t  s_last_hsdi = 0xFFU;
      static uint16_t s_last_rly  = 0xFFFFU;
      uint8_t  h = app_demo_hsdi_bits();
      uint16_t r = app_demo_relays();
      { extern uint8_t app_diag_di_changed(void);   /* backplane EX_16DI bitmaps join the same
                                                     * report-on-change policy (2026-08-13) */
        if (s_mqtt_up && app_diag_di_changed()) { s_status_dirty = 1; } }
      if ((h != s_last_hsdi) || (r != s_last_rly))
      {
        s_last_hsdi = h; s_last_rly = r; s_status_dirty = 1;
      }
    }
    /* status publish: periodic (every 5s) + report-on-change (s_status_dirty: state change sends immediately, <=500ms, doesn't wait 5s) */
    if (s_mqtt_up && (((tick % 10U) == 0U) || s_status_dirty))
    {
      extern volatile uint32_t eth_irq_count;
      char mb_str[48];
      { extern void app_mbcfg_hb(char *out, int cap); app_mbcfg_hb(mb_str, sizeof(mb_str)); }
      s_status_dirty = 0;
      /* health block (dash: "working" vs "working while sick", 2026-07-25): cpu%/task table/
       * min stack headroom from app_diag, plus last reset cause and previous-run fault record */
      { extern void app_diag_hb(char *o7, uint16_t cap7, char *o4, uint16_t cap4,
                                uint8_t *c7, uint8_t *c4, uint32_t *sm);
        extern void app_diag_fault_str(char *o, uint16_t cap);
        extern void app_diag_mtsk(char *o, uint16_t cap);
        extern void app_diag_mdata(char *o, uint16_t cap);
        extern const char *app_diag_reset_cause(void);
        static char tsk7_str[300]; static char tsk4_str[320];
        static char flt_str[48]; static char mtsk_str[520]; static char mdata_str[768]; /* PH_EC 8 fields + EX_16DI di+16 counters per module */
        uint8_t cpu7 = 0, cpu4 = 0; uint32_t stkmin = 0;
        app_diag_hb(tsk7_str, sizeof(tsk7_str), tsk4_str, sizeof(tsk4_str), &cpu7, &cpu4, &stkmin);
        app_diag_fault_str(flt_str, sizeof(flt_str));
        app_diag_mtsk(mtsk_str, sizeof(mtsk_str));   /* raw JSON fragment: ,"m2cpu":.. per module */
        app_diag_mdata(mdata_str, sizeof(mdata_str));   /* PH_EC engineering values, fresh each beat */
        { extern uint8_t app_diag_mods_dirty(void);  /* module joined/left -> desc gains/loses its card */
          if (app_diag_mods_dirty()) { desc_build(); s_desc_pub_err = 1; tcpip_callback(mqtt_desc_republish_cb, NULL); } }
      { extern uint32_t app_se_hs608_count(void); extern uint32_t app_se_hs608_fail(void);
        snprintf(s_pub_buf, sizeof(s_pub_buf),
               "{\"ver\":\"%s\",\"bkr\":\"%s\",\"sec\":\"%s\",\"hs608\":%lu,\"hs608f\":%lu,\"dsc\":%d,\"mqfix\":%u,\"mqk\":%u,\"mqkc\":%u,\"bank\":%lu,\"ota\":\"%s\",\"evt\":\"%s\",\"fws\":\"%s\",\"fwk2\":\"%s\",\"time\":%lu,\"recv\":%lu,\"tick\":%lu,\"rxirq\":%lu,\"heap\":%u,\"pub_ok\":%lu,\"beep\":%d,\"temp\":%d,\"hsdi\":%u,\"c0\":%lu,\"c1\":%lu,\"c2\":%lu,\"c3\":%lu,\"c4\":%lu,\"g1\":%u,\"g2\":%u,\"g3\":%u,\"g4\":%u,\"ga\":%u,\"flow\":%lu,\"flowr\":%u,\"rly\":%u,\"vent\":%u,\"ventm\":\"%s\",\"aer\":%u,\"aerm\":\"%s\",\"dose\":\"%s\",\"mb\":\"%s\",\"p5\":\"%s\",\"cpu\":%u,\"cpu4\":%u,\"stkmin\":%lu,\"tsk7\":\"%s\",\"tsk4\":\"%s\",\"rst\":\"%s\",\"flt\":\"%s\",\"cmdr\":\"%s\",\"dlog\":\"%s\",\"wda\":\"%s\",\"mqa\":\"%s\",\"bf\":\"%s\"%s%s}",
               FW_VERSION, (s_broker == BRK_AWS) ? "aws" : (s_broker == BRK_LAN) ? "lan" : "pub",
               (s_clone == APP_AC_GENUINE) ? "ok" : (s_clone == APP_AC_ABSENT) ? "nose" : "clone",
               (unsigned long)app_se_hs608_count(), (unsigned long)app_se_hs608_fail(), (int)s_desc_pub_err,
               (unsigned)s_mq_autopsies, (unsigned)s_mq_kicks, (unsigned)s_mq_kick_cures,
               (unsigned long)(ota_swap_active() + 1U), ota_state_str(), ota_last_evt(),
               ota_signer_str(), app_fwkey2_fp(),
               (unsigned long)app_time_now(),
               (unsigned long)ota_recv(),
               (unsigned long)tick, (unsigned long)eth_irq_count,
               (unsigned int)xPortGetFreeHeapSize(), (unsigned long)s_mqtt_pub_ok,
               (int)({ extern uint8_t app_beep_get(void); app_beep_get(); }), app_temp_read(),
               (unsigned)app_demo_hsdi_bits(),
               (unsigned long)app_demo_count(0), (unsigned long)app_demo_count(1),
               (unsigned long)app_demo_count(2), (unsigned long)app_demo_count(3),
               (unsigned long)app_demo_count(4),
               (unsigned)({ extern uint16_t app_gutter_rate_dlmin(uint8_t); app_gutter_rate_dlmin(0); }),
               (unsigned)({ extern uint16_t app_gutter_rate_dlmin(uint8_t); app_gutter_rate_dlmin(1); }),
               (unsigned)({ extern uint16_t app_gutter_rate_dlmin(uint8_t); app_gutter_rate_dlmin(2); }),
               (unsigned)({ extern uint16_t app_gutter_rate_dlmin(uint8_t); app_gutter_rate_dlmin(3); }),
               (unsigned)({ extern uint8_t app_flowmon_alarms(void); app_flowmon_alarms(); }),
               (unsigned long)({ extern uint32_t app_flow_dl(void); app_flow_dl(); }),
               (unsigned)({ extern uint16_t app_flow_rate_dlmin(void); app_flow_rate_dlmin(); }),
               (unsigned)app_demo_relays(),
               (unsigned)({ extern uint8_t app_vent_stage(void); app_vent_stage(); }),
               ({ extern const char *app_vent_mode(void); app_vent_mode(); }),
               (unsigned)({ extern uint8_t app_aer_state(void); app_aer_state(); }),
               ({ extern const char *app_aer_mode(void); app_aer_mode(); }),
               ({ extern const char *app_dose_hb(void); app_dose_hb(); }),
               mb_str, app_p5_str(),
               (unsigned)cpu7, (unsigned)cpu4, (unsigned long)stkmin,
               tsk7_str, tsk4_str, app_diag_reset_cause(), flt_str, s_mbreg_result,
               ({ extern const char *app_datalog_hb(void); app_datalog_hb(); }), s_wda, s_autopsy,
               ({ extern const char *app_diag_boot_ring_str(void); app_diag_boot_ring_str(); }),
               mtsk_str, mdata_str); } }
      tcpip_callback(mqtt_pub_cb, NULL);
    }
    /* publish watchdog: only at periodic points (every 5s) evaluate whether pub_ok has stalled (half-open TCP) -> force reconnect.
     * Independent of the "report-on-change" above, so on-change publishes don't disturb the stale count. */
    if (s_mqtt_up && ((tick % 10U) == 0U))
    {
      if (s_mqtt_pub_ok != wd_last_pub)
      {
        wd_last_pub = s_mqtt_pub_ok;
        wd_stale = 0;
        if (s_kick_armed) { s_kick_armed = 0; s_mq_kick_cures++; }   /* stage-1 kick unwedged it: no reconnect needed */
      }
      else if (wd_stale < 2U)   /* stages 1+2 (5s/10s stalled): gentle kick — flush stranded unsent TCP segments */
      {
        if (wd_stale == 0U) { s_mq_kicks++; s_kick_armed = 1U;
                              s_mmc_tx_stall = ETH->MMCTPCGR; }   /* count once per stall; MAC TX baseline for the wda mtx delta */
        wd_stale++;
        tcpip_callback(mqtt_kick_cb, NULL);
      }
      else if (++wd_stale >= 3U)
      {
        s_kick_armed = 0;
        wd_stale = 0;
        /* Wedge autopsy (2026-08-23): five pcap-verified cases show a 15 s total TX halt with
         * the TCP pipe empty and fully ACKed while the ring still holds data — the halt lives
         * between the MQTT ring and tcp_write. Photograph allocator/ring/req state BEFORE the
         * teardown; the snapshot rides every following heartbeat as "wda". Racy reads of
         * tcpip-owned structs are acceptable here: the wedge state has been static for 15 s. */
        if (s_mqtt != NULL)
        {
          uint8_t nreq = 0;
          struct mqtt_request_t *r = s_mqtt->pend_req_queue;
          while ((r != NULL) && (nreq < 9U)) { nreq++; r = r->next; }
          s_wda_cnt++;
          snprintf(s_wda, sizeof(s_wda),
                   "#%u t=%lu ring=%u req=%u snd=%u st=%u perr=%d lmem=%u/%u/e%u seg=%u/e%u pbuf=%u/e%u mtx=+%lu dsr=%04lx %s",
                   (unsigned)s_wda_cnt, (unsigned long)tick,
                   (unsigned)((u16_t)(s_mqtt->output.put - s_mqtt->output.get)), (unsigned)nreq,
                   (unsigned)((s_mqtt->conn != NULL) ? altcp_sndbuf(s_mqtt->conn) : 0U),
                   (unsigned)s_mqtt->conn_state, (int)s_diag_pub_last,
                   (unsigned)lwip_stats.mem.used, (unsigned)lwip_stats.mem.max, (unsigned)lwip_stats.mem.err,
                   (unsigned)lwip_stats.memp[MEMP_TCP_SEG]->used, (unsigned)lwip_stats.memp[MEMP_TCP_SEG]->err,
                   (unsigned)lwip_stats.memp[MEMP_PBUF_POOL]->used, (unsigned)lwip_stats.memp[MEMP_PBUF_POOL]->err,
                   (unsigned long)(ETH->MMCTPCGR - s_mmc_tx_stall), (unsigned long)(ETH->DMADSR & 0xFFFFU),
                   s_kda);
        }
        printf("[MQTT] pub watchdog: half-open suspected, force reconnect\n\r");
        printf("[MQTT] wda: %s\n\r", s_wda);
        tcpip_callback(mqtt_force_reconnect_cb, NULL);
      }
    }
    /* offline autopsy (observation only): cloud down for 120s straight despite the 10s retry
     * cadence -> photograph the client + both allocators, touch NOTHING. The 2026-07-24 recovery
     * watchdog that used to live here (client rebuild / TLS-pool reset) was withdrawn same day:
     * the true root cause is an lwip-heap leak on the connect-fail path, and every forced
     * rebuild cycle leaked MORE (plus the pool reset wedged the client at NULL) — the cure was
     * worse than the disease. Instrumentation stays: it is what exposed the real culprit. */
    {
      static uint32_t s_offline_ticks = 0;
      static uint32_t s_offline_total = 0;   /* ticks since this offline episode began (heal-ladder clock) */
      if (s_mqtt_up || !ip_shown)
      {
        s_offline_ticks = 0;
        s_offline_total = 0;
        s_mmc_tx_base = ETH->MMCTPCGR;   /* keep the MAC TX baseline fresh so autopsy #1's delta covers only offline time */
      }
      else if (++s_offline_ticks >= 240U)
      {
        s_offline_ticks = 0;
        size_t pcur = 0, pmax = 0;
        { extern void app_tls_pool_usage(size_t *cur, size_t *maxu); app_tls_pool_usage(&pcur, &pmax); }
        /* lwip allocators (the ERR_MEM source): heap used/err + TCP-PCB pool used/err.
         * A climbing used or monotonically rising err here = the lwip-heap leak is active. */
        unsigned lmem_u = (unsigned)lwip_stats.mem.used, lmem_e = (unsigned)lwip_stats.mem.err;
        unsigned tpcb_u = (unsigned)lwip_stats.memp[MEMP_TCP_PCB]->used;
        unsigned tpcb_e = (unsigned)lwip_stats.memp[MEMP_TCP_PCB]->err;
        /* MAC hardware TX-frame delta over this 120s window: the retry loop generates SYNs and
         * ARP requests the whole time, so tx=+0 convicts the MAC/DMA TX path while a normal
         * count convicts the wire/bridge beyond it (2026-08-25 mute discriminator). */
        uint32_t mmc_now = ETH->MMCTPCGR;
        uint32_t mmc_d   = mmc_now - s_mmc_tx_base;
        s_mmc_tx_base = mmc_now;
        if (s_mqtt != NULL)
        {
          snprintf(s_autopsy, sizeof(s_autopsy),
                   "st=%u pcb=%d tlspool=%u/%uK lmem=%u/e%u tpcb=%u/e%u cerr=%d cing=%u fails=%u tx=+%lu dsr=%04lx",
                   (unsigned)s_mqtt->conn_state, (int)(s_mqtt->conn != NULL),
                   (unsigned)pcur, (unsigned)(pmax / 1024U),
                   lmem_u, lmem_e, tpcb_u, tpcb_e,
                   (int)s_diag_conn_err, (unsigned)s_mqtt_connecting, (unsigned)s_conn_fails,
                   (unsigned long)mmc_d, (unsigned long)(ETH->DMADSR & 0xFFFFU));
        }
        else
        {
          snprintf(s_autopsy, sizeof(s_autopsy),
                   "client=NULL tlspool=%u/%uK lmem=%u/e%u tpcb=%u/e%u cerr=%d tx=+%lu dsr=%04lx",
                   (unsigned)pcur, (unsigned)(pmax / 1024U),
                   lmem_u, lmem_e, tpcb_u, tpcb_e, (int)s_diag_conn_err,
                   (unsigned long)mmc_d, (unsigned long)(ETH->DMADSR & 0xFFFFU));
        }
        s_mq_autopsies++;
        printf("[MQTT] autopsy #%u: %s\n\r", (unsigned)s_mq_autopsies, s_autopsy);
      }
      /* Heal ladder (2026-08-25 four-hour reconnect mute): the 08-22 backplane lesson applied
       * to the cloud link — a one-shot recovery whose failure is swallowed must escalate on its
       * own, never wait for a human. Rung 1 = the 10s connect retry + 20s abort above. */
      if (!s_mqtt_up && ip_shown)
      {
        s_offline_total++;
        if ((s_offline_total % 720U) == 0U)   /* rung 2 every 6 min: re-announce ourselves */
        {
          printf("[MQTT] heal rung2: gratuitous ARP + DHCP renew (offline %lus)\n\r",
                 (unsigned long)(s_offline_total / 2U));
          tcpip_callback(net_refresh_cb, NULL);
        }
        /* rung 3 once per episode at 30 min: self reset. Guards: cloud must have worked this
         * boot (a board on a genuinely dead network must not reboot-loop) and at most 2 resets
         * per episode (PF_RETAIN budget, refilled on the next successful connect). Reset gives
         * the full cure chain: ETH re-init + DHCP broadcast + gratuitous ARP + link renegotiation. */
        if ((s_offline_total == 3600U) && s_cloud_was_up && (s_pf_mute_resets < 2U))
        {
          s_pf_mute_resets++;
          { extern void app_log_event_src(const char *type, const char *src, const char *fmt, ...);
            app_log_event_src("SYSTEM", "net", "cloud mute 30min -> self reset (#%u this episode)",
                              (unsigned)s_pf_mute_resets); }
          printf("[MQTT] heal rung3: cloud mute 30min -> self reset (#%u)\n\r", (unsigned)s_pf_mute_resets);
          s_reboot_why = "net heal rung3";
          s_reboot_cnt = 5;   /* reuse the cloud-reboot deferred path: journal flushes, then NVIC_SystemReset */
        }
      }
    }
#endif
    /* ~8s without a lease -> fall back to static (addresses in app_cfg.h) */
    if (!ip_shown && (tick == 16U))
    {
      app_netcfg_load();                     /* defaultTask: littlefs read before the tcpip thread uses the cached strings */
      tcpip_callback(set_static_ip_cb, NULL);
      printf("[CM7] DHCP timeout -> static fallback (DHCP retry on link cycle)\n\r");
    }
    /* fallback-state rescue, two triggers (2026-08-31 ruling supersedes the 07-17
     * "retry only on cable replug" rule — that guarded the half-broken PC-ICS DHCP, where
     * every periodic DISCOVER left a static ARP corpse; ICS is retired, and the board-2
     * Starlink stranding proved a rebooted board that misses its one DHCP window otherwise
     * sits on the fallback address forever):
     *  - link down->up (cable replug): one retry after 2s of stability;
     *  - PERIODIC every 60s while stranded: lease arrival -> mqtt_dhcp_bound_check
     *    switches to the dynamic address and rebuilds the MQTT connection. */
    {
      static uint8_t s_prev_link = 1, s_link_settle = 0;
      static uint16_t s_dhcp_period = 0;
      uint8_t link = netif_is_link_up(&gnetif) ? 1U : 0U;
      if (!s_prev_link && link && s_ip_fallback) { s_link_settle = 4; }   /* back from replug: try after 2s of stability */
      s_prev_link = link;
      if ((s_link_settle > 0U) && (--s_link_settle == 0U))
      {
        printf("[CM7] link restored -> one DHCP retry\n\r");
        tcpip_callback(dhcp_retry_cb, NULL);
      }
      if (!s_ip_fallback) { s_dhcp_period = 0; }
      else if (link && (++s_dhcp_period >= 120U))   /* 60s cadence while stranded (0.5s ticks) */
      {
        s_dhcp_period = 0;
        printf("[CM7] fallback: periodic DHCP retry\n\r");
        tcpip_callback(dhcp_retry_cb, NULL);
      }
    }
    if (app_netcfg_take_dirty() && s_ip_fallback)   /* netcfg set while stranded in fallback: re-apply + reconnect now */
    {
      printf("[CM7] netcfg changed in fallback -> re-apply\n\r");
      tcpip_callback(set_static_ip_cb, NULL);
#if APP_ENABLE_CLOUD
      tcpip_callback(mqtt_force_reconnect_cb, NULL);
#endif
    }
#if APP_ENABLE_CLOUD
    mqtt_dhcp_bound_check();
#endif
    if ((tick & 1U) == 0U)
    {
      extern volatile uint32_t eth_irq_count;
      extern volatile uint32_t dbg_sem_wake, dbg_rx_frames, dbg_input_err, dbg_alloc_bad;
      printf("[CM7] tick=%lu %s link=%s rxirq=%lu wake=%lu frm=%lu ierr=%lu abad=%lu heap=%u\n\r",
             (unsigned long)tick,
             app_rpc_alive() ? "RPC-OK" : "CM4-NO-RESP",
             netif_is_link_up(&gnetif) ? "UP" : "DOWN",
             (unsigned long)eth_irq_count,
             (unsigned long)dbg_sem_wake, (unsigned long)dbg_rx_frames,
             (unsigned long)dbg_input_err, (unsigned long)dbg_alloc_bad,
             (unsigned int)xPortGetFreeHeapSize());
    }
    tick++;
    osDelay(500);
  }
}
