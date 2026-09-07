/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : Target/lwipopts.h
  * Description        : This file overrides LwIP stack default configuration
  *                      done in opt.h file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion --------------------------------------*/
#ifndef __LWIPOPTS__H__
#define __LWIPOPTS__H__

#include "main.h"

/*-----------------------------------------------------------------------------*/
/* Current version of LwIP supported by CubeMx: 2.2.1 -*/
/*-----------------------------------------------------------------------------*/

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */
#define MQTT_OUTPUT_RINGBUF_SIZE 8192  /* must hold the largest desc (~5KB with per-module dash groups incl. PH_EC engineering points, 2026-08-09) plus a concurrent heartbeat/SUBSCRIBE; a too-small ring silently fails desc publish with ERR_MEM (dash then shows a stale sn / missing module cards — bitten twice: 07-18 sn, 08-09 module cards) */
/* CubeMX does not generate MEM_SIZE -> opt.h defaults to only 1600B; the ETH driver holds up to 4 in-flight TX pbufs waiting for descriptor recycling,
 * 1600B is bound to run dry and deadlock as "alloc fails -> don't send -> don't recycle".
 * heap is at LWIP_RAM_HEAP_POINTER=0x30030000 (upper half of SRAM2, 64KB). Corresponds to LWIP->MEM_SIZE in CubeMX.
 * 2026-09-03: 16K -> 56K. Field evidence (608a, 24 min cloud mute): heap peak 16120/16384 with alloc errors
 * climbing on both boards (e32 / e95) and a connect attempt dying with ERR_MEM; a TLS reconnect burst under
 * a lossy uplink needs headroom. The OTA ring that used to sit at 0x30038000 moved to SRAM3 (app_ota.c), so
 * the heap may grow to 0x3003E000 (8K overflow sentinel gap kept up to the SRAM2 end 0x30040000). Stays inside
 * MPU region 2 (0x30020000/128K non-cacheable): ETH DMA reads TX pbufs straight from this heap. */
#define MEM_SIZE (56 * 1024)

/* lwIP memp pools (static .bss, DTCM). Defaults are the opt.h minimums sized for a single socket toy:
 * MEMP_NUM_TCP_PCB 5 was seen exhausted in the field (mqa "tpcb=5/e25": one connecting PCB + zombies in
 * FIN_WAIT/TIME_WAIT left by watchdog reconnects on a lossy link, Modbus TCP clients share the same pool),
 * turning every reconnect into an instant ERR_MEM until a zombie timed out. 2026-09-03: 5 -> 16, segments
 * and pbuf headers scaled with the bigger heap (~3K more .bss). */
#define MEMP_NUM_TCP_PCB        16
#define MEMP_NUM_TCP_SEG        64
#define MEMP_NUM_PBUF           32

/* mTLS: ALTCP = LwIP's "pluggable transport layer"; once enabled the mqtt client can transparently run over TLS;
 * glue source in Middlewares/.../altcp_tls/, mbedTLS config in App/app_mbedtls_config.h */
#define LWIP_ALTCP              1
#define LWIP_ALTCP_TLS          1
#define LWIP_ALTCP_TLS_MBEDTLS  1

/* DNS: connecting to AWS by domain name requires resolution; CubeMX LWIP defaults it off. dns_gethostbyname/dns_setserver depend on this */
#define LWIP_DNS                1

/* MEMP_NUM_SYS_TIMEOUT: if undefined it defaults to LWIP_NUM_SYS_TIMEOUT_INTERNAL,
 * i.e. counts only LwIP's internal periodic timers (ARP/TCP/DNS/DHCP×2/IGMP...), giving the application layer "zero headroom".
 * But our mqtt client hangs a keepalive periodic timer via sys_timeout on every TCP establishment (mqtt.c:1085),
 * which is not in the internal count. When the internal timers are full (especially with DNS on connecting to AWS + frequent broker-switch reconnects), the app hanging another
 * timer -> memp_malloc(MEMP_SYS_TIMEOUT) returns NULL -> assertion blows up in the tcpip thread (timeouts.c:190),
 * the networking thread dies, and the board's network dies right after it gets an IP on boot. Explicitly give ample headroom (steady-state uses ~5, giving 12 = more than double headroom). */
#define MEMP_NUM_SYS_TIMEOUT    12

/* SNTP time service: resolves an NTP pool by domain name, hands the sync result to App/app_time.c (anchor + RTC keep-alive).
 * sntp.c copied from FW_H7_V1.13.0 into src/apps/sntp (only the header was originally in the tree), build hooked in app_tls.cmake.
 * uses 1 sys_timeout (within the 12 headroom above). */
#define SNTP_SERVER_DNS         1
/* 2026-09-02 field finding: the greenhouse board's RTC ticks at ~45-55% of real time
 * with hour-to-hour jitter (LSE crystal oscillating sick — outdoor-cabinet humidity on
 * the 32k crystal is the prime suspect; PRER ruled out: a divider error would be an
 * exact jitter-free ratio). Every SNTP sync rewrites the calendar successfully, so a
 * 15-min poll caps the accumulated RTC error at ~7 min instead of the default hour's
 * ~27 min — that error only ever surfaces as the boot-seed clock before first sync. */
#define SNTP_UPDATE_DELAY       900000
#define SNTP_SET_SYSTEM_TIME(sec) \
  do { extern void app_time_sntp_cb(unsigned long s_); app_time_sntp_cb(sec); } while (0)
/* USER CODE END 0 */

#ifdef __cplusplus
 extern "C" {
#endif

/* STM32CubeMX Specific Parameters (not defined in opt.h) ---------------------*/
/* Parameters set in STM32CubeMX LwIP Configuration GUI -*/
/*----- WITH_RTOS enabled (Since FREERTOS is set) -----*/
#define WITH_RTOS 1
/*----- CHECKSUM_BY_HARDWARE enabled -----*/
#define CHECKSUM_BY_HARDWARE 1
/*-----------------------------------------------------------------------------*/

/* LwIP Stack Parameters (modified compared to initialization value in opt.h) -*/
/* Parameters set in STM32CubeMX LwIP Configuration GUI -*/
/*----- Value in opt.h for LWIP_DHCP: 0 -----*/
#define LWIP_DHCP 1
/*----- Default value in ETH configuration GUI in CubeMx: 1524 -----*/
#define ETH_RX_BUFFER_SIZE 1536
/*----- Value in opt.h for MEM_ALIGNMENT: 1 -----*/
#define MEM_ALIGNMENT 4
/*----- Default Value for H7 devices: 0x30004000 -----*/
#define LWIP_RAM_HEAP_POINTER 0x30030000 /* manual edit: original 0x30004000 = D2 SRAM1 = CM4's RAM (0x10000000 alias)! the two cores step on each other, crashes on receive. Moved to CM7-exclusive upper half of SRAM2. WARNING: change the CubeMX LWIP parameter to match, otherwise regeneration reverts it */
/*----- Value supported for H7 devices: 1 -----*/
#define LWIP_SUPPORT_CUSTOM_PBUF 1
/*----- Value in opt.h for LWIP_ETHERNET: LWIP_ARP || PPPOE_SUPPORT -*/
#define LWIP_ETHERNET 1
/*----- Value in opt.h for LWIP_DNS_SECURE: (LWIP_DNS_SECURE_RAND_XID | LWIP_DNS_SECURE_NO_MULTIPLE_OUTSTANDING | LWIP_DNS_SECURE_RAND_SRC_PORT) -*/
#define LWIP_DNS_SECURE 7
/*----- Value in opt.h for TCP_SND_QUEUELEN: (4*TCP_SND_BUF + (TCP_MSS - 1))/TCP_MSS -----*/
#define TCP_SND_QUEUELEN 9
/*----- Value in opt.h for TCP_SNDLOWAT: LWIP_MIN(LWIP_MAX(((TCP_SND_BUF)/2), (2 * TCP_MSS) + 1), (TCP_SND_BUF) - 1) -*/
#define TCP_SNDLOWAT 1071
/*----- Value in opt.h for TCP_SNDQUEUELOWAT: LWIP_MAX(TCP_SND_QUEUELEN)/2, 5) -*/
#define TCP_SNDQUEUELOWAT 5
/*----- Value in opt.h for TCP_WND_UPDATE_THRESHOLD: LWIP_MIN(TCP_WND/4, TCP_MSS*4) -----*/
#define TCP_WND_UPDATE_THRESHOLD 536
/*----- Value in opt.h for LWIP_NETIF_LINK_CALLBACK: 0 -----*/
#define LWIP_NETIF_LINK_CALLBACK 1
/*----- Value in opt.h for TCPIP_THREAD_STACKSIZE: 0 -----*/
#define TCPIP_THREAD_STACKSIZE 16384 /* TLS handshake (mbedTLS ECC) + 608A soft-I2C signing need >8K of stack; 16K leaves headroom. Keep CubeMX in sync */
/*----- Value in opt.h for TCPIP_THREAD_PRIO: 1 -----*/
#define TCPIP_THREAD_PRIO 24
/*----- Value in opt.h for TCPIP_MBOX_SIZE: 0 -----*/
#define TCPIP_MBOX_SIZE 6
/*----- Value in opt.h for SLIPIF_THREAD_STACKSIZE: 0 -----*/
#define SLIPIF_THREAD_STACKSIZE 1024
/*----- Value in opt.h for SLIPIF_THREAD_PRIO: 1 -----*/
#define SLIPIF_THREAD_PRIO 3
/*----- Value in opt.h for DEFAULT_THREAD_STACKSIZE: 0 -----*/
#define DEFAULT_THREAD_STACKSIZE 1024
/*----- Value in opt.h for DEFAULT_THREAD_PRIO: 1 -----*/
#define DEFAULT_THREAD_PRIO 3
/*----- Value in opt.h for DEFAULT_UDP_RECVMBOX_SIZE: 0 -----*/
#define DEFAULT_UDP_RECVMBOX_SIZE 6
/*----- Value in opt.h for DEFAULT_TCP_RECVMBOX_SIZE: 0 -----*/
#define DEFAULT_TCP_RECVMBOX_SIZE 6
/*----- Value in opt.h for DEFAULT_ACCEPTMBOX_SIZE: 0 -----*/
#define DEFAULT_ACCEPTMBOX_SIZE 6
/*----- Value in opt.h for RECV_BUFSIZE_DEFAULT: INT_MAX -----*/
#define RECV_BUFSIZE_DEFAULT 2000000000
/*----- Value in opt.h for LWIP_STATS: 1 -----*/
/* 2026-07-24: enable ONLY mem + memp stats (heap free / per-pool used+err) so the offline-watchdog
 * autopsy can tell mbedTLS-pool exhaustion apart from lwip-memp/heap exhaustion — the real wedge
 * returned connect=ERR_MEM and either allocator could be the source. Other stat groups off = lean. */
#define LWIP_STATS 1
#define LWIP_STATS_DISPLAY 0
#define MEM_STATS 1
#define MEMP_STATS 1
#define LINK_STATS 0
#define ETHARP_STATS 0
#define IP_STATS 0
#define IPFRAG_STATS 0
#define ICMP_STATS 0
#define UDP_STATS 0
#define TCP_STATS 0
#define SYS_STATS 0
#define IGMP_STATS 0
#define MIB2_STATS 0
/*----- Value in opt.h for CHECKSUM_GEN_IP: 1 -----*/
#define CHECKSUM_GEN_IP 0
/*----- Value in opt.h for CHECKSUM_GEN_UDP: 1 -----*/
#define CHECKSUM_GEN_UDP 0
/*----- Value in opt.h for CHECKSUM_GEN_TCP: 1 -----*/
#define CHECKSUM_GEN_TCP 0
/*----- Value in opt.h for CHECKSUM_GEN_ICMP6: 1 -----*/
#define CHECKSUM_GEN_ICMP6 0
/*----- Value in opt.h for CHECKSUM_CHECK_IP: 1 -----*/
#define CHECKSUM_CHECK_IP 0
/*----- Value in opt.h for CHECKSUM_CHECK_UDP: 1 -----*/
#define CHECKSUM_CHECK_UDP 0
/*----- Value in opt.h for CHECKSUM_CHECK_TCP: 1 -----*/
#define CHECKSUM_CHECK_TCP 0
/*----- Value in opt.h for CHECKSUM_CHECK_ICMP6: 1 -----*/
#define CHECKSUM_CHECK_ICMP6 0
/*-----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /*__LWIPOPTS__H__ */
