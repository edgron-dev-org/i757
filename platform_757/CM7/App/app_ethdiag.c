/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * app_ethdiag.c - Ethernet receive- and transmit-path diagnostics (observation only, 2026-10-09 / 10-10).
 *
 * Origin: field board i757-0003. link=UP, DHCP lease held, SYNs leaving the MAC, yet for 5 h the
 * board answered no ARP on its own LAN (a PC on the same switch got no reply); a cable re-plug
 * (PHY/MAC re-init through the link thread) cured it instantly. Nothing on the console could say
 * WHERE frames stopped, so this module makes the path observable stage by stage. It never resets
 * or re-initialises anything - a self-heal that fires on a misread counter is a worse bug than
 * the one it hunts.
 *
 *   wire -> PHY    energy detect, symbol errors            sampled on the link thread every 10 s (MDIO owner)
 *   PHY  -> MAC    CRC / alignment / good unicast          MMC counters, free-running, read any time
 *   MAC  -> ring   RX FIFO overflow, missed, DMA dropped   MTL/DMA counters (clear-on-read: accumulated on the link thread)
 *   ring -> lwIP   frames handed to tcpip_input, last age  netif->input wrapper (installed by app_ethdiag_init)
 *   lwIP           ARP/IP/ICMP/TCP/UDP packet counters     lwip_stats (LINK/ETHARP/IP/ICMP/UDP/TCP_STATS=1 in lwipopts.h)
 *
 * Reading the next deaf board: rx frames frozen + age climbing while rxirq still ticks = frames
 * die between MAC and lwIP (ring/driver); crcerr or symerr climbing = wire/PHY signal; energy=0
 * with link=UP = PHY lies; arp rx climbing but lwip arp xmit flat = lwIP sees requests and does
 * not answer (address/netif state); everything flat = nothing reaches the MAC at all.
 *
 * Transmit side (2026-10-10): every forced MQTT reconnect on 608a/0003 is a sent TLS record whose ACK
 * never comes, and the broker accepts the fresh connection within a second - so the wire was fine
 * and the question is whether that record ever left the MAC. The netif->linkoutput wrapper counts
 * frames lwIP hands to the driver (and error returns / blocking time); the MAC's own MMC good-TX
 * counter says how many reached the wire; the TX descriptor OWN bits say how many sit in the DMA.
 * lwIP-handed minus MAC-good growing by one at a stall = the segment died in the driver/DMA; the two
 * tracking = it was on the wire and the loss (or the ACK loss) is beyond this board.
 *
 * Threading: the input wrapper runs on EthIf; the linkoutput wrapper on tcpip; the PHY/DMA sampler runs on EthLink (the only MDIO
 * user, so it cannot interleave with LAN8742_GetLinkState and corrupt a link reading); printing
 * runs on defaultTask. Counters are 32-bit plain stores, read racily on purpose. */
#include <stdio.h>
#include <string.h>
#include "app_ethdiag.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "lan8742.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/memp.h"

extern struct netif gnetif;                   /* lwip.c */
extern ETH_HandleTypeDef heth;                /* ethernetif.c */
extern const struct memp_desc memp_RX_POOL;   /* ethernetif.c: LWIP_MEMPOOL_DECLARE(RX_POOL, ...) */

/* ---- ring -> lwIP (EthIf thread) ---- */
static netif_input_fn s_orig_input = NULL;
static volatile uint32_t s_rx_frames = 0, s_rx_bytes = 0, s_rx_arp = 0, s_in_err = 0;
static volatile uint32_t s_rx_last_tick = 0;   /* kernel ms of the last frame handed to lwIP (0 = none yet) */

/* ---- link-thread samples (EthLink thread) ---- */
static volatile uint32_t s_polls = 0;
static volatile uint16_t s_link_ups = 0, s_link_downs = 0;
static volatile uint8_t  s_link_was_up = 0;
static volatile uint32_t s_phy_addr = 0;
static volatile uint8_t  s_phy_ok = 0;
static volatile uint16_t s_bsr = 0, s_mcsr = 0, s_scsir = 0, s_physcsr = 0, s_secr = 0;
static volatile uint32_t s_sym_err = 0;        /* SECR deltas accumulated across the 16-bit roll-over (LAN8742 SECR is NOT clear-on-read; reset-cleared) */
static volatile uint16_t s_sym_last10 = 0;     /* SECR delta of the last 10 s window */
static volatile uint32_t s_dma_dropped = 0, s_mtl_ovf = 0, s_mtl_miss = 0;   /* clear-on-read hardware counters, accumulated */
static volatile uint32_t s_mtl_uf = 0;         /* MTLTQUR underflow packets (clear-on-read), accumulated */

/* ---- lwIP -> driver (tcpip thread) ---- */
static netif_linkoutput_fn s_orig_linkoutput = NULL;
static volatile uint32_t s_tx_frames = 0, s_tx_bytes = 0, s_tx_err = 0, s_tx_slow = 0, s_tx_max_ms = 0;
static volatile uint32_t s_tx_last_tick = 0;   /* kernel ms of the last frame handed to the driver (0 = none yet) */
static volatile uint32_t s_sample_tick = 0;    /* kernel ms of the last 10 s sample (0 = none yet) */

static err_t ethdiag_input(struct pbuf *p, struct netif *inp)   /* EthIf thread, replaces tcpip_input in gnetif.input */
{
  err_t e;
  const uint8_t *h = (const uint8_t *)p->payload;
  s_rx_frames++;
  s_rx_bytes += p->tot_len;
  if ((p->len >= 14U) && (h[12] == 0x08U) && (h[13] == 0x06U)) { s_rx_arp++; }   /* EtherType 0x0806 = ARP */
  s_rx_last_tick = osKernelGetTickCount();
  e = s_orig_input(p, inp);          /* tcpip_input: posts to the tcpip mailbox; on failure the caller frees p */
  if (e != ERR_OK) { s_in_err++; }
  return e;
}

static err_t ethdiag_linkoutput(struct netif *n, struct pbuf *p)   /* tcpip thread, replaces low_level_output in gnetif.linkoutput */
{
  uint32_t t0 = osKernelGetTickCount();
  err_t e = s_orig_linkoutput(n, p);        /* low_level_output: HAL_ETH_Transmit_IT, blocks up to ETHIF_TX_TIMEOUT when no descriptor is free */
  uint32_t dt = osKernelGetTickCount() - t0;
  s_tx_frames++;
  s_tx_bytes += p->tot_len;
  if (e != ERR_OK) { s_tx_err++; }
  if (dt >= 100U) { s_tx_slow++; }          /* waited for a TX descriptor: the DMA was not draining */
  if (dt > s_tx_max_ms) { s_tx_max_ms = dt; }
  s_tx_last_tick = osKernelGetTickCount();
  return e;
}

void app_ethdiag_init(void)
{
  if ((s_orig_input == NULL) && (gnetif.input != NULL))
  {
    s_orig_input = gnetif.input;
    gnetif.input = ethdiag_input;    /* single pointer store: safe against a concurrent EthIf call */
  }
  if ((s_orig_linkoutput == NULL) && (gnetif.linkoutput != NULL))
  {
    s_orig_linkoutput = gnetif.linkoutput;
    gnetif.linkoutput = ethdiag_linkoutput;   /* single pointer store: safe against a concurrent tcpip call */
  }
}

static uint32_t tx_desc_owned(void)   /* TX descriptors the DMA still owns (OWN bit set) - frames queued, not yet sent */
{
  uint32_t i, n = 0;
  for (i = 0; i < ETH_TX_DESC_CNT; i++)
  {
    const ETH_DMADescTypeDef *d = (const ETH_DMADescTypeDef *)heth.TxDescList.TxDesc[i];
    if ((d != NULL) && ((d->DESC3 & ETH_DMATXNDESCWBF_OWN) != 0U)) { n++; }
  }
  return n;
}

void app_ethdiag_link_poll(uint32_t phy_addr, int32_t link_state)   /* EthLink thread, every 100 ms */
{
  uint8_t up = (link_state > LAN8742_STATUS_LINK_DOWN) ? 1U : 0U;
  s_phy_addr = phy_addr;
  if (up != s_link_was_up)
  {
    if (up) { s_link_ups++; } else { s_link_downs++; }
    s_link_was_up = up;
  }
  if ((++s_polls % 100U) != 0U) { return; }   /* 100 ms cadence -> sample every 10 s */
  {
    uint32_t v;
    uint8_t ok = 1;
    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, LAN8742_BSR,     &v) == HAL_OK) { s_bsr     = (uint16_t)v; } else { ok = 0; }
    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, LAN8742_MCSR,    &v) == HAL_OK) { s_mcsr    = (uint16_t)v; } else { ok = 0; }
    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, LAN8742_SCSIR,   &v) == HAL_OK) { s_scsir   = (uint16_t)v; } else { ok = 0; }
    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, LAN8742_PHYSCSR, &v) == HAL_OK) { s_physcsr = (uint16_t)v; } else { ok = 0; }
    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, LAN8742_SECR,    &v) == HAL_OK)
    {
      uint16_t d = (uint16_t)((uint16_t)v - s_secr);   /* 16-bit roll-over safe; first sample = errors since PHY reset */
      s_secr = (uint16_t)v;
      s_sym_err += d;
      s_sym_last10 = d;
    }
    else { ok = 0; }
    s_phy_ok = ok;
  }
  /* RM0399: DMACMFCR.MFC[10:0] (packets dropped by the DMA, no descriptor / bus error) and
   * MTLRQMPOCR OVFPKTCNT[10:0] / MISPKTCNT[26:16] (RX FIFO overflow / flush) are cleared when read. */
  s_dma_dropped += (ETH->DMACMFCR & 0x7FFU);
  {
    uint32_t m = ETH->MTLRQMPOCR;
    s_mtl_ovf  += (m & ETH_MTLRQMPOCR_OVFPKTCNT) >> ETH_MTLRQMPOCR_OVFPKTCNT_Pos;
    s_mtl_miss += (m & ETH_MTLRQMPOCR_MISPKTCNT) >> ETH_MTLRQMPOCR_MISPKTCNT_Pos;
  }
  s_mtl_uf += (ETH->MTLTQUR & ETH_MTLTQUR_UFPKTCNT);   /* TX FIFO underflow packets, clear-on-read like the RX counters above */
  s_sample_tick = osKernelGetTickCount();
}

uint32_t app_ethdiag_rx_frames(void) { return s_rx_frames; }
uint32_t app_ethdiag_in_err(void)    { return s_in_err; }
uint32_t app_ethdiag_rx_age_s(void)
{
  uint32_t t = s_rx_last_tick;
  return ((t != 0U) ? (osKernelGetTickCount() - t) : osKernelGetTickCount()) / 1000U;
}
uint32_t app_ethdiag_tx_frames(void) { return s_tx_frames; }
uint32_t app_ethdiag_tx_err(void)    { return s_tx_err; }
uint32_t app_ethdiag_tx_age_s(void)
{
  uint32_t t = s_tx_last_tick;
  return ((t != 0U) ? (osKernelGetTickCount() - t) : osKernelGetTickCount()) / 1000U;
}
int app_ethdiag_tx_snap(char *o, uint32_t cap)
{
  return snprintf(o, cap, "tx=%lu/%lu own=%lu/%lu txe=%lu uf=%lu tps=%u",
                  (unsigned long)s_tx_frames, (unsigned long)ETH->MMCTPCGR,
                  (unsigned long)tx_desc_owned(), (unsigned long)heth.TxDescList.BuffersInUse,
                  (unsigned long)s_tx_err, (unsigned long)s_mtl_uf,
                  (unsigned)((ETH->DMADSR >> 12) & 0xFU));   /* RM0399 DMADSR.TPS0: 0 stopped, 1-3/7 running, 6 suspended */
}

const char *app_ethdiag_hb(void)   /* defaultTask: cloud status "eth" field */
{
  static char buf[256];
  snprintf(buf, sizeof(buf),
           "rx=%lu age=%lu arp=%lu ie=%lu pe=%u crc=%lu aln=%lu ovf=%lu miss=%lu drop=%lu sym=%lu en=%u flap=%u/%u"
           " tx=%lu/%lu txe=%lu txs=%lu uf=%lu col=%lu own=%lu",
           (unsigned long)s_rx_frames, (unsigned long)app_ethdiag_rx_age_s(), (unsigned long)s_rx_arp,
           (unsigned long)s_in_err, (unsigned)memp_RX_POOL.stats->err,
           (unsigned long)ETH->MMCRCRCEPR, (unsigned long)ETH->MMCRAEPR,
           (unsigned long)s_mtl_ovf, (unsigned long)s_mtl_miss, (unsigned long)s_dma_dropped,
           (unsigned long)s_sym_err, (unsigned)((s_mcsr & LAN8742_MCSR_ENERGYON) ? 1U : 0U),
           (unsigned)s_link_ups, (unsigned)s_link_downs,
           /* tx = frames lwIP handed to the driver / frames the MAC put on the wire (MMC good TX);
            * txe = driver refusals, txs = calls that waited >=100 ms for a descriptor, uf = TX FIFO
            * underflow, col = single+multiple collisions (half-duplex tell-tale), own = descriptors
            * the DMA still holds */
           (unsigned long)s_tx_frames, (unsigned long)ETH->MMCTPCGR, (unsigned long)s_tx_err, (unsigned long)s_tx_slow,
           (unsigned long)s_mtl_uf, (unsigned long)(ETH->MMCTSCGPR + ETH->MMCTMCGPR), (unsigned long)tx_desc_owned());
  return buf;
}

void app_ethdiag_print(void)   /* defaultTask: 'stat' */
{
  uint32_t now = osKernelGetTickCount();
  printf("eth: rx=%lu(%luKB) age=%lus arp=%lu inerr=%lu pool=%u/%u/e%u link up/down=%u/%u\n\r",
         (unsigned long)s_rx_frames, (unsigned long)(s_rx_bytes / 1024U), (unsigned long)app_ethdiag_rx_age_s(),
         (unsigned long)s_rx_arp, (unsigned long)s_in_err,
         (unsigned)memp_RX_POOL.stats->used, (unsigned)memp_RX_POOL.stats->max, (unsigned)memp_RX_POOL.stats->err,
         (unsigned)s_link_ups, (unsigned)s_link_downs);
  printf("tx : lwip=%lu(%luKB) err=%lu slow=%lu max=%lums age=%lus mac-good=%lu col=%lu/%lu fifo-uf=%lu desc-own=%lu inuse=%lu cur=%lu rel=%lu\n\r",
         (unsigned long)s_tx_frames, (unsigned long)(s_tx_bytes / 1024U), (unsigned long)s_tx_err, (unsigned long)s_tx_slow,
         (unsigned long)s_tx_max_ms, (unsigned long)app_ethdiag_tx_age_s(),
         (unsigned long)ETH->MMCTPCGR, (unsigned long)ETH->MMCTSCGPR, (unsigned long)ETH->MMCTMCGPR, (unsigned long)s_mtl_uf,
         (unsigned long)tx_desc_owned(), (unsigned long)heth.TxDescList.BuffersInUse,
         (unsigned long)heth.TxDescList.CurTxDesc, (unsigned long)heth.TxDescList.releaseIndex);
  printf("mac: txgood=%lu rxucast=%lu crcerr=%lu alnerr=%lu fifo-ovf=%lu fifo-miss=%lu dma-drop=%lu dmacsr=%08lx dmadsr=%08lx hal=st%u/e%lx/dma%lx\n\r",
         (unsigned long)ETH->MMCTPCGR, (unsigned long)ETH->MMCRUPGR,
         (unsigned long)ETH->MMCRCRCEPR, (unsigned long)ETH->MMCRAEPR,
         (unsigned long)s_mtl_ovf, (unsigned long)s_mtl_miss, (unsigned long)s_dma_dropped,
         (unsigned long)ETH->DMACSR, (unsigned long)ETH->DMADSR,
         (unsigned)heth.gState, (unsigned long)heth.ErrorCode, (unsigned long)heth.DMAErrorCode);
  if (s_sample_tick == 0U)
  {
    printf("phy: not sampled yet (link thread samples every 10s)\n\r");
  }
  else
  {
    printf("phy: addr=%lu %s bsr=%04x mcsr=%04x energy=%u scsir=%04x physcsr=%04x symerr=%lu(+%u/10s) sampled %lus ago\n\r",
           (unsigned long)s_phy_addr, s_phy_ok ? "ok" : "MDIO-FAIL",
           (unsigned)s_bsr, (unsigned)s_mcsr, (unsigned)((s_mcsr & LAN8742_MCSR_ENERGYON) ? 1U : 0U),
           (unsigned)s_scsir, (unsigned)s_physcsr,
           (unsigned long)s_sym_err, (unsigned)s_sym_last10, (unsigned long)((now - s_sample_tick) / 1000U));
  }
  printf("lwip: arp r%u/x%u/d%u/e%u ip r%u/x%u/d%u/rt%u/pr%u/ck%u icmp r%u/x%u tcp r%u/x%u/d%u/m%u/e%u udp r%u/x%u/d%u\n\r",
         (unsigned)lwip_stats.etharp.recv, (unsigned)lwip_stats.etharp.xmit, (unsigned)lwip_stats.etharp.drop, (unsigned)lwip_stats.etharp.err,
         (unsigned)lwip_stats.ip.recv, (unsigned)lwip_stats.ip.xmit, (unsigned)lwip_stats.ip.drop,
         (unsigned)lwip_stats.ip.rterr, (unsigned)lwip_stats.ip.proterr, (unsigned)lwip_stats.ip.chkerr,
         (unsigned)lwip_stats.icmp.recv, (unsigned)lwip_stats.icmp.xmit,
         (unsigned)lwip_stats.tcp.recv, (unsigned)lwip_stats.tcp.xmit, (unsigned)lwip_stats.tcp.drop,
         (unsigned)lwip_stats.tcp.memerr, (unsigned)lwip_stats.tcp.err,
         (unsigned)lwip_stats.udp.recv, (unsigned)lwip_stats.udp.xmit, (unsigned)lwip_stats.udp.drop);
}
