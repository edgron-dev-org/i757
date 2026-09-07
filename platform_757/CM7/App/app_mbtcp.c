/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* app_mbtcp.c — Modbus TCP shell (CM7/LwIP raw API; contract docs/Universal_Modbus_Port_Config.md §5)
 * Thread model: all LwIP callbacks run on the tcpip thread (no printf / no RPMsg there); pending requests are
 * dispatched by defaultTask's app_mbtcp_service() via op9 to the CM4 board-level map (RPMsg single-user discipline),
 * the reply is sent back via tcpip_callback.
 * v1 convention: one question at a time per connection (Modbus TCP synchronous-client habit; a pipelined 2nd
 * question is parsed on the next segment); reply latency <= defaultTask tick (500ms), client timeout recommended >=2s.
 * Idle 120s kicks the line. */
#include <string.h>
#include <stdio.h>
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/ip_addr.h"
#include "cmsis_os.h"
#include "app_mbtcp.h"
#include "app_mbcfg.h"
#include "app_rpc.h"

#define MBT_PORT       502U
#define MBT_NCONN      2U      /* contract §5: 2 concurrent, reject the 3rd */
#define MBT_IDLE_S     120U
#define MBT_PDU_MAX    253U

typedef struct {
  struct tcp_pcb *pcb;
  uint8_t  rx[300];
  uint16_t fill;
  volatile uint8_t pend;       /* 1 = a complete request is waiting for defaultTask to process */
  uint8_t  tid[2], uid;
  uint8_t  pdu[MBT_PDU_MAX];
  uint16_t pdu_len;
  uint8_t  rsp[MBT_PDU_MAX + 7U];
  uint16_t rsp_len;
  uint32_t last_ms;
} mbt_conn_t;

static mbt_conn_t s_c[MBT_NCONN];
static struct tcp_pcb *s_listen = NULL;

/* ---- server: tcpip-thread side ---- */
static void conn_free(mbt_conn_t *c)
{
  if (c->pcb != NULL)
  {
    tcp_arg(c->pcb, NULL);
    tcp_recv(c->pcb, NULL);
    tcp_err(c->pcb, NULL);
    tcp_poll(c->pcb, NULL, 0);
    (void)tcp_close(c->pcb);
  }
  memset(c, 0, sizeof(*c));
}

static void conn_parse(mbt_conn_t *c)   /* MBAP framing (tcpip thread; leaves rx untouched while pend) */
{
  while (!c->pend && (c->fill >= 7U))
  {
    uint16_t len = (uint16_t)(((uint16_t)c->rx[4] << 8) | c->rx[5]);
    if ((c->rx[2] != 0U) || (c->rx[3] != 0U) || (len < 2U) || (len > (MBT_PDU_MAX + 1U)))
    {
      c->fill = 0;               /* non-Modbus protocol / bad header: drop buffer (no TCP resync, wait for peer to reconnect) */
      return;
    }
    uint16_t total = (uint16_t)(6U + len);
    if (c->fill < total) { return; }
    c->tid[0] = c->rx[0]; c->tid[1] = c->rx[1];
    c->uid = c->rx[6];
    c->pdu_len = (uint16_t)(len - 1U);
    memcpy(c->pdu, &c->rx[7], c->pdu_len);
    c->fill = (uint16_t)(c->fill - total);
    memmove(c->rx, &c->rx[total], c->fill);
    c->pend = 1;                 /* defaultTask takes over */
  }
}

static err_t srv_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
  mbt_conn_t *c = (mbt_conn_t *)arg;
  if ((p == NULL) || (err != ERR_OK))
  {
    if (p != NULL) { pbuf_free(p); }
    conn_free(c);
    return ERR_OK;
  }
  c->last_ms = sys_now();
  uint16_t room = (uint16_t)(sizeof(c->rx) - c->fill);
  uint16_t n = (uint16_t)pbuf_copy_partial(p, &c->rx[c->fill], (p->tot_len < room) ? p->tot_len : room, 0);
  c->fill = (uint16_t)(c->fill + n);
  tcp_recved(pcb, p->tot_len);
  pbuf_free(p);
  conn_parse(c);
  return ERR_OK;
}

static void srv_err(void *arg, err_t err)
{
  (void)err;
  mbt_conn_t *c = (mbt_conn_t *)arg;
  if (c != NULL) { c->pcb = NULL; memset(c, 0, sizeof(*c)); }   /* pcb already reclaimed by LwIP */
}

static err_t srv_poll(void *arg, struct tcp_pcb *pcb)
{
  mbt_conn_t *c = (mbt_conn_t *)arg;
  (void)pcb;
  if ((c != NULL) && ((sys_now() - c->last_ms) > (MBT_IDLE_S * 1000U))) { conn_free(c); }
  return ERR_OK;
}

static err_t srv_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  (void)arg;
  if ((err != ERR_OK) || (newpcb == NULL)) { return ERR_VAL; }
  if (!app_mbcfg_tcp_on()) { tcp_abort(newpcb); return ERR_ABRT; }
  for (unsigned i = 0; i < MBT_NCONN; i++)
  {
    if (s_c[i].pcb == NULL)
    {
      memset(&s_c[i], 0, sizeof(s_c[i]));
      s_c[i].pcb = newpcb;
      s_c[i].last_ms = sys_now();
      tcp_arg(newpcb, &s_c[i]);
      tcp_recv(newpcb, srv_recv);
      tcp_err(newpcb, srv_err);
      tcp_poll(newpcb, srv_poll, 20);   /* 20×500ms=10s idle-check tick */
      return ERR_OK;
    }
  }
  tcp_abort(newpcb);                     /* reject the 3rd connection (contract §5) */
  return ERR_ABRT;
}

static void listen_cb(void *a)
{
  (void)a;
  struct tcp_pcb *p = tcp_new();
  if (p == NULL) { return; }
  if (tcp_bind(p, IP_ADDR_ANY, MBT_PORT) != ERR_OK) { tcp_abort(p); return; }
  s_listen = tcp_listen(p);
  tcp_accept(s_listen, srv_accept);
}

void app_mbtcp_init(void)
{
  if (s_listen == NULL) { tcpip_callback(listen_cb, NULL); }
}

/* ---- defaultTask side: op9 adjudication + send-back ---- */
static void send_cb(void *a)   /* tcpip thread: put reply on the wire */
{
  mbt_conn_t *c = (mbt_conn_t *)a;
  if (c->pcb != NULL)
  {
    (void)tcp_write(c->pcb, c->rsp, c->rsp_len, TCP_WRITE_FLAG_COPY);
    (void)tcp_output(c->pcb);
  }
  c->rsp_len = 0;
  c->pend = 0;                 /* clear pending = allow parsing the next question (cleared on tcpip thread, same thread as conn_parse so no race) */
}

void app_mbtcp_service(void)
{
  for (unsigned i = 0; i < MBT_NCONN; i++)
  {
    mbt_conn_t *c = &s_c[i];
    if ((c->pcb == NULL) || (!c->pend) || (c->rsp_len != 0U)) { continue; }   /* rsp_len nonzero = send-back in flight */
    static uint8_t req[300], rsp[300];
    req[0] = 0x09U;
    memcpy(&req[1], c->pdu, c->pdu_len);
    uint16_t r = app_rpc_transact(req, (uint16_t)(1U + c->pdu_len), rsp, sizeof(rsp), 150);
    uint16_t pl;
    if ((r >= 2U) && (rsp[0] == 0U))
    {
      pl = (uint16_t)(r - 1U);
      memcpy(&c->rsp[7], &rsp[1], pl);
    }
    else                        /* CM4 no reply: gateway target-failed exception (0x0B) */
    {
      c->rsp[7] = (uint8_t)(c->pdu[0] | 0x80U);
      c->rsp[8] = 0x0BU;
      pl = 2;
    }
    c->rsp[0] = c->tid[0]; c->rsp[1] = c->tid[1];
    c->rsp[2] = 0; c->rsp[3] = 0;
    c->rsp[4] = (uint8_t)((pl + 1U) >> 8); c->rsp[5] = (uint8_t)((pl + 1U) & 0xFFU);
    c->rsp[6] = c->uid;         /* echo Unit ID back verbatim (contract §5) */
    c->rsp_len = (uint16_t)(7U + pl);
    tcpip_callback(send_cb, c);
  }
}

/* ---- client single transaction (defaultTask blocking; used by CLI mbpoll tcp) ---- */
typedef struct {
  ip_addr_t ip;
  uint16_t port;
  uint8_t  unit;
  const uint8_t *pdu;
  uint16_t pn;
  uint8_t  rx[300];
  uint16_t fill;
  uint8_t  rsp[MBT_PDU_MAX];
  volatile int16_t result;     /* 0=in progress >0=reply PDU length <0=error */
  struct tcp_pcb *pcb;
  uint16_t tid;
} mbt_cli_t;

static mbt_cli_t s_q;
static uint16_t s_tid = 1;

static void cli_finish(int16_t res)   /* tcpip thread */
{
  if (s_q.pcb != NULL)
  {
    tcp_arg(s_q.pcb, NULL);
    tcp_recv(s_q.pcb, NULL);
    tcp_err(s_q.pcb, NULL);
    (void)tcp_close(s_q.pcb);
    s_q.pcb = NULL;
  }
  s_q.result = res;
}

static err_t cli_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
  (void)arg;
  if ((p == NULL) || (err != ERR_OK))
  {
    if (p != NULL) { pbuf_free(p); }
    cli_finish(-2);
    return ERR_OK;
  }
  uint16_t room = (uint16_t)(sizeof(s_q.rx) - s_q.fill);
  uint16_t n = (uint16_t)pbuf_copy_partial(p, &s_q.rx[s_q.fill], (p->tot_len < room) ? p->tot_len : room, 0);
  s_q.fill = (uint16_t)(s_q.fill + n);
  tcp_recved(pcb, p->tot_len);
  pbuf_free(p);
  if (s_q.fill >= 7U)
  {
    uint16_t len = (uint16_t)(((uint16_t)s_q.rx[4] << 8) | s_q.rx[5]);
    if ((len >= 2U) && (s_q.fill >= (uint16_t)(6U + len)))
    {
      uint16_t pl = (uint16_t)(len - 1U);
      if (pl > sizeof(s_q.rsp)) { pl = sizeof(s_q.rsp); }
      memcpy(s_q.rsp, &s_q.rx[7], pl);
      cli_finish((int16_t)pl);
    }
  }
  return ERR_OK;
}

static void cli_err(void *arg, err_t err)
{
  (void)arg; (void)err;
  s_q.pcb = NULL;              /* pcb already reclaimed */
  s_q.result = -3;
}

static err_t cli_connected(void *arg, struct tcp_pcb *pcb, err_t err)
{
  (void)arg;
  if (err != ERR_OK) { cli_finish(-4); return ERR_OK; }
  uint8_t hdr[7];
  hdr[0] = (uint8_t)(s_q.tid >> 8); hdr[1] = (uint8_t)(s_q.tid & 0xFFU);
  hdr[2] = 0; hdr[3] = 0;
  hdr[4] = (uint8_t)((s_q.pn + 1U) >> 8); hdr[5] = (uint8_t)((s_q.pn + 1U) & 0xFFU);
  hdr[6] = s_q.unit;
  (void)tcp_write(pcb, hdr, 7, TCP_WRITE_FLAG_COPY);
  (void)tcp_write(pcb, s_q.pdu, s_q.pn, TCP_WRITE_FLAG_COPY);
  (void)tcp_output(pcb);
  return ERR_OK;
}

static void cli_start_cb(void *a)
{
  (void)a;
  struct tcp_pcb *p = tcp_new();
  if (p == NULL) { s_q.result = -5; return; }
  s_q.pcb = p;
  tcp_arg(p, NULL);
  tcp_recv(p, cli_recv);
  tcp_err(p, cli_err);
  if (tcp_connect(p, &s_q.ip, s_q.port, cli_connected) != ERR_OK) { cli_finish(-6); }
}

static void cli_abort_cb(void *a)
{
  (void)a;
  if (s_q.pcb != NULL) { tcp_abort(s_q.pcb); s_q.pcb = NULL; }
  s_q.result = -7;
}

int app_mbtcp_query(const char *ip_str, uint16_t port, uint8_t unit,
                    const uint8_t *pdu, uint16_t pn,
                    uint8_t *rsp_pdu, uint16_t cap, uint32_t timeout_ms)
{
  if ((s_q.result == 0) && (s_q.pcb != NULL)) { return -10; }   /* previous question not yet wrapped up */
  memset(&s_q, 0, sizeof(s_q));
  if (!ipaddr_aton(ip_str, &s_q.ip)) { return -11; }
  s_q.port = (port != 0U) ? port : MBT_PORT;
  s_q.unit = unit;
  s_q.pdu = pdu; s_q.pn = pn;
  s_q.tid = s_tid++;
  tcpip_callback(cli_start_cb, NULL);
  uint32_t waited = 0;
  while ((s_q.result == 0) && (waited < timeout_ms)) { osDelay(10); waited += 10U; }
  if (s_q.result == 0)
  {
    tcpip_callback(cli_abort_cb, NULL);
    while (s_q.result == 0) { osDelay(10); }   /* wait for abort to land (tcpip is guaranteed to run it) */
  }
  int16_t r = s_q.result;
  if (r <= 0) { return (int)r; }
  uint16_t n = ((uint16_t)r > cap) ? cap : (uint16_t)r;
  memcpy(rsp_pdu, s_q.rsp, n);
  return (int)n;
}
