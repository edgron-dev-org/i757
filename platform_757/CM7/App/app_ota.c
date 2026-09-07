/* SPDX-License-Identifier: LicenseRef-Edgron-Source-Available
 * Copyright (c) 2026 Edgron. See LICENSE at the SDK root. */
/* ota.c — A3 dual-Bank OTA implementation (v2: ring-buffer decoupled)
 * Protocol (MQTT dev/<type>/<sn>/dn/cmd text commands + .../dn/fw chunked binary):
 *   ota-begin <offset_hex> <size_dec> <crc32_hex>   prepare to receive a segment (first segment triggers a full bank erase)
 *   chunk message = 8B header['O','T','A','1', u32LE in-segment offset] + data (4KB, QoS1 throttled)
 *   ota-apply / ota-revert   swap bank + reset (swap is an involution, swapping again = rollback)
 *   ota-provision            one-time: BOOT_CM4_ADD0=0x08080000 + reset
 *   ota-sign <sig_hex>       firmware signature (ECDSA-P256/SHA256, over cm7||cm4) verify -> pass then write footer / allow swap
 *   broker-pub / broker-lan  force switch MQTT broker (public VPS / LAN PC) and reconnect
 * Architectural iron law: never write Flash in a tcpip-thread callback — the CPU stall during programming
 * fights the ETH RX path (IMPRECISERR HardFault). The callback only pushes data into the SPSC ring buffer (D2 SRAM2 bare region),
 * defaultTask's ota_poll() consumes it and does all Flash operations.
 * On H7, after SWAP the flash register set swaps with the mapping -> FLASH_BANK_2 always points to the 0x08100000 window = the non-running bank.
 */
#include "app_ota.h"
#include "app_mqtt.h"
#include "app_init.h"
#include "app_cfg.h"   /* APP_ENABLE_CLOUD */
extern void app_beep_set(uint8_t on);   /* buzzer (app_leds_beep.c, dash do.beep point) */
#include "app_certs.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "mbedtls/sha256.h"
#include "mbedtls/pk.h"
#include "mbedtls/ecp.h"
#include "app_identity.h"   /* app_fwkey2: board-registered customer signing key (second OTA verify key) */

#define OTA_WIN_BASE   0x08100000UL
#define OTA_BANK_SIZE  0x00100000UL
#define OTA_WORD       32U
#define OTA_CM4_BOOT   0x08080000UL
#define OTA_CM4_OFF    0x00080000UL

/* Standby-bank persistent footer (anti-brick gate v3 = anti-brick + anti-forgery): written in the last 160B of the bank, survives power loss.
 * Key design: the footer is written only "after firmware signature verification passes" => "footer present + CRC valid" transitively proves "signature was verified"
 *   —— an attacker can neither forge a valid signature (no private key) nor write flash directly (MQTT access only), so cannot produce a legal footer.
 * The swap gate need only check footer CRC + presence, no need to redo cryptography on every swap (signature verification is done once in ota-sign). */
#define FOOT_OFF       (OTA_BANK_SIZE - 160U)  /* 0xFFF60, 32B aligned = 5 flash words */
#define FOOT_MAGIC     0x4F544632UL            /* 'OTF2' (v3 layout, with signature) */
typedef struct {
  uint32_t magic;
  uint32_t cm7_size, cm7_crc;
  uint32_t cm4_size, cm4_crc;
  char     ver[40];       /* copied from the image ID tag (FW_INFO_OFF): version + build date/time */
  uint8_t  sig[72];       /* firmware signature (ECDSA-P256 DER, over SHA256(cm7||cm4)); archived + future secure boot */
  uint16_t sig_len;       /* actual DER length */
  uint16_t _pad;
  uint8_t  signer;        /* key that verified sig: 'E' = embedded Edgron key, 'C' = board-registered customer key (fwkey2); 0xFF/0 = pre-2026-09-07 footer */
  uint8_t  _rsv[19];      /* pad to 160B (5 flash words); reserved for future fields */
  uint32_t self_crc;      /* CRC32 of the first 156 bytes of this struct (footer self-integrity) */
} ota_footer_t;           /* exactly 160B = 5 flash words */
_Static_assert(sizeof(ota_footer_t) == 160, "footer must be exactly 5 flash words");
static void ota_write_footer(void);
static uint8_t ota_standby_safe(void);

/* ---- automatic rollback (trial period) ----
 * Core problem: "roll back if not confirmed within N seconds after boot" would wrongly kill a good firmware that "just happened to have no network on this power-up" -> need a marker to distinguish
 * "the very first boot right after an OTA swap" from "a normal power-up". Marker scheme:
 *  - confirm word: one flash word below each bank's footer (OTA_CONF_OFF). A full bank erase naturally resets it to all FF.
 *    "footer valid + confirm word all FF" = this bank is a trial firmware just OTA'd in; already programmed = veteran, never rolls back.
 *  - boot count + event: SRAM4 0x38000100 (D3 domain, reset-retained; lost on power loss = recount, harmless; magic guards against power-up random values).
 * Trial-period judgment (three paths, all exist only during the trial period):
 *  - repeated crashes: each trial-period boot ota_boot_guard (before RTOS) counts +1, exceeding OTA_TRIAL_MAX_BOOTS directly flips
 *    SWAP_BANK to go home. A hang is bitten into a reset by IWDG (32s, app_init.c), a HardFault stuck in while(1) is likewise bitten.
 *  - cloud confirm: MQTT connected + >=3 heartbeats sent + CM4 ping-pong normal -> program the confirm word, promote.
 *  - timeout: not met within OTA_TRIAL_TIMEOUT_MS of this boot -> roll back. Rationale for daring to use "can't reach network" as a criterion:
 *    the firmware was pushed over the network minutes ago (network was definitely up before the swap), the prime suspect for going dark after swap = the new firmware itself;
 *    and this is the only self-rescue path when "new firmware broke the network stack" (a manual revert button can never reach it); cost of a false kill =
 *    return to the old firmware that was working minutes ago, just re-push once.
 * Swaps during the trial period (auto rollback / manual revert) bypass the footer gate: the rollback target = the old firmware that was just running, which may be
 * an SWD-flashed footerless image (the gate would reject it). The normal manual-revert gate stays as usual. */
#define OTA_RUN_BASE   0x08000000UL           /* running bank is always in this window (alias follows after SWAP) */
#define OTA_CONF_OFF   (FOOT_OFF - 32U)       /* 0xFFF40: confirm word (1 flash word, right below the footer) */
#define GUARD_MAGIC    0x47554152UL           /* 'GUAR' */
#ifndef OTA_TRIAL_MAX_BOOTS
#define OTA_TRIAL_MAX_BOOTS  3U               /* boots allowed during the trial period, exceeding = repeated crashes */
#endif
#ifndef OTA_TRIAL_TIMEOUT_MS
#define OTA_TRIAL_TIMEOUT_MS 600000UL         /* 10 min without cloud confirm -> roll back (test builds can shorten with -D) */
#endif
typedef struct { uint32_t magic; uint32_t attempts; char evt[56]; uint32_t swap_req; } ota_guard_t;
#define GUARD ((ota_guard_t *)0x38000100UL)   /* SRAM4; IPC ping-pong occupies 0x00/0x04, staggered (address convention see boundary rules §four) */
#define SWAP_REQ_MAGIC 0x53575021UL           /* 'SWP!': flip failed at runtime -> redo from virgin early boot (field appended
                                               * past the old 64B layout; magic-guarded, so stale RAM can't fake a request) */
static uint8_t s_trial = 0;
static uint32_t crc32_sw(uint32_t crc, const uint8_t *d, uint32_t n);   /* defined below */

static void ota_evt_set(const char *e)   /* write event to SRAM4 (reset-safe, reported by old firmware after rollback) + report on trigger */
{
  strncpy(GUARD->evt, e, sizeof(GUARD->evt) - 1U);
  GUARD->evt[sizeof(GUARD->evt) - 1U] = 0;
  mqtt_mark_dirty();
}
const char *ota_last_evt(void)
{
  /* fail-safe: a brief power loss half-decays SRAM4 — the magic happens to survive while evt content rots,
   * garbage would pollute the heartbeat JSON. Byte-by-byte verify printable ASCII, treat anything unclean as an empty string */
  if (GUARD->magic != GUARD_MAGIC) { return ""; }
  for (uint32_t i = 0; i < sizeof(GUARD->evt); i++)
  {
    char c = GUARD->evt[i];
    if (c == 0) { return GUARD->evt; }             /* clean all the way to the terminator */
    if ((c < 0x20) || (c >= 0x7F)) { GUARD->evt[0] = 0; return ""; }   /* rotten: clear it */
  }
  GUARD->evt[0] = 0;                               /* no terminator, likewise clear */
  return "";
}
uint8_t  ota_in_trial(void)     { return s_trial; }
uint32_t ota_boot_attempts(void){ return (GUARD->magic == GUARD_MAGIC) ? GUARD->attempts : 0U; }

static uint8_t ota_conf_erased(void)   /* is our own (running) bank's confirm word still erased? */
{
  const uint32_t *c = (const uint32_t *)(OTA_RUN_BASE + OTA_CONF_OFF);
  for (uint32_t i = 0; i < (OTA_WORD / 4U); i++) { if (c[i] != 0xFFFFFFFFUL) { return 0U; } }
  return 1U;
}
static uint8_t ota_own_footer_valid(void)
{
  const ota_footer_t *f = (const ota_footer_t *)(OTA_RUN_BASE + FOOT_OFF);
  return ((f->magic == FOOT_MAGIC) &&
          (crc32_sw(0, (const uint8_t *)f, sizeof(*f) - 4U) == f->self_crc)) ? 1U : 0U;
}
const char *ota_signer_str(void)
{
  const ota_footer_t *f = (const ota_footer_t *)(OTA_RUN_BASE + FOOT_OFF);
  if (!ota_own_footer_valid()) { return "-"; }
  return (f->signer == 'C') ? "customer" : (f->signer == 'E') ? "edgron" : "legacy";
}
/* ---- bank-swap engine (2026-07-26 rebuild; closes the "silent no-flip" ledger open since 07-24) ----
 * Old form trusted HAL return codes and they LIE: a failed OPT unlock leaves OPTLOCK set until
 * the next reset, after which OPTSR_PRG writes are ignored, OPTSTART is ignored, OPT_BUSY never
 * rises — every call returns HAL_OK and nothing happens (field: boards rebooting on the same
 * bank with evt=swapped; the "early-boot window" theory was survivor bias — the real variable
 * is which BOOT is poisoned, uptime was only a proxy for "a reset happened in between").
 * The only judge that cannot lie is reading FLASH_OPTSR_CUR back after the launch (the CUR
 * register takes the new value when the option change completes; the MAPPING flips on the next
 * system reset — RM0399 bank swapping). Three layers:
 *   1. verified attempts (clear OPTCHANGEERR, unlock, program, launch, READ BACK) x3;
 *   2. still not flipped -> capture OPTCR/OPTSR_CUR into evt (the poison photographs itself)
 *      and pend SWAP_REQ_MAGIC -> reset -> retry at the TOP of ota_boot_guard, pre-RTOS,
 *      where the OPT engine is virgin — nothing has had a chance to poison it yet;
 *   3. even that failing leaves the board booting its old bank with the evidence in the
 *      heartbeat evt, instead of a false "swapped". */
static int ota_swap_ob_try(void)   /* <=3 verified flip attempts; 0 = flipped (awaiting reset) */
{
  uint32_t want = ota_swap_active() ? 0U : FLASH_OPTSR_SWAP_BANK_OPT;
  for (int attempt = 0; attempt < 3; attempt++)
  {
    FLASH_OBProgramInitTypeDef ob;
    memset(&ob, 0, sizeof(ob));
    ob.OptionType = OPTIONBYTE_USER;
    ob.USERType   = OB_USER_SWAP_BANK;
    ob.USERConfig = want ? OB_SWAP_BANK_ENABLE : OB_SWAP_BANK_DISABLE;
    FLASH->OPTCCR = FLASH_OPTCCR_CLR_OPTCHANGEERR;   /* a stale change-error blocks OPTSTART */
    HAL_FLASH_Unlock();
    (void)HAL_FLASH_OB_Unlock();                     /* status worthless (see above): judge by readback only */
    if (HAL_FLASHEx_OBProgram(&ob) == HAL_OK) { (void)HAL_FLASH_OB_Launch(); }
    if ((FLASH->OPTSR_CUR & FLASH_OPTSR_SWAP_BANK_OPT) == want) { return 0; }
    HAL_Delay(10);
  }
  {
    char e[56];
    snprintf(e, sizeof(e), "swapfail cr=%08lx sr=%08lx",
             (unsigned long)FLASH->OPTCR, (unsigned long)FLASH->OPTSR_CUR);
    ota_evt_set(e);                                  /* cr bit0 = OPTLOCK: 1 here proves the poisoned-unlock theory */
  }
  return -1;
}

static void ota_swap_toggle_reset(void)   /* flip SWAP_BANK + reset (shared by apply/rollback); does not return */
{
  if (ota_swap_ob_try() != 0)
  {
    GUARD->swap_req = SWAP_REQ_MAGIC;     /* this boot's OPT engine is poisoned: redo from the next boot's clean start */
  }
  { extern void app_reset(const char *reason); app_reset("ota bank swap"); }
}

void ota_boot_guard(void)   /* called in app_periph_init (before RTOS); serial not up, no printf, all evidence left in GUARD->evt */
{
  if (GUARD->magic != GUARD_MAGIC)   /* power-up SRAM random -> initialize */
  {
    GUARD->magic = GUARD_MAGIC;
    GUARD->attempts = 0;
    GUARD->evt[0] = 0;
    GUARD->swap_req = 0;
  }
  else if (GUARD->swap_req == SWAP_REQ_MAGIC)   /* runtime flip failed last boot: retry from the virgin OPT engine */
  {
    GUARD->swap_req = 0;                        /* one-shot: success resets into the new bank, failure boots on with evidence */
    if (ota_swap_ob_try() == 0)
    {
      ota_evt_set("swapped-earlyboot");
      { extern void app_reset(const char *reason); app_reset("ota swap earlyboot"); }   /* mapping engages on this reset */
    }
    /* failed even here: evt already holds OPTCR/OPTSR_CUR; continue booting the old bank */
  }
  s_trial = (ota_own_footer_valid() && ota_conf_erased()) ? 1U : 0U;
  if (!s_trial)                      /* normal power-up (confirmed / SWD image): don't enter any rollback logic */
  {
    GUARD->attempts = 0;
    return;
  }
  /* Debugger attached (2026-07-25, parity with the H503 slave guard): debug sessions reset
   * the core routinely — don't let those resets count toward the boot-loop rollback while a
   * probe is on. C_DEBUGEN clears on POR only, so protection resumes after a power cycle. */
  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U) { return; }
  GUARD->attempts++;
  if (GUARD->attempts > OTA_TRIAL_MAX_BOOTS)   /* repeated crashes (IWDG/HardFault reset loop) */
  {
    ota_evt_set("rollback:boot-loop");
    ota_swap_toggle_reset();
  }
}

/* USB service confirm (2026-07-25): a technician on the USB console may promote a trial
 * firmware with `ota-confirm yes` when the site has no network at all (pure-USB OTA).
 * One-shot request flag; the promotion still requires CM4 ping-pong health below. */
static uint8_t s_usb_confirm_req = 0;

static void ota_trial_poll(void)   /* defaultTask every 500ms (relayed from ota_poll): trial-period judgment */
{
  extern uint8_t app_rpc_alive(void);   /* app_rpc.h (avoid a header dependency cycle) */
#if APP_ENABLE_CLOUD
  if ((mqtt_cloud_ok() || s_usb_confirm_req) && app_rpc_alive())
#else
  if (app_rpc_alive())   /* no cloud: confirm as soon as both cores are healthy (nothing else to wait for) */
#endif
  {
    /* cloud link + both cores healthy -> program confirm word to promote. Note this writes to the currently running bank (once in a lifetime):
     * during programming, same-bank instruction fetch stalls ~hundreds of µs, ETH interrupt is delayed at most one frame, TCP will retransmit */
    uint8_t conf[OTA_WORD];
    memset(conf, 0, sizeof(conf));
    memcpy(conf, "OTACONF1", 8);
    HAL_FLASH_Unlock();
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, OTA_RUN_BASE + OTA_CONF_OFF, (uint32_t)conf) == HAL_OK)
    {
      printf("[OTA] trial CONFIRMED (%s+ipc ok, boot %lu) -> permanent\n\r",
             s_usb_confirm_req ? "usb" : "cloud", (unsigned long)ota_boot_attempts());
      s_trial = 0;
      GUARD->attempts = 0;
      s_usb_confirm_req = 0;
      ota_evt_set("confirmed");
    }
    else
    {
      printf("[OTA] confirm word program FAIL, retry next tick\n\r");
    }
    HAL_FLASH_Lock();
  }
#if APP_ENABLE_CLOUD
  else if (HAL_GetTick() >= OTA_TRIAL_TIMEOUT_MS)
  {
    ota_evt_set("rollback:no-cloud");
    printf("[OTA] TRIAL FAILED: no cloud confirm in %lus -> rollback\n\r",
           (unsigned long)(OTA_TRIAL_TIMEOUT_MS / 1000UL));
    ota_swap_toggle_reset();
  }
#endif
}

/* signature pending footer-write for this session (stashed after ota-sign verifies, taken by the consumer when writing the footer) */
static uint8_t  s_sig[72];
static uint16_t s_sig_len = 0;
static uint8_t  s_sig_by = 0;     /* 'E'/'C': which key verified s_sig (recorded in the footer, reported as heartbeat "fws" after the swap) */

/* SPSC ring buffer: D2 SRAM3 bare region (not managed by the linker; clock enabled in app_periph_init).
 * 2026-09-03: moved here from SRAM2 0x30038000 so the LwIP heap at 0x30030000 could grow 16K -> 56K
 * (lwipopts.h). CPU-only traffic (tcpip thread produces, defaultTask consumes, no DMA), so the cacheable
 * default of SRAM3 is fine and no MPU region is needed. */
#define RING_BASE      0x30040000UL   /* SRAM3 start */
#define RING_SIZE      0x00008000UL   /* 32KB = the whole SRAM3 (0x30040000..0x30048000) */
static volatile uint32_t s_ring_head = 0;  /* producer (tcpip) writes */
static volatile uint32_t s_ring_tail = 0;  /* consumer (task) writes */
#define RING_BYTE(i)   (((volatile uint8_t *)RING_BASE)[(i) % RING_SIZE])
typedef struct { uint32_t dst; uint16_t len; uint16_t magic; } rec_hdr_t;  /* 8B */
#define REC_MAGIC      0xA55AU

typedef enum {
  OTA_IDLE = 0, OTA_ERASE_PEND, OTA_READY, OTA_RECV,
  OTA_VERIFY_OK, OTA_VERIFY_FAIL, OTA_APPLY_PEND, OTA_PROVISION_PEND,
  OTA_FOOTER_PEND,          /* signature verified -> consumer writes footer (defaultTask, flash) */
  OTA_SIGNED,               /* footer on flash, ready to apply */
  OTA_SIG_FAIL              /* signature verification failed -> reject, don't write footer */
} ota_state_t;

static volatile ota_state_t s_st = OTA_IDLE;
static uint32_t s_offset, s_size, s_crc_exp;
static volatile uint32_t s_recv;       /* bytes written on the consumer side */
static uint8_t  s_erased = 0;
static uint8_t  s_dirty = 0;           /* wrote a word after this erase -> a re-begin (off=0) must re-erase (flash word forbids second programming, ECC would break) */
/* dedup bitmap: 1KB granularity (1024 blocks = 1MB, 128B). Granularity must be <= push block size, else multiple blocks in the same cell collide and get wrongly judged "already received".
 * Why 1KB and not 4KB: a single TLS decrypt pbuf is ~1.5KB, an MQTT message >1.5KB = multiple pbufs
 * triggering the mqtt parser's multi-segment handling defect so it doesn't reach the app; a 1KB block in a single pbuf always arrives. The push side aligns block size to this (ota_push CHUNK) */
#define OTA_CHUNK_SHIFT  10U                        /* 1KB */
static uint8_t  s_chunk_map[(OTA_BANK_SIZE >> OTA_CHUNK_SHIFT) / 8U];   /* 128B */
/* anti-brick swap gate: this session's dual-segment verified flags (fast path) + each segment's size/CRC record (for writing the footer) */
static uint8_t  s_ok_cm7, s_ok_cm4;
static uint32_t s_cm7_size, s_cm7_crc, s_cm4_size, s_cm4_crc;
/* producer (tcpip thread) state */
static uint8_t  s_hdr[8];
static uint32_t s_hdr_fill;
static uint8_t  s_msg_bad;             /* discard this message (bad header / ring full / duplicate block) */
static uint32_t s_prod_dst;
static uint32_t s_msg_total;           /* this message's total length (including 8B header) */
/* consumer (task) state */
static uint8_t  s_wbuf[OTA_WORD];
static uint32_t s_wfill, s_cons_dst;

static uint32_t crc32_sw(uint32_t crc, const uint8_t *d, uint32_t n)
{
  crc = ~crc;
  while (n--)
  {
    crc ^= *d++;
    for (int k = 0; k < 8; k++) { crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL))); }
  }
  return ~crc;
}

uint32_t ota_swap_active(void)
{
  return (FLASH->OPTSR_CUR & FLASH_OPTSR_SWAP_BANK_OPT) ? 1U : 0U;
}

const char *ota_state_str(void)
{
  switch (s_st)
  {
    case OTA_IDLE:           return s_trial ? "trial" : "idle";
    case OTA_ERASE_PEND:     return "erasing";
    case OTA_READY:          return "ready";
    case OTA_RECV:           return "recv";
    case OTA_VERIFY_OK:      return "verify-ok";
    case OTA_VERIFY_FAIL:    return "verify-fail";
    case OTA_APPLY_PEND:     return "applying";
    case OTA_PROVISION_PEND: return "provisioning";
    case OTA_FOOTER_PEND:    return "footer";
    case OTA_SIGNED:         return "signed";
    case OTA_SIG_FAIL:       return "sig-fail";
    default:                 return "?";
  }
}

/* ---------------- producer: tcpip thread, only touches the ring buffer ---------------- */

static uint32_t ring_free(void) { return RING_SIZE - (s_ring_head - s_ring_tail) - 1U; }

static void ring_push(const uint8_t *d, uint32_t n)
{
  uint32_t h = s_ring_head;
  for (uint32_t i = 0; i < n; i++) { RING_BYTE(h + i) = d[i]; }
  __DMB();
  s_ring_head = h + n;
}

void ota_fw_msg_start(uint32_t tot_len)
{
  s_msg_total = tot_len;
  s_hdr_fill = 0;
  s_msg_bad = 0;
}

void ota_fw_chunk(const uint8_t *data, uint16_t len, uint8_t last)
{
  (void)last;
  if (((s_st != OTA_READY) && (s_st != OTA_RECV)) || s_msg_bad) { return; }
  while ((s_hdr_fill < 8U) && (len > 0U))
  {
    s_hdr[s_hdr_fill++] = *data++;
    len--;
    if (s_hdr_fill == 8U)
    {
      uint32_t off, gidx;
      if (memcmp(s_hdr, "OTA1", 4) != 0) { s_msg_bad = 1; return; }
      off = (uint32_t)s_hdr[4] | ((uint32_t)s_hdr[5] << 8) | ((uint32_t)s_hdr[6] << 16) | ((uint32_t)s_hdr[7] << 24);
      if (((off & (OTA_WORD - 1U)) != 0U) || (s_offset + off >= OTA_BANK_SIZE)) { s_msg_bad = 1; return; }
      gidx = (s_offset + off) >> OTA_CHUNK_SHIFT;        /* block number (granularity see OTA_CHUNK_SHIFT) */
      if ((s_chunk_map[gidx >> 3] & (1U << (gidx & 7U))) != 0U)
      {
        s_msg_bad = 1;   /* already received -> silently skip (flash word forbids second programming) */
        return;
      }
      if (ring_free() < (sizeof(rec_hdr_t) * 4U + s_msg_total))
      {
        s_msg_bad = 1;   /* ring can't hold the whole message -> don't mark the bitmap, re-push next round to fill in */
        return;
      }
      s_chunk_map[gidx >> 3] |= (uint8_t)(1U << (gidx & 7U));   /* accept: mark immediately (the ring won't lose accepted data) */
      s_prod_dst = OTA_WIN_BASE + s_offset + off;
      s_st = OTA_RECV;
    }
  }
  if (len == 0U) { return; }
  rec_hdr_t rh = { s_prod_dst, len, REC_MAGIC };
  ring_push((const uint8_t *)&rh, sizeof(rh));
  ring_push(data, len);
  s_prod_dst += len;
}

/* ---------------- consumer: defaultTask, all Flash operations here ---------------- */

static void cons_flush_word(void)
{
  if (s_wfill == 0U) { return; }
  memset(&s_wbuf[s_wfill], 0xFF, OTA_WORD - s_wfill);
  s_dirty = 1;
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, s_cons_dst, (uint32_t)s_wbuf) != HAL_OK)
  {
    printf("[OTA] program FAIL @0x%08lx\n\r", (unsigned long)s_cons_dst);
    s_st = OTA_VERIFY_FAIL;
  }
  s_cons_dst += OTA_WORD;
  s_wfill = 0;
}

static void cons_drain(void)
{
  while ((s_ring_head - s_ring_tail) >= sizeof(rec_hdr_t))
  {
    rec_hdr_t rh;
    uint32_t t = s_ring_tail;
    for (uint32_t i = 0; i < sizeof(rh); i++) { ((uint8_t *)&rh)[i] = RING_BYTE(t + i); }
    if (rh.magic != REC_MAGIC) { printf("[OTA] ring desync!\n\r"); s_st = OTA_VERIFY_FAIL; s_ring_tail = s_ring_head; return; }
    if ((s_ring_head - s_ring_tail) < (sizeof(rh) + rh.len)) { return; }   /* data not fully arrived */
    t += sizeof(rh);
    if (rh.dst != (s_cons_dst + s_wfill)) { cons_flush_word(); s_cons_dst = rh.dst; }  /* offset jump = new block */
    for (uint16_t i = 0; i < rh.len; i++)
    {
      s_wbuf[s_wfill++] = RING_BYTE(t + i);
      if (s_wfill == OTA_WORD)
      {
        s_dirty = 1;
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, s_cons_dst, (uint32_t)s_wbuf) != HAL_OK)
        {
          printf("[OTA] program FAIL @0x%08lx\n\r", (unsigned long)s_cons_dst);
          s_st = OTA_VERIFY_FAIL;
        }
        s_cons_dst += OTA_WORD;
        s_wfill = 0;
      }
    }
    __DMB();
    s_ring_tail = t + rh.len;
    s_recv += rh.len;
  }
  if ((s_st == OTA_RECV) && (s_recv >= s_size))
  {
    cons_flush_word();
    HAL_FLASH_Lock();
    uint32_t crc = crc32_sw(0, (const uint8_t *)(OTA_WIN_BASE + s_offset), s_size);
    if ((s_recv == s_size) && (crc == s_crc_exp))
    {
      s_st = OTA_VERIFY_OK;
      if (s_offset == 0U) { s_ok_cm7 = 1U; s_cm7_size = s_size; s_cm7_crc = crc; }
      else                { s_ok_cm4 = 1U; s_cm4_size = s_size; s_cm4_crc = crc; }  /* dual-segment layout: nonzero means CM4@0x80000 */
      printf("[OTA] segment OK: %lu bytes, crc=%08lx (cm7=%u cm4=%u)%s\n\r",
             (unsigned long)s_recv, (unsigned long)crc, s_ok_cm7, s_ok_cm4,
             (s_ok_cm7 && s_ok_cm4) ? " -> await ota-sign" : "");
      /* both segments present but don't write the footer yet: must wait for ota-sign to verify (anti-forgery). Footer = the proof the signature was verified */
    }
    else
    {
      s_st = OTA_VERIFY_FAIL;
      printf("[OTA] VERIFY FAIL: recv=%lu/%lu crc=%08lx exp=%08lx\n\r",
             (unsigned long)s_recv, (unsigned long)s_size, (unsigned long)crc, (unsigned long)s_crc_exp);
    }
  }
}

/* write footer (defaultTask context): version string read from the pushed image ID tag (FW_INFO_OFF) */
static void ota_write_footer(void)
{
  ota_footer_t f = {0};
  const fw_info_t *fi = (const fw_info_t *)(OTA_WIN_BASE + FW_INFO_OFF);
  f.magic    = FOOT_MAGIC;
  f.cm7_size = s_cm7_size; f.cm7_crc = s_cm7_crc;
  f.cm4_size = s_cm4_size; f.cm4_crc = s_cm4_crc;
  strncpy(f.ver, (fi->magic == FW_INFO_MAGIC) ? fi->ver : "unknown", sizeof(f.ver) - 1U);
  if (s_sig_len > 0U && s_sig_len <= sizeof(f.sig)) { memcpy(f.sig, s_sig, s_sig_len); f.sig_len = s_sig_len; }
  f.signer = (s_sig_by != 0U) ? s_sig_by : 0xFFU;
  f.self_crc = crc32_sw(0, (const uint8_t *)&f, sizeof(f) - 4U);
  HAL_FLASH_Unlock();
  uint8_t ok = 1;
  for (uint32_t i = 0; i < sizeof(f); i += OTA_WORD)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                          OTA_WIN_BASE + FOOT_OFF + i, (uint32_t)((const uint8_t *)&f + i)) != HAL_OK) { ok = 0; }
  }
  HAL_FLASH_Lock();
  /* 2026-08-31 no-footer case (2 of 5 pushes, always standby=bank1, retry always cured):
   * the apply gate found no magic although this write reported OK. Self-check: drop any
   * cached lines over the footer range and read back what flash actually holds — a
   * mismatch now shouts in the OTA event instead of surfacing later as refused:no-footer. */
  SCB_InvalidateDCache_by_Addr((void *)(OTA_WIN_BASE + FOOT_OFF), (int32_t)sizeof(f));
  if (memcmp((const void *)(OTA_WIN_BASE + FOOT_OFF), &f, sizeof(f)) != 0)
  {
    ok = 0;
    ota_evt_set("footer-verify-fail");
  }
  printf("[OTA] footer %s: ver=\"%s\"\n\r", ok ? "written+verified" : "WRITE-FAIL", f.ver);
}

/* pre-swap standby-bank safety check (defaultTask context): footer magic + self CRC + recomputed CRC of both segment contents all correct to pass */
static uint8_t ota_standby_safe(void)
{
  const ota_footer_t *f = (const ota_footer_t *)(OTA_WIN_BASE + FOOT_OFF);
  /* the footer was programmed through the flash controller (cache-unaware); any stale
   * cached line here would masquerade as no-footer/footer-corrupt (2026-08-31 case) */
  SCB_InvalidateDCache_by_Addr((void *)(OTA_WIN_BASE + FOOT_OFF), (int32_t)sizeof(*f));
  if (f->magic != FOOT_MAGIC)
  {
    printf("[OTA] SWAP REFUSED: standby has no footer (incomplete/legacy image)\n\r");
    ota_evt_set("refused:no-footer");
    return 0;
  }
  if (crc32_sw(0, (const uint8_t *)f, sizeof(*f) - 4U) != f->self_crc)
  {
    printf("[OTA] SWAP REFUSED: footer corrupt\n\r");
    ota_evt_set("refused:footer-crc");
    return 0;
  }
  if ((f->cm7_size == 0U) || (f->cm7_size > OTA_CM4_OFF) ||
      (f->cm4_size == 0U) || (f->cm4_size > (OTA_CONF_OFF - OTA_CM4_OFF)))
  {
    printf("[OTA] SWAP REFUSED: footer sizes insane\n\r");
    ota_evt_set("refused:size");
    return 0;
  }
  uint32_t c7 = crc32_sw(0, (const uint8_t *)(OTA_WIN_BASE), f->cm7_size);
  uint32_t c4 = crc32_sw(0, (const uint8_t *)(OTA_WIN_BASE + OTA_CM4_OFF), f->cm4_size);
  if ((c7 != f->cm7_crc) || (c4 != f->cm4_crc))
  {
    printf("[OTA] SWAP REFUSED: content CRC mismatch (cm7 %08lx/%08lx cm4 %08lx/%08lx)\n\r",
           (unsigned long)c7, (unsigned long)f->cm7_crc, (unsigned long)c4, (unsigned long)f->cm4_crc);
    ota_evt_set("refused:content-crc");
    return 0;
  }
  printf("[OTA] standby verified: ver=\"%s\" cm7=%lu cm4=%lu\n\r",
         f->ver, (unsigned long)f->cm7_size, (unsigned long)f->cm4_size);
  return 1;
}

/* firmware signature verification (tcpip thread: same thread as TLS, mbedTLS memory pool serial use is safe; read-only flash, no write).
 * SHA256 over the standby bank's two segment images, then verify the ECDSA signature against, in order:
 *   1. the build-embedded Edgron release key (app_certs);
 *   2. the customer key registered on THIS board over the USB console (app_identity fwkey2), if any.
 * Either passing = genuine (1); the winner is remembered in s_sig_by for the footer. The cloud only carries
 * bytes — trust is decided here, by the board. Returns 0 = forged/corrupt/unknown signer. */
static uint8_t ota_verify_signature(const uint8_t *sig, uint16_t sig_len)
{
  mbedtls_sha256_context sha;
  uint8_t hash[32];
  int r;
  mbedtls_sha256_init(&sha);
  if (mbedtls_sha256_starts_ret(&sha, 0) != 0) { mbedtls_sha256_free(&sha); return 0U; }
  mbedtls_sha256_update_ret(&sha, (const uint8_t *)(OTA_WIN_BASE), s_cm7_size);
  mbedtls_sha256_update_ret(&sha, (const uint8_t *)(OTA_WIN_BASE + OTA_CM4_OFF), s_cm4_size);
  mbedtls_sha256_finish_ret(&sha, hash);
  mbedtls_sha256_free(&sha);

  mbedtls_pk_context pk;
  s_sig_by = 0;
  mbedtls_pk_init(&pk);
  r = mbedtls_pk_parse_public_key(&pk, (const uint8_t *)app_fwsign_pub_pem, app_fwsign_pub_len);
  if (r == 0) { r = mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, hash, sizeof(hash), sig, sig_len); }
  mbedtls_pk_free(&pk);
  if (r == 0) { s_sig_by = 'E'; return 1U; }

  {
    size_t klen = 0;
    const unsigned char *raw = app_fwkey2(&klen);   /* raw 65-byte point: no ASN.1 to parse at OTA time */
    if (raw == NULL) { return 0U; }
    mbedtls_pk_init(&pk);
    r = mbedtls_pk_setup(&pk, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY));
    if (r == 0)
    {
      mbedtls_ecp_keypair *ec = mbedtls_pk_ec(pk);
      r = mbedtls_ecp_group_load(&ec->grp, MBEDTLS_ECP_DP_SECP256R1);
      if (r == 0) { r = mbedtls_ecp_point_read_binary(&ec->grp, &ec->Q, raw, klen); }
      if (r == 0) { r = mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, hash, sizeof(hash), sig, sig_len); }
    }
    mbedtls_pk_free(&pk);
    if (r == 0) { s_sig_by = 'C'; return 1U; }
  }
  return 0U;
}

/* hex string -> bytes; returns byte count (<=maxout), stops on an illegal character */
static int ota_hex2bin(const char *h, uint8_t *out, int maxout)
{
  int n = 0;
  for (; (h[0] != 0) && (h[1] != 0) && (n < maxout); h += 2)
  {
    int hi = (h[0] <= '9') ? (h[0] - '0') : ((h[0] | 0x20) - 'a' + 10);
    int lo = (h[1] <= '9') ? (h[1] - '0') : ((h[1] | 0x20) - 'a' + 10);
    if ((hi < 0) || (hi > 15) || (lo < 0) || (lo > 15)) { break; }
    out[n++] = (uint8_t)((hi << 4) | lo);
  }
  return n;
}

void ota_cmd(const char *c, uint16_t len)
{
  (void)len;
  if (strncmp(c, "ota-begin ", 10) == 0)
  {
    char *end;
    uint32_t off, size, crc;
    if (s_trial)
    {
      /* trial period forbids erasing the standby bank: the only rollback escape route lies there, erasing then rolling back = jumping into a half-erased bank = a real brick.
       * Wait for promotion (or rollback) before pushing */
      ota_evt_set("refused:in-trial");
      return;
    }
    off  = strtoul(c + 10, &end, 16);
    size = strtoul(end, &end, 10);
    crc  = strtoul(end, &end, 16);
    if ((size == 0U) || (off + size > OTA_CONF_OFF) || ((off & (OTA_WORD - 1U)) != 0U))  /* tail = confirm word + footer reserved area */
    {
      return;
    }
    s_offset = off; s_size = size; s_crc_exp = crc;
    s_recv = 0; s_wfill = 0; s_hdr_fill = 0; s_msg_bad = 0;
    s_ring_head = 0; s_ring_tail = 0;
    s_cons_dst = OTA_WIN_BASE + off;
    /* off=0 is treated as a new upgrade session: if written after erase, must re-erase (flash word forbids second programming);
       off!=0 is the second segment of the same session (e.g. CM4@0x80000), reuse this erase */
    if ((off == 0U) && (s_dirty || !s_erased)) { s_st = OTA_ERASE_PEND; }
    else                                       { s_st = s_erased ? OTA_READY : OTA_ERASE_PEND; }
  }
  else if (strncmp(c, "ota-confirm yes", 15) == 0)   /* USB console: promote trial fw on a no-network site */
  {
    if (!s_trial) { printf("[OTA] not in trial, nothing to confirm\n\r"); }
    else          { s_usb_confirm_req = 1; printf("[OTA] usb confirm requested (needs CM4 ipc ok)\n\r"); }
  }
  else if ((strncmp(c, "ota-apply", 9) == 0) || (strncmp(c, "ota-revert", 10) == 0))
  {
    s_st = OTA_APPLY_PEND;   /* the anti-brick gate is in ota_poll's APPLY_PEND (needs to recompute CRC, don't block the tcpip thread) */
  }
  else if (strncmp(c, "ota-sign ", 9) == 0)
  {
    /* anti-forgery gate: both segments present + signature verified -> allow writing footer; otherwise sig-fail (footer not written, swap will be refused).
     * Verification completes immediately on this (tcpip) thread — writing the footer / swapping are flash operations, left to defaultTask */
    uint8_t sig[72];
    int slen;
    if (!(s_ok_cm7 && s_ok_cm4)) { s_st = OTA_SIG_FAIL; return; }
    slen = ota_hex2bin(c + 9, sig, (int)sizeof(sig));
    if ((slen > 0) && ota_verify_signature(sig, (uint16_t)slen))
    {
      memcpy(s_sig, sig, (size_t)slen);
      s_sig_len = (uint16_t)slen;
      s_st = OTA_FOOTER_PEND;   /* defaultTask's ota_poll writes the footer */
      ota_evt_set((s_sig_by == 'C') ? "signed:customer" : "signed:edgron");   /* which key admitted this image (heartbeat evt) */
    }
    else
    {
      s_st = OTA_SIG_FAIL;
    }
  }
  else if (strncmp(c, "ota-provision", 13) == 0)
  {
    s_st = OTA_PROVISION_PEND;
  }
  else if (strncmp(c, "broker-pub", 10) == 0)
  {
    mqtt_broker_select(0U);   /* mqtt.c; this function is on the tcpip thread, direct call safe */
  }
  else if (strncmp(c, "broker-lan", 10) == 0)
  {
    mqtt_broker_select(1U);
  }
  else if (strncmp(c, "broker-aws", 10) == 0)
  {
    mqtt_broker_select(2U);   /* switch to AWS IoT (mTLS+SNI+DNS) */
  }
  else if (strncmp(c, "beep ", 5) == 0)
  {
    app_beep_set((uint8_t)(c[5] == '1'));   /* direct control: beep 1 / beep 0 (757: led3 retired, replaced by buzzer) */
  }
  /* --- generic protocol (used by panel): set <point-id> <value> / cmd <cmd-id> [arg] --- */
  else if (strncmp(c, "set do.beep ", 12) == 0)
  {
    app_beep_set((uint8_t)(c[12] == '1'));
  }
  else if (strncmp(c, "mota-begin ", 11) == 0)      /* mota-begin <type> <size> <crc32hex> (v0.24 stored into repo by model) */
  {
    extern int mota_begin(uint8_t type, uint32_t size, uint32_t crc);
    char *e = 0;
    uint8_t a = (uint8_t)strtoul(c + 11, &e, 10);
    uint32_t sz = strtoul(e, &e, 10);
    uint32_t cr = strtoul(e, 0, 16);
    printf("[MOTA] begin -> %d\n\r", mota_begin(a, sz, cr));
  }
  else if (strncmp(c, "mota-flash ", 11) == 0)      /* mota-flash <addr> */
  {
    extern int mota_flash(uint8_t addr);
    (void)mota_flash((uint8_t)strtoul(c + 11, 0, 10));
  }
  else if ((strncmp(c, "mbcfg", 5) == 0) || (strncmp(c, "mbpoll ", 7) == 0))
  {
    /* cloud-configure Modbus port: same syntax as the CLI (contract §2.3); tcpip thread only enqueues, defaultTask executes */
    extern void app_mbcfg_cloud(const char *line);
    app_mbcfg_cloud(c);
  }
  else if (strncmp(c, "set rly.do ", 11) == 0)
  {
    extern void app_demo_relay_set(uint16_t mask);   /* application owns the point (app_user.c) */
    unsigned long m = strtoul(c + 11, 0, 0);         /* bits point: the whole bitmap at once */
    app_demo_relay_set((uint16_t)m);
    mqtt_mark_dirty();                               /* the real state is reported by the next read-back */
  }
  else if (strncmp(c, "cmd broker.set ", 15) == 0)
  {
    const char *a = c + 15;
    mqtt_broker_select((strncmp(a, "aws", 3) == 0) ? 2U : (strncmp(a, "lan", 3) == 0) ? 1U : 0U);
  }
  else if (strncmp(c, "cmd ota.revert", 14) == 0)
  {
    s_st = OTA_APPLY_PEND;   /* rollback = swap bank (same anti-brick gate path as ota-apply/revert) */
  }
}

void ota_poll(void)
{
  FLASH_OBProgramInitTypeDef ob;
  if (s_trial) { ota_trial_poll(); }   /* trial-period judgment (cloud confirm / timeout rollback); clears s_trial itself after promotion */
  switch (s_st)
  {
    case OTA_ERASE_PEND:
    {
      FLASH_EraseInitTypeDef e = {0};
      uint32_t sector_err = 0;
      printf("[OTA] erasing standby bank (8 sectors, ~8s)...\n\r");
      HAL_FLASH_Unlock();
      e.TypeErase    = FLASH_TYPEERASE_SECTORS;
      e.Banks        = FLASH_BANK_2;
      e.Sector       = FLASH_SECTOR_0;
      e.NbSectors    = 8;
      e.VoltageRange = FLASH_VOLTAGE_RANGE_3;
      if (HAL_FLASHEx_Erase(&e, &sector_err) == HAL_OK)
      {
        s_erased = 1;
        s_dirty = 0;
        s_ok_cm7 = 0;
        s_ok_cm4 = 0;
        s_sig_len = 0;   /* new session: clear the previous signature, prevent a stale signature slipping through */
        memset(s_chunk_map, 0, sizeof(s_chunk_map));
        s_st = OTA_READY;
        printf("[OTA] erase done, ready for data\n\r");
      }
      else
      {
        HAL_FLASH_Lock();
        s_st = OTA_IDLE;
        printf("[OTA] erase FAIL sector=%lu\n\r", (unsigned long)sector_err);
      }
      break;
    }
    case OTA_READY:
    case OTA_RECV:
      HAL_FLASH_Unlock();   /* idempotent; flash may be locked after the second-segment begin */
      cons_drain();
      break;
    case OTA_FOOTER_PEND:   /* signature verified (tcpip side) -> write footer (flash, this defaultTask) */
      ota_write_footer();
      s_st = OTA_SIGNED;    /* ready to swap */
      break;
    case OTA_APPLY_PEND:
    {
      if (s_trial)
      {
        /* a swap during the trial period = manual rollback (dash Revert button): the target is the old firmware that was just running,
         * which may have no footer (SWD image), so pardon the footer gate and swap straight back */
        ota_evt_set("rollback:manual");
        printf("[OTA] trial manual rollback -> swap back, reset...\n\r");
        ota_swap_toggle_reset();
        break;
      }
      /* anti-brick + anti-forgery gate v3: always pass based on the standby bank's persistent footer. The footer is written only after signature verification passes,
       * so "footer present + CRC valid" = "signature was verified" (attacker can't write a footer). Never use "CRC already verified" as a fast path —
       * CRC correct != signature correct (firmware with a tampered signature still has a correct CRC), that would bypass anti-forgery. */
      if (!ota_standby_safe())
      {
        s_st = OTA_IDLE;
        break;
      }
      ota_evt_set("swapped");   /* after reset the new firmware (trial period) can show "where it came from" in the heartbeat */
      printf("[OTA] toggling bank swap, reset...\n\r");
      ota_swap_toggle_reset();
      break;
    }
    case OTA_PROVISION_PEND:
    {
      printf("[OTA] provisioning BOOT_CM4_ADD0=0x%08lx, reset...\n\r", (unsigned long)OTA_CM4_BOOT);
      memset(&ob, 0, sizeof(ob));
      ob.OptionType    = OPTIONBYTE_CM4_BOOTADD;
      ob.CM4BootConfig = OB_BOOT_ADD0;
      ob.CM4BootAddr0  = OTA_CM4_BOOT;
      HAL_FLASH_Unlock();
      HAL_FLASH_OB_Unlock();
      if (HAL_FLASHEx_OBProgram(&ob) == HAL_OK) { HAL_FLASH_OB_Launch(); }
      { extern void app_reset(const char *reason); app_reset("ota cm4 provision"); }
      break;
    }
    default:
      break;
  }
}

uint32_t ota_recv(void)
{
  return s_recv;
}
