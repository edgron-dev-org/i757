# CubeMX Code Boundary Rules

> Origin: MQTT/application code was once scattered across main.c, making refactoring painful. This document = the rules + the complete list of manual edits to CubeMX-generated files.
> **Read this document before any CubeMX regeneration; any new manual edit must be back-filled into this document.**

## Rules

1. **Put your own code in your own files, with dual insurance of directory + prefix**: one **`App/` directory** per core, all files with the **`app_` prefix** — directory isolation prevents mixing, the prefix prevents name collisions (e.g. the former mqtt.c colliding with LwIP's own mqtt.c). CubeMX never touches App/; the USER CODE blocks in generated files hold **at most one line of interface call**.
2. **Exception = CubeMX has a major defect** (like ethernetif, where no GUI option is offered and the template itself has pitfalls) — direct edits to generated code are then allowed, but each must be registered in this document's "Defect-Patch List", noting what was changed, why, and how to reapply it after regeneration.
3. Purpose: when porting to the production board (cube/) or regenerating the .ioc, the user files can be copied over as-is, the defect patches reapplied per the list, and no archaeology is needed.

## I. User File List (unaffected by regeneration; port = copy the whole thing over)

| File | Responsibility |
|---|---|
| CM7 `App/app_cfg.h` | All configurable items: broker / account / topics / static IP |
| CM7 `App/app_mqtt.c/h` | Network + MQTT + OTA application main loop, broker auto-failover, all MQTT callbacks, IPC ping-pong macros |
| CM7 `App/app_ota.c/h` | Dual-Bank OTA: chunk protocol / ring buffer / anti-brick gate / footer; identity-tag type and FW_INFO_OFF convention; **trial-period auto-rollback** |
| CM7 `App/app_init.c/h` | MPU + Cache init, peripheral patches (IPC clear / SRAM2 clock / ETH NVIC / **ota_boot_guard + IWDG startup**), **IWDG1 32s watchdog (fed by mqtt_app_task every 500ms)**, ETH_IRQHandler, g_fw_info identity tag (OTA_TEST_* builds flag TEST), key B1 dual-edge EXTI + LED3 linkage, ADC3 internal temperature (bare registers), _gettimeofday (hooked to app_time real time, falls back to boot milliseconds when unsynced) |
| CM7 `App/app_time.c/h` | Time service: RAM anchor as primary + RTC (LSE **bare registers**, following the ADC3 precedent of not touching CubeMX) reset-survival + SNTP callback; downstream = heartbeat time field / TLS timestamp / backplane Modbus broadcast time-sync |
| CM7 `App/app_cli.c/h` | Diagnostic CLI: VCP (USART3 = BSP COM1) idle RX enabled — **USART3_IRQHandler is defined here** (bare registers only fill the ring buffer; legal because CubeMX NVIC didn't tick USART3, precedent = ETH_IRQHandler); defaultTask consumes / echoes / executes each tick; commands help/stat/time/ota/mb/can (rerun self-test)/reboot/revert yes |
| CM7 `App/app_can.c/h` | FDCAN1 internal loopback self-test: HAL driver, internal loopback uses no pins and no transceiver; 500k @ HSE 8MHz; covers init / TX-RX / filter reject / burst; runs Stop + DeInit afterward to leave a clean state |
| CM7 `App/app_mbport.c/h` | Modbus RTU transport layer + on-board master-slave dialogue: USART2 (PD5/6 master) / USART6 (PC6/7 slave) bare registers, **USART2/USART6_IRQHandler defined here** (NVIC not taken over by CubeMX); RTO = 35-bit framing; **USART6 CR2.SWAP adapts to the actual jumper wiring (PD5↔PC6/PD6↔PC7)**; CLI wires/wscan/mbx |
| CM4 `App/app_cm4.c/h` | Yellow LED heartbeat + ping-pong reply (including LED HAL init) |
| CM7 `App/app_mbedtls_config.h` | mbedTLS trimmed config (TLS1.2 / ECDHE-ECDSA / AES-GCM / P-256) |
| CM7 `App/app_certs.h` + `app_certs.c.in` | TLS certificate declarations and template (real entities generated from keys/ca into build at build time; private key not checked in) |
| CM7 `App/app_tls.cmake` | TLS build integration (mbedTLS library + altcp glue + certificate generation + cryptography -O2); appends OTA_TEST test hooks at the end (-DOTA_TEST=HANG\|NONET, for auto-rollback verification) + **Modbus frame core integration** (software/modbus source + header paths) |
| `software/modbus/` (shared, outside the project) | Modbus RTU frame core: CRC/ADU/PDU frame building and parsing (modbus_core.c/h) + boot self-test (modbus_selftest.c, 38 cases run on-board); **pure C99, no HAL**, shared by the front-panel 485 master stack / backplane master-slave (reused by H503) / future TCP; contract = docs/Backplane_Bus_Protocol.md v0.2 |
| CM7 `App/app_se.h` + `app_se_soft.c` + `app_se_608b.c` | Secure-element abstraction (anti-cloning): software backend (dev/test) / 608B backend (production-board stub) |
| CM7 `App/app_anticlone.c/h` | Anti-cloning challenge-response (SE signature + embedded public-key verification) |
| Both `.ld` | Linker scripts are not regenerated with the .ioc; managed as user files (contents in List III) |

The CMakeLists.txt user-source region (`# Add user sources here`) must list the above .c files; this region is not overwritten by CubeMX.

## II. Interface Call Sites Inside Generated Files (all allowed intrusions; copy back after regeneration)

| File | USER CODE block | Content (one line) |
|---|---|---|
| CM7 `main.c` | Includes | `#include "app_init.h"` / `#include "app_mqtt.h"` / `<stdio.h>` |
| CM7 `main.c` | 1 | `app_mpu_cache_init();` (must precede all peripherals) |
| CM7 `main.c` | 2 | `app_periph_init();` |
| CM7 `main.c` | 5 | `mqtt_app_task();` |
| CM7 `main.c` | `BSP_PB_Callback` function body | `app_button_isr();` (key EXTI → App; ⚠️ this function body is NOT inside a USER CODE block, must be copied back after regeneration) |
| CM7 `main.c` | PD + Boot_Mode_Sequence_* | ST dual-core boot synchronization official template (not counted as intrusion, preserved on regeneration) |
| CM4 `main.c` | Includes | `#include "app_cm4.h"` |
| CM4 `main.c` | 5 | `app_cm4_task();` |
| CM4 `main.c` | PD + Boot_Mode_Sequence_* | Same dual-core official template as above |
| CM7 `stm32h7xx_it.c` | 2 | ① Comment (ETH_IRQHandler is in app_init.c; if CubeMX NVIC ticks the ETH interrupt, delete the hand-written version in app_init.c); ② **fault black box** [registered 2026-07-19]: the USER CODE regions of the four fault handlers (HardFault/MemManage/BusFault/UsageFault) write magic `0xFA017CB7` + CFSR/BFAR/faulting PC/PSP to SRAM4 **0x3800E440**, then reset immediately (relocated 2026-07-19 from 0x38008480 — the old address was never registered in the resource table and the process image claimed it, landing on the hsdi configuration). Must be reapplied after regeneration |

## III. Defect-Patch List (Rule 2 exceptions; regeneration reverts them, must be reapplied)

| Location | Manual edit | Reason / CubeMX equivalent |
|---|---|---|
| CM7 `main.c` defaultTask_attributes | stack_size 128 → **1024 words** | MX_LWIP_Init runs in this task, 512B stack overflow = HardFault; GUI: FreeRTOS(M7) → Tasks → 1024 |
| CM7 `LWIP/Target/lwipopts.h` | `LWIP_RAM_HEAP_POINTER` → **0x30030000** | Default 0x30004000 = CM4 memory, dual cores stomp each other and crash on packet receive; GUI has this item |
| CM7 `LWIP/Target/lwipopts.h` USER CODE | `MEM_SIZE=16KB` | When not generated, defaults to 1600B → TX pbuf deadlock; USER CODE auto-preserved ✓ |
| CM7 `LWIP/Target/lwipopts.h` | `TCPIP_THREAD_STACKSIZE` → **2048** | Overflows under OTA load; GUI has this item |
| CM7 `LWIP/Target/ethernetif.c` | `INTERFACE_THREAD_STACK_SIZE` 350 → **2048** | Common ST template flaw, overflow stomps the tcpip mutex = assertion spin fake-hang; no GUI item, reapply manually after regeneration |
| CM7 `LWIP/Target/ethernetif.c` | `ETH_RX_BUFFER_CNT` 12 → **24** | A burst of public-net OTA chunks blows through the 12-buffer pool; no GUI item; RAM_D2 must not cross the 0x30030000 LwIP heap |
| CM7 `LWIP/Target/lwipopts.h` USER CODE | `LWIP_ALTCP`/`_TLS`/`_TLS_MBEDTLS`=1 + `TCPIP_THREAD_STACKSIZE` 2048 → **8192** (TLS handshake runs in this thread, ECC eats stack) + `MEM_SIZE` kept at 16K | mTLS; USER CODE auto-preserved, the stack item has a GUI entry |
| CM7 `Core/Inc/FreeRTOSConfig.h` | `configTOTAL_HEAP_SIZE` 30720 → **40960** | Companion to the enlarged tcpip stack; GUI: FreeRTOS → TOTAL_HEAP_SIZE |
| `Middlewares/.../altcp_tls/altcp_tls_mbedtls_mem.c` | `mbedtls_platform_set_calloc_free` in `altcp_mbedtls_mem_init` **disabled (`&& 0`)** | Original code hijacks the allocator to the 16K LwIP heap → the SSL 16K buffer is guaranteed to fail; switched to an independent 40K static pool in app_init.c |
| `Middlewares/.../mqtt/mqtt.c` | `mqtt_parse_incoming` chained-pbuf fix (`pbuf_skip`) + zero-advance broken-chain fuse | Original code's `p->len-in_offset` computes 0 for subsequent nodes of a chained pbuf → cpy_len=0 infinite loop starves the whole machine (guaranteed under TLS / high traffic) |
| `Middlewares/.../mbedTLS/` whole library | Copied in from local ST firmware pack FW_H7_V1.13.0 | Compilation needs `-O2` (-O0 software ECC handshake 20s+ gets killed by the connection watchdog) |
| `Middlewares/.../altcp_tls/altcp_tls_mbedtls.c` | Added SNI injection after ssl_setup (reads global g_altcp_sni_hostname, calls mbedtls_ssl_set_hostname) | This version of altcp_tls has no SNI API; AWS routes by SNI |
| `Middlewares/.../altcp_tls/altcp_tls_mbedtls.c` | `altcp_mbedtls_dealloc`: also free `state->rx_app` | Upstream defect: decoded-but-undelivered application-data pbufs are never freed on abnormal close; repeated occurrences starve PBUF_POOL (2026-07-24 leak audit) |
| CM7 `LWIP/Target/lwipopts.h` USER CODE | `LWIP_DNS=1` | Connecting to AWS by domain name needs DNS resolution |
| CM7 `LWIP/Target/lwipopts.h` USER CODE 0 | `MEMP_NUM_SYS_TIMEOUT=12` | Default = number of internal timers, leaving 0 headroom for the app layer; the mqtt client hangs 1 keepalive sys_timeout per connection (not in the internal count) + enabling DNS to reach AWS + frequent broker-switch reconnects → pool empty → assertion blows up the tcpip thread (timeouts.c:190) → network dies right after getting an IP at boot. |
| CM7 `LWIP/Target/lwipopts.h` USER CODE 0 | `SNTP_SERVER_DNS=1` + `SNTP_SET_SYSTEM_TIME(sec)` → app_time_sntp_cb | Time service; occupies 1 sys_timeout (within the headroom of 12); USER CODE auto-preserved ✓ |
| CM7 `Core/Inc/FreeRTOSConfig.h` | `configAPPLICATION_ALLOCATED_HEAP=1` | Offload DTCM: the RTOS heap is defined in app_init.c into the AXI .axisram section; reapply after regeneration |
| CM7 `.ld` | +RAM_AXI (0x24000000/512K) + `.axisram` NOLOAD section | mbedTLS pool + RTOS heap moved to AXI; folded into the existing .ld user-file management |
| CM7 `Core/Inc/stm32h7xx_hal_conf.h` | `HAL_FDCAN_MODULE_ENABLED` turned on | A5 FDCAN loopback self-test; GUI: CubeMX enables it automatically when the FDCAN peripheral is activated; reapply manually if regeneration reverts |
| `Drivers/.../Src/stm32h7xx_hal_fdcan.c` + `Inc/stm32h7xx_hal_fdcan.h` | Copied in from FW_H7_V1.13.0 (the project's trimmed HAL tree didn't originally include it; version cross-checked identical 1.11.x) | Build hooked in App/app_tls.cmake (target_sources to the STM32_Drivers target) |
| `Middlewares/.../LwIP/src/apps/sntp/sntp.c` | Copied in from local ST firmware pack FW_H7_V1.13.0 (the tree originally had only the header) | SNTP client source; build hooked in App/app_tls.cmake (target_sources to the LwIP target) |
| CM7 `App/app_mbedtls_config.h` | RSA_C/PKCS1_V15/ECDHE_RSA (verify AWS RSA server cert chain), **MBEDTLS_SSL_SERVER_NAME_INDICATION** (send SNI extension, absence → AWS rejects), IN_CONTENT_LEN=8192 (hold AWS's large CertReq) | Required to connect to AWS; the device side still uses an ECDSA certificate |
| CM7 `LWIP/Target/ethernetif.c` USER CODE 2 | `ETH_TxPacketConfig TxConfig;` | Generation defect: definition missing; USER CODE auto-preserved ✓ |
| CM7 `Core/Inc/FreeRTOSConfig.h` | `configTOTAL_HEAP_SIZE` → **30720** | Companion to the enlarged stack; GUI: FreeRTOS → TOTAL_HEAP_SIZE |
| CM7 `.ld` | FLASH LENGTH 1024K → **512K**; +RAM_D2 (0x30020000, 128K) + `.lwip_sec`; + **`.fw_info` section @ORIGIN+0x400** | OTA dual-bank layout; ETH DMA cannot reach DTCM; identity tag at fixed offset (synced with app_ota.h FW_INFO_OFF) |
| CM4 `.ld` | FLASH ORIGIN → **0x08080000/512K**; RAM 288K → **128K** | OTA layout (companion to ota-provision flashing BOOT_CM4_ADD0); SRAM2 yielded to CM7 |
| CM4 `Core/Inc/FreeRTOSConfig.h` | `configTOTAL_HEAP_SIZE` 15360 → **49152** [2026-07-19] | With one thread per bus, the 7×3 KB thread stacks are allocated from the FreeRTOS heap; 15 K only covers defaultTask + OpenAMP — **without this change `osThreadNew` silently returns NULL and every bus is dead**. `app_bus_cm4.c` writes the number of successfully started threads into the breadcrumb word 0x38008404 (expected **0xB0000008** — 7 bus threads + rpc = 8 threads) to defeat the silence |

### platform_h503

| File | Direct edit | Reason |
|---|---|---|
| `STM32H503xx_FLASH.ld` / `_RAM.ld` | Delete the **`(READONLY)`** keyword from all section declarations | The GCC 10.2 linker doesn't recognize it (≥GCC11 only); the .ld's own comment says so too; **CubeMX regenerating the .ld restores it, rerun after regeneration**: `sed -i 's/ (READONLY) :/ :/' STM32H503xx_*.ld` |
| `main.c` / `stm32h5xx_it.c` | USER CODE blocks only (include / bsp_16o_init / poll + feed-dog / 3 interrupt forwards) | Compliant, auto-preserved on regeneration |
| `CMakeLists.txt` user-source region | +modbus×3 / uart_drv / app_16o×2 + three include directories | The generator does not overwrite this file |



### platform_757

> **Overall policy: the .ioc is frozen = the June whole-machine version's generated tree is treated as a "library"; thereafter all configuration/trimming goes through code, no going back around the CubeMX loop** (avoiding the two-headed maintenance where "AI-edited code gets wiped by regeneration"). If regeneration is truly required (not expected): first reapply all direct edits per this section.

## IV. Coding Discipline and Address Conventions (belong to no single file, belong to all files)

- LwIP core APIs must never be called directly across threads, always dispatch via `tcpip_callback`; **printf is forbidden inside tcpip-thread callbacks** (printf belongs only to defaultTask).
- No re-programming of the same flash word between two erases (the ECC breaks, read-back bus error).
- **D2 SRAM partitioning**: SRAM1 (0x30000000/128K) = CM4; SRAM2 (0x30020000/128K) = CM7 network (descriptors + RX pool link allocation, 0x30030000 LwIP heap 16K, 0x30038000 OTA ring 32K — the latter two are **bare addresses**, unknown to the linker, so RAM_D2 linker usage must not exceed 64K); SRAM4 (0x38000000) = dual-core IPC.
- The SRAM4 ping-pong addresses 0x38000000/04 are hard-coded in two places, CM7 `App/app_mqtt.h` and CM4 `App/app_cm4.c`, changes must be synced; **64B starting at 0x38000100 = OTA trial-period guard (app_ota.c, magic + boot counter + event string, reset-persistent)**, new SRAM4 occupants queue after it.
- **IWDG1 cannot be stopped by hardware once started** (started by app_periph_init, 32s): any newly added blocking operation in the defaultTask main loop must be < 32s (current longest = full-bank OTA erase ~8s); debug breakpoints already set DBGMCU freeze to stop the timer.
- ⚠️ ninja doesn't track .ld dependencies; do a full rebuild after editing a .ld.
- **Hardware-resource numbering discipline**: before adding any peripheral/interrupt/DMA/memory occupation, you must first check and sync-update `docs/Hardware_Resource_Allocation.md` (UART master ledger / interrupt vectors / SRAM4 partitioning / DMA channels are all there).

## V. Post-Regeneration Check Flow

1. List II: copy the interface-call lines back into the USER CODE blocks (in theory auto-preserved, verify once).
2. List III: check line by line, confirm the GUI value for items that have a GUI entry, manually reapply items with no GUI entry.
3. Verify the CMakeLists user-source region (in theory not overwritten).
4. Full rebuild + one round of local OTA push verification.
