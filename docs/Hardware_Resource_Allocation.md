# I757-M (H757 dual-core) — Hardware Resource Allocation

> **Iron rule: any newly added peripheral / interrupt / DMA / memory usage MUST update this table in sync** (same-level discipline as `CubeMX_Code_Boundary_Rules.md`).
> Scope = resources in use by the I757-M dual-core firmware. Streams/pins below are the actual allocation; where the generated layer (`.ioc`) already fixes a DMA stream, do not hand-code it.

## 1. Core Division of Labor

| Core | Clock | Responsibilities | Main Loop |
|---|---|---|---|
| CM7 | 480 MHz | Networking (ETH/LwIP/MQTT/mTLS), OTA + trial period, time (RTC/SNTP), CLI (USB CDC), RPMsg host, application/cloud | defaultTask 500 ms tick |
| CM4 | 240 MHz | RPMsg remote, **Modbus transport + host scheduler + slave map**, 6×485 front ports, backplane bus master | 1 ms tick (slave-response real-time relies on it) |

## 2. Flash (2 MB dual bank, OTA swap)

| Address (offset within bank) | Size | Purpose |
|---|---|---|
| +0x00000 | 512K region (usable ~264K) | CM7 image; **+0x400 = fw_info identity tag** (magic `I757` + version string) |
| +0x80000 | 512K region (usable ~39K) | CM4 image (`BOOT_CM4_ADD0 = 0x08080000` option byte) |
| +0xFFF40 | 32B (1 flash word) | **OTA trial-period confirmation word** (all-FF = trial period; ota-begin write upper bound = this address) |
| +0xFFF60 | 160B | **OTA footer** (size/CRC/version/signature; swap-gate credential) |

The running bank is always mapped to 0x08000000, the spare always to 0x08100000 (erase/write target). Option bytes: `SWAP_BANK`, `BOOT_CM4_ADD0`.

## 3. RAM Overall Map

| Region | Address / Size | Owner | Purpose |
|---|---|---|---|
| DTCM | 0x20000000 / 128K | CM7 | Main RAM (.data/.bss/stack, self-test static buffers) |
| AXI SRAM | 0x24000000 / 512K | CM7 | **.axisram section (NOLOAD)**: mbedTLS pool + FreeRTOS heap; bulk of the block free |
| ITCM | 0x00000000 / 64K | — | Unused (candidate for extreme-fast code) |
| D2 SRAM1 | 0x30000000 / 128K | CM4 | CM4 main RAM (aliased via 0x10000000 on the CM4 side) |
| D2 SRAM2 | 0x30020000 / 128K | CM7 | Networking region, subdivided below |
| ├ | 0x30020000~2FFFF (≤64K) | CM7 | ETH DMA descriptors + RX pool (linker-allocated, must not exceed 64K) |
| ├ | 0x30030000 / 16K | CM7 | LwIP heap (`LWIP_RAM_HEAP_POINTER`, raw address) |
| ├ | 0x30034000 / 16K | — | Isolation band (overrun sentinel) |
| └ | 0x30038000 / 32K | CM7 | OTA ring buffer (raw address, produced by tcpip / consumed by defaultTask) |
| D2 SRAM3 | 0x30040000 / 32K | — | Free |
| D3 SRAM4 | 0x38000000 / 64K | Dual-core shared | Subdivided below (**new occupants go after 0x3800E460**) [2026-07-19] |
| ├ | 0x38000100 / 64B | CM7 | OTA trial-period guard (magic + boot count + evt string, retained across reset) |
| ├ | 0x38000200 / 0x200 | Dual-core | OpenAMP `resource_table` (~140B) |
| ├ | 0x38000400 / 31K | Dual-core | OPEN_AMP_SHMEM: vring RX@+0 / TX@+0x400 / buffer@+0x800 (`RPMSG_BUFFER_SIZE=512`) |
| ├ | 0x38008400 / 0x20 | CM4 write / SWD read | **CM4 debug breadcrumbs** (boot stage / callback count / last event; read via `STM32_Programmer_CLI HOTPLUG -r32`) |
| 0x38008420 / 0x20 | both cores | **Fault records** (stack-overflow / heap-exhaustion hooks): CM7@+0x00, CM4@+0x10, 16 B each (magic|type + offending task name). Reset-persistent, printed by the `tasks` CLI, cleared by `health clear`. Header = `Common/app_health.h` |
| ├ | 0x38008440 / ≤24K | Dual-core | **Process image** [2026-07-19]: ctrl + scan config table + input/output images + status region (CM4 scans and writes, CM7 application reads/writes; seqlock consistency; layout single source of truth = `Common/app_pimage.h`). Contract = `Process_Image_and_IO_Mapping.md` |
| └ | 0x3800E440 / 32B | CM7 write / SWD read | **CM7 fault black box** [2026-07-19]: the four fault handlers (HardFault/MemManage/BusFault/UsageFault) write magic `0xFA017CB7` + CFSR/BFAR/faulting PC/PSP here, then reset immediately; read it back over SWD to get the truth. **Relocated 2026-07-19** from 0x38008480: the old address was never registered in this table, and when the process image claimed its block it landed exactly on the `hsdi[2..5]` configuration — claiming an address MUST register it here |
| Backup SRAM | 0x38800000 / 4K | VBAT (battery) | **Entire block reserved for the application** (power-fail retained data; see `App/app_pwrfail.h`). Platform survival state uses SRAM4 + TAMP instead. |

MPU (CM7): Region0 = 0x30020000 / 128K non-cacheable (ETH DMA region), Region1 = 0x38000000 / 64K non-cacheable (guard/OpenAMP). CM4 has no cache, so no configuration needed.

## 4. UART / USART Ledger (CM4 owns all field ports)

| Peripheral | Role | Pins | DE | DMA (RX ring + IDLE / TX) |
|---|---|---|---|---|
| USART1 | **485A** configurable Modbus port (isolated CA-IS2092A) | PB14/PB15 | PJ15 (GPIO) | RX=DMA2_S4 / TX=DMA2_S5 |
| USART2 | **485B** configurable Modbus port (isolated CA-IS2092A) | PD5/PD6 | PD4 (hardware) | RX=DMA1_S2 / TX=DMA1_S3 |
| USART3 | **485C** configurable Modbus port (non-isolated) | PB10/PB11 | PI1 (GPIO) | RX=DMA2_S2 / TX=DMA2_S3 |
| UART4 | **485D** configurable Modbus port (non-isolated; ⚠️ PA0 shares a pad with AI0_C) | PA0/PI9 | PA15 (hardware) | RX=DMA2_S0 / TX=DMA2_S1 |
| USART6 | **485E** configurable Modbus port (non-isolated; terminal on board 2) | PG14/PG9 | PA8 (GPIO) | RX=DMA1_S6 / TX=DMA1_S7 |
| UART7 | **485F** configurable Modbus port (non-isolated; terminal on board 2) | PB4/PB3 | PH15 (GPIO) | RX=DMA1_S4 / TX=DMA1_S5 |
| UART8 | **Backplane expansion-bus master** | PJ8/PJ9 | PD11 | RX=DMA1_S0 / TX=DMA1_S1 |
| UART5 | Spare | PB5=RX | — | — |
| LPUART1 | Slot-C sub-card serial | PA9/PA10 | — | — |
| USB OTG_FS | **CLI console (USB-C service port)** | PA11/PA12 | — | USBD stack (no UART consumed) |

The six front ports (485A~F) are software-configurable as Modbus master or slave; contract = `Universal_Modbus_Port_Config.md`. The backplane bus (UART8) is fixed master; contract = `Backplane_Bus_Protocol.md`.

## 5. Interrupt Vectors (hand-written handlers reside in App user files)

| Core | IRQ | Priority | Handler | Purpose |
|---|---|---|---|---|
| CM7 | ETH_IRQn | 7 | app_init.c | Ethernet TX/RX |
| CM7 | OTG_FS_IRQn | — | USBD stack | USB CDC console (CLI) |
| CM7 | HSEM1_IRQn | 7 | app_rpc.c | RPMsg mailbox kick |
| CM4 | HSEM2_IRQn | 6 | app_rpc_cm4.c | RPMsg mailbox kick |
| CM4 | UART8_IRQn | 6 | app_mbport_cm4.c | Backplane master (IDLE frame-end + errors) |
| CM4 | DMA1_Stream0~1_IRQn | 6 | app_mbport_cm4.c | Backplane UART8 TX/RX DMA |
| CM4 | USART1/2/3, UART4, USART6, UART7 IRQn | 6 | app_mbfront_cm4.c | 485A~F ports (IDLE frame-end + errors, raise-flag only) |
| CM4 | DMA1_Stream2~7_IRQn, DMA2_Stream0~5_IRQn | 6 | app_mbfront_cm4.c | 485A~F TX/RX DMA |

Discipline: FreeRTOS syscall priority ceiling = 5; any ISR using RTOS APIs must have priority ≥ 5. All ISRs in this table **do not touch RTOS** — they only feed the ring buffer / kick the mailbox.

## 6. DMA

| Controller | Occupancy |
|---|---|
| DMA1 Stream0~1 | CM4 backplane master UART8: S0=RX (ring) S1=TX |
| DMA1 Stream2~3 | CM4 485B / USART2: S2=RX S3=TX |
| DMA1 Stream4~5 | CM4 485F / UART7: S4=RX S5=TX |
| DMA1 Stream6~7 | CM4 485E / USART6: S6=RX S7=TX (all eight DMA1 streams claimed) |
| DMA2 Stream0~1 | CM4 485D / UART4: S0=RX S1=TX |
| DMA2 Stream2~3 | CM4 485C / USART3: S2=RX S3=TX |
| DMA2 Stream4~5 | CM4 485A / USART1: S4=RX S5=TX |
| DMA2 Stream6~7 / BDMA / MDMA | Free (SDMMC uses IDMA, does not occupy this) |
| ETH dedicated DMA | In use (descriptors + buffers in SRAM2, see §3) |

Stream allocation is fixed in the generated layer (`.ioc` / `it.c` / `usart.c`) — do not hand-code it (`Universal_Modbus_Port_Config.md`).

## 7. Other Peripherals / System Resources

| Resource | State | Notes |
|---|---|---|
| HSEM 0/1 | Dual-core | 0 = boot sync + RPMsg kick; 1 = RPMsg kick reverse direction |
| IWDG1 | CM7 always on | 32 s, fed by defaultTask every 500 ms; **defaultTask blocking must be < 32 s** (longest = OTA bank erase ~8 s); DBGMCU freeze at breakpoints |
| IWDG2 | CM4 always on | 32 s, fed by the 1 ms main loop; a bite = full-system reset → falls into the OTA trial-period safety net |
| RTC | CM7 | LSE 32768, raw registers; time service survives reset |
| RNG | CM7 | HSI48 clock source; mbedTLS entropy / ECDSA |
| ADC3 | CM7 | Internal temperature (ch18); ADC1/2 free for the analog front end |
| FDCAN1/2 | Field CAN ×2 | one isolated; `can` self-test uses internal loopback |
| USB OTG_FS | Service port | USB-C CDC console (CLI) |
| FLASH controller | CM7 | OTA erase/write (all in defaultTask; tcpip thread forbidden to write) |
| TIM5 | CM7 | Buzzer PWM — CH4 @ PI0 (~2.7 kHz, MLT-8530 passive) |
| TIM1 / TIM3 / TIM8 | **CM4** | Onboard HSDI counters / quadrature encoders (contract: `HSDI_Configuration_and_Counting.md`; owned by the realtime core with the field buses) [2026-07-19 correction — an earlier row said CM7] |
| Other TIM / LPTIM | Free | — |
| Status LED | CM7 | LD1 green run/heartbeat @ **PG10** (GPIO output; LD1 is a dual-color LED, common cathode). [2026-07-19] Correction: PG10, **not PB0** — PB0 belongs to HSDI2. No user push-button |

## 8. Clock Tree Key Points

HSE 25 MHz crystal → PLL1 (M=5 → 5 MHz PFD, N=192 → VCO 960 MHz, P=2) → **SYSCLK 480 MHz**; AHB (HCLK) = SYSCLK ÷2 = **240 MHz**; APB1/2/3/4 = HCLK ÷2 = **120 MHz**. HSI48 → RNG; LSI → IWDG; LSE 32.768 kHz → RTC; FDCAN kernel clock = HSE. 480 MHz is reached via the H7 SMPS→LDO cascade.

## 9. Memory Block Personality Profiles (address-selection basis)

| Block | Address / Size | Domain | Who Can Access | Cache | Power-loss / Reset | Role |
|---|---|---|---|---|---|---|
| ITCM | 0x00000000 / 64K | CM7 private | CM7 only (+MDMA back door) | Not cached | Lost on reset | Unused (extreme-fast code candidate) |
| DTCM | 0x20000000 / 128K | CM7 private | CM7 (+MDMA via AHBS); **DMA1/2/BDMA/ETH cannot reach it!** | Not cached | Lost on reset | CM7 stack/heap/mbedTLS pool/RTOS heap |
| AXI SRAM | 0x24000000 / 512K | D1 | Dual-core + MDMA + DMA1/2 | CM7 via D-Cache (maintain / disable when shared with DMA) | Lost on reset | Large buffers (bulk free) |
| D2 SRAM1 | 0x30000000 / 128K | D2 | Dual-core + DMA1/2 + ETH/SDMMC/USB; CM4 native | CM7 via cache | Lost on reset | CM4 main RAM (alias 0x10000000) |
| D2 SRAM2 | 0x30020000 / 128K | D2 | Same as above | Cache disabled by MPU | Lost on reset | CM7 networking region (ETH DMA / LwIP / OTA ring, §3) |
| D2 SRAM3 | 0x30040000 / 32K | D2 | Same as above | — | Lost on reset | Free |
| D3 SRAM4 | 0x38000000 / 64K | D3 always-awake | Dual-core + **only RAM reachable by BDMA** | Cache disabled by MPU | **Retained across system reset**, lost on power-off | Inter-core shared (guard / RPMsg, §3) |
| Backup SRAM | 0x38800000 / 4K | VBAT | Dual-core | — | **Not lost on power-off with battery** | **Reserved for the application** (power-fail retained data) |
| **FMC SDRAM** | 0xC0000000 / **32MB** | D1 (FMC), SDCLK 100 MHz | CM7 (CM4 can reach it physically; unused there) | MPU Region4 = WBWA cacheable — do cache maintenance before handing SDRAM buffers to DMA | Lost on power-off, and NOT preserved across reset (controller re-init required) | **[2026-07-19] In the linker memory pool**: `SDRAM` region + `.sdram` NOLOAD section; applications use the `APP_SDRAM` attribute (app_platform.h) for MB-scale buffers. ⚠️ The controller comes up in `app_p5_test_run()` on the FIRST main-loop pass (same place as the 32MB self-test) — nothing in SDRAM is loaded, zeroed, or touchable before that; tasks created in app_user_init are safe |


## QSPI serial flash partition map (W25Q256 32 MB, JEDEC EF4019) [registered 2026-07-19]

The single code source of this layout is `CM7/App/app_qflash.h`; changing the split means changing both.

| Chip offset | Size | Use | Notes |
|---|---|---|---|
| 0x0000000 – 0x1DFFFFF | **30 MB** | **littlefs** (7680 × 4 KB blocks) | Config (mbport.cfg), staged firmware (t*.fw), application files (`app_lfs()`, manual §5.5). A superblock size change fails the first mount and auto-formats |
| 0x1E00000 – 0x1FFFFFF | **2 MB** | **Power-fail save area (raw page writes)** | Layer 2 of the power-fail scheme: **170 slots × 12 KB**, rotated; kept pre-erased in normal operation, only sequential page programming at end-of-life — **no erase, no file system** inside the ~39 ms dying window (iron rule); `app_pf_blackbox_set/last` (app_pwrfail.h) |

**Address-selection mnemonic**: CPU-private and fast → DTCM; buffers feeding ETH/DMA1/2 → D2; two-core shared / reset-survival → SRAM4; just big → AXI; REALLY big (MB-scale, late init acceptable) → SDRAM (`APP_SDRAM`); must survive power-off (with battery) → Backup; extreme-fast code → ITCM.
**Two classic pitfalls**: ① a DMA buffer in DTCM = DMA silently cannot reach it (the ETH buffer lives in D2 for exactly this reason); ② CM7 accessing a DMA-shared region via cache without MPU/maintenance = data ghosts.
