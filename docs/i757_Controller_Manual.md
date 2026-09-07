# Industry_757 (I757-M) Modular Industrial Controller — Controller Manual v1.0

> Audience: integrators, electrical/field engineers, installers, application developers.
> **One manual per board**: this volume covers the complete main unit (main board + Board 2 + option cards) — specifications, installation & wiring, and operation. Expansion modules have their own manuals (`EX_16DO_Module_Manual.md`, `PH_EC_Transmitter_User_Manual.md`); on-board application development = `Software_Manual_Application_Development.md`; each interface's authoritative definition is in the corresponding contract doc under `docs/`.

---

## 1. Overview

I757-M is a modular industrial controller. A **main board** provides compute, networking,
storage and field buses; I/O capacity scales horizontally via **expansion modules** on a
backplane bus (1 internal blade slot + up to 15 side-mounted, 16 module positions total,
auto-discovered). Every capability the controller exposes is published to the cloud as a
**self-describing device model** — upstream software renders and controls it with zero
per-device code.

```
              Cloud (MQTT/TLS)
                  │ Ethernet
   ┌──────────────┴───────────────┐
   │         I757-M main board      │── Field buses: 6×RS-485 / 2×CAN / 8×fast DI / ADC
   │   CM7 (network)  CM4 (realtime)│
   └──────────────┬───────────────┘
                  │ Backplane bus (Modbus RTU)
   ┌────┬────┬────┴────┬───────────┐
  mod1 mod2  …       mod16   (relay / water-quality / …)
```

### 1.1 Configuration Tiers

All three tiers have **identical communication interfaces, I/O and expansion capacity**;
they differ only in three main-board options:

| Configuration | Basic | Pro | Full |
|---|---|---|---|
| Dual-core H757 + 2 MB dual-bank flash + 1 MB RAM | ✅ | ✅ | ✅ |
| microSD + RTC battery + all communication interfaces | ✅ | ✅ | ✅ |
| ATECC608A anti-clone / hardware identity (cloud mTLS) | ✅ | ✅ | ✅ |
| 32 MB external SDRAM | — | ✅ | ✅ |
| 32 MB external SPI flash (filesystem + power-fail save area) | — | ✅ | ✅ |
| SE050 high-security element (RSA / multi-identity / full cert chain in chip) | — | — | ✅ |

## 2. Specifications

| Item | Spec |
|---|---|
| MCU | STM32H757XIH6 dual-core |
| CM7 | Cortex-M7 @ 480 MHz — networking (Ethernet/TLS/MQTT), OTA, storage, application/cloud |
| CM4 | Cortex-M4 @ 240 MHz — real-time field I/O, Modbus master/slave, backplane bus master |
| Internal flash | 2 MB dual-bank (A/B images, fail-safe OTA) |
| Internal RAM | 1 MB (DTCM/AXI/D2/D3 SRAM domains) |
| External SDRAM | 32 MB (W9825G6KH, FMC; Pro/Full) |
| External SPI flash | 32 MB (W25Q256, on-board; first 30 MB = LittleFS, last 2 MB = power-fail save area; Pro/Full) |
| microSD | FAT filesystem (removable, PC-readable; logs / data export) |
| RTC | 32.768 kHz crystal + battery (CR1220); SNTP network time sync |
| Secure elements | ATECC608A (standard) + SE050 (Full) |
| Power supply | 9–36 V DC (24 V nominal) |
| Dimensions | approx. 90 × 105 × 83 mm, DIN 35 mm rail mount |

## 3. Installation & Wiring

### 3.1 Safety Notes (read first)

- This is a **SELV low-voltage device**: 9–36 V DC supply; all I/O terminals are low-voltage. **Never connect mains voltage (110/230 VAC) to any terminal.**
- The **PE protective-earth terminal must be reliably connected** (power terminal block, two ring-in/ring-out positions).
- **Never plug or unplug expansion modules under power**: switch off the 24 V supply before inserting or removing any module (internal blade slot or side-mounted).
- Inductive loads (solenoid valves, contactors, locks) on relay/output modules require a flyback diode (DC) or RC snubber (AC) at the load.
- The USB-C service port **does not power the board** (VBUS not connected); it is communication-only. The controller needs its 24 V supply during debugging.

### 3.2 Mechanical Installation

- **DIN 35 mm rail mount**; blade-style modular enclosure, main unit approx. **90 × 105 × 83 mm**, expansion modules use the narrow 18-terminal housing.
- The main enclosure contains: main board (board 1) + Board 2 + **1 blade slot** (accepts any expansion module); **up to 15 more expansion modules mount on the side** of the enclosure (16 module positions total, daisy-chained backplane bus).
- Environment: cabinet installation; leave ventilation clearance above and below; keep away from strong interference sources (VFDs, contactors).

### 3.3 Power Wiring (5.0 mm terminal block, 6 positions)

| Terminal | Function |
|---|---|
| V+ ×2 | Supply positive, 9–36 V DC (two terminals = ring in / ring out) |
| GND ×2 | Supply negative (ring in / ring out) |
| PE ×2 | Protective earth (ring in / ring out) |

- The second position of each pair may feed downstream devices; the 5.0 mm block has limited current capacity — **do not route heavy downstream loads (several amps or more) through the controller**.
- No UPS required: the controller is designed to tolerate power loss at any moment (built-in power-fail data retention, see §8).

### 3.4 RS-485 (6 ports)

| Port | UART | Terminal location | Isolation | DE | Recommended use |
|---|---|---|---|---|---|
| Port 1 (485A) | USART1 | Main board, P3 | ✅ isolated (CA-IS2092A, dedicated ground GND_A) | PJ15 (GPIO) | Field / long lines |
| Port 2 (485B) | USART2 | Main board, P3 | ✅ isolated (CA-IS2092A, dedicated ground GND_B) | PD4 (hardware) | Field / long lines |
| Port 3 (485C) | USART3 | Main board, P3 | ❌ non-isolated | PI1 (GPIO) | In-cabinet / short runs |
| Port 4 (485D) | UART4 | Main board, P3 | ❌ non-isolated | PA15 (hardware) | In-cabinet / short runs |
| Port 5 (485E) | USART6 | **Board 2 terminals** | ❌ non-isolated | PA8 (GPIO) | In-cabinet; access-control (OSDP) etc. |
| Port 6 (485F) | UART7 | **Board 2 terminals** | ❌ non-isolated | PH15 (GPIO) | In-cabinet; access-control (OSDP) etc. |

- Wiring: A–A, B–B daisy-chain bus topology; third wire to the matching ground (GND_A/GND_B for isolated ports, GND for non-isolated ports).
- **No on-board termination**: if the controller sits at the end of the bus, clamp an external **120 Ω** resistor directly into the A/B terminal positions. Fail-safe biasing is already on board.
- Use twisted pair; for long field runs always use the isolated ports (1/2) with shielded cable, shield grounded at one end.
- Every port is software-configurable as **Modbus master or slave** (see §7.2).

### 3.5 CAN (2 channels, main board P3)

- CAN1 = non-isolated (TJA1051, referenced to GND); CAN2 = **isolated** (CA-IS2062A, referenced to isolated ground GND_CA).
- H–H, L–L; 120 Ω termination at both bus ends (not on board — clamp at the terminals).
- Hardware provisioned; firmware support per roadmap.

### 3.6 Ethernet & USB-C Service Port

- **Ethernet**: 10/100 RJ45 (LAN8742 PHY, RMII); standard cable, Cat-5e or better; use shielded cable in low-voltage trays for long field runs. TLS 1.2 mutual auth; each device has a unique X.509 identity.
- **USB-C service port**: virtual COM port (CDC, enumerates as a COM port on the PC), full diagnostic CLI (see §7.1); debug / firmware download only, **does not supply power**; located inside the enclosure as a maintenance interface.

### 3.7 Main Board P3 Terminal Pinout (3.5 mm, two vertical rows, 2×9 = 18 positions)

Arranged as physically seen **facing the front panel**: **right row = odd positions 1–17, left row = even positions 2–18**, top to bottom:

| #  | Left row signal                | #  | Right row signal               |
|----|--------------------------------|----|--------------------------------|
| 2  | 485 port 2 A (isolated)        | 1  | 485 port 1 A (isolated)        |
| 4  | 485 port 2 B (isolated)        | 3  | 485 port 1 B (isolated)        |
| 6  | GND_B (port 2 isolated ground) | 5  | GND_A (port 1 isolated ground) |
| 8  | 485 port 3 A                   | 7  | CAN2 H (isolated)              |
| 10 | 485 port 3 B                   | 9  | CAN2 L (isolated)              |
| 12 | GND (port 3)                   | 11 | GND_CA (CAN2 isolated ground)  |
| 14 | 485 port 4 A                   | 13 | CAN1 H                         |
| 16 | 485 port 4 B                   | 15 | CAN1 L                         |
| 18 | GND (port 4)                   | 17 | GND (CAN1)                     |

(Always follow the silkscreen printed on the board.)

### 3.8 Board 2 Field Terminal Pinout (P1, 3.5 mm, two vertical rows, 2×10 = 20 positions)

Arranged as physically seen **facing the front panel**: **left row = odd positions 1–19, right row = even positions 2–20**, top to bottom:

| #  | Left row signal            | #  | Right row signal           |
|----|----------------------------|----|----------------------------|
| 1  | HSDI0_A / HSDI1_A (shared) | 2  | HSDI4_A / HSDI5_A (shared) |
| 3  | HSDI0_B                    | 4  | HSDI4_B                    |
| 5  | HSDI1_B                    | 6  | HSDI5_B                    |
| 7  | HSDI2_A                    | 8  | HSDI6_A                    |
| 9  | HSDI2_B                    | 10 | HSDI6_B                    |
| 11 | HSDI3_A                    | 12 | HSDI7_A                    |
| 13 | HSDI3_B                    | 14 | HSDI7_B                    |
| 15 | 485E_A (485 port 5 A)      | 16 | 485F_A (485 port 6 A)      |
| 17 | 485E_B (485 port 5 B)      | 18 | 485F_B (485 port 6 B)      |
| 19 | GND                        | 20 | GND                        |

- Each fast DI channel = a two-wire A/B pair (bridge input). ⚠️ **HSDI0 and HSDI1 share the A terminal at position 1; HSDI4 and HSDI5 share the A terminal at position 2** — the signal sources of each shared pair must join on one common wire at that terminal.
- **485 ports 5 and 6**: A/B at positions 15–18, GND at 19/20.
- Always follow the silkscreen printed on the board.
- ⚠️ Field finding (2026-09-03, pending bench verification): pulses fed into left-row terminals 7/9 appear on channel HSDI5 (not HSDI2 as tabled), and pulses on right-row terminals 2/6 make the controller reset repeatedly — **do not use terminals 2/6** until this table has been re-verified pin by pin.

### 3.9 Grounding & EMC

- Connect PE to the cabinet earth bar by the shortest path; ground cable shields at one end (normally the controller end).
- **Never tie the isolated grounds (GND_A / GND_B / GND_CA) to the device GND** — doing so defeats the isolation.
- Long field lines always on isolated ports with shielded twisted pair; non-isolated ports stay inside the cabinet.
- All communication ports include on-board TVS/GDT protection; in lightning-prone sites add an external surge protection device.

## 4. On-Board I/O

### 4.1 Fast Digital Inputs ×8 (HSDI0–7)

- 24 V digital inputs, optocoupler-isolated, constant-current front end; optos on Board 2, logic returned to the main MCU via CN4; timer-backed (TIM3/TIM1/TIM8).
- Each channel configurable as **regular DI / high-speed counter / quadrature encoder** (capability combinations and rates: contract `HSDI_Configuration_and_Counting.md`); encoder A/B phases go to a channel pair of the same timer (see Software Manual).
- Panel LEDs IND0–7 directly indicate the field level of each DI channel (driven by field signals, not MCU-controlled).
- Terminal wiring: see §3.8.

### 4.2 Analog Inputs

On-board ADC front end, **2.5 V precision reference** (REF3025). ⚠️ Field ADC full-scale = 2.5 V (by design). A sub-card form also offers 8 AI channels.

### 4.3 Status LED & Buzzer

- **Run LED**: LD1 green on **PG10**, toggled ~1 Hz by the main loop = firmware-alive heartbeat.
- **Buzzer**: BUZZER1, passive magnetic (MLT-8530) on **PI0 = TIM5_CH4** PWM (~2.7 kHz resonant); control via CLI `beep 1|0` or the app hook `app_beep_set()`.
- No user push-button on the main board.

## 5. Option Cards (factory-installed per order)

RTD temperature card (4 channels, 2/3/4-wire) and EOL loop-monitoring card (8 channels) are installed into the main enclosure's option slots and calibrated at the factory; on site, wire the corresponding terminals only (a wiring sheet ships with each card).

## 6. Expansion Modules & Backplane Bus

**Bus specification**:

- **Physical**: RS-485 half-duplex multidrop, in-cabinet ≤~30 cm; default 1 Mbps 8N1 (measured limit ~2M; kept at 1M for compliance).
- **Protocol**: standard Modbus RTU (third-party pymodbus / QModMaster can probe directly). Master = main board CM4 (UART8, DE=PD11). Contract = `Backplane_Bus_Protocol.md`.
- **Attention line**: open-drain wired-OR; a module pulls it low → master does event-driven poll.
- **Protection**: SM712 TVS per node; master-side fixed 120 Ω termination + fail-safe bias.

**Installation steps**:

1. **Switch off the 24 V supply** (rule, see §3.1).
2. Set the **DIP address** (4-position switch on the module): **address = DIP value + 1** (0000 = address 1 … 1111 = address 16); addresses must be unique within one system; 0 = broadcast.
3. Internal blade slot: plug in directly. Side-mounted modules: join the enclosures and mate the 8-pin backplane feed-through connector (**ground-first staggered design**).
4. Power up — the master discovers the module automatically (type / version / capabilities self-identified, no configuration files); the new module's points appear on the host/cloud immediately.

- A module that loses the master for 3 s automatically enters its safe state (all outputs off) — the bus is self-failsafing.
- Termination is fixed at the master end, nothing to configure on modules; module baud rates are fully automatic on the backplane.

**Registered expansion modules** (type registry = board/version registry doc):

| Type | Module | I/O | Manual |
|---|---|---|---|
| 1 | EX_16DO | 16× isolated digital outputs (PhotoMOS, 200 mA/ch) | `EX_16DO_Module_Manual.md` |
| 2 | PH_EC | 2× pH + 2× EC + temperature | `PH_EC_Transmitter_User_Manual.md` |
| 3 | EX_16DI | 16× isolated digital inputs (optocoupler; in development) | (in development) |
| 4~255 | (unassigned) | new modules get a type code | |

Modules are STM32H503 slave MCUs sharing the CORE_503 standard core; the backplane auto-identifies each module's type / hardware version / firmware version / capability bits. Blade (embedded) and external modules share the same PCB; whether the 10-pin adapter is populated distinguishes them.

## 7. Operation & Software

### 7.1 USB Console

USB-C to a PC enumerates as a virtual COM port with a full diagnostic CLI: network/cloud status, Modbus port statistics, task/stack/CPU usage, power-fail persistence and black box, OTA status, and more (command list: Software Manual §5.9).

### 7.2 Modbus Port Configuration

All six front-panel RS-485 ports are configurable as Modbus RTU **master or slave** (port pool / configuration model / board-level slave register map / Modbus TCP = contract `Universal_Modbus_Port_Config.md`); field I/O is scanned cyclically into a process image for non-blocking application access (Software Manual §5.3).

### 7.3 Cloud & Dashboard

The device publishes a self-describing model to the cloud (MQTT/TLS, contract `Device_Cloud_Protocol.md`); the web dashboard renders points and controls with zero code. Connecting to your own AWS IoT is supported (guide: `Connect_Your_Own_AWS_IoT.md`).

### 7.4 OTA Updates

Dual-bank fail-safe OTA: trial period with automatic rollback + brick-proofing (IWDG always on); expansion-module firmware is updated over the backplane bus. Operation = `OTA_and_MQTT_User_Guide.md`; internals = `OTA_Update_and_Brick_Proofing.md`.

### 7.5 On-Board Application Development

Customer applications run on the board as FreeRTOS tasks (CM7 or CM4), with a full platform C API (Modbus / process image / power-fail persistence / cloud reporting) — **see `Software_Manual_Application_Development.md`**.

## 8. Power & Power-Fail Persistence

- **Input**: 24 V nominal (operating range 9–36 V); SMPS→LDO cascade for 3.3 V.
- **No UPS / super-cap needed**: surviving arbitrary power loss without damage is a design goal.
- **Power-fail detection**: an external comparator (LM393) watches the 24 V rail and triggers the last-gasp routine via PJ4/EXTI. **Measured warning window: 31~39 ms** (same under full load).
- **Three-layer power-fail persistence** (available to the application, contract = `app_pwrfail.h`, usage: Software Manual §5.4):
  1. BKPSRAM 4 KB (battery domain, permanently retained) — cumulative counters
  2. Last-gasp snapshot into BKPSRAM (50 µs)
  3. SPI-flash black-box page (≥8 KB writable in the last-gasp window; Pro/Full)
- **Battery**: CR1220, backs RTC + BKPSRAM.

## 9. Security & Identity

| Chip | Role |
|---|---|
| ATECC608A (standard on all tiers) | Anti-clone + TLS client private-key host. Private key generated inside the chip, physically non-readable; each device has a unique hardware identity to the cloud (including your own AWS, ECC certificates). |
| SE050 (Full) | RSA / multi-curve / multi-identity / full certificate chain stored in chip / factory pre-provisioning / attestation; for customer's own cloud or high-security scenarios. |

**Verified: the 608A private key completes the mTLS handshake and the server accepts it** — a cloner who copies the flash still cannot obtain the chip's private key and cannot connect.

## 10. Board Type, Version & Maintenance

- Main board type code **I757-M**, hardware version 0x0100 (v1.0 first board); Modbus slave-map BOARD_TYPE = 0x0757.
- **Debugger**: PW-LINK2 (standard DAPLink / CMSIS-DAPv2) + OpenOCD; name dual probes with `adapter serial`.
- **Flashing**: OTA (recommended, see §7.4) or SWD.
  - ⚠️ **SWAP trap**: when the board runs on bank 2 (swapped state), SWD `program 0x08000000` writes physical bank 1 = the standby area, so it "has no effect". On bank 2, always use OTA, or revert to bank 1 first, then SWD.
- **Bench power**: current-limit ≥500 mA (480 MHz + Ethernet + relay transients; 100 mA collapses the power domain).

---

## Revision

- v1.0 (2026-08-13): consolidated volume — reorganized from `Hardware_Manual.md` v1.0 + `Installation_and_Wiring.md` v0.1 under the "one manual per board" policy; both superseded volumes retired. Includes the measured P3 / Board 2 pinout tables (2026-08-12) and the new configuration-tier table (§1.1). Open item: external 8-pin feed-through connector part number to be added once finalized.
