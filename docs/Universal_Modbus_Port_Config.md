# Industry_757 — Generic Modbus Port Configuration v0.1

> This document = the single authoritative contract for **front-panel serial-port-configurable Modbus master/slave + Ethernet Modbus TCP**.
> Purpose: any of the serial ports can be configured as a Modbus master or slave, and Ethernet supports Modbus master/slave — the integrator configures it.
> Boundary: the backplane bus (UART8) is **fixed as master, not part of the configurable pool**, its contract remains `Backplane_Bus_Protocol.md`; new inter-core ops are registered in `Inter_Core_RPMsg_Protocol.md` v1.3; this document governs: the port pool, the configuration model, the unified slave register map (board-level), and the Modbus TCP conventions.
> This map = the foundation of the future `Display Modbus Register Map` (Display Modbus register map): the display is simply "a Modbus master connected to one of the slave ports" (like connecting a PLC).

## 1. Port Pool

| Port name (silkscreen/config name) | UART | TX/RX | DE | Isolation | Notes |
|---|---|---|---|---|---|
| **485A** (=485_1) | USART1 @CM4 | PB14/PB15 | PJ15 (GPIO-DE) | Isolated (CA-IS2092A) | |
| **485B** (=485_2) | USART2 @CM4 | PD5/PD6 | **PD4 (hardware DE)** | Isolated (CA-IS2092A) | |
| **485C** (=485_3) | USART3 @CM4 | PB10/PB11 | PI1 (GPIO-DE) | Non-isolated | Bench status: connected to a PC USB-485 dongle (first acceptance port) |
| **485D** (=485_4) | UART4 @CM4 | PA0/PI9 | **PA15 (hardware DE)** | Non-isolated | ⚠️ PA0 shares a pad with AI0_C (analog switch, pin assignment §6-1) |
| **485E** (=485_5) | USART6 @CM4 | PG14/PG9 | PA8 (GPIO-DE) | Non-isolated | Terminal on board 2 (transceiver on board 1, differential via CN4) |
| **485F** (=485_6) | UART7 @CM4 | PB4/PB3 | PH15 (GPIO-DE) | Non-isolated | Terminal on board 2 |
| **ETH** | — @CM7 | — | — | — | Modbus TCP server + client |
| Backplane | UART8 @CM4 | PJ8/PJ9 | PD11 | Non-isolated | **Fixed master, not configurable** (`Backplane_Bus_Protocol.md`) |

> The six ports include a mix of hardware DE (PD4/PA15) and GPIO-DE — uart_drv covers both modes. The whole table was verified against the EDA netlist ✓.
> Resource allocation (already back-filled into `Hardware_Resource_Allocation.md` §4/5/6; **stream allocation = pre-existing in the generated-layer .ioc, do not make it up yourself**): RX ring buffer + IDLE framing, TX each takes one — 485A=DMA2 S4/S5, 485B=DMA1 S2/S3, 485C=DMA2 S2/S3, 485D=DMA2 S0/S1, 485E=DMA1 S6/S7, 485F=DMA1 S4/S5; 6×UART IRQ @CM4 priority 6 (flag-raise only).

## 2. Configuration Model ("let customers configure it themselves")

### 2.1 Per-Port Entry

| Field | Value | Default | Description |
|---|---|---|---|
| role | `off` / `slave` / `master` | off | off = UART disabled to save resources |
| baud | 2400/4800/9600/19200/38400/57600/115200 | 9600 | Officially supported tiers (display/PLC domain); >115200 experimental tier not guaranteed |
| parity | `N` / `E` / `O` (8 data bits fixed) | N | Modbus spec default is 8E1, the de-facto default in the tool community is 8N1 → factory N |
| stop | 1 / 2 | 1 | |
| slave_addr | 1~247 | 1 | Only used by slave role |

### 2.2 Storage and Activation [decided]

- Config file = littlefs **`/mbport.cfg`** (text line format, CM7 owns it), loaded at boot and pushed down to CM4; **runtime change = takes effect immediately + auto-persists** (no save command needed).
- File line format (CLI syntax is the file syntax, human-readable directly):
  ```
  485a slave 9600 8N1 addr=1
  485b master 9600 8N1
  tcp server=on
  ```

### 2.3 Configuration Channels (three; v1 delivers the first two)

1. **CLI** (USB-C COM console): the `mbcfg` command family, see §6.
2. **Cloud**: `dn/cmd` payload = a string with the same syntax as the CLI (`mbcfg 485a slave 9600 8N1 addr=1`); current acknowledgment goes via the heartbeat `mb` field + desc point `sys.mb` (str, ro).
3. [reserved] Display: configure via writing the holding-register config area on a slave port (§4 reserved 0x3000~), pending `Display Modbus Register Map`.

## 3. Role Semantics

### 3.1 slave (slave port)

- Each slave port = a uart_drv instance + a modbus core `mb_slave_handle`; **all ports answer the same "board-level register map" (§4)**; the Modbus TCP server uses the same map (its PDU enters CM4 for arbitration via RPMsg op 0x09) — **a serial-port query and an Ethernet-port query give bit-for-bit identical answers**.
- RTU timing: IDLE framing (≥3.5 characters), the response is framed within the CM4 1ms main loop (typically <2ms).
- Exception codes: standard Modbus (0x01 illegal function / 0x02 illegal address / 0x03 illegal value).

### 3.2 master (master port)

- v1 = **CM7-driven transaction mode** (RPMsg op 0x0A: specify port + slave address + PDU, CM4 goes on the bus and waits for the response): CLI `mbpoll` one query at a time; cloud cmd uses the same syntax. A periodic poll table (read-back values normalized into the point model `node`, already reserved in `Device_Cloud_Protocol.md` §5A) = v2 [planned]; a CM4 autonomous poll table is also left for v2 — v1 first gets "any port as master querying any device" working.
- Transaction timeout defaults to 100ms (can be specified via an op parameter); the CM7 defaultTask is single-threaded serial, naturally free of concurrency.

### 3.3 off

UART disabled (uart_close), DMA/interrupt released, DE pin set to receive.

## 4. Board-Level Slave Register Map v1 (unified across all ports; = the board-level isomorph of the backplane slave data model §3)

> The 757's own map when acting as a slave. **The customer scheduling/program/config areas are reserved** (0x2000~); refinement = `Display Modbus Register Map` (to be established, extended from this map, non-conflicting).

> **Slot window width: each slot is windowed at 32 channels** (headroom for future 32-terminal expansion modules) — 32 coil/discrete bits per slot, an 8-cell HR block per slot, a 16-cell IR block per slot. Even a fully-populated 16 slots only uses coils 0~511 and registers 0x0100~0x01FF; the number range is ample.

### 4.1 Input Registers (FC04, read-only)

| Address | Meaning |
|---|---|
| 0x0000 | MAP_VER = 1 |
| 0x0001 | BOARD_TYPE = **0x0757** (I757-M) |
| 0x0002 | FW_VER 0xMMmm |
| 0x0003 | CAPS: bit0=backplane slot bit1=time bit2=onboard AI (v1=0) |
| 0x0004/0x0005 | UPTIME seconds high word / low word |
| 0x0006 | Board temperature °C (signed) |
| 0x0007 | FLAGS: bit0=cloud online bit1=backplane scheduler active |
| 0x0008 | HW_VER 0xMMmm (=0x0100 first board) |
| 0x0010+s (s=1..16) | Slot s module type (0=not present; = transcribed from backplane slave IR 0x0001) — the master (display) thereby **auto-discovers which module is plugged in** |
| 0x0020+s (s=1..16) | Slot s slave firmware version 0xMMmm (= transcribed from backplane IR 0x0002; not present=0) [reserved, v1 may fill later] |
| 0x0100+(s-1)×16 ~ +15 | **Slot s analog/input window (16 cells per slot)**: layout depends on module type (= transcribed from that model's backplane IR 0x0100+ area; PH_EC=2pH+2EC+4RTD…) [reserved, v1 not filled] |
| **0x2000+** | **Process-image window (input image, words)** [opened 2026-07]: register `0x2000+n` = input-image bytes [2n, 2n+1]. An external master reads all the field data this board concentrates. Contract = `Process_Image_and_IO_Mapping.md` section 10 |

### 4.2 Holding Registers (FC03 read / FC06 single write / FC16 multi write)

| Address | Meaning |
|---|---|
| 0x0000~0x0002 | Time unix high word / low word / milliseconds (v1 read-only; display time-sync write = TBD, backlogged §8) |
| 0x0100+(s-1)×8 ~ +7 | **Slot s output block (8 cells per slot)**: +0=DO mask low 16 channels (= cloud point `m<s>.do`, same semantics), +1=DO mask high 16 channels (for 32-terminal modules), +2~+7 reserved (AO/parameters). Write → backplane FC15 pushed down; read = the most recent FC01 true read-back cache (≤5s old) |
| **0x2000+** | **Process-image window (output image, words)** [opened 2026-07]: read = command read-back, write = pushed down to the field devices. Register `0x2000+n` = output-image bytes [2n, 2n+1]. Contract = `Process_Image_and_IO_Mapping.md` section 10 |
| 0x3000~0x30FF | [reserved] Port config area (display config channel) |

### 4.3 Coils (FC01 read / FC05 single write / FC15 multi write)

| Address | Meaning |
|---|---|
| (s-1)×32 + c (s=slot 1..16, c=channel 0..31) | The DO bit view of slot s channel c (same data as the slot HR block, two granularities; **32-bit window per slot**) — FC05 single-point turns on one relay; slot 2 channel 0 = coil 32 |
| **0x2000+b** | **Process-image window (output image, bits)** [opened 2026-07]: bit `0x2000+b` = output-image bit b. An external master commands DO here. Contract = `Process_Image_and_IO_Mapping.md` section 10 |

### 4.3bis Discrete Inputs (FC02, read-only)

| Address | Meaning |
|---|---|
| (s-1)×32 + c | The DI bit view of slot s channel c (32-bit window per slot; 16O has no DI, for future DI modules) [reserved, v1 not filled] |
| **0x2000+b** | **Process-image window (input image, bits)** [opened 2026-07]: bit `0x2000+b` = input-image bit b. An external master reads field DI here. Contract = `Process_Image_and_IO_Mapping.md` section 10 |

### 4.4 Consistency Discipline

- The single source of slot data = the CM4 backplane scheduler (true read-back, no trust of shadows); the serial slave / TCP slave / cloud point all share the same source.
- In-flight write semantics: writing a coil/mask is immediately turned into a backplane FC15 (within hundreds of ms), the read-back cache refreshes on the next backplane poll — **reading a stale value ≤5s is contract-defined behavior** (consistent with the cloud `m2.do` read-back semantics).

## 5. Modbus TCP (CM7, LwIP)

| Item | Convention |
|---|---|
| Server port | **502** (standard); listener always on (`tcp server=off` can disable it) |
| Concurrent connections | **2** (display/SCADA + debug; a 3rd connection is refused) |
| Unit ID | **Accept any value and echo it verbatim** (direct-device convention); recommend clients use 1 |
| MBAP | Standard 7-byte header; TID echoed verbatim; PDU exactly the same map as RTU (§4) |
| Client (master) | v1 = CLI/cloud cmd one query at a time (`mbpoll tcp <ip[:port]> <unit> <fc> <reg> <n>`); resident poll table = v2 |
| Idle kick | Disconnect after 120s with no request (prevents half-open squatting) |

## 6. CLI Command Family (USB-C console; cloud cmd uses the same syntax)

| Command | Semantics |
|---|---|
| `mbcfg` | Show all port configurations + runtime statistics |
| `mbcfg 485a slave 9600 8N1 addr=1` | Configure a slave port (takes effect immediately + persists) |
| `mbcfg 485b master 19200 8E1` | Configure a master port |
| `mbcfg 485a off` | Disable a port |
| `mbcfg tcp on\|off` | TCP server on/off |
| `mbpoll 485b <addr> <fc> <reg> <n> [value]` | One transaction on a master port (read = fc 3/4/1/2, write = fc 5/6/15/16) |
| `mbpoll tcp <ip[:port]> <unit> <fc> <reg> <n> [value]` | One TCP client transaction |

## 7. Acceptance Script (customer perspective, = tomorrow's plan §4; bench status: 485C already connected to a PC USB-485 dongle)

1. `mbcfg 485c slave 9600 8N1 addr=1` → PC pymodbus/QModMaster via USB-485: FC04 read ident (0x0757/version) ✓ FC03 read time ✓ **FC05 write coil 32 (=slot 2 channel 0), watch the real relay** ✓.
2. Isolated-port answer test: connect 485A↔485B terminals, `mbcfg 485a slave …` + `mbcfg 485b master …` → `mbpoll 485b 1 4 0 9` reads back ident bit-for-bit identical to what 485C queried ✓ (hardware DE + GPIO-DE, isolated power all tested together).
3. PC pymodbus TCP connects to the board's 502: reads the same map, bit-for-bit identical to serial ✓; `mbpoll tcp <PC_IP> 1 3 0 4` the board actively queries the pymodbus server on the PC ✓.
4. Change config → reboot → `mbcfg` shows it restored (littlefs persistent) ✓.

## 8. TBD / Backlog

- [ ] HR 0x0000 time area allows the display to write time-sync (involves arbitration of SNTP/cloud time-sync priority)
- [ ] master resident poll table + read-back values normalized to `node` points (cloud protocol §5A); CM4 autonomous poll table
- [ ] Display Modbus register map (scheduling/program/config area refinement, jointly defined with the customer)
- [ ] Enable 485C~F (transceivers already on board, just expand the firmware port pool)
- [ ] TCP client resident poll table
- [ ] >115200 high-speed tier evaluation (verify against the CA-IS2092A spec upper limit)

## 9. Version History

- v0.1: contract established. Port pool = **all 6 ports** (485A~F) + TCP; board-level slave map v1 (ident/time/slot window, **32 channels per slot** reserved for 32-terminal expansion) + customer reservation; TCP 502×2; CLI/cloud same-syntax configuration; BOARD_TYPE=0x0757 allocated; all-port pins verified against the EDA netlist measurement.
- **v0.1 bench acceptance**: scripts ①③④ all passed live — 485C slave (pymodbus/COM7): ident=0x0757/time/slot 2 auto-discovery/FC05 coil 32 = real relay open-close + FC01 true read-back/FC06 slot mask write/out-of-bounds = exception code 2 ✓; TCP 502: same map bit-for-bit identical + Unit ID any value echoed verbatim ✓; TCP client `mbpoll tcp` reads PC pymodbus server 111/222/333/444 ✓; reboot config restored + slave ports auto-revived ✓.
- **Six-port shared-bus stress test**: 485D/E/F passed on first try; 9600×1800 transactions + 115200×3000 transactions **zero failures** (latency 9600=32ms steady / 115200=6ms); board-side ledger = each of the six ports received ~93KB/~9100 frames (**every frame received by all 6 ports** — eavesdropping immunity, each frame tested 6 times) **ovr=0 err=0 all zero**, ring ledger wr==rd balanced; concurrently the backplane scheduler had 10855 polls with 0 failures, cloud heartbeat uninterrupted — proving the CM4 1ms main loop handles 6×485 + backplane + RPMsg concurrently. Hot line-parameter change (9600→115200, six-port uart_set_line) validated in practice ✓.
- **Script ② closed the same day (after wiring)**: 485A↔485B **bidirectional** answer test all passed (two isolated ports, hardware DE / GPIO-DE each tested as master and slave in turn). One issue cracked along the way: **same-board self-query-self-answer deadlock** — the master transaction spun waiting inside the RPMsg callback, hogging the CM4 main loop, so the slave-port service couldn't run (ring ledger evidence fw=1/rd=0 = the response was framed but nobody consumed it); fix = the master wait loop pumps `mbfront_service()` on its behalf (single-threaded sequential call, no re-entrancy). **Affects only same-board two-port cross-testing; the external-device scenario has no such landmine.** The port statistics op0B was expanded to 40B (adds the ring ledger internals wr/rd/fs/f_wr/f_rd, a resident troubleshooting instrument).
