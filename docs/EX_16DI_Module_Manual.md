# EX_16DI 16-Channel Isolated Input Module — User Manual

> Companion documents: `i757_Controller_Manual.md` (host controller), `Backplane_Bus_Protocol.md` (bus contract).

---

## 1. Overview

EX_16DI is a **16-channel isolated digital input** module with a **32-bit counter per channel** and **configurable debounce**. It connects to any Modbus master (PLC / gateway / industrial PC) over **RS-485 (Modbus RTU slave)**, and also works as a backplane expansion module of the Edgron i757 controller (plug-and-play, zero configuration).

- 16 wide-range inputs (approx. 5–30 V DC) with a constant-current front end — input current does not rise with voltage, so long lines and high supply voltages do not cause heating;
- **Bridge-type inputs, polarity-free**: dry contacts, NPN (open-collector), PNP (sourcing) and two-wire sensors all wire up the same way;
- Per-channel debounce (0–255 ms) plus a 32-bit rising-edge counter — water meters, slow flow meters, travel/limit feedback in one module;
- Field side galvanically isolated from the bus/logic side; transient protection on every channel;
- One panel LED per channel: LED on = input conducting, what you see is the input state;
- Inputs are passive data: **sampling and counting keep running during a bus outage** — nothing is lost, the master reads complete data once it returns.

## 2. Specifications

| Item | Value |
|---|---|
| Power | 9–36 V DC wide range (24 V nominal, via the bus connector) |
| Communication | RS-485 half duplex, Modbus RTU slave, 8N1 |
| Inputs | 16 channels, isolated, bridge-type (polarity-free), constant current approx. 3.5 mA per channel |
| Input voltage | 24 V DC rated; approx. 5–30 V wide range |
| Thresholds | ON ≈ ≥4 V; OFF = open circuit or ≤3 V (typical) |
| Sampling | 1 kHz (1 ms period) |
| Debounce | 0–255 ms per channel, default 10 ms (0 = off) |
| Counting | 32-bit per channel, increments on debounced rising edges; max count rate approx. 200 Hz at debounce = 0 |
| Isolation | Field side isolated from bus/logic side; channels share a common per group (not isolated within a group); TVS transient protection per channel |
| Terminals | 2×9 = 18 positions (row A / row B), one common per group |
| Indicators | One LED per channel, on = conducting |
| Mounting | i757 internal blade slot / side-mounted stack / stand-alone |

> ⚠️ The thresholds are engineering values of the wide-range constant-current design (not IEC 61131-2 Type 3): **5 V logic-level signals are not recommended as direct inputs** (may read as ON), and long floating lines may pick up induced voltage — dry contacts and 24 V sensors are unaffected.

## 3. Installation and Wiring

Field terminals, two rows of 9 (row A / row B), 18 positions total:

| Terminal | Function                  |
|----------|---------------------------|
| A1–A8    | Inputs DI1–DI8            |
| A9       | COM_A (common for DI1–8)  |
| B1–B8    | Inputs DI9–DI16           |
| B9       | COM_B (common for DI9–16) |

Wiring notes:

- **Common connection (bridge inputs, polarity-free)**: connect COM to **either pole** of the field supply (0 V or 24 V), and feed each input from the **other pole** — a closed loop reads ON. Groups A and B may choose independently.
  - **Dry contact** (button / limit switch / relay contact): COM to 0 V (or 24 V), contact in series between DIn and 24 V (or 0 V);
  - **NPN sensor** (open-collector, sinking): COM to 24 V, sensor output to DIn;
  - **PNP sensor** (sourcing): COM to 0 V, sensor output to DIn.
- Approx. 3.5 mA constant current per channel doubles as contact wetting current; long lines are fine — route signal wiring away from power wiring.
- Input loops are SELV low-voltage circuits — **never connect mains**.
- Channel numbers correspond one-to-one to terminals and panel LEDs (DI1–DI16).
- **Cut the 24 V supply before inserting or removing the module** (hot-plugging is not allowed).

Mounting:

- **As an i757 expansion module**: plug into an internal blade slot, or stack side-mounted via the backplane feed-through — power and bus connect automatically, zero wiring.
- **Stand-alone**: wire 24 V and the RS-485 bus (A/B) to the corresponding positions of the side bus connector (see the wiring sheet shipped with the unit / silkscreen).
- **Slave address**: 4-position DIP switch on the module, **address = DIP value + 1** (0000 = address 1 … 1111 = address 16); addresses must be unique on a bus.

## 4. Modbus Communication

### 4.1 Link Parameters

- **Slave address**: DIP + 1 (1–16), see §3; address 0 = broadcast (write-type function codes only; executed without a response);
- **Serial format**: 8 data bits, no parity, 1 stop bit (8N1);
- **Baud rate**: **4800 / 9600 (factory default) / 19200 / 38400 / 57600 / 115200 / 250K / 500K / 1M**; changing it: see 4.3;
- **Supported function codes**: FC02 (read discrete inputs), FC03 (read holding), FC04 (read input), FC06 (write single register), FC16 (write multiple registers); the module has no outputs, so FC01/05/15 answer "illegal function";
- Register values are big-endian (Modbus standard); exceptions are standard Modbus exception codes (01/02/03/04).

### 4.2 System Identification (FC04, read-only)

| Address | Meaning |
|---|---|
| 0x0001 | Module type = 3 (EX_16DI) |
| 0x0002 | Firmware version |
| 0x0008 | Hardware version (0xMMmm, 0x0100 = v1.0) |
| 0x0004/0x0005 | Uptime seconds (high word / low word) |
| 0x0006 | CRC error-frame counter (link-quality diagnostic) |
| 0x0007 | Status word: bit0 = in safe state, bit1 = unclaimed events pending |

### 4.3 Baud Rate Configuration (0x0011 immediate switch + 0x0281 power-up rate)

Baud code table (shared by both registers):

| Code | Rate | Code | Rate |
|---|---|---|---|
| 0 | 1 M | 7 | 9600 (factory default) |
| 1 | 250 K | 8 | 19200 |
| 2 | 500 K | 9 | 38400 |
| 6 | 4800 | 10 | 57600 |
|   |      | 11 | 115200 |

**Ships at 9600** — most masters connect first try with no configuration. When another rate is needed:

- **0x0281 power-up baud rate** (FC06 write + persist): write the target code, then write 0xA55A to 0x0200 to save — after the next power cycle the module runs at the new rate. Example: switch to 19200 = write 0x0281 = 8 → write 0x0200 = 0xA55A → restart;
- **0x0011 immediate switch** (FC06 write, advanced): the response is sent at the **old** rate, then the module switches about 20 ms later — the master must follow in the same window; a switch that was not persisted **falls back to the power-up rate after 3 s of bus silence** (anti-lockout protection);
- Recovery when the module cannot be found: scan the rates in the table above (reading 0x0011, at most 12 attempts);
- On the Edgron i757 backplane no setup is needed — the master discovers the module automatically and pins its power-up rate to the backplane operating rate (one-time; every later power-up connects directly).

## 5. Inputs and Counters

### 5.1 Input Levels (FC02, discrete inputs 0–15)

| Address | Meaning |
|---|---|
| 0–15 | DI1–DI16 debounced levels (1 = conducting, 0 = off/open) |

Reads match the panel LEDs; the level bits and the counter edges share the same debounce result (see 5.3).

### 5.2 Counters (FC04, 0x0200–0x021F)

One **32-bit unsigned counter** per channel, 2 registers each, **high word first**:

| Address | Meaning |
|---|---|
| 0x0200/0x0201 | CNT1 high/low word (DI1) |
| … | … |
| 0x021E/0x021F | CNT16 high/low word (DI16) |

- Increments by 1 on each debounced **rising edge** (off → conducting);
- Starts at 0 on power-up; wraps around freely (no saturation) — consume as an **unsigned difference**: `delta = (new − old) mod 2³²`;
- Reading the whole block in one frame (start 0x0200, quantity 32) gives a same-instant snapshot of all channels;
- Clearing: see 5.4; a power cycle also resets to zero (see §7, event bit0).

### 5.3 Debounce (holding registers 0x0020–0x002F, FC03 read / FC06 single write / FC16 block write)

| Address | Meaning |
|---|---|
| 0x0020–0x002F | Debounce time DI1–DI16, in ms, 0–255 (default 10, 0 = off) |

- Semantics: a level must stay **continuously stable** for this many milliseconds before the new state is accepted; applies to both the level bits (FC02) and the counter edges (0x0200–);
- Takes effect immediately; **writing 0xA55A to 0x0200 persists the values with the module configuration — they are restored automatically after a power cycle**. Set once, keeps forever (FC16 writes all 16 channels in one frame; if any value in a block write is out of range the whole frame is rejected and nothing changes). Changes that were not persisted revert to the last saved values (or the 10 ms default) on restart;
- Suggested values: 10–50 ms for mechanical contacts; smaller or 0 for electronic sensors / higher-rate counting (mind the count-rate limit in §2).

### 5.4 Counter Clear (holding register 0x0030, FC06 write)

Write a 16-bit bitmap: bit0 = 1 clears the DI1 counter … bit15 = 1 clears the DI16 counter. **Executes immediately, self-clears, always reads 0.** Example: writing 0x0005 clears DI1 and DI3 together.

> Normal incremental consumption (the difference method of 5.2) does **not** require clearing; clearing suits "new meter / restart from zero" operations.

## 6. Safe State

- **No valid frame addressed to this module (or broadcast) for 3 seconds → safe state**: status word bit0 and event bit1 are set; the module has no outputs, and safe state **does not affect sampling or counting** — input changes and pulses keep accumulating during a bus outage, so the master reads complete data on return;
- Any valid frame exits safe state immediately;
- The safe-state flag is set at power-up and cleared by the first frame; on the i757 backplane all of this is handled by the host automatically.

## 7. Event Register

**Event register 0x0010** (FC03 read / FC06 **write-1-to-clear**):

| Bit | Meaning |
|---|---|
| bit0 | Powered up / was reset (polling-free evidence of a module restart) |
| bit1 | Entered safe state (bus was interrupted) |
| bit2 | Bad-frame count over threshold (line-quality alarm) |

Events latch until the master clears them by writing 1. **bit0 matters especially for this module**: a restart means the **counters have reset to zero** (persisted settings such as debounce are restored automatically, no re-download needed) — on seeing bit0 the master should handle the counter baseline as the application requires (e.g. discard the previous reading and restart the delta).

## 8. Integration with Third-Party Modbus Masters

Minimum integration with any Modbus master (PLC / SCADA / gateway) is three steps:

1. Set the DIP address and the serial parameters of 4.1 (ships at 9600 8N1);
2. **Poll FC02, inputs 0–15, periodically (≤1 s)** for all levels; add one FC04 read of 0x0200 × 32 when counters are needed;
3. At commissioning, set debounce as needed (FC16 to 0x0020–0x002F, then write 0x0200 = 0xA55A to persist — once is enough); optionally poll 0x0010 (events) and 0x0007 (status).

**Water-meter / flow-meter example** (reed-pulse output on DI1):

- Set debounce to match the pulse width (e.g. 20 ms);
- Once per minute, read 2 registers from 0x0200 with FC04 and join them into the 32-bit reading;
- `pulses this minute = (new − old) mod 2³²`, multiplied by the volume per pulse gives the flow;
- No clearing needed, wrap-around safe; a module power cycle is caught via event bit0.

## Revision

- v1.0 (2026-08-13): initial release (one manual per board; template = PH_EC / EX_16DO module manuals). Firmware baseline v0.3 (persistent power-up baud rate via 0x0281; persistent per-channel debounce).
