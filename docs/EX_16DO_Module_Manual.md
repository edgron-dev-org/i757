# EX_16DO 16-Channel Isolated Output Module — User Manual

> Companion documents: `i757_Controller_Manual.md` (host controller), `Backplane_Bus_Protocol.md` (bus contract).

---

## 1. Overview

EX_16DO is a **16-channel isolated digital output** module (PhotoMOS solid-state outputs). It connects to any Modbus master (PLC / gateway / industrial PC) over **RS-485 (Modbus RTU slave)**, and also works as a backplane expansion module of the Edgron i757 controller (plug-and-play, zero configuration).

- 16 independently controlled outputs, AC or DC loads; no mechanical contacts, no switching noise, lifetime not limited by switching cycles;
- Field side electrically isolated from the bus/logic side; transient protection on every channel;
- Loss of communication automatically enters the safe state (all outputs off) — a broken bus fails safe by itself;
- Per-channel output LEDs (O1–O16) on the front panel — what you see is the output state.

## 2. Specifications

| Item | Value |
|---|---|
| Power supply | 9–36 V DC wide-range (24 V nominal, via the bus connector) |
| Communication | RS-485 half-duplex, Modbus RTU slave, 8N1 |
| Outputs | 16× PhotoMOS solid-state, AC or DC |
| Per-channel rating | **≤200 mA, ≤60 V loop voltage**; not for switching mains |
| Isolation | Field side isolated from bus/logic side; TVS transient protection per channel |
| Terminals | 2×9 = 18 positions (row A / row B), one common per group |
| Indicators | Per-channel output LEDs O1–O16, following output state |
| Mounting | i757 internal blade slot / side-mounted / standalone |

## 3. Installation & Wiring

Two field-terminal rows of 9 positions each (row A / row B), 18 positions total:

| Terminal | Function                       |
|----------|--------------------------------|
| A1–A8    | Outputs O1–O8                  |
| A9       | COM_A (common for O1–8)        |
| B1–B8    | Outputs O9–O16                 |
| B9       | COM_B (common for O9–16)       |

Wiring notes:

- **Common wiring**: COM to 0 V = low-side switching (sinking); COM to 24 V = high-side switching (sourcing); rows A and B choose independently.
- Load current stays entirely on the field side (never through the bus connector); **inductive loads (solenoid valves, contactors, locks) require a flyback diode (DC) or RC snubber (AC) at the load**.
- Channel numbers match the front-panel silkscreen O1–O16.
- **Switch off the 24 V supply before plugging or unplugging the module** (never under power).

Mounting forms:

- **As an i757 expansion module**: plug into the internal blade slot, or side-mount by joining enclosures and mating the backplane feed-through connector; power and bus connect automatically, no wiring.
- **Standalone**: wire 24 V power and the RS-485 bus (A/B) to the corresponding positions of the bus connector on the module side (follow the wiring sheet shipped with the module / the silkscreen).
- **Slave address**: 4-position DIP switch on the module, **address = DIP value + 1** (0000 = address 1 … 1111 = address 16); addresses must be unique on one bus.

## 4. Modbus Communication

### 4.1 Link Parameters

- **Slave address**: DIP + 1 (1–16), see §3; address 0 = broadcast (write-type function codes only; executed without a response);
- **Serial format**: 8 data bits, no parity, 1 stop bit (8N1);
- **Baud rate**: **4800 / 9600 (factory default) / 19200 / 38400 / 57600 / 115200 / 250K / 500K / 1M**; changing it: see 4.3;
- **Supported function codes**: FC01 (read coils), FC05 (write single coil), FC15 (write multiple coils), FC03 (read holding), FC04 (read input), FC06 (write single register), FC16 (write multiple registers);
- Register values are big-endian (Modbus standard); exceptions are standard Modbus exception codes (01/02/03/04).

### 4.2 System Identification (FC04, read-only)

| Address | Meaning |
|---|---|
| 0x0001 | Module type = 1 (EX_16DO) |
| 0x0002 | Firmware version |
| 0x0008 | Hardware version (0xMMmm, 0x0100 = v1.0) |
| 0x0004/0x0005 | Uptime seconds (high word / low word) |
| 0x0006 | CRC error-frame counter (link-quality diagnostic) |
| 0x0007 | Status word: bit0 = in safe state, bit1 = unclaimed events pending |

### 4.3 Baud Rate Configuration (0x0011 immediate switch + 0x0281 power-up rate) — firmware v2.8+

Baud codes: 6 = 4800, 7 = 9600 (factory default), 8 = 19200, 9 = 38400, 10 = 57600, 11 = 115200, 1 = 250K, 2 = 500K, 0 = 1M (shared by both registers).

**Ships at 9600** — most masters connect first try with no configuration. When another rate is needed:

- **0x0281 power-up baud rate** (FC06 write + persist): write the target code, then write 0xA55A to 0x0200 to save — after the next power cycle the module runs at the new rate. Example: switch to 19200 = write 0x0281 = 8 → write 0x0200 = 0xA55A → restart;
- **0x0011 immediate switch** (FC06 write, advanced): the response is sent at the **old** rate, then the module switches about 20 ms later — the master must follow in the same window; a switch that was not persisted **falls back to the power-up rate after 3 s of bus silence** (anti-lockout protection) — keep the master polling (see §6);
- Recovery when the module cannot be found: scan the rates in the table above (reading 0x0011, at most 12 attempts);
- On the Edgron i757 backplane no setup is needed — the master discovers the module automatically and pins its power-up rate to the backplane operating rate (one-time; every later power-up connects directly).

## 5. Output Control

### 5.1 Coils (FC01 read / FC05 write single / FC15 write multiple)

| Coil address | Meaning |
|---|---|
| 0–15 | Outputs O1–O16 (1 = on, 0 = off) |

- Writes take effect immediately; reads return the output read-back (consistent with the panel LEDs);
- **All outputs default off at power-up**, waiting for the master to write.

### 5.2 Output Word Mirror (holding register 0x0203, FC03 read / FC06 write)

The 16 outputs packed into one 16-bit word (bit0 = O1 … bit15 = O16), fully equivalent to coils 0–15 — convenient for masters that only speak register function codes, or for reading/writing all 16 channels in one frame.

## 6. Safe State & Recovery

- **No valid frame addressed to the module (or broadcast) for 3 seconds → safe state: all 16 outputs switch off**, status-word bit0 set; power-up default is the safe state (all off).
- Any valid frame exits the safe state immediately, but **outputs remain off until the master rewrites them** — recovery belongs to the master; the module never restores old outputs on its own.
- The master should therefore: **poll/refresh at a period < 3 seconds (≤1 second recommended)** and rewrite the output coils cyclically (not just once) — after a brief bus interruption the outputs then recover automatically on the next scan.
- On the i757 backplane all of the above is handled by the master automatically.

## 7. Event Register

**Event register 0x0010** (FC03 read / FC06 **write-1-to-clear**):

| Bit | Meaning |
|---|---|
| bit0 | Powered up / was reset (poll-free evidence the module rebooted) |
| bit1 | Entered the safe state |
| bit2 | Bad-frame count over threshold (line-quality warning) |

Bits latch on the event and do not clear on recovery; the master clears them by writing 1 — from bit0/bit1 the master knows "the module rebooted / dropped off and its outputs were switched off", and can decide to re-issue the outputs.

## 8. Integrating with a Third-Party Modbus Master

To integrate with any Modbus master (PLC / SCADA / gateway), the minimum set is three steps:

1. Set the DIP address and configure the serial port per 4.1 (factory 9600 8N1);
2. **Cyclically (≤1 s) write coils 0–15 with FC15** (or FC06 to 0x0203) = refresh all outputs;
3. Optional: poll 0x0010 for "rebooted / entered safe state" events, and 0x0007 for the current status.

## Revision

- v1.0 (2026-08-13): initial release (per the "one manual per board" policy; template = PH_EC user manual). Sources: former `Installation_and_Wiring.md` §8 (terminal table / common wiring / load rules) + backplane bus contract register usage.
