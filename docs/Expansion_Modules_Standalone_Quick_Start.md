# Expansion Modules Without a Controller — Standalone Quick Start

> For EX-16DI, EX-16DO and PH_EC modules used **on their own** as plain Modbus RTU slaves, driven by
> your PLC, gateway or PC instead of an I757-M. Covers assembly, wiring, the first test with a
> USB-RS485 adapter, and the registers your own master needs. Each module's full register map is in
> its own manual (`EX_16DI_Module_Manual.md`, `EX_16DO_Module_Manual.md`,
> `PH_EC_Transmitter_User_Manual.md`); the entry board is `EX_BUS_Module_Manual.md`.

## 1. One rule: the group needs an EX_BUS board

Expansion modules have **no power or RS-485 terminals of their own**. Inside an I757-M they get
both from the backplane. On their own they get both the same way, from an **EX_BUS bus access
board** stacked at the end of the row: the external 24 V and the RS-485 pair enter on the EX_BUS
front terminals and run along the row through the backplane feed-through connectors. The module
front terminals stay free for field wiring.

So a standalone order is always *modules + one EX_BUS per group*. There is no other supported way
to power a module or to reach its bus.

## 2. What you need

| Item | Notes |
|---|---|
| EX_BUS board | One per group of modules. Passive: no address, no firmware. |
| 24 V DC supply | 9–36 V DC. Budget roughly 0.1 A per module plus your field loads; a bench supply with a 100 mA limit is not enough. |
| USB-RS485 adapter | FTDI or CH340 based, any brand. Terminals are labelled A/B or D+/D−: **A = D+, B = D−**. An adapter with a GND terminal is preferable. |
| Twisted pair | Any 2-wire twisted cable for the bench; shielded twisted pair in the field. |
| PC with Python 3 | `pip install pymodbus pyserial`. The test script is `tools/u485_quickstart.py`. Any Modbus RTU master tool (QModMaster, Modbus Poll, mbpoll) works the same way. |

## 3. Assemble and wire (power off)

1. **Set each module's DIP address**: address = DIP value + 1 (0000 → 1, 0001 → 2, …, 1111 → 16).
   Unique within the group. Addresses are read at power-up, so set them before step 4.
2. **Stack** the modules side by side with the EX_BUS at one end, mating the backplane feed-through
   connectors (ground-first design, but never under power).
3. **Wire the EX_BUS front terminals**:

   | Terminal | Connect |
   |---|---|
   | A1 (and B1) | 24 V DC + |
   | A2 (and B2) | 0 V |
   | A3 / B3 | PE, to the cabinet earth bar |
   | A4 | RS-485 **A** ← adapter A / D+ |
   | B4 | RS-485 **B** ← adapter B / D− |
   | A5–B5 | Bridge with a short wire link = on-board 120 Ω terminator. Do it when this group is at the end of the line, which on a bench it always is. |

   If the adapter has a GND terminal, connect it to A2 (0 V). It is not required for a short bench
   lead but it removes the common-mode guesswork on longer runs.
4. **Power on.** Module status LEDs light. Nothing else happens until a master talks.

## 4. Serial settings

- **9600 baud, 8 data bits, no parity, 1 stop bit** out of the box. Slave address = DIP + 1.
- Every module supports 4800 … 115200, 250 k, 500 k and 1 M. The power-up rate is holding register
  `0x0281` (code 7 = 9600, 8 = 19200, 9 = 38400, 10 = 57600, 11 = 115200, 0 = 1 M), saved by writing
  `0xA55A` to `0x0200`, effective after a power cycle. Details: module manual §4.3.
- **A module that has ever been plugged into an I757-M runs at 1 Mbps.** The controller pins the
  module's power-up rate to the backplane rate the first time it discovers it. Taken out and used
  standalone, such a module does not answer at 9600. Either scan the rates
  (`u485_quickstart.py --baud scan`, if your adapter goes to 1 M) and set `0x0281 = 7` + `0x0200 =
  0xA55A`, or plug it back into the controller and change it from there.

## 5. First test with the script

```
pip install pymodbus pyserial
python tools/u485_quickstart.py --port COM7          # Linux: --port /dev/ttyUSB0
```

The script scans addresses 1–16, prints every module's identity block, and shows its data:

```
--- COM7 @ 9600 8N1 ---
addr  1: EX_16DI fw=0x0003 hw=0x0100 uptime=42s crc_err=0 flags=0x0000
   DI1..16 = 0000000000000000 (1 = loop closed)
   counters = [0, 0, 0, ...] (rising edges since power-up)
addr  3: EX_16DO fw=0x0208 hw=0x0100 uptime=42s crc_err=0 flags=0x0001  [SAFE STATE]
   O1..16 read-back = 0000000000000000
   (--toggle blinks O1 for 10 s; note the 3 s safe-state rule in the manual, section 6)
addr  4: PH_EC   fw=0x0026 hw=0x0200 uptime=41s crc_err=0 flags=0x0000
   pH1 = invalid  pH2 = invalid
   EC1 = invalid  EC2 = invalid
   T_pH1 = invalid  T_pH2 = invalid  T_EC = invalid
found 3 module(s) at 9600 baud
```

Then exercise each module:

- **EX-16DI**: wire COM_A (terminal A9) to 0 V and touch DI1 (A1) to 24 V. Run the script again:
  `DI1..16 = 1000…` and counter 1 has incremented once per touch (10 ms debounce by default).
- **EX-16DO**: `python tools/u485_quickstart.py --port COM7 --addr 3 --toggle` blinks O1 for 10 s
  while refreshing every second; the O1 LED follows. When the script stops polling, **all outputs
  drop after 3 s** and `flags` shows bit0 = safe state. That is the design: a master must keep
  polling (period under 3 s, 1 s recommended) and rewrite the coils cyclically. Read-back (FC01)
  matches the panel LEDs.
- **PH_EC**: with probes connected the values appear; `invalid` (raw `0x7FFF`) means no probe, out
  of range or a fault on that channel. Poll every 2 s or slower, the module measures on a 2 s cycle.

## 6. Using your own master (PLC, SCADA, gateway)

What every module answers, with **0-based register addresses as they appear in the frame** (some
tools display them +1, or as 30001/40001-style numbers; check your tool's convention):

| | EX-16DI (type 3) | EX-16DO (type 1) | PH_EC (type 2) |
|---|---|---|---|
| Identity | FC04 `0x0000`–`0x0008`: `0x0001` type, `0x0002` firmware, `0x0008` hardware, `0x0004/5` uptime, `0x0006` CRC-error count, `0x0007` status (bit0 safe state) | same | same |
| Data | FC02 inputs 0–15 (levels); FC04 `0x0200`–`0x021F` 16 × u32 counters, high word first | FC01 coils 0–15 read-back; FC05/FC15 write coils 0–15; or holding `0x0203` = all 16 as one word | FC04 `0x0108`–`0x010F`: pH1, pH2 (×100), EC1, EC2 (µS/cm at 25 °C), T_pH1, T_pH2, (T_EC1 unused), T_EC (×10 °C); `0x7FFF` = invalid |
| Configuration | holding `0x0020`–`0x002F` debounce ms; `0x0030` counter clear bitmap | — | `0x0200` area: probe type, calibration, compensation source (manual §6) |
| Persist | write `0xA55A` to `0x0200` | write `0xA55A` to `0x0200` | write `0xA55A` to `0x0200` |
| Baud | `0x0281` power-up code (+ persist) | same | same |
| Safe state | none (inputs only) | **outputs off after 3 s without a valid frame**; master must refresh cyclically | none |
| Poll period | as fast as you like; counters are edge-counted on the module | < 3 s, 1 s recommended | ≥ 2 s |

Values are big-endian (Modbus standard). Exceptions are the standard codes 01/02/03/04. Broadcast
address 0 is honoured for write function codes and never answered.

## 7. Troubleshooting

| Symptom | Check |
|---|---|
| No module answers at all | A/B swapped (try it, it is the most common cause); rate is 1 M because the module came off an I757-M (§4); DIP set after power-up; 24 V present on EX_BUS; adapter GND to 0 V on longer leads. |
| Some addresses answer, one does not | That module's DIP duplicates another, or was set after power-up. |
| `crc_err` keeps climbing | Missing termination (A5–B5), untwisted or very long cable, a second terminator somewhere, or the adapter's own bias resistors fighting the module's. |
| EX-16DO outputs switch off by themselves | Safe state: the master stopped polling for 3 s. Poll faster and rewrite the coils every cycle. |
| PH_EC reads `0x7FFF` | No probe on that channel, probe out of range, or a fault: manual §7 lists the fault events. Temperature channels need a PT100/PT1000/NTC selected in the configuration area (manual §6.1). |
| Works on the bench, not in the cabinet | Shielded twisted pair, shield earthed at one end, 485 wiring away from motor cables; terminate both ends of the line. |

Everything in this document is what the I757-M does for these modules automatically: discovery,
rate pinning, cyclic refresh, safe-state recovery and the cloud dashboard. Standalone use trades
that for a plain Modbus device you can drop into any system.

---
Revision: v1.0 (2026-09-29) first issue.
