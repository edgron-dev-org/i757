# EX_BUS Bus Access Board — User Manual

> Companion documents: `i757_Controller_Manual.md` (host controller), module manuals (`EX_16DO_Module_Manual.md`, `EX_16DI_Module_Manual.md`, `PH_EC_Transmitter_User_Manual.md`).

---

## 1. Overview

EX_BUS is a **passive bus access board** in the same standard 18-terminal expansion enclosure as the expansion modules. It brings **power (9–36 V DC), protective earth (PE) and the RS-485 bus** from front-panel terminals onto the module-row backplane bus, with filtering and transient protection on the power and bus entries.

Two typical uses:

1. **Backplane power injection**: when an i757 backplane row carries many modules and the total current gets large, insert EX_BUS mid-row or at the far end and feed the field 24 V in locally — reducing the voltage drop along a long row;
2. **Bus entry for a stand-alone module group**: when expansion modules are used on their own (without an i757), EX_BUS is the group's external interface — the third-party master's RS-485 and the supply enter here, the modules stack alongside via the backplane feed-through, and **no module needs to give up front terminals for bus wiring**; entry filtering and protection are handled centrally by this board.

## 2. Specifications

| Item | Value |
|---|---|
| Form | Standard 18-terminal expansion enclosure; passive (no MCU / no firmware / no bus address) |
| Power | 9–36 V DC (24 V nominal); two paralleled terminals per rail |
| Bus | RS-485 half duplex (A/B pair) |
| Protection | Filtering and transient protection on power and bus entries; PE terminal for protective earth |
| Terminals | 2×9 = 18 positions (row A / row B), see §3 |
| Mounting | i757 internal blade slot / side-mounted stack (interconnects with neighbouring modules via the backplane feed-through) |

## 3. Installation and Wiring

Front terminal assignment (rows A / B, 9 positions each):

| Terminal | Function |
|---|---|
| A1, B1 | Supply + , 9–36 V DC (two positions in parallel, sharing the current) |
| A2, B2 | 0 V (two positions in parallel) |
| A3, B3 | PE protective earth |
| A4 | RS-485 A |
| B4 | RS-485 B |
| A5, B5 | **Termination enable pair**: bridge A5–B5 with a wire link to engage the on-board 120 Ω bus terminator; leave open when no termination is needed (A5 is at the 485 A potential — do not use it for anything else) |
| A6–A9, B6–B9 | Reserved (not connected) |

Wiring notes:

- **Power**: each rail has two paralleled terminals — at higher currents wire both to share the terminal load; size the supply wiring for the whole group's current;
- **PE**: bond A3/B3 to the cabinet earth bar nearby; whether PE and 0 V are bonded is a site earthing-policy decision (the board does not force it);
- **RS-485**: twisted pair, A to A, B to B — polarity matters; route away from power wiring;
- **Stacking**: EX_BUS interconnects with neighbouring modules via the backplane feed-through — power and bus run along the row automatically, zero wiring inside the group;
- **Cut the 24 V supply before inserting or removing any module, including this board** (hot-plugging is not allowed).

## 4. The Two Use Cases

### 4.1 Backplane power injection (inside an i757 system)

Insert EX_BUS anywhere in the module row (preferably near the far end or the heavy loads) and feed the field 24 V into the front terminals:

- **Use the same field supply as the main feed** (a second local feed from the same 24 V bus) — the goal is reducing the row voltage drop, not dual-supply redundancy;
- 0 V and PE are continuous along the row; leave the RS-485 terminals (A4/B4) **unwired** in this use case (the bus is already inside the backplane).

### 4.2 Bus entry for a stand-alone module group (without an i757)

Place EX_BUS at the end of the group and stack the modules next to it:

- Wire the third-party Modbus master's RS-485 to A4/B4, the field 24 V to A1/A2 (parallel B1/B2 when needed), PE to A3/B3;
- Set each module's DIP address (address = DIP + 1, unique within the group); serial parameters per each module's manual (all ship at 9600 8N1);
- For multi-drop runs continuing to another group, the next bus segment may share the same terminals;
- **Bus termination**: when this group sits at the end of the line, bridge A5–B5 to engage the on-board 120 Ω terminator (see §3); leave it open when the group is mid-line.

## Revision

- v1.0 (2026-08-13): initial release.
