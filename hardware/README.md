# Hardware schematics

Schematics for the boards listed below, published so you can see exactly what the
firmware in this repository runs on.

| File | Board | Contents |
|---|---|---|
| `i757_Main_Board_Schematic.pdf` | i757 controller, main board (Board 1) | System structure, block diagram, power, MCU, SDRAM / QSPI flash / microSD, Ethernet, CAN, six RS-485 ports, backplane bus, connectors |
| `i757_Board2_Schematic.pdf` | i757 controller, Board 2 (stacked on the main board) | Eight isolated high-speed inputs, USB-C service port, field terminals for RS-485 ports 5 and 6 |
| `EX_16DO_Schematic.pdf` | EX-16DO expansion module | STM32H503 core, 16 PhotoMOS outputs, backplane bus interface |

## Notes

- Exported 2026-11-19 from the current design. The sheet title blocks still carry the
  original drawing date and "VER1.0".
- Pin and peripheral assignment as used by the firmware:
  `docs/Hardware_Resource_Allocation.md`. Wiring and terminals:
  `docs/i757_Controller_Manual.md` and `docs/EX_16DO_Module_Manual.md`.
- Not published here: PCB layout, Gerber files, bills of materials, and the
  schematics of the other modules.

## Licence

These files are hardware design files under section 5 of the repository `LICENSE`
(Edgron Source-Available License v1.0): you may reproduce and study them for
personal use. Commercial use of them, including making hardware from them for
sale, is not permitted.
