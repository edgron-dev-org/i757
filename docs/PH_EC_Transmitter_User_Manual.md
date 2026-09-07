# PH_EC Water-Quality Transmitter Module — User Manual

**Model:** i757 PH_EC (module type 2) · **Applies to:** firmware ≥ v0.21

---

## 1. Product Overview

The PH_EC is a **2-channel pH + 2-channel conductivity (EC) + 3-channel temperature** water-quality transmitter. It connects to any Modbus master (PLC / gateway / industrial PC) over **RS-485 (Modbus RTU slave)**, and can also be used as a backplane expansion module of the Edgron i757 controller.

- Every channel is calibrated independently; calibration parameters survive power cycles;
- Measurements are published directly in engineering units (pH × 100, µS/cm, °C × 10) — the master needs no conversion formulas;
- EC readings are automatically temperature-compensated to 25 °C (coefficient configurable);
- Published values pass a built-in smoothing filter (configurable), fault detection is debounced, and a faulted channel reports an explicit invalid marker instead of fake data;
- Dual pH / dual EC probes support redundant installation in the same tank (measured cross-influence data: Appendix A).

## 2. Specifications

| Item | Value |
|---|---|
| Power supply | 9–36 V DC wide-range (24 V nominal) |
| Communication | RS-485 half-duplex, Modbus RTU slave, 8N1 |
| pH | 2 channels, 0–14 pH, resolution 0.01 pH; two-point calibration, typical accuracy ±0.05 pH after calibration (electrode-dependent) |
| EC | 2 channels, 0.025–10 mS/cm, resolution 1 µS/cm; ±1.5% across the range after standard-solution calibration; auto-compensated to 25 °C |
| Temperature | 3 channels, PT100 (default) / PT1000 / NTC configurable, resolution 0.1 °C |
| Probe interface | 2-wire; pH = measuring + reference electrode, EC = electrode pair (interchangeable), temperature = sense + return |
| Indicators | Run LED 1 Hz heartbeat; channel LEDs solid = OK, 2 Hz blink = probe fault |

## 3. Installation and Wiring

The terminal block has 9 rows × 2 positions (A left / B right), arranged from the bus end downward:

| Row | A                       | B                       | Notes                                             |
|-----|-------------------------|-------------------------|---------------------------------------------------|
| 1   | 24V                     | 0V                      | Power                                             |
| 2   | 485_A                   | 485_B                   | RS-485 bus                                        |
| 3   | EC1 electrode           | EC1 electrode           | Wires interchangeable                             |
| 4   | Temp EC (shared)        | GND                     | Shared water temperature for EC1/EC2 compensation |
| 5   | EC2 electrode           | EC2 electrode           | Wires interchangeable                             |
| 6   | Temp PH1                | GND                     | Dedicated compensation for pH1                    |
| 7   | PH1 measuring electrode | PH1 reference electrode | **Not interchangeable**                           |
| 8   | PH2 measuring electrode | PH2 reference electrode | **Not interchangeable**                           |
| 9   | Temp PH2                | GND                     | Dedicated compensation for pH2                    |

Wiring notes:

- **Do not ground the solution**: the measurement side is galvanically floating by design; with a 3-wire pH probe, leave the third (solution-ground) wire unconnected;
- Connect the probe cable shield to the module-side GND terminal at **one end only** (verify the shield is open-circuit to the cores first);
- The pH measuring (glass) and reference electrodes must not be swapped; EC pairs and temperature probes have no polarity;
- For multi-probe installations in the same tank, spacing and cross-influence margins are covered in Appendix A.

## 4. Modbus Communication

### 4.1 Link Parameters

- **Slave address**: 4-bit DIP switch, address = DIP value + 1 (1–16); address 0 = broadcast (write function codes only, executed without a reply);
- **Serial format**: 8 data bits, no parity, 1 stop bit (8N1);
- **Baud rate**: **4800 / 9600 (factory default) / 19200 / 38400 / 57600 / 115200 / 250K / 500K / 1M**; fixed-rate design, see 4.3 to change it;
- **Supported function codes**: FC03 (read holding), FC04 (read input), FC06 (write single register), FC16 (write multiple registers; not used for the 0x0200 configuration area);
- Register values are big-endian (Modbus standard); errors are reported as standard Modbus exceptions (01/02/03/04).

### 4.2 System Identification (FC04, read-only)

| Address | Meaning |
|---|---|
| 0x0001 | Module type = 2 (PH_EC) |
| 0x0002 | Firmware version |
| 0x0008 | Hardware version (0xMMmm, 0x0100 = v1.0) |
| 0x0004/0x0005 | Uptime in seconds (high/low word) |
| 0x0006 | CRC error-frame counter (link-quality diagnostic) |

### 4.3 Baud-Rate Configuration (0x0011 live switch + 0x0281 power-up rate)

Baud-rate code table (shared by both registers):

| Code | Rate | Code | Rate |
|---|---|---|---|
| 0 | 1 M | 7 | 9600 (factory default) |
| 1 | 250 K | 8 | 19200 |
| 2 | 500 K | 9 | 38400 |
| 5 | 5 M | 10 | 57600 |
| 6 | 4800 | 11 | 115200 |

(Codes 3/4 = 2M/3M, reserved for system use.)

**The module ships at 9600** — most masters connect on first contact with no configuration. To use a different rate:

- **0x0281 power-up baud** (FC06 write + persist): write the target code → write 0xA55A to 0x0200 to save → after a power cycle the module runs at the new rate. Example, 19200: write 0x0281=8 → write 0x0200=0xA55A → restart;
- **0x0011 live switch** (FC06 write, advanced): the reply is sent at the **old** rate, and the module switches about 20 ms later — the master must follow within the same window. An unsaved live switch **falls back to the power-up rate** after 3 s of bus silence (anti-lockout protection);
- Recovering a module at an unknown rate: scan the table from the master (read 0x0011, at most 12 attempts);
- When plugged into an Edgron i757 backplane, no setup is needed — the host automatically manages the module's bus rate.

## 5. Measurement Data (FC04, 0x0108–0x010F)

| Address | Meaning | Scaling | Type |
|---|---|---|---|
| 0x0108 | pH1 | pH × 100 (e.g. 685 = pH 6.85) | int16 |
| 0x0109 | pH2 | same | int16 |
| 0x010A | EC1 | µS/cm, compensated to 25 °C | uint16 |
| 0x010B | EC2 | same | uint16 |
| 0x010C | Temp PH1 | °C × 10 | int16 |
| 0x010D | Temp PH2 | °C × 10 | int16 |
| 0x010E | Temp EC1 (not brought out on this model; always reads 0x7FFF) | °C × 10 | int16 |
| 0x010F | Temp EC2 (= the shared EC water-temperature terminal) | °C × 10 | int16 |

- **Invalid marker 0x7FFF**: a faulted or disconnected channel publishes 0x7FFF — never a stale or fabricated number. A fault is declared only after 3 s of consecutive invalid readings (configurable, §6.6); recovery is immediate;
- Published values are smoothed (median + damping, default 2 s, configurable); the unfiltered instantaneous values are available at 0x0290–0x0297 (FC03) for diagnostics;
- 0x0100–0x0107 is a factory diagnostic area; not needed in normal use.

## 6. Configuration and Calibration (FC03 read / FC06 write, 0x0200 area)

**General rule**: writes take effect immediately but are not persisted; writing **0xA55A to 0x0200 saves to flash** (survives power cycles). Writing 0x0F0F restores factory defaults (RAM only; save again to persist). Status register 0x0201: bit0 = unsaved changes, bit1 = valid flash copy exists, bit2 = last save succeeded.

### 6.1 Temperature Channels (n = 0,1 → PH1/PH2 at base 0x0210 + 8n; shared EC channel at base 0x0228)

| Offset | Name | Description |
|---|---|---|
| +0 | TYPE | 0 = PT100 (default) / 1 = PT1000 / 2 = NTC / 0xFF = disabled |
| +1 | PULLUP | Reference resistor in Ω (factory-set to match the hardware; normally leave alone) |
| +2 | R25 | NTC resistance at 25 °C, units of 10 Ω (TYPE = 2 only) |
| +3 | B | NTC β value (TYPE = 2 only) |
| +4 | OFFSET | Calibration offset, °C × 10, int16 (may be written directly) |
| +5 | CAL | **One-touch calibration**: with the probe settled, write the true temperature × 10 → the offset is computed automatically; write 0x7FFF to clear |

### 6.2 pH Channels (n = 0,1, base 0x0240 + 8n)

| Offset | Name | Description |
|---|---|---|
| +0 | ZERO | Electrode zero, 0.1 mV, int16 (calibration product; experts may write directly) |
| +1 | SLOPE | Electrode slope, 0.1% (1000 = 100.0%; 90–105% = healthy electrode) |
| +2 | CAL_A | Two-point calibration, first point: with the probe settled in buffer A, write its pH × 100 (e.g. 686) |
| +3 | CAL_B | Second point: switch to buffer B (≥ 1 pH away from A), write its pH × 100 → zero + slope are solved |
| +4 | CAL_STATE | Read-only: 0/1/2 = points calibrated; write 0 = reset calibration to defaults |
| +5 | TSRC | Compensation source: 0 = Temp PH1 / 1 = Temp PH2 / 3 = Temp EC / 0xFE = none (fixed 25 °C) / 0xFF = auto (default: own channel); see §6.4 |

**pH calibration procedure**: probe into buffer A (e.g. pH 6.86) → wait for a stable reading → write 686 to CAL_A → rinse, probe into buffer B (e.g. pH 4.00) → stable → write 400 to CAL_B → check SLOPE is within 90–105% → write 0xA55A to persist. Calibration normalizes for the actual liquid temperature automatically — buffers need not be at 25 °C.

### 6.3 EC Channels (n = 0,1, base 0x0260 + 8n)

| Offset | Name | Description |
|---|---|---|
| +0 | K | Cell constant, units of 0.001/cm (1000 = K 1.000; the probe's nameplate value may be written directly) |
| +1 | TCOEF | Temperature coefficient, 0.01%/°C (default 2.00%/°C; KCl standards ≈ 1.9, nutrient solutions 1.9–2.2; 0 = compensation off) |
| +2 | CAL | **One-touch calibration**: with the probe settled in a standard solution, write the solution's 25 °C nominal value in µS/cm (e.g. 1413) → K is solved automatically |
| +5 | TSRC | Compensation source: as §6.2 (default auto = the shared EC temperature); see §6.4 |
| +6/+7 | (factory) | Factory electronics calibration — **do not modify** |

**EC calibration procedure**: confirm TCOEF matches your working liquid → probe into the standard solution (1413 µS/cm recommended) and let it settle → write 1413 to CAL → write 0xA55A to persist. The solution need not be at 25 °C — compensation reconciles both sides; **one probe per calibration vessel at a time**.

### 6.3bis pH Board-Zero Audit (E_ZERO, firmware ≥ v0.29)

The pH conversion is `mV = ADC − E_ZERO − ZERO`. E_ZERO is the input offset of **this transmitter** (front-end amplifier, ADC); ZERO/SLOPE belong to the **electrode**. Without E_ZERO an electrode calibration absorbs the board offset into ZERO, so a set of probe parameters is only valid on the module that calibrated it; between audited modules, probe parameters can be moved as they are.

| Channel | E_ZERO (0.1 mV, r/w) | E_ZCAL (one-shot audit) |
|---|---|---|
| pH1 | 0x0246 | 0x0247 |
| pH2 | 0x024E | 0x024F |

Procedure: unplug the electrode, short the BNC centre to its shield (a shorting cap is best), wait 10 s; read E_ZCAL to see the raw input (0.1 mV) settle; write 1 to E_ZCAL — the module stores the present reading as E_ZERO (a reading beyond ±50 mV is refused: not shorted); repeat for the other channel; write 0x0200 = 0xA55A to persist. Then calibrate the electrodes as in 6.3. Done once per board at the factory and recorded in its unit file.

### 6.3ter Two pH Electrodes in One Tank (firmware ≥ v0.37)

With two combination pH electrodes in the same liquid the transmitter ties both references to its reference potential (0x0230 bit 2 = 1, factory default). Lifting or removing either electrode moves the other by about 0.04 pH at that instant (the mutual loading of the two reference junctions disappears) and it then stays steady at the normal 2 s update rate. **Calibrate with both electrodes in the liquid** so that this loading offset is absorbed by the calibration. Both electrodes **must sit in the same liquid**: with one in a buffer and the other still in the tank, the two liquids are coupled through the shared reference node and the reading is off (0.2 pH measured), so move both electrodes together when calibrating. **Leave the temperature probes in the tank, not in the buffer cup**: with a shared compensation source (TSRC mean) the cup temperature replaces the tank temperature and EC is compensated with the wrong value (a 0.5–0.6 °C cup/tank difference measured as ±1.5 % on EC); the pH-side cost of leaving them in the tank is below 0.005 pH. With a single electrode simply leave the other channel unconnected.

### 6.4 Compensation Source Selection (TSRC) — One Temperature Probe Feeding All Channels

Both the pH electrode slope and the 25 °C EC normalization (§5) need the **true temperature of the measured liquid**. Which temperature input each measurement channel uses is selected independently by its TSRC register:

| Channel | TSRC address | Writable values |
|---|---|---|
| pH1 | 0x0245 | 0 = Temp PH1 / 1 = Temp PH2 / 3 = shared EC water temperature |
| pH2 | 0x024D | 0xFE = no compensation (fixed 25 °C) |
| EC1 | 0x0265 | 0xFF = auto (default): own channel; with no probe of its own, |
| EC2 | 0x026D | pH falls back to the 25 °C slope and EC borrows the shared EC water temperature |
|  |  | **0x20 + mask (firmware >= v0.28) = the mean of the valid channels in the mask**: bit0 = Temp PH1, bit1 = Temp PH2, bit2 = Temp EC1, bit3 = Temp EC2; e.g. 0x23 = mean of Temp PH1 and Temp PH2. A channel that reads invalid drops out of the mean; with one probe left, that probe is used alone |

**Typical setup: a single water-temperature probe** (on the shared EC temperature terminal). Write 3 to all four TSRC registers, then persist with 0xA55A:

```
FC06 write 0x0245 = 3    (pH1)
FC06 write 0x024D = 3    (pH2)
FC06 write 0x0265 = 3    (EC1)
FC06 write 0x026D = 3    (EC2)
FC06 write 0x0200 = 0xA55A  (save)
```

**Two water-temperature probes averaged** (one beside each electrode pair, each backing up the other): write 0x23 (= 35) to all four TSRC registers, then 0xA55A to persist. If either probe is unplugged or faults, compensation falls back to the other one without interruption.

Notes:

- **Never compensate water chemistry with air temperature.** If one of the temperature terminals carries an ambient-air probe (for logging only), set each measurement channel's TSRC explicitly as above — leaving it on "auto" would make the corresponding pH channel compensate its in-water electrode with the air temperature on its own terminal;
- **Set TSRC before calibrating**: calibration captures temperature through the same selection, so a wrong source skews the calibration with it;
- After switching sources, a **one-time step in the EC reading is expected** (the compensation reference changed; magnitude ≈ temperature difference × TCOEF, e.g. a 2 °C difference ≈ 4%). It is not a fault;
- When the module is connected to an Edgron controller, these registers can be read, written, and persisted remotely from the cloud with the `mbr`/`mbw` commands (see the *i757 Controller Manual*) — no site visit needed.

### 6.5 Output Smoothing (per channel, n = 0–7 matching the 8 values at 0x0108–0x010F, base 0x0270 + 2n)

| Offset | Name | Description |
|---|---|---|
| +0 | SM_MEDIAN | 0 = off / 1 = median filter (default 1) — single-sample outliers are dropped entirely |
| +1 | SM_TAU | Damping time constant, units of 0.1 s (default 20 = 2 s; 0 = off; max 600) |

### 6.6 Fault Debounce and Instantaneous Values

| Address | Name | Description |
|---|---|---|
| 0x0280 | FAULT_N | Consecutive invalid seconds before a fault is declared (1–10, default 3); recovery is not debounced — a re-inserted probe reads again within seconds |
| 0x0281 | BAUD_BOOT | Power-up baud-rate code (see 4.3; default 7 = 9600) |
| 0x0290–0x0297 | Instantaneous values (read-only) | Pre-filter engineering values, same order as 0x0108–; for diagnostics |

## 7. Events and Faults

- **Event register 0x0010** (FC03 read / FC06 **write-1-to-clear**): bit8 = PH1 fault, bit9 = EC1 fault, bit10 = PH2 fault, bit11 = EC2 fault. A bit sets on entering the fault and stays set until the master clears it — polling this register is an alternative to checking each channel for 0x7FFF;
- Fault detection is debounced (§6.6): brief transients (splashes, momentary lift-out) do not trigger it;
- The module has no output channels: a communication outage does not affect measurement; readings resume as soon as communication returns (at a non-default rate the module falls back to its power-up rate, see 4.3).

## 8. Quick Start with a Third-Party Modbus Master

Integration with any Modbus master (PLC / SCADA / gateway) takes three steps:

1. Set the DIP address and configure the serial port per 4.1 (9600, 8N1 out of the box);
2. Cyclically read 8 registers starting at FC04 0x0108 = all measurements (a period of ≥ 2 s is recommended — it matches the module's measurement cycle; polling faster adds nothing);
3. Treat 0x7FFF as "channel invalid"; optionally poll 0x0010 for fault events.

Calibration can be performed either through the registers described in this manual, or through the Edgron cloud when the module is connected to an Edgron controller.

## Appendix A. Probe Cross-Influence Test Report

**Date:** 2026-08-12
**Module:** i757 pH/EC water-quality module (2× pH + 2× EC + temperature)
**Test medium:** live nutrient solution (fertigation stock), ≈1.5 mS/cm, pH ≈6.0, 17 °C
**Probes:** 2× pH + 2× EC; **within each pair the two probes are of different makes/models (not a matched pair)** — the agreement figures below were obtained between dissimilar probes.
**Method:** probes introduced and removed one at a time; all channels logged continuously at 5 s intervals over the cloud link. Every figure below is from this recorded data.

### A.1 Summary

- **At working conductivity (~1.5 mS/cm), inserting or removing any probe disturbs the other channels by less than 1%, and the disturbance is fully reversible** — readings return to their exact prior values within seconds, with zero hysteresis.
- pH probes have **no measurable effect on EC readings** when entering or leaving the same tank.
- Two independent pH probes in the same working solution agree to within **0.05 pH**.
- Maintenance hot-swap is production-safe in the working range: pulling one EC probe shifts its neighbor by ~0.6% until it is re-inserted, then everything snaps back.

### A.2 Cross-influence matrix (measured)

"Neighbor EC" = the other EC channel in the same tank.

| Action | Effect on neighbor EC | Effect on pH channels |
|---|---|---|
| EC probe inserted, **touching** the neighbor probe | +1.3% | none |
| EC probe inserted, spacing ≥ 5 cm | < 0.5% | none |
| EC probe **removed** (5 cm spacing) | −0.6%; restored within one sample on re-insertion, zero hysteresis | none |
| pH probe inserted (either of two, in sequence) | within noise (< 0.5%) | no effect on the other pH probe |
| Probe lifted out of the liquid | channel reports a fault instead of false data (see A.4) | others unaffected |
| Temperature probe moved to a different-temperature location | temperature compensation tracks correctly (this is compensation working, not an error) | none |

### A.3 Dual-probe redundancy behavior

- Removal test: with both EC probes (two different makes) at 5 cm spacing, one probe was pulled out for 15 s and re-inserted. The remaining channel moved −0.6% while alone and returned to its exact prior reading within one sample — the before/after baselines matched to the last digit.
- Two pH probes of **different makes** in the same clean working solution converge to within 0.05 pH of each other. (A growing disagreement between two pH probes is itself a useful diagnostic — it usually indicates an aging reference junction on one of them.)

### A.4 Fault reporting and reading stability (firmware ≥ v0.18)

- Published values are **smoothed**: a median filter drops single-sample glitches entirely, followed by configurable damping (default time constant 2 s). Instantaneous values remain available in a separate register block for diagnostics.
- **Fault debounce:** a probe must read invalid for 3 consecutive seconds before the channel reports a fault — brief transients (splashes, momentary lift-out) are invisible. Recovery is immediate: the first valid sample after re-insertion restores the reading within 2–3 s.
- A faulted channel reports a distinct invalid marker (0x7FFF), never a stale or fabricated number.

*All figures in this report were measured on production hardware in live nutrient solution and are reproducible from the logged data. Percentages are relative changes on the affected channel; absolute EC accuracy is set by the standard-solution calibration (±1.5% across the range after the factory electronics audit).*
