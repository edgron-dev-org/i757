# Onboard HSDI Configuration and Counting Protocol v0.2 (design frozen 2026-07-19; pulse qualifier + edge recorder 2026-09-04)

> The **sole authoritative contract** for the 8 high-speed isolated digital inputs (CN4) on the
> main board: each channel can be configured as **plain DI / high-speed counter / encoder /
> Z index / software counter**, the data lands in the **ONBOARD region of the process image**, and
> the application and an external master (the Modbus window) see one and the same data.
> Upstream contracts: `Process_Image_and_IO_Mapping.md` (image layout and external window),
> `Hardware_Resource_Allocation.md` (timer / EXTI allocation).
> Modification discipline: changing channel capabilities, combination rules or the image layout
> requires updating this document **and** `PIMG_HSDI_*` in `Common/app_pimage.h`.

## 1. Front end and rate parameters (to be published in the manual)

| Stage | Parameter | Bottleneck? |
|---|---|---|
| ISOM8711 isolator | 25 Mbps | No |
| RC pre-filter 1k/22pF | τ≈22 ns → **−3 dB ≈ 7 MHz** | **Yes — the front-end ceiling** |
| 74LVC14 Schmitt trigger | ns class | No |

| Mode | Design figure | Limited by |
|---|---|---|
| Hardware counter / encoder | **≥1 MHz** input edge rate | Front-end RC (7 MHz in theory), ample margin left |
| Encoder speed limit | 1 MHz ÷ (4×PPR); e.g. 1000 PPR → **15000 rpm** | Same as above (×4 quadrature decoding) |
| Software counter (EXTI) | **≤10 kHz** per channel, **≤50 kHz** in total (CPU <10%) | Interrupt overhead |
| **Capture counter (COUNT_CAP)** [added 2026-09-03, mode value 8; **all 8 channels + pulse qualifier since 2026-09-04**] | **any channel** (every HSDI pin is a timer capture channel: TIM3_CH1~4 / TIM1_CH1~2 / TIM8_CH2~3): hardware ICF filter (`filter` 0..15, 15 ≈ 1 µs) → **both edges** captured and timestamped in the interrupt → **pulse qualifier `min_us`** (§10). Uses no EXTI line (HSDI2/7 and HSDI3/4 no longer exclude each other) and coexists with a hardware counter or encoder on the same timer. **The recommended mode for field pulse sensors**; ≤10 kHz (interrupt cost, same class as EXTI) | hardware filter + interrupt + qualifier; no polling, no task-load dependence. **Why (2026-09-04)**: four field hall flow meters, 15 ms pulses at 50 % duty, carry microsecond dips inside the high phase — every plain edge method (EXTI, ICF15 capture, timer external clock) read 1.5..2×; with `min_us = 1000` the count equals 1 kHz sampling and the measuring jug |
| **Sampled counter (COUNT_POLL)** [added 2026-09-03, mode value 7] | any channel, **≤200 Hz** (the CM4 samples the pin in its 1 ms main loop and counts edges; `edge` applies, `debounce_ms` does not apply to the count). **Fallback / cross-check mode since 2026-09-04** — the dirty-signal answer is COUNT_CAP + `min_us` | sampling theorem (1 kHz); the poll runs in a task, so a heavy load can miss an edge ≥1 ms wide. Not subject to the validator's EXTI/timer constraints |
| Software debounce `debounce_ms` [2026-07-19] | 0–255 ms (0 = off; **recommended 1–50**, mechanical contacts 5–20) | Software; applies to the **level bit** of every channel and to **software-counter edges**. Hardware-counter / encoder counting uses the timer input filter instead and is unaffected by this parameter |

> ⚠️ The figures above are **engineering design values, pending bench measurement**; only after
> that confirmation may they be used as published specifications.

## 2. Pins and timers (measured against the EDA netlist)

| Channel | Pin | Timer channel | EXTI line |
|---|---|---|---|
| HSDI0 | PC6 | TIM3_CH1 | EXTI6 |
| HSDI1 | PC7 | TIM3_CH2 | EXTI7 |
| HSDI2 | PB0 | TIM3_CH3 | EXTI0 |
| HSDI3 | PB1 | TIM3_CH4 | EXTI1 |
| HSDI4 | PK1 | TIM1_CH1 | EXTI1 |
| HSDI5 | PJ11 | TIM1_CH2 | EXTI11 |
| HSDI6 | PJ10 | TIM8_CH2 | EXTI10 |
| HSDI7 | PK0 | TIM8_CH3 | EXTI0 |

## 3. Four hard constraints (all imposed by the silicon, none can be worked around)

1. **A timer has exactly one counter** → an "accumulating count" requires external clock mode, so
   **one timer can count only one channel**. Four-channel input capture can only measure frequency
   or period; it is not an accumulating count.
2. **A timer's external clock can only be taken from TI1/TI2 (CH1/CH2)**, never from CH3/CH4 →
   **HSDI2 / HSDI3 / HSDI7 can never do hardware counting**.
3. **Encoder mode requires CH1+CH2** → only **TIM3 (HSDI0+1)** and **TIM1 (HSDI4+5)** can act as
   encoders; HSDI6/7 on TIM8 fall on CH2+CH3 and **cannot be an encoder**.
4. **EXTI lines are shared across ports by pin number** → **HSDI2 (PB0) and HSDI7 (PK0) contend
   for EXTI0**; **HSDI3 (PB1) and HSDI4 (PK1) contend for EXTI1**. The two members of a pair
   cannot use the interrupt (software counter / Z) at the same time.

## 4. Per-channel capability table

| Channel | Plain DI | Hardware counter | Encoder | Z index | Software counter (EXTI) |
|---|---|---|---|---|---|
| HSDI0 | ✓ | ✓ (TIM3) | ✓ **phase A** | — | ✓ EXTI6 |
| HSDI1 | ✓ | ✓ (TIM3) | ✓ **phase B** | — | ✓ EXTI7 |
| HSDI2 | ✓ | ✗ | ✗ | ✓ **Z of encoder 0** (TIM3_CH3 input capture, uses no EXTI) | ✓ EXTI0※ |
| HSDI3 | ✓ | ✗ | ✗ | ✓ **Z of encoder 1** (EXTI1) | ✓ EXTI1※ |
| HSDI4 | ✓ | ✓ (TIM1) | ✓ **phase A** | — | ✓ EXTI1※ |
| HSDI5 | ✓ | ✓ (TIM1) | ✓ **phase B** | — | ✓ EXTI11 |
| HSDI6 | ✓ | ✓ (TIM8) | ✗ | ✓ | ✓ EXTI10 |
| HSDI7 | ✓ | ✗ | ✗ | ✓ | ✓ EXTI0※ |

※ Restricted by constraint 4: EXTI0 can be given to either HSDI2 or HSDI7, not both; EXTI1 can be
given to either HSDI3 or HSDI4, not both.

**Resource-allocation rule (each timer picks exactly one role)**:
- **TIM3** = encoder (HSDI0+1) **or** one hardware counter (HSDI0 **or** HSDI1) **or** off
- **TIM1** = encoder (HSDI4+5) **or** one hardware counter (HSDI4 **or** HSDI5) **or** off
- **TIM8** = one hardware counter (**HSDI6 only**) **or** off

## 5. ★ Configuration combination table (not all configurations are simultaneously realisable)

| # | Combination | TIM3 | TIM1 | TIM8 | Encoders | Hardware counters | Remaining channels |
|---|---|---|---|---|---|---|---|
| **①** | **Two encoders + two Z (recommended)** | encoder 0+1, **Z = HSDI2** (CH3 capture) | encoder 4+5, **Z = HSDI3** (EXTI1) | counter HSDI6 | **2** | 1 | HSDI7 = DI or software counter (EXTI0) |
| ② | Two encoders without Z | encoder 0+1 | encoder 4+5 | counter HSDI6 | 2 | 1 | HSDI2/3/7 = DI or software counter (EXTI0/1 each one of two) |
| ③ | Three high-speed counters | counter HSDI0 or 1 | counter HSDI4 or 5 | counter HSDI6 | 0 | **3** | the other 5 channels = DI / software counter |
| ④ | One encoder + two counters | encoder 0+1, Z = HSDI2 | counter HSDI4 | counter HSDI6 | 1 | 2 | HSDI3/5/7 |
| ⑤ | All DI / all software counters | off | off | off | 0 | 0 | all 8 channels (EXTI0/1 each one of two) |

**The core trade-off: two encoders and three hardware counters cannot both be had** (the encoders
consume TIM1/TIM3, leaving only TIM8 for a single counter).

**Z index has fixed pins** (software-switchable; switching it off returns that channel to
DI / software counter):
- **Encoder 0** (TIM3, A = HSDI0, B = HSDI1) → **Z = HSDI2**, via **TIM3_CH3 input capture**: the
  Z edge **latches the encoder position in hardware**, with zero interrupts and zero jitter.
- **Encoder 1** (TIM1, A = HSDI4, B = HSDI5) → **Z = HSDI3**, via EXTI1 (HSDI4 is then in timer
  AF mode and does not occupy EXTI1, so there is no conflict).
- [2026-07-19, final] The numbering is unified as **0/1** (matching the API and the image); older
  revisions of this document used the letters A/B, reversed with respect to the code numbering —
  abolished. **ZREV/ZLATCH slot ownership**: `[0]` = encoder 0 (TIM3), `[1]` = encoder 1 (TIM1),
  i.e. FC04 `0x2012`–`13` / `0x2016`–`17` = encoder 0 and `0x2014`–`15` / `0x2018`–`19` =
  encoder 1.
- The Z action is configurable: `none / zero the position / latch position + increment revolution
  count`.

## 6. 16-bit counters → 32-bit software extension

TIM1/TIM3/TIM8 are all **16-bit**. CM4 reads the hardware counters in its 1 ms main loop and
accumulates the **signed 16-bit delta** into a 32-bit software accumulator:
- At a 1 MHz input the 1 ms delta = 1000 « 32768, so wrap detection is never ambiguous (safe up to
  ~32 MHz in theory).
- Encoder position is a **signed 32-bit** value (may be positive or negative); counters are
  unsigned 32-bit.

## 7. Process-image layout (ONBOARD region, input image)

The region map has been enlarged: **ONBOARD = bytes 0..63** (SLOTC→64, SLOTD→128, GP→192;
**SCAN is still at 256, so customer addresses are unaffected**).

| Offset | Size | Content | External Modbus |
|---|---|---|---|
| +0 | 1 B | **level bits** of the 8 channels (valid in every mode) | FC02 bits `0x2000`–`0x2007` |
| +1 | 1 B | per-channel "configured" status bit | — |
| +4 | 8×u32 | **32-bit slot per channel**: hardware count / software count / encoder position (signed) | FC04 registers `0x2002`–`0x2011` |
| +36 | 2×u32 | encoder Z revolution count | FC04 `0x2012`–`0x2015` |
| +44 | 2×u32 | encoder position latched at the most recent Z edge | FC04 `0x2016`–`0x2019` |

**Same region in the output image**: `+0`, 1 byte = the **counter reset bit** of each channel
(writing 1 clears that channel; **write-1-clears, one-shot**: CM4 acts and self-clears the bit
within the same 1 ms tick, no write-back of 0 needed) → externally FC01/05/15 bits
`0x2000`–`0x2007`. [2026-07-19, final: the first implementation set the bit but never cleared
it, so a single reset pinned the count at 0 forever — the W1C bit must self-clear.]

> Every channel gets a **fixed 32-bit slot** that does not change with the mode — the simplest
> possible address table for master-side integration.

## 8. Ownership and configuration model

- **Owned by CM4** (the real-time field-I/O rule): TIM1/TIM3/TIM8 plus every EXTI line used are
  assigned to CM4; CM4 writes the ONBOARD input region and reads the reset commands. The former
  `app_hsdi.c` on CM7 was only a CLI self-test and is being moved over.
- **Configuration = a compile-time static table** (consistent with the scan table), one
  `{mode, filter, edge, z_action, debounce_ms, min_us}` (`min_us` added 2026-09-04, §10) per channel. The **combination validator is one
  shared source compiled on both cores** (`app_pimage.h`: `pimg_hsdi_check`): CM7 runs it first
  inside `app_hsdi_setup()`, which **returns a negative error code immediately** on an illegal
  table (-1x counter placement / -2x..-4x encoder placement / -5x EXTI clash / -6x, -7x timer
  exclusivity / **-80..-83 encoder phases must come as a pair, and Z must belong to a live
  pair**); CM4 re-validates defensively on reload and falls back to all-DI on failure.
  [2026-07-19, final: originally only CM4 validated, silently, and `app_hsdi_setup()` always
  returned 0 — a rejected configuration was invisible to the customer; and pairing was not
  checked, so a single-phase "encoder" counted while floating and an orphan Z dereferenced a
  null timer handle inside the interrupt.]

## 9. Open items / pending measurement

1. Every rate figure in section 1 is pending bench measurement and confirmation.
2. The correspondence between the timer input-filter settings and the actual debounce behaviour
   (field contacts vs encoder).
3. Acceptance of the three Z actions on a real encoder.

## Bench commands (2026-09-03, app_user.c; identical over cloud `dn/cmd` and the USB console)

- `pulse <coil 4..16> [Hz]` / `pulse off`: a dedicated 1 ms task toggles the relay (half period = 500/Hz ms); while active the relay scan row runs at 1 ms, `off` restores 50 ms. Measured: the GAQW212 PhotoMOS relay makes a clean 100 Hz square wave (scope 100.0 Hz / 24 Vpp; Ton 2.7..3.2 ms caps it near 150 Hz); over a 5-minute window four channels (2 hardware + 2 polled/interrupt/capture) matched the source pulse for pulse.
- `pulse sw <EXTI line> <Hz>` (0 Hz = off): the CM4 fires that EXTI line from software (`EXTI->SWIER1`) in its 1 ms poll, ≤1 kHz, no wiring — exercises the software-counter interrupt path (RPC op 0x13).
- `hsdi <0..7> off|di|hw [icf]|poll`: re-mode one channel at run time (same validator as at boot; a rejected table is restored).
- `hsdi <ch> cap [icf] [min_us]` (defaults icf 15, min_us 1000) / `hsdi <ch> sw [min_us]` (default 0 = every edge counts): re-mode with parameters.
- `hsdi <ch> stat` (RPC 0x11): `cnt` = count, `poll`/`hi` = rising edges / high samples seen by a 1 kHz shadow sampler (what polling would read), `min` = shortest interval between interrupts, four interval bins, `rej` = phases the qualifier dropped, `coinc` = two channels dropping within 200 µs (common-cause indicator), `blip` = edge pairs inside the interrupt latency, `storm` = storm-breaker trips, `isrmax` = longest service time of one edge interrupt. **Acceptance: cnt ≈ poll.**
- `hsdi rec <mask hex|all> [n]` / `hsdi rec stat` / `hsdi rec get`: edge recorder (§10.2).

## 10. Pulse qualifier and edge recorder (2026-09-04)

### 10.1 Qualifier (`min_us`, one engine shared by COUNT_CAP and COUNT_SW)

- **Rule**: both edges raise an interrupt that timestamps the edge (DWT, 4.2 ns). A level that has held for at least `min_us` is a *phase*, and its level becomes the **qualified level**; the counter advances only when the qualified level changes, per `edge`. A phase shorter than `min_us` (a dip inside a pulse, a spike inside a gap) is discarded whole and counted in `stat.rej`.
- **Latency**: the 1 ms main loop qualifies a phase early once it has already lasted `min_us` (inside a critical section), so a count appears ~1 ms after the rising edge and a line parked at one level settles too. The poll only *confirms*, it never has to *catch* an edge — task load cannot cost a pulse.
- **Edge pairs inside the interrupt latency** (the pin reads the same level as the current phase) are sub-microsecond blips: the phase continues, `stat.blip` counts them.
- **Choosing the value**: `min_us` below half the shortest genuine half-period of the sensor, above three times the widest glitch. Hall flow meters with 15 ms pulses and ≤1 ms chatter phases: **3000** (derive from the maximum rate for fast meters, e.g. 1000 for 200 Hz). 0 = off (classic per-edge counting with level confirmation and `debounce_ms`). `filter` (ICF) is the first, hardware, stage — 15 ≈ 1.07 µs at the 240 MHz timer clock; use 0 while recording so the recorder sees the raw signal.
- **Design position**: the timer capture with its hardware filter is the hardware method, the qualifier is the configurable parameter — a new sensor is two numbers in the table, not a new counting scheme.

### 10.2 Edge recorder (the board as its own logic analyser, RPC 0x12)

- The CM4 logs every interrupt edge of the channels in `mask` into a 2048-entry buffer (16 KB): `{DWT timestamp u32, timer CCR u16, ch | level<<4, flags}`, flags bit0 = an edge was lost before this one (capture overrun), bit1 = EXTI source, bit2 = blip, bit3 = synthesised by the poll after a storm mute. Recording stops by itself when full.
- `hsdi rec all [n]` arms it → wait until `hsdi rec stat` shows `armed=0` → `hsdi rec get`: a CM7 task pulls 60 entries per RPC and streams ~1 KB text chunks to `up/hsdirec/<seq>` (first chunk starts with `HSDIREC n= mask= cyc_us= lim=`, last chunk ends with `END`), one edge per line `t_us,ch,level,ccr,flags`.
- PC side `tools/hsdi_rec.py arm|stat|get|analyze`: phase-length histograms, short phases (< threshold) with width and position inside the pulse (deciles), dips per pulse, cross-channel coincidence within 200 µs (common cause vs. per-sensor), the first short phases with context; `--plot` writes overview and zoom PNGs. Raw CSV and report land in `plan/hsdi_rec/`.
- Two edges of the same timer closer than 250 µs are refined with the CCR difference (240 MHz), which removes interrupt-latency jitter.

### 10.3 Interrupt storm breaker (2026-09-04, added after the field recording)

- **Trip**: a channel that raises ≥16 edge interrupts inside one millisecond (a legitimate ≤10 kHz signal never gets there) has its interrupt source switched off on the spot (CAP: its CCxIE bit; SW: its EXTI C2IMR1 bit); `stat.storm` increments.
- **Reset**: the next 1 ms poll clears the pending flag, re-enables the source and reads the pin once — if the level differs from the phase the qualifier is tracking, the missed transition is fed in with "now" as its timestamp (recorder flag bit3 = SYNTH). Real phases are milliseconds long, so a timestamp up to 1 ms late does not affect `min_us`.
- **Why it is needed**: the gutter-2 meter emits ~97 kHz oscillation bursts with edges 5.17 µs apart; one edge interrupt costs about 6 µs, so a burst held the CM4 back-to-back in the ISR for up to 8 ms (RPMsg and every bus thread stalled). The timer filter cannot reach it (ICF tops out near 4.3 µs even with CKD/4). With the breaker a channel costs at most 16 interrupts per ms ≈ 10 % CPU, whatever the line does.

### 10.4 Field findings (2026-09-04, greenhouse board, four 1200 p/L hall flow meters; data in the internal repo)

- The pulses themselves are clean: 15..19 ms high, 33..46 ms period, ≈50 % duty.
- **The dirt is oscillation bursts**: the gutter-2 line (HSDI5) intermittently carries a ~97 kHz oscillation (capture-register deltas of exactly 5.17 µs, low duty) in bursts of 1..8 ms, roughly every 36 ms while it lasts; inside a low phase they look like spikes, inside a high phase like the "dips" seen the day before. Only this one meter does it.
- **It couples into the others**: 110 of the 131 sub-microsecond blips on HSDI7 fall inside HSDI5 burst windows; the HSDI4 and HSDI6 blips share identical timestamps 33.4 ms apart = the HSDI5 pulse period. The other three channels are not dirty by themselves.
- **Settled parameters**: `filter 15` (ICF ≈ 1 µs, removes every coupled blip — 500 k of them in six minutes) + `min_us 1000` (drops the bursts) + the storm breaker (bounds the load). All five hall meters (the water meter on HSDI0 included) run `{COUNT_CAP, 15, RISING, 0, 0, 1000}`. Acceptance: `cnt == poll` on all four gutter channels (1520/1520, 1697/1698, 1687/1687, 1942/1943; 5032/5032 over six minutes) and rates matching the measuring jug.
- **`cnt` 1..3 % below `poll` after the second build (storm breaker + ICF15)**: a 21.8 s recording (664 pulses) was re-counted on the PC by both rules — qualifier 664, 1 kHz sampler 667, and every extra sampler count was a **0.5..0.9 ms spike or dip** (a 582 µs dip, a 925 µs spike; an impeller with a 33 ms period cannot produce those), the qualifier missed none. **Verdict: 1 kHz sampling over-counts this signal by 1..3 % (the previous "true" values were high by that much, below what a measuring jug resolves); the qualifier is the reference, and `cnt` slightly below `poll` is correct.** 925 µs is too close to 1000, so the gutter rows now use `min_us` **3000** (still allows 166 Hz = 8 L/min; the water meter on HSDI0 keeps 1000, it can reach 200 Hz). In the tree, ships with the next batch.
- **Open physical question**: why that one meter/line self-oscillates at 97 kHz (sensor sample, cable length?) — swap the cable or the meter, or add an input RC, to find out; the firmware no longer depends on the answer.
