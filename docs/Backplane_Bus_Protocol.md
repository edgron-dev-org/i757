# Industry_757 — Backplane Expansion Bus Protocol v0.34 (= Modbus RTU Application Specification)

> v0.34 (2026-09-20, first user = EX_8AI, module type 5): **AI module register details** — FC04 0x0100~0x0107 RAW front-end node voltage in mV, 0x0108~0x010F ENG engineering value (mA×100 / mV), 0x0110 AI_STATUS fault bitmap; holding registers 0x0210~0x0217 AI_TYPE range code per channel, 0x0218 AI_FILTER (both persisted by 0xA55A). MODULE_TYPE table adds 4 = EX_8RLY, 5 = EX_8AI. (v0.29–v0.33 — COILS mirror 0x0203, baud-rate table extension and power-up baud rate 0x0281, rescue/naturalization, persisted debounce — are documented in the module manuals; this English contract will be caught up separately.)
> v0.28 (2026-08-09): **module configuration/calibration area + flash persistence** — holding registers gain **0x0200~0x02FF** (⚠️ a separate address space from the FC04 input-register 0x0200~ counter block; no conflict): 0x0200 CMD (write 0xA55A = save to flash / 0x0F0F = factory defaults (RAM only); anything else = exception 03), 0x0201 STATUS (ro: bit0 = unsaved changes, bit1 = a valid flash copy exists, bit2 = last save ok), 0x0202 = framework version (1), **0x0210~ module-defined** (details in each module's firmware spec; PH_EC = probe types/calibration/auto-range). Writes act immediately without persisting; only 0xA55A touches flash — **storage = the last 8K sector of BOTH banks**, newest valid copy wins (sequence + CRC), corrupt copies fall back to defaults. Companion ruling: **OTA image limit 64K→56K** (FC41 size >56K = exception 03), erase 8→7 sectors — the config sector is never touched by OTA and survives bank swaps. Modules without config (e.g. current EX_16DO) answer exception 02 for the whole area (compatible).
> v0.27 (2026-07-25): **slave diagnostic block** — CAPS adds bit4=DIAG (set automatically by the slavecore platform); FC04 adds the 0x0300~ diagnostic block (task count / total CPU% / min stack headroom + one 8-register slot per task: name/state/priority/cpu%/stack-free). Slaves recompute every ~2 s (delta window); the host polls at a low rate (~20 s) into the heartbeat. Old firmware answers exception 02 on 0x0300 → host treats as "no diagnostics" (compatible).
> v0.26: §4 gains two implementation rulings — ① **safe-state recovery semantics**: leaving safe state only clears the flag, outputs stay off; recovery = the host's periodic rewrite (latency = that module's DO-point scan period); ② **event consumption belongs to the MASTER**: the scanner round-robins one online module every 2 s, FC03-reads 0x0010 and writes the same value back (W1C) when nonzero, independent of the attention-line level. Also: the slave safe-state timer is hardened against tick wrap (EX_16DO firmware v1.6).
> v0.25: **DI counting extension** (first user = EX_16DI, module type 3) — CAPS adds bit3=CNT; FC04 adds the 0x0200~ counter block (u32 per channel, high word first); holding registers add 0x0020~0x002F per-channel debounce + the 0x0030 CNT_CLR clear bitmap (W1C, self-clearing).
> v0.21: §1 slave serial port corrected USART2/PD2 → **USART3 (PC10/11)/DE=PC9** — both the EX_16DO production-board netlist and the CORE_503 standard core use these pins; the original text lagged behind the hardware (doc changed by decision).
> v0.2. **v0.1 custom frame (STX/ADDR/CMD/LEN/CRC) was abolished the same day** — the user decided "use Modbus wherever Modbus can be used": the backplane switches to **standard Modbus RTU**, sharing one core with the front-panel 6×485 Modbus master stack and the future Modbus TCP (overall-plan modules 14/29 merge; A4+A8 combine into one line). Rationale for abolition on record: all of the custom frame's non-standard requirements (safe state / attention line / time broadcast / slave OTA) can be placed within the Modbus framework, and it is not worth maintaining a second protocol stack for; moreover third-party tools (pymodbus/QModMaster) can debug the backplane directly.
> This document = the sole contract between the host (H757) and the slaves (H503 expansion modules); frame-core implementation = `software/modbus/` (pure C, host/slave/self-test from the same source).

## 1. Physical Layer (carries over existing hardware decisions; summary only, nothing new)

| Item | Value |
|---|---|
| Medium | RS-485 half-duplex multidrop, non-isolated, in-cabinet ≤~30cm electrically short [fixed] |
| Host | Board 1 UART8, DE=PD11; **slave H503 USART3 (PC10=TX/PC11=RX), DE=PC9 (GPIO, CORE_503 standard pins)** [fixed v0.21; the former USART2/PD2 was an old convention, overturned by the production board] |
| Serial parameters | **Default 1 Mbps, 8N1** [set v0.2; the private backplane does not use Modbus's default 8E1, since CRC already covers it; the whole line can be pushed to 5M] |
| Termination/bias/protection | Host end fixed 120Ω + fail-safe bias (unique on the whole line); SM712 TVS at each node [fixed] |
| Slave address | 4-bit DIP, **address = DIP + 1 → 1~16**; 0 = broadcast; 17~247 reserved [fixed] |
| Attention line EX_IO | Open-drain wired-OR, host PH4 (read) / PI15 (drive, kept high-impedance in v0.2), slave single open-drain pin [fixed] |

## 2. Link Layer = Standard Modbus RTU + Private Timing Profile

- **ADU = Address (1B) + PDU (function code + data) + CRC16 (2B, low byte first)**; ADU ≤256B. CRC-16/MODBUS (0xA001 reflected, initial value 0xFFFF), covering address through end of data. **Register values always go on the wire big-endian** (Modbus convention; for 32-bit quantities the high-word register comes first).
- **Framing relies on silent intervals** (RTU standard): no STX. The receiver uses the UART idle interrupt to get frame boundaries.
- **Private timing profile** [set v0.2, calibrated during integration]: the standard fixes t3.5 at 1.75ms for >19200bps — but on the private backplane both ends are ours, so we tighten it: **inter-frame silence t3.5 = 100µs** (≈2.6 character times at 1M, still >the 35µs of 3.5 character times in spirit); slave response start ≤1ms; host response timeout 5ms, 2 retries; 3 consecutive polling cycles with no response = offline (drop to ~1s probe interval; re-insertion auto-recruits).
- **Broadcast (address 0)**: write-class function codes only, slave executes without responding (standard Modbus semantics) — time synchronization goes this way.
- **Host single outstanding transaction** serial scheduling; retry safety is naturally guaranteed by Modbus write semantics (write = overwrite, idempotent).

## 3. Slave Data Model (unified across all expansion modules; each module class uses a subset)

### 3.1 Coils (FC01 read / FC05 single write / FC15 multiple write)
| Address | Meaning |
|---|---|
| 0~15 | DO channels 1~16 (all used by EX_16DO; other modules per CAPS) |

### 3.2 Discrete Inputs (FC02)
| 0~n | DI channels (EX_16DI uses 0~15; debounced level, conducting = 1) |

### 3.3 Input Registers (FC04, read-only)
| Address | Meaning |
|---|---|
| 0x0000 | MAP_VER data-model version (this document = 1) |
| 0x0001 | MODULE_TYPE (1 = EX_16DO, 2 = PH_EC, 3 = EX_16DI, 4 = EX_8RLY, 5 = EX_8AI, others to be assigned) |
| 0x0002 | FW_VER (slave firmware version) |
| 0x0003 | CAPS capability bits: bit0=DO bit1=DI bit2=AI **bit3=CNT (DI counters)** [v0.25] |
| 0x0004/0x0005 | UPTIME seconds (high word / low word) |
| 0x0006 | CRC error-frame count |
| 0x0007 | FLAGS: bit0=in safe state bit1=has unretrieved events bit2=never received time |
| 0x0008 | **HW_VER hardware version** [v0.24]: 0xMMmm (0x0100 = v1.0) |
| 0x0100~ | AI raw values ch1~ (PH_EC: see its manual). **EX_8AI [v0.34]**: 0x0100~0x0107 **AI_RAW ch1~8** = front-end node voltage in mV (u16, 0~2750; a physical quantity independent of the DIP switch, for troubleshooting) |
| 0x0108~0x010F | **AI_ENG ch1~8** [v0.34, EX_8AI]: engineering value converted per AI_TYPE — mA ranges = mA×100 (4-20 mA normal 400~2000; <360 = open loop; >2100 = over-range), V ranges = mV (0-10 V → 0~10000; 0-5 V → 0~5000). Engineering units stop at mA/mV; user units (pressure, level, …) are scaled on the host scan table / cloud side |
| 0x0110 | **AI_STATUS fault bitmap** [v0.34]: bit k−1 = channel k faulted (mA ranges: open loop / over-range; V ranges: over-range >10500 mV); debounced by FAULT_N; setting a bit also sets the EVENTS bit and pulls the attention line |
| 0x0200~ | **DI counters CNT ch1~** [v0.25]: u32 per channel = 2 registers (high word first, §2 big-endian convention); increments on the debounced rising edge, unsigned free-running wrap (the host consumes it as an unsigned delta); 0 at power-on; clearing goes through holding register 0x0030 |

### 3.4 Holding Registers (FC03 read / FC06 single write / FC16 multiple write)
| Address | Meaning |
|---|---|
| 0x0000~0x0002 | Time: unix seconds high word / low word / milliseconds — **the host broadcasts FC16 to write these three cells = line-wide time sync** (every 10s + whenever the RTC is set) |
| 0x0010 | EVENTS event bitmap, **write-1-to-clear (W1C)**; after clearing, the slave releases the attention line (a read with side effects is not in the Modbus spirit, hence W1C) |
| 0x0011 | **BAUD_SEL baud-rate selection** [v0.22]: 0=1M (default)/1=250K/2=500K/3=2M/4=3M/5=5M. FC06/FC16 write: a legal value is first **ACKed at the old rate**, then switches ≥20ms after the ACK is sent; an illegal value = exception 03. FC03 can read the current value. **Survival rule: at a non-default rate, 3 consecutive seconds with no legal frame (addressed to this node or broadcast) → the slave automatically falls back to 1M** (same timer and trigger as the safe state); a host that loses contact likewise falls back to default and re-recruits. ⚠️ Transceiver physical limit: BL3085 @3.3V is rated 250kbps in the datasheet — 1M and above = out-of-spec operation; the production rate is set by mbspeed bench measurement |
| 0x0020~0x002F | **DI debounce DEBOUNCE_MS ch1~16** [v0.25]: 0~255ms, 0 = off, >255 = exception 03; applies to both the level bits (FC02) and the counting edges (0x0200~); **volatile — the host downloads it after recruiting the slave** (configuration-restore authority belongs to the host, the same philosophy as "output-restore authority after safe state belongs to the host"); slave default 10 |
| 0x0030 | **CNT_CLR counter-clear bitmap** [v0.25]: bitN = 1 → clears the counter of DI channel N+1; **self-clears after execution, always reads 0** (one-shot W1C — lesson learned from the HSDI reset bit: "set-only without clearing = the counter is pinned at 0 forever", see HSDI_Configuration_and_Counting.md §7) |
| 0x0210~0x0217 | **AI_TYPE ch1~8** [v0.34, EX_8AI]: 0 = 0-10 V (default), 1 = 4-20 mA, 2 = 0-20 mA, 3 = 0-5 V; other = exception 03. **Must match the front-panel DIP switch** (the switch decides whether the 125 Ω shunt is in circuit; the register only selects the conversion — a mismatch gives obviously wrong numbers, no damage). Persisted by 0xA55A |
| 0x0218 | **AI_FILTER** [v0.34, EX_8AI]: first-order low-pass time constant ×100 ms, 0 = off, 1~100 (0.1~10 s), default 10 (1 s); applies to AI_ENG only, AI_RAW is unfiltered. Persisted by 0xA55A |
| 0x0200~0x02FF | **module configuration/calibration area** [v0.28]: 0x0200 CMD (0xA55A = save / 0x0F0F = factory), 0x0201 STATUS (ro), 0x0202 framework version; 0x0210~ module-defined (details = each module's firmware spec). FC03 block reads inside the area / FC06 single writes (FC16 does not enter this area); writes act immediately, persisting is explicit; storage = the last sector of both banks (untouched by OTA). **Blob layout evolution is append-only**: the platform accepts a SHORTER stored blob (loaded as a prefix over defaults, appended fields keep defaults) and rejects a longer one (downgrade) back to defaults. Modules without config = exception 02 for the whole area |

## 4. Behavioral Specification

- **Slave safe state** [C2 acceptance item]: **3s** without receiving "a legal frame addressed to itself or broadcast" → enter safe state (EX_16DO = all 16 channels off, FLAGS.bit0 set); any subsequent legal frame exits it immediately. Power-on default = safe state, waiting for the host to recruit it.
  - **Recovery semantics [ruled v0.26]**: leaving safe state **only clears the flag; outputs stay off**. Output recovery = the host's periodic rewrite (FC15/FC16 scan); recovery latency = the scan period of that module's DO point (50 ms in the reference configuration). Applications must expect "outputs briefly off after a safe-state episode" — restore authority belongs to the host, the same philosophy as configuration restore.
  - **Implementation discipline [v0.26, from the EX_16DO v1.6 field case]**: the safe-state timer comparison must be tick-wrap safe — if the `now` used in the comparison predates the moment `last_ok` was stamped, the unsigned difference wraps to a huge value = spurious safe entry (two independent crystals' beat frequency makes it fire periodically). The slavecore idiom: re-read the tick at the point of comparison + use signed interval comparison.
- **Host scan**: at power-on, read FC04 0x0000~0x0003 (with retries) from each of 1~16 to build the online table (type/version/capabilities); online-table changes (join/leave) are reported to the application layer and the cloud.
- **Attention line and event consumption [implementation ruled v0.26]**: slave has an event → sets the EVENTS bit + pulls EX_IO low and holds it; **the host scanner round-robins one online module every 2 s, FC03-reads 0x0010 and, when nonzero, writes the same value back (W1C)**; once all bits are clear the slave releases the attention line. Consumption does not depend on the attention-line level (PH4 monitoring = an optimization, not a prerequisite); application-side consumption of the bits (e.g. POWERUP → re-download debounce configuration) is a future hook — at this stage the sweep exists to keep the register and the line from rotting.
- **Exception responses**: standard Modbus (function code | 0x80 + exception code 01 = illegal function / 02 = illegal address / 03 = illegal value / 04 = slave device failure).
- **Slave OTA** [v0.23; image limit changed to 56K in v0.28]: user-defined function codes **0x41~0x44** (0x45~0x48 still reserved). Prerequisite = H503 128K dual bank (2×64K) + SWAP_BANK; the image must be **≤56K** (the last sector = config area, v0.28), linked hard to 0x08000000 (after swap the spare bank is mapped to 0); the .ld FLASH LENGTH is 56K to match.
  - **FC 0x41 BEGIN**: request `[41][size u32 BE][crc32 u32 BE]` (crc = zlib CRC32) → response `[41][00]` returned upon acceptance, **erase is done asynchronously in the slave main loop** (dual-bank RWW, no stop; only sectors 0~6, i.e. 7 sectors); size >56K or OTA busy = exception 03. The host then polls 0x44 to wait for ready.
  - **FC 0x42 DATA**: request `[42][seq u16 BE][payload ≤240B]` → response `[42][st][next_seq u16 BE]`. **Strict ordering**: seq must equal the expected value (out-of-order = st 1 + report expected seq; the host resends to continue); the slave writes the spare bank at offset seq×240 (240 = 15 H5 quadwords, naturally aligned) and **accumulates a rolling CRC**; a tail block shorter than 16B is padded with 0xFF.
  - **FC 0x43 COMMIT**: request `[43]` → the slave compares the rolling CRC: match = response `[43][00]`, then **≥20ms after sending, set the SWAP_BANK option bit + reset** (the new bank takes the stage); mismatch = `[43][02]` and abandon the upgrade (return to idle).
  - **FC 0x44 STATUS**: → `[44][state][err][next_seq u16 BE]`, state: 0=idle 1=erasing 2=ready (receiving data) 3=verified.
  - **Brick protection (slave side)**: ① throughout BEGIN/DATA/COMMIT the old firmware keeps running, so a bad transfer can't kill it; ② after the swap, **the new firmware's main entry first increments a TAMP BKP0R boot counter; ≥3 → switch SWAP_BANK back + reset (automatic rollback)**; the counter is **cleared when the first legal request addressed to this node is handled = promotion to permanent** (host polls at 100ms, healthy firmware promotes within ~1s). Residual risk: firmware so broken it can't even reach main (bare hard fault) = hangs dead awaiting SWD / power cycle; the production fix = IWDG option-byte hardware watchdog (burned at provisioning, on the backlog).
  - **Host = module firmware repository (stored by model, v0.24)**: cloud images arrive via MQTT and are stored in the main controller's littlefs as **`t<type>.fw`** (one file per board type, shared by all slaves of that type; CRC verified before storing); `mota-flash <addr>` **first reads the slave model via FC04**, then selects the file to stream in; the whole thing runs over the standard op=1 transaction (RPMsg passthrough; the custom FCs are transparent to the transport layer). 49KB@1M measured ≈3s (including retries).
  - **Implementation discipline**: ① the slave erase/write window must **disable ICACHE + disable interrupts** (speculative prefetch / exception stacking colliding with erase/write = IACCVIOL+STKERR lockup); ② **on the H5, BKSEL selects the physical bank and "SWAP_BANK setting is ignored" (RM0492 p169), the opposite of the H7 where "registers swap along with the mapping"** — the erase bank number must be converted by the swap bit, otherwise after a bank switch the second OTA erases itself (lockup); address-type operations (program 0x08010000) follow the mapping and need no conversion. ③ **Lost-response continuation**: a DATA resend that is rejected but reports expected = seq+1 means the original send was already received (write is idempotent), so the host advances without error; **the COMMIT response cannot be fully trusted (stale-frame misjudgment); the verdict is always based on "the slave restarts and returns + FC04 version read-back"**.
  - **Acceptance**: 0x0103→0x0104 fully over the pure bus (no SWD involved), mbx 9/9 after the bank switch; 3-boot survival + promotion-clear fully exercised in the field.

## 5. Implementation and Test Map

| Item | Location | Notes |
|---|---|---|
| Frame core (CRC/ADU/PDU frame build & parse) | `software/modbus/modbus_core.c/h` | **Pure C99, no HAL**; full coverage of host request + slave response + exceptions; reused in all three places: front panel / backplane / TCP (drop the ADU, swap in the MBAP) |
| Self-test | `software/modbus/modbus_selftest.c` | Runs on the board at boot (includes byte-by-byte comparison against the classic example frames from the Modbus spec, pre-rehearsed with an independent Python implementation) |
| UART driver + idle framing + host scheduler | `software/uartdrv/` | Multi-instance DMA + IDLE framing; the CM4 host scheduler drives it |
| Slave firmware skeleton (H503) | `software/modbus/` reused | Core + register table shared with the host |
