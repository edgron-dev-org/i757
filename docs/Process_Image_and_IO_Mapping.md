# Process Image and I/O Mapping v0.2

> The authoritative contract for **cyclic field I/O**: a background scanner keeps a process image
> in shared memory, the application reads/writes that image as plain memory, and an external
> master can reach the same image over Modbus. Peer documents: `Backplane_Bus_Protocol.md`,
> `Inter_Core_RPMsg_Protocol.md`, `Universal_Modbus_Port_Config.md`.
>
> Why it exists: the on-demand helpers (`app_mb_read_holding()` and friends) each block on one
> bus round-trip, which couples your control loop to bus timing. The process image is the
> standard industrial answer — a scanner polls every configured point in the background, and
> your code touches memory instead of the wire.
>
> **A generic Modbus scan table is the parent concept**: the internal backplane is simply a set
> of entries on its bus, and third-party devices on the front 485 ports are entries on theirs.
> Modification discipline: changing the scan-table layout, image layout, seqlock rules or the
> shared-region split requires updating this document **and** `Common/app_pimage.h` (the single
> code-side source of truth). Both cores include that header; neither hardcodes offsets.

## 1. Architecture — two cores, split by plane

| | CM7 = interface / management plane | CM4 = scan / data plane |
|---|---|---|
| Role | Cloud, dashboard, USB CLI, your application; **owns the configuration**; reads the input image, writes the output image | Owns every UART; **polls the scan table**; maintains the process image; real-time timing (t3.5, timeouts, retries); reports health |
| Data access | Reading an input = a RAM read; writing an output = a RAM write (nanoseconds, never blocks) | Writes the input image, reads the output image |
| Why this split | Has a D-cache, runs features, not hard real-time | No cache, holds real time — a natural scan engine |

**I/O data never travels over RPMsg.** It lives in shared SRAM4. RPMsg carries only a control
doorbell (op 0x0C: reload / start / stop) plus the aperiodic pass-through transactions used for
diagnostics and OTA. Health is polled from the status region — no asynchronous notifications.

## 2. Shared memory layout (SRAM4, non-cacheable, reset-persistent)

The whole block lives in **D3 SRAM4** at `0x38008440`, inside the MPU region that is already
mapped non-cacheable — which is exactly why it is placed there: CM7 has a D-cache, CM4 does
not, and a cached view of cross-core shared memory is a classic source of phantom data.

| Sub-region | Budget | Writer | Reader | Content |
|---|---|---|---|---|
| `ctrl` | 48 B | CM7 | CM4 | magic, `cfg_ver`, `cfg_applied`, base tick, entry count, run flag. [2026-07-19] Correction: 48 B, and there are **no region-size fields** — the single source of truth for the layout is `app_pimage.h` |
| `hsdi[8]` | 64 B (8×8 B) | CM7 | CM4 | onboard HSDI channel configuration (`HSDI_Configuration_and_Counting.md`). [2026-07-19] Registered late: this block was originally missing from this table, so computing offsets from the document alone shifted everything after it |
| `cfg[]` | 1 K (**64 entries × 16 B**) | CM7 | CM4 | the scan table (section 3). [2026-07-19] Correction: the earlier "128 × 32 B" did not match the code; `PIMG_MAX_ENTRIES = 64`, 16 B per entry |
| `in[]` | ≤8 K | CM4 | CM7 | input image |
| `out[]` | ≤8 K | CM7 (+CM4 for external writes) | CM4 | output image |
| `st[]` | ≤4 K | CM4 | CM7 | per-entry seqlock counters + health, per-bus counters |

On a cold power-on SRAM4 holds garbage, so `app_io_setup()` wipes the whole block when the magic
does not match; on a reconfiguration the status region is cleared **by CM4 only** — each bus
thread clears its own entries as it swaps in the new table (section 8) [2026-07-19].

**The OUTPUT image is cleared on EVERY reset** [ruled 2026-07-25, = the PLC model]:
`app_io_outputs_clear_boot()` runs pre-RTOS in `app_periph_init`, even when the magic is valid.
Outputs are a function of logic, never a memory — industry PLCs clear outputs on both cold and
warm restart and retain only *data*; the first scan of the program computes them back. This also
closes the SRAM-remanence trap: after a brief mains dip the magic can survive and retained
commands would otherwise silently re-energize the loads (the "unexpected restart" industrial
safety forbids). Restoring outputs is explicit and upper-layer only: the cloud/dashboard
re-commands, or the application keeps its process state in `PF_RETAIN` (battery-backed BKPSRAM,
see app_pwrfail.h) and replays it on first scan after checking `app_pf_retain_valid()`. The
host-reboot window is bounded by the slave's 3 s safe state (outputs reach zero on the module
side within 3 s regardless). The clear must be pre-RTOS: after a warm reset the CM4 scanner can
otherwise resume the retained table and re-energize relays from the stale image before the
application layer gets a say.

### 2.1 Region map inside each image — **addresses are a published contract**

The input and output images use the **same** region split. Platform I/O that does not exist yet
still gets its space reserved **now**, and customer scan points are allocated only above it, so
that an HMI wired to these addresses never has to be re-mapped when that I/O lands.

| Byte offset | Size | Region | Content |
|---|---|---|---|
| 0 | 32 | `PIMG_RGN_ONBOARD` | onboard I/O — 8 × HSDI today, room for 256 bits / 16 words |
| 32 | 64 | `PIMG_RGN_SLOTC` | sub-card slot C: DI/AI (input) or DO/AO (output) |
| 96 | 64 | `PIMG_RGN_SLOTD` | sub-card slot D |
| 160 | 96 | `PIMG_RGN_GP` | general purpose / future platform use |
| **256** | rest | **`PIMG_RGN_SCAN`** | **your scan-table points start here** |

Reserved regions read as 0 until their BSP lands — that is intentional placeholding, not a
defect. **Every point is aligned to a 2-byte boundary** so it lands on a clean Modbus register
boundary in the external window (register *n* = image bytes [2n, 2n+1]).

## 3. Scan-table entry

One entry = one unidirectional Modbus transaction, repeated at its own period.

| Field | Type | Meaning |
|---|---|---|
| `port` | u8 | 0..5 = front 485A..F, **6 = backplane** (same enumeration as `APP_PORT_*`) |
| `addr` | u8 | Modbus slave address (backplane modules use their DIP address 1..16) |
| `access` | u8 | direction + function code + granularity, see below |
| `start` | u16 | first coil / register address **inside that device** |
| `count` | u16 | quantity (bits for coil/discrete, words for registers) |
| `img_off` | u16 | byte offset into the input or output image (assigned by `app_io_setup`) |
| `period` | u16 | desired poll interval in base ticks; 0 = disabled |
| `seq_idx` | u16 | which seqlock counter guards this entry's slice |
| `flags` | u8 | bit0 = byte-swapped word device; bit1 = **hold last values while offline**. Default (bit1=0): the entry's input slice is **zeroed at the moment the entry crosses 3 consecutive failures** — stale sensor data must not look alive, and `ok[]` drops to 0 at the same instant [2026-07-19] |
| `retry` | u8 | extra attempts on **silence or mangled frames only** (a Modbus exception IS an answer — the device is online, no retry); 0 = bus default (backplane 2, front ports 1) [2026-07-19] |

| `access` | Direction | Function code | Granularity |
|---|---|---|---|
| `IN_COILS` / `IN_DISCRETE` | input | FC01 / FC02 | bits |
| `IN_INPUT_REG` / `IN_HOLDING` | input | FC04 / FC03 | words |
| `OUT_COILS` | output | FC15 | bits |
| `OUT_HOLDING` | output | FC16 | words |

To both command and read back a holding register, register **two** entries (one out, one in).

## 4. Image format

- **Bits** are packed, LSB = first coil (the Modbus wire convention). `count` bits occupy
  `(count+7)/8` bytes.
- **Words** are stored as **native little-endian u16**; CM4 converts from the big-endian wire.
- Application accessors: `app_io_di()` / `app_io_ai()` read, `app_io_do_set()` / `app_io_hr_set()`
  write. See `Software_Manual_Application_Development.md`.

## 5. Consistency — seqlock

A multi-word value (a 32-bit quantity split over two registers, a whole DI block, an AI array)
can be read while the writer is halfway through it, producing a value that never existed. Each
entry therefore carries a sequence counter:

- **Writer**: `seq++` (now odd = "writing"), barrier, write the data, barrier, `seq++` (even).
- **Reader**: read `seq`; if odd, retry. Otherwise copy the data, then re-read `seq` — if it is
  unchanged the snapshot is consistent, else retry.

The barriers matter: they pin the ordering between the counter flip and the data. Because the
image is in a non-cacheable region there is no cache reordering to fight, only the store buffer.

A **single aligned 16/32-bit value** is already atomic on this core, so single-word points can
skip the retry loop. The writer never blocks and never waits for readers — essential for the
real-time scan loop.

**Writer parity self-heal + bounded reader spin** (2026-07-26, after a reset-loop field case):
SRAM4 survives warm resets, so a reset landing inside a write leaves that seq odd forever —
every later write inverts parity and readers spin for good, starving the watchdog feed into a
permanent 32 s reset loop (the block is not re-wiped because its magic is valid, and a seq with
no scan-table owner is cleaned by nobody). Two rules, implemented once for both cores in
`app_pimage.h` (`pimg_seq_write`/`pimg_seq_read`):

1. A writer that finds the seq **odd on entry bumps it even first**. Writers are serialized by
   the HSEM, so an odd seq at entry can only be the residue of a writer killed mid-write —
   the lock heals itself on the first write after any such reset.
2. The reader's spin is **bounded** (`PIMG_SEQ_SPIN_MAX`, ~ms scale; a real write window is
   microseconds). On exhaustion it copies anyway and returns: one torn/stale telemetry sample
   beats a watchdog corpse, and a dead peer core is detected by the RPC heartbeat, not by
   starving the IWDG.

## 6. Scheduling

- **Base tick** defaults to 1 ms (`ctrl.base_tick_ms`). An entry's `period` is a multiple of it:
  1 for fast digital, 5 for analog, and so on.
- `period` is a **request, not a guarantee**. One Modbus transaction at 1 Mbps costs roughly
  0.5–1 ms; if the entries due on a bus cannot all fit, the **overrun** counter for that bus
  increments. Overrun is the signal that the bus is over-subscribed — `period` is also how you
  budget its bandwidth.
- Timeouts, retries and offline detection follow the bus profile
  (`Backplane_Bus_Protocol.md` section 2 for the backplane: 5 ms response timeout, 2 retries,
  3 consecutive silences = offline — the entry then drops to a **2 s** probe until it answers
  again, and recovery is automatic [2026-07-19: fixed per the implementation, previously stated
  as ~1 s]). The front-port scan timeout is fixed at **20 ms** (ample even for short frames at
  9600 baud).
- **Bus arbitration**: aperiodic transactions (diagnostics, OTA, configuration) share the wire
  with the scanner; CM4 injects them between scan transactions. Cyclic I/O belongs to the image,
  everything else to the on-demand path.

## 7. Status / health (CM4 writes, CM7 polls)

Per entry: seqlock counter, `ok` (last transaction succeeded), consecutive `fails`, last Modbus
exception code. Per bus: poll / failure / overrun counters, plus `cycle_us` = the measured
duration (µs) of the most recent service pass that ran at least one transaction. The online
bitmap is **backplane-only** (addresses 1..16 map one bit each); front-port slaves can use any
address 1..247, so their liveness is read from the per-entry `ok[]` instead [2026-07-19: fixed
per the implementation]. CM7 polls this to detect modules coming and going, an overloaded bus,
or a degrading link — no asynchronous channel is needed.

## 8. Configuration reload handshake

1. CM7 writes the whole table into the shared configuration region, then increments `cfg_ver`.
2. CM7 rings the doorbell (RPMsg op 0x0C, sub-command 0).
3. On CM4, **each bus thread individually** notices the `cfg_ver` change and swaps **its own
   private snapshot** at **its own safe point** (between two of that thread's transactions).
   The snapshot copy is guarded against tearing by reading `cfg_ver` before and after and
   retrying on a mismatch. Only when **all seven buses** have swapped is
   `ctrl.cfg_applied = cfg_ver` written back. [2026-07-19, v0.2: the first version had a single
   thread rebuild on behalf of everyone and raced the other buses' in-flight transactions — a
   torn entry could write the image out of bounds; abolished.]
4. CM7 polls until `cfg_applied == cfg_ver` — `app_io_setup()` performs this poll itself
   (≤200 ms) and returns **-4** on timeout (the table is stored but not live).

**Outputs are held across a live reload** — a runtime reconfiguration does **not** clear the output image (the
cold-boot wipe guarantees zeros on first power-on). Explicit corollary [2026-07-19]: if the new
table changes the point layout, the held bytes land according to the **new** layout — editing the
table means accepting the remap. The bus threads keep driving from the old image and then the new
one continuously; nothing twitches.
The status region (including the live seqlock counters) is cleared **only by CM4** — each bus
thread clears its own entries' `ok`/`fails` as it swaps tables. CM7 must never memset it: a seq
counter wiped mid-write has its parity inverted permanently and readers spin forever
[2026-07-19, real case].
CM4 never scans straight out of the shared table, so a rewrite in progress can never be observed.

## 9. Safe state

- **CM7 stops (crash, OTA restart) → outputs are held.** CM4 simply keeps flushing the output
  image; if CM7 died mid-write, the seqlock makes CM4 use the last complete value.
- A dead bus is still caught deeper down: backplane slaves drop to their own safe state after 3 s
  without a valid frame (`Backplane_Bus_Protocol.md` section 4).
- Backplane housekeeping that is **not** cyclic I/O — time broadcast, the attention line, slave
  OTA, roll-call — stays outside the scan table in its own protocol-specific layer.

## 10. External access — the process-image window (register 0x2000)

**The controller acts as a data concentrator / Modbus gateway.** Field points collected on one
port are readable and writable by an external HMI / SCADA / PLC through **any front 485 port set
to Modbus slave**, and through **Modbus TCP** — both are served from the same board register map
at the same addresses, with no extra firmware.

This is cheap because the process image is owned by CM4 and the board slave register map is
**also** served by CM4, so requests are answered straight out of the same RAM: zero copy, no
RPMsg involved.

### 10.1 Address mapping (window base `0x2000`)

Modbus defines four independent address spaces, so the input and output images can share one
numeric base without colliding:

| Modbus space | Maps to | External rights | Addressing |
|---|---|---|---|
| FC04 input registers | **input image** (words) | read-only | register `0x2000+n` = image bytes [2n, 2n+1] |
| FC02 discrete inputs | **input image** (bits) | read-only | bit `0x2000+b` = image bit *b* |
| FC03 / 06 / 16 holding registers | **output image** (words) | read + write | as above (read = command read-back) |
| FC01 / 05 / 15 coils | **output image** (bits) | read + write | as above |

A point's external address follows from its `img_off`: **word spaces** `0x2000 + img_off/2`,
**bit spaces** `0x2000 + img_off*8`. The CLI command **`pio dump` prints the external address of
every point** — hand that listing to whoever integrates the HMI. Out-of-range access returns the
standard exception 02 (illegal address). The older per-slot windows (coils below 512, holding
`0x0100`..`0x017F`) are unchanged and coexist.

### 10.2 Dual-writer rule (**mandatory**)

Once the window is open the output image has **three writer classes** [2026-07-19, roster
completed]: the CM7 application (`app_io_do_set` / `app_io_hr_set`), the CM4 user application
(the same-named API in `app_platform_cm4.h`), and CM4 acting for an external master (the 0x2000
window). Concurrent writers would corrupt both the data and the seqlock counter, so:

- **Every writer of the output image takes the hardware semaphore first**
  (`pimg_out_lock()` / `pimg_out_unlock()`, HSEM **2**; HSEM 0/1 belong to OpenAMP). With the
  writers mutually excluded, exactly one is ever active — which is all seqlock requires.
  **Each side additionally stacks a same-core FreeRTOS mutex layer** (CM4: `app_cm4_out_lock`;
  CM7: `s_io_mtx`): the HSEM one-step lock always succeeds for the same core, so it cannot
  protect a core's own threads from each other. All locking lives inside the API — applications
  **never lock manually** [2026-07-19].
- **In-core API readers take no lock**; they keep using the plain seqlock.
- **Window readers** (the external master) get zero-copy **aligned 16-bit atomic loads**: a
  single register can never tear; a 32-bit value spanning two registers is confirmed by the
  master **re-reading until two passes agree**, standard Modbus concentrator practice
  [2026-07-19: the window path does not use the seqlock — the earlier "readers keep using
  seqlock" wording did not match the implementation]. Every window entry point is gated on
  `ctrl.magic`: while the image is unbuilt or rebuilding, the response is exception 02 —
  cold-boot garbage configuration once let an external write compute an unbounded copy.
- The **input image is externally read-only** (FC04/02); no window path can write it. In-core it
  still has a single writer (the scanner) and needs no lock.
- An external write that lands inside a point's slice is committed **as the whole slice under
  that point's sequence counter**, so the scanner can never read a torn value. A write into a
  reserved region (no point owns it) is applied directly.
- On the CM7 side the FreeRTOS mutex serialises **task against task**; the HSEM covers
  **core against core**. Both layers are required.

### 10.3 Typical deployment

Field devices on 485A (master, scanned) → HMI on 485B (slave, given an address that cannot clash
with the field devices) or over Ethernet via Modbus TCP. The application and the external master
may own different channels of the same point: measured on hardware, the application writing
channel 0 every 20 ms and an external master writing channel 1 coexist without interference.

## 11. Application-visible interface

- **Cyclic I/O** → the process-image API (`app_io_*`, non-blocking).
- **Aperiodic access** (setup, diagnostics, OTA) → the blocking `app_mb_read/write_*` helpers.
- The scan table is a **compile-time table in a user file** (`app_user.c`); edit the rows for your
  devices. Runtime reconfiguration (persisted, CLI/cloud editable) is a later step.
