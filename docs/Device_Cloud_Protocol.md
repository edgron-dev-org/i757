# Device Cloud Protocol v0.1 (Draft)

> The shared contract between firmware and the cloud web dashboard. One protocol covers all board types / vertical applications: the board **self-describes** its capabilities, and the dashboard **renders generically**. Adding a board type only requires changing the description, not the code.
> This document is the "constitution"; changes must go through a version bump. All implementation disagreements are resolved in favor of this document.
> Related: transport/security is carried by `OTA_and_MQTT_User_Guide.md`.

---

## 0. Design Principles

1. **Self-description first**: The dashboard has no built-in knowledge of any board type; it renders everything based on the `desc` (capability manifest) reported by the device. Adding a board type = changing the desc in the firmware, with zero backend/frontend changes.
2. **Generic point model**: All I/O, states, and parameters are abstracted into "points". Six kinds cover the greatest common denominator of industrial monitoring and control (see §7).
3. **Closed-loop control**: Every downlink command / point write has an acknowledgment (ack). The UI shows "executed / rejected / timed out", never "fire and forget".
4. **Forward compatibility**: Every message carries a protocol version `pv`; unknown fields are ignored, unknown kinds are downgraded to raw display. An old dashboard connecting to new firmware doesn't crash; a new dashboard connecting to old firmware still works.
5. **Embedded-friendly**: JSON is compact and has a size budget; desc can be chunked; it doesn't assume the board has large memory / RTC (see §18).
6. **Secure by default**: mTLS device identity; dangerous commands require permission + secondary confirmation + audit; the device side also validates (defense in depth), not relying on the server alone.

---

## 1. Terminology

| Term | Meaning |
|---|---|
| device | One board (main controller or slave), with a unique `type`+`sn` |
| point | A readable/writable data item: DI/DO/AI/AO/state/parameter |
| command | An action: start/stop/reset/calibrate/OTA/reboot… |
| desc (description) | The capability manifest self-reported by the device: which points, which commands, what topology |
| module | A slave/expansion (e.g. EX_16DO) attached to the main controller, occupying a slot |
| rid | Request ID, links a downlink command with its acknowledgment |

---

## 2. Transport and Security

- **Protocol**: MQTT 3.1.1 over TLS (mTLS mutual authentication).
- **Port**: 8883/18884 (production); plaintext is limited to on-machine debugging, forbidden on the public internet.
- **Identity**: One X.509 certificate per device. **[decided 2026-07-24] `<sn>` = the certificate CN, verbatim** (parsed from the embedded certificate at startup), so CN = the topic `<sn>` segment = the MQTT client-id = the mosquitto username (`use_identity_as_username`) — all four identical. The broker ACL is live (2026-07-24): the ops certificate gets full access, plus one line — `pattern readwrite dev/+/%u/#` — so each board may only touch its own topics (cross-board subscribe/publish measured as denied, retained messages withheld too). **The certificate is the identity and travels with the device; the trust anchor travels with the deployment** (self-hosted VPS uses a self-built CA, AWS uses Amazon Root CA). Production provisioning + private keys never leaving the chip: see the production checklist.
- **QoS**: Telemetry QoS0 (high-frequency, droppable, made up on the next beat); commands/point-writes/acks QoS1 (at-least-once, deduplicated by rid); desc QoS1+retain.
- **retain**: `desc` and the latest `status` are recommended to be retained, so the dashboard gets the current state as soon as it subscribes (no need to wait for the next beat).
- **LWT (Last Will and Testament)**: On connection the device registers a will on `.../up/state` with content `{"online":false,...}`; while normally online it publishes `{"online":true}` itself. The broker publishes the will on behalf of the device when it goes offline → the dashboard senses offline within seconds.

---

## 3. Topic Namespace

```
dev/<type>/<sn>/up/desc      device→cloud  capability self-description   QoS1 retain
dev/<type>/<sn>/up/state     device→cloud  online/offline (incl. LWT)   QoS1 retain
dev/<type>/<sn>/up/status    device→cloud  telemetry: point snapshot/delta QoS0 (retain latest)
dev/<type>/<sn>/up/event     device→cloud  alarms/events                QoS1
dev/<type>/<sn>/up/ack       device→cloud  command/point-write ack       QoS1
dev/<type>/<sn>/dn/set       cloud→device  point-write/setpoint          QoS1
dev/<type>/<sn>/dn/cmd       cloud→device  action command                QoS1
```

- The backend subscribes to all device uplinks with a single **`dev/+/+/up/#`**; downlinks are sent to a specific `type/sn`.
- `<type>`: board-type code (e.g. `I757-M` main controller, `H503-16O` slave), see §19.
- `<sn>`: device serial number (unique in production, recommended to match the certificate CN).
- Migration: **completed (2026-07-24, clean cut, no dual-publish)** — firmware, dashboard, logger and tools all moved into this namespace; the old `i757/*` topics are retired. `<sn>` = the device certificate CN (parsed by the firmware at startup, doubling as the MQTT client-id).

---

## 4. Common Message Envelope

All messages are UTF-8 JSON objects, common fields:

| Field | Type | Description |
|---|---|---|
| `pv` | int | Protocol version (this doc = 1). Required |
| `ts` | int | Device-side timestamp (ms; if there is no synchronized clock, use power-on milliseconds `up`, see §14) |
| `sn` | str | Redundantly included for logging convenience (already in the topic) |
| `seq` | int | Uplink message sequence number (monotonically increasing); the dashboard uses it to detect packet loss/restart (reset to zero = restart) |
| `rid` | str | Command class only: request ID, echoed verbatim in the ack |

Unknown fields are always ignored (forward compatibility). Field defaults are given in each section.

---

## 5. Device Identity and Topology

- device = `type` + `sn`.
- **Topology**: The main controller can carry slaves / expansion modules (blade or external), expressed via `modules[]` in the desc; point ids use a module prefix to distinguish (see §7 id convention).
- A slave (H503) can connect directly to the cloud as an independent device (with its own desc), or be proxied by the main controller as one of its modules — **both are supported**, decided by the deployment:
  - Direct: the slave has its own `dev/<type>/<sn>/...`;
  - Proxy: the main controller lists `modules[]` in its own desc and relays the slave's points (point ids carry an `mN.` prefix).

## 5A. Gateway Mode: Third-Party Bus Device (Modbus / CAN) Integration ★

This controller is not just an "IO device", it is an **edge gateway**: RS485 acts as a Modbus master polling a bunch of third-party slaves, CAN connects to CANopen/J1939 nodes, and it **normalizes** the registers/IO of these heterogeneous buses **into "points"** to feed the cloud. **To the dashboard/protocol, a Modbus register is just an ordinary point** — the dashboard doesn't care whether it is a local pin or a slave three hops away; the difference is entirely on the firmware side (what data source the point binds to: local GPIO / Modbus register / CAN signal).

### 5A.1 Sub-devices (node)
A third-party slave = one node (parallel to our own H503 modules, uniformly expressed as a node). In the desc:
```json
"nodes":[
  {"id":"n5","bus":"rs485","addr":5,"proto":"modbus","template":"PM2100","name":"Incoming meter","online":true},
  {"id":"n6","bus":"rs485","addr":6,"proto":"modbus","template":"VFD-XX","name":"Pump 1 VFD"}
]
```
Points belong to a node: `{"id":"n5.p.total","node":"n5","name":"Total power","kind":"num","unit":"kW","rw":"ro"}`.
The dashboard groups by node into a **device tree**: controller → RS485 bus → [Incoming meter n5] [Pump VFD n6], each point can be viewed/controlled inside.

### 5A.2 Data Quality q (critical)
Modbus slaves may go offline / time out — **never display the last stale value as if it were live data**. Point values carry quality:
- In status, a point value may carry quality: `{"n5.p.total":{"v":12.3,"q":0}}`, `q`= 0 good / 1 stale (not refreshed after timeout) / 2 comm-fail (slave lost). A plain value `12.3` is treated as q=0.
- The node-level `online` + communication statistics (timeout rate / last success time) are reported separately.
- Dashboard: for stale/fail points, **grey out + annotate "data stale / lost"**, so as not to mislead the operator.

### 5A.3 Binding = Register ↔ Point (firmware side)
The controller must know "who to poll, which register to read, how to interpret it, which point to map to". Each binding =
`{node/addr, function code (coil/discrete/holding/input register), register address, data type (u16/i16/u32/float/bit), scaling (raw×gain+offset), byte order, poll period, point id, read/write}`.
**Writing a point = reverse-writing the corresponding register (write single/multiple coils, write holding register), also going through the `set`→board writes Modbus→`ack` closed loop.** Write failure (slave no response) returns code 8.

### 5A.4 Where Does the Mapping Configuration Come From (decides whether the product is usable)
| Approach | Description | Verdict |
|---|---|---|
| Compiled into firmware | Every slave/wiring requires a recompile | Not productizable |
| On-board config file (SD/flash) | Editable, survives power loss | Usable |
| **Dashboard-pushed template** ★ | The dashboard has a "device template library" (register table of a certain meter/VFD model); select a template + fill in Modbus address → push down → board polls → points automatically enter the desc | **Product-grade, recommended** |

Recommended: **template push**: onboarding any third-party Modbus device **requires no firmware touch**. Uses a dedicated config channel:
```
dev/<type>/<sn>/dn/cfg   cloud→device  push/update Modbus/CAN mapping table (nodes+bindings)  QoS1
dev/<type>/<sn>/up/cfg   device→cloud  read back current mapping table + apply result         QoS1 retain
```
After the push the device **recomputes the desc (dv changes) and re-publishes** → the dashboard automatically grows new points. The MVP may first use a static on-board mapping; the protocol reserves the `dn/cfg`/`up/cfg` channel and the template concept up front.

### 5A.5 CAN Is the Same
CANopen/J1939 nodes are also nodes (`bus:"can"`), with PDO/SDO/signals mapped to points — one model handles all. Heterogeneous buses are normalized inside the gateway, and the cloud only sees unified points — this is exactly the value of an "edge gateway".

---

## 6. Self-Description desc (core)

Sent once after the device connects (retained); re-sent when the content changes (firmware upgrade / module insertion-removal). The dashboard renders everything based on it.

```json
{
  "pv": 1, "ts": 172..., "sn": "0001",
  "dv": "a3f1",                          // desc version/hash, echoed in status; dashboard uses it to decide whether to re-fetch desc
  "type": "I757-M", "name": "Line 1 main controller",
  "fw": "A3P Jul 5 2026 06:48:34",
  "vendor": "edgron",
  "groups": [                            // UI grouping/paging (optional)
    {"id":"in","name":"Inputs"},{"id":"out","name":"Outputs"},{"id":"proc","name":"Process"},{"id":"sys","name":"System"}
  ],
  "modules": [                           // topology (optional)
    {"slot":1,"type":"H503-16O","name":"Relay expansion","io":16}
  ],
  "points": [ /* see §7 */ ],
  "commands": [ /* see §11 */ ]
}
```

---

## 6bis. Dynamic Backplane Module Groups (est. 2026-07-25)

The controller dynamically generates one desc group plus points for every **online backplane module**, so each module gets its own card on the dash instead of sharing the System group (scales with module count, stays distinguishable):

- **Group**: `id="m<addr>"` (addr = backplane address), `name="Slot <addr>: <type name> v<maj>.<min>"` (type name / firmware version from the FC04 ident; type registry = Board Type and Version Registry).
- **Points** (data source = backplane contract v0.27 diagnostic block at 0x0300~, polled by the controller every ~20 s): `m<addr>.cpu` (num, %) / `m<addr>.stkmin` (num, B) / `m<addr>.tsk` (str, task table "name:stateLetter cpu%/stackfreeB").
- **status fields**: `m<addr>cpu` / `m<addr>stk` / `m<addr>tsk`; old slave firmware (no diagnostic block) yields only `m<addr>tsk:"-"`.
- **Membership change republishes desc** (retained): after a module joins or leaves, the panel adds/removes the card on the next desc it receives — the self-description principle ("add a module without touching panel code") cashed in.
- **Type-specific process points (est. 2026-08-09)**: beyond the three diagnostic points, the controller appends process points per module type, **on a card of their own** (group `id="m<addr>p"`) — the same logic that splits Onboard IO from System and the Relay Module card from the Slot 2 diagnostics card; the `m<addr>` card carries health diagnostics only. **Card order groups by nature, not by slot (ruled 2026-08-09)**: the desc groups array order IS the dash card order — all process cards first (Onboard IO, then each slot's process card in address order), then all health cards (System, then each slot's cpu/stack card in address order), Network last. Principle: **engineering-unit conversion happens on the module itself** (every expansion board must also work as a plain stand-alone Modbus transmitter), the controller only relays and the desc carries `scale`/`unit`; process values are read fresh on **every** heartbeat (the diagnostic block stays on the ~20 s slow cadence). An invalid channel (module reports the 0x7FFF sentinel) **omits its field** for that beat — the panel shows a gap, not a fabricated value.
  - **PH_EC (type 2, fw ≥ v0.4; data source = the module's engineering-value region 0x0108~, contract = the PH_EC slave firmware spec §2)**: group `m<addr>p` (name = "Slot <addr>: PH_EC Measurements"), points `m<addr>.ph1/.ph2` (num, scale 0.01), `m<addr>.ec1/.ec2` (num, µS/cm, temperature-compensated to 25 °C), `m<addr>.t1~.t4` (num, °C, scale 0.1); status fields `m<addr>ph1` etc. Older module firmware without the region → the whole block is omitted (the process card still appears, values stay blank).

## 7. Point Model (point)

### 7.1 Point Attributes

| Field | Required | Description |
|---|---|---|
| `id` | ✓ | Point unique id, convention `<class>.<index>` (see 7.3) |
| `name` | ✓ | Display name (Chinese is fine) |
| `kind` | ✓ | See 7.2: `bool`/`num`/`enum`/`bits`/`str`/`num` |
| `rw` | ✓ | `ro` read-only / `rw` read-write |
| `unit` | | Engineering unit (℃/%/A/bar/rpm…), used by `num` |
| `min`/`max` | | Range / writable range, used by `num` (out-of-range writes are rejected) |
| `step` | | Writable step (for slider/input) |
| `enum` | | Used by `enum`: `[{"v":0,"label":"Auto"},{"v":1,"label":"Manual"}]` |
| `width` | | Used by `bits`: bit width (e.g. 16-channel DO) |
| `alarm` | | Alarm rule: `{"hi":80,"hihi":90,"lo":5,"sev":"alarm"}` |
| `deadband` | | Used by `num`: only report when the change exceeds this value (suppresses jitter flooding) |
| `group` | | Belonging group id (corresponds to desc.groups) |
| `node` | | Belonging sub-device id (corresponds to desc.nodes, third-party Modbus/CAN slave); local points omit it |
| `src` | | (optional, for diagnostics) data source hint, e.g. `"mb:5/hr/40001"` (Modbus slave 5 / holding register / address) |
| `widget` | | Rendering hint (see §20); if omitted, defaults per kind |
| `hist` | | `true` = this point needs history stored / trend plotted |
| `role` | | Minimum permission to write this point (see §15), default operator |

### 7.2 Six Kinds (covering the common denominator)

| kind | Semantics | Value type | Typical | Read-only widget | Writable widget |
|---|---|---|---|---|---|
| `bool` | On/off | 0/1 | DI, DO, enable, run/fault | Indicator light | Switch/momentary |
| `num` | Analog | number | Temperature/pressure/flow/setpoint/PID | Value/gauge/trend | Input box/slider |
| `enum` | Enumerated state | int | Mode, gear | Status badge | Mode selection |
| `bits` | Bit group | int bitmap | 16-channel DI/DO packed | Bit-array indicator | Bit-array switches |
| `str` | Text | string | Version, label, fault-code text | Text | Text box (rarely) |
| `evt`? | (alarms go via event, not a point) | | | | |

> `num` serves both as a read-only sensor and a writable setpoint (via `rw`). `bits` is an optional optimization to pack multi-channel IO and save bandwidth, expanded into `width` switches for rendering.

### 7.3 point id Convention

- System points: `sys.*` — `sys.bank`/`sys.ota`/`sys.broker`/`sys.heap`/`sys.uptime`/`sys.rssi`/`sys.clone`.
- I/O points: `di.N`/`do.N`/`ai.N`/`ao.N`/`enc.N` (encoder)/`cnt.N` (count).
- Process points: custom semantic names, e.g. `t.return` (return water temp), `p.pump1` (pump 1 opening), `pid.kp`.
- Module points: prefix `m<slot>.`, e.g. `m1.do.3` (DO3 of the slave in slot 1).

---

## 8. Telemetry Report status

Two types, used together:

- **Full snapshot**: low-frequency period (e.g. every 5–30s) sending all current point values, doubling as heartbeat. The dashboard aligns on reconnect/initial via this.
- **Delta**: sends the changed points immediately when a point changes (`num` constrained by `deadband`). Low latency, saves bandwidth.

```json
// Full (f=1) or delta (f=0)
{ "pv":1,"ts":..., "seq":123, "dv":"a3f1", "f":1,
  "p": { "di.0":1, "do.3":0, "t.return":72.4, "sys.bank":2, "sys.ota":"idle" } }
```

- `p`: dictionary of point id → current value. A delta carries only changed points.
- `dv`: current desc version; if it doesn't match the one the dashboard holds → the dashboard re-fetches the desc (handles "capabilities changed after firmware upgrade").
- `bits` point value is given as an integer bitmap; `enum` as int; `bool` as 0/1; `num` as a number.
- **Quality q**: for points from an external bus (Modbus/CAN), the value may carry quality `{"v":..,"q":0/1/2}` (see §5A.2); local points default to good and may omit q. The dashboard greys out / annotates stale/comm-fail.
- **Write-readback**: after any `set` takes effect, the point should reflect the **actual applied value** in the next status beat (may be clamped / unchanged if rejected), by which the dashboard confirms.

---

## 9. Event / Alarm event

```json
{ "pv":1,"ts":..., "seq":.., 
  "id":"al.t.return.hi", "src":"t.return",
  "sev":"alarm", "state":"active", "msg":"Return water temp too high 82.1℃", "val":82.1 }
```

| Field | Description |
|---|---|
| `id` | Alarm instance id (multiple triggers of the same alarm use the same id, for dedup/acknowledgment) |
| `src` | Triggering point id (may be empty; system events have no source) |
| `sev` | `info`/`warn`/`alarm`/`critical` |
| `state` | `active` (triggered) / `clear` (recovered) / `ack` (acknowledged) |
| `msg` | Human-readable description |

- **Alarm state machine**: active →(recover) clear; active →(dashboard ack) ack; supports "latch": stays active even after recovery until ack, decided by the point's `alarm.latch`.
- **Acknowledge**: the dashboard sends `cmd` `alarm.ack {id}` → the device sets that alarm to ack / clears the latch → replies event `state:ack`.

---
- **First implementation [2026-09-04]**: topic `dev/<type>/<sn>/up/event` (QoS0, shares the datalog uplink slot; the board retries on its 10 s tick until sent). Alarm ids `al.gutter1..4.flow` (src `io.g1..4`, val = current L/min) and `al.feed.supply` (all four lines low at once = pump/supply). `state` uses active/clear only; acknowledgement is the on-board command `flow ack` (silences the buzzer, does not change state). The same alarm also lands in the on-board event journal (FAULT) and in the heartbeat bitmap `ga` (point `io.galm`). **Dashboard side**: the event is pushed to open pages over SSE and relayed to a phone (ntfy or Telegram, environment `DASH_NTFY_TOPIC` / `DASH_TG_TOKEN` + `DASH_TG_CHAT`), de-duplicated per id+state for 60 s. Producer = `app_flowmon.c`.

## 10. Downlink Point-Write set

```json
// cloud→device dev/<type>/<sn>/dn/set
{ "pv":1, "rid":"r-8842", "sets": { "ao.0": 65, "mode": 1, "pid.kp": 2.5 } }
```

- Can write multiple points at once. The device validates each point (type/range/rw/permission/operating-condition interlock); **partial success is also acknowledged per point**.
- The device **must** return an ack (see §12) and reflect the applied value in status.

---

## 11. Downlink Command cmd

Declared in desc.commands:

```json
{ "id":"start", "name":"Start", "confirm":"soft", "role":"operator",
  "args":[ {"id":"line","kind":"enum","enum":[{"v":1,"label":"Line 1"},{"v":2,"label":"Line 2"}],"required":true} ] }
```

Invocation:
```json
// cloud→device dev/<type>/<sn>/dn/cmd
{ "pv":1, "rid":"r-8843", "cmd":"start", "args":{"line":1} }
```

| desc field | Description |
|---|---|
| `id`/`name` | Command identifier / display name |
| `args` | Argument list, each {id,kind,unit,min,max,enum,required,default} (kind same as point model) |
| `confirm` | `none`/`soft` (click to confirm)/`strong` (enter device name / secondary credential), used for dangerous commands |
| `role` | Minimum permission (see §15) |
| `danger` | true = UI red high-risk (OTA/reboot/forced output) |

Built-in standard commands (recommended to implement on all board types): `reboot`, `ota.begin/chunk/sign/apply/revert` (incorporates existing OTA), `broker.set`, `alarm.ack`, `ident` (make the board blink to identify itself), `selftest`.

---

## 12. Acknowledgment ack

```json
// device→cloud dev/<type>/<sn>/up/ack
{ "pv":1, "rid":"r-8843", "ok":true, "code":0, "msg":"started",
  "results":{ "ao.0":{"ok":true,"applied":65}, "mode":{"ok":false,"code":13,"msg":"Only manual mode can change"} } }
```

- `rid` links the request; `ok` overall success; `results` per-point/per-item results (used by set).
- **Result code (recommended table)**: 0 OK / 1 unknown command / 2 bad argument / 3 out of range / 4 read-only / 5 insufficient permission / 6 operating-condition interlock rejection / 7 busy / 8 hardware error / 9 verification failure (e.g. signature).
- Dashboard: after sending a command, wait for ack (timeout N seconds judged as "no ack") and show the result; never "green on send".

---

## 13. Online / Offline

- On connect: send `up/state` `{"online":true,"fw":...}` (retained) + `up/desc` + one full status frame (birth).
- On disconnect: broker triggers LWT → `up/state` `{"online":false}`. The dashboard greys out accordingly.
- `seq` reset to zero = device restarted, the dashboard prompts "device has restarted" accordingly.

---

## 14. Time

- Device has a synchronized clock (main controller broadcast / NTP) → `ts` uses real UTC ms.
- No clock (e.g. H503 slave) → `ts` uses power-on milliseconds, and sets `tsrel:true`; **the server uses the receive moment as the authoritative timestamp** for storage, the device time is only for ordering within the device.
- The main controller can time-sync the slave/itself via `cmd` `time.set`.

---

## 15. Security and Permissions

- **Authentication**: mTLS (device side); dashboard users log in via web (account/role).
- **Roles** (dashboard side, enforced by backend): `viewer` (read-only) / `operator` (point-write + routine commands) / `admin` (dangerous commands / config / OTA).
- **Command authorization**: every command/point declares a `role`; the backend permits based on the logged-in user's role; **the device side re-validates** (operating-condition interlock, dangerous commands require a confirm token) — defense in depth.
- **Dangerous commands**: `confirm:strong` + `danger:true`; UI secondary confirmation (enter device name), backend records the audit.
- **Audit**: all set/cmd record who/when/what/result (backend persists).
- **Rate limiting**: the backend throttles downlinks, preventing accidental touch / spamming.
- **Privilege control point**: some outputs are writable only in a specific mode (e.g. "manual mode"), and the device rejects with code 6 — safety cannot rely solely on the UI greying out.

---

## 16. OTA Incorporated into the Command Model

The existing OTA (ota-begin/chunk/sign/apply/revert + brick-proofing footnote + signature) is mapped to a command family of `danger:true, confirm:strong, role:admin`. Firmware data goes via `dn/cmd`'s `ota.chunk` (binary chunks, base64 or a dedicated binary topic). Progress/status is reported via the `sys.ota` point + event. **The dashboard's "firmware upgrade" button = a high-risk command flow, with a progress bar + ack + brick-proofing status display.**

**Customer signing key (2026-09-07)**: the board can register a second verification public key (USB console `fwkey2 set` only, no cloud command); it then accepts firmware signed by either the Edgron key or the customer key. The dashboard upload page only carries bytes plus the `ota-sign` signature hex the customer computed locally — the private key never touches the server. New heartbeat fields `fws` (signer of the running firmware) and `fwk2` (customer key fingerprint), desc points `sys.fws` / `sys.fwk2`. Mechanism: *OTA Update and Brick-Proofing*, gate ②bis.

---

## 17. Version and Forward Compatibility

- `pv` is required in every message; incompatible changes bump the `pv` major version.
- Dashboard encountering unknown `kind`/`widget`/field: downgrade (raw value display) without crashing.
- Device encountering unknown downlink field: ignore, may hint in the ack.
- desc uses a `dv` hash; firmware upgrade that changes capabilities → dv changes → the dashboard automatically re-fetches the desc.

---

## 18. Embedded Constraints (must be observed)

- **payload budget**: a single MQTT publish is limited by the LwIP mqtt output ring buffer (currently `MQTT_OUTPUT_RINGBUF_SIZE` is on the small side). desc may be large → two routes: ① enlarge the output buffer to fit the largest desc; ② **chunk the desc**: `up/desc` carries `{part:i,of:n}` fragments, the dashboard reassembles. The protocol reserves chunking fields.
- **status frequency**: full at low frequency (5–30s) + delta on demand; `num` uses `deadband` to suppress jitter; don't do a full refresh every 100ms (blows out bandwidth / broker).
- **point-count upper bound**: the per-device point count is constrained by RAM / desc size; use `bits` packing for very large I/O.
- **No RTC**: see §14, do not depend on the device wall clock.
- **Binary**: don't stuff OTA firmware chunks into JSON (base64 inflates 33%); use a dedicated binary topic or chunked binary (carries over the existing `i757/dn/fw` approach).

---

## 19. Naming / Encoding Convention

- **type code**: `<series>-<variant>`, e.g. `I757-M` (757 main controller), `H503-16O` (503 + 16-channel relay), `H503-DI` (digital input slave). Registered on record to avoid collisions.
- **sn**: unique in production, recommended = certificate CN = `<sn>` in the topic, all three consistent for traceability.
- **unit**: use common symbols (℃ ℉ % A V W kWh bar Pa L/min rpm Hz), the dashboard displays by symbol.
- **point id**: see §7.3; stable and unchanging (to rename, change `name` not `id`, otherwise history breaks).

---

## 20. Dashboard Rendering Convention (widget mapping)

| kind + rw | Default widget | Optional widget (overridden by desc.widget) |
|---|---|---|
| bool ro | `led` indicator | `badge` |
| bool rw | `switch` | `button` (momentary) |
| num ro | `value` | `gauge` / `trend` (hist=true) |
| num rw | `input` box | `slider` |
| enum ro | `badge` | |
| enum rw | `select` dropdown | `segmented` |
| bits | `bitgrid` bit array | |
| command | `button` (danger→red, confirm→with confirmation) | |

- Color / alarm state: a point changes color within its `alarm` thresholds (green/yellow/red), linked with events.
- Grouping: cards/pages by desc.groups; ungrouped go into "Other".

---

## 21. Extension Guide

- **Add a board type**: write that type's desc (points/commands/modules) in the firmware → it appears on the dashboard as soon as it connects, no backend/frontend changes needed.
- **Add point-type semantics**: prefer reusing the 6 kinds + widget hints; only when truly insufficient, bump `pv` to add a new kind, and provide the dashboard downgrade rendering.
- **Add a widget style**: backend/frontend add a widget implementation + reference it via `widget` in the desc; old devices are unaffected.

---

## 22. Full Example (one I757-M)

**desc** (excerpt):
```json
{ "pv":1,"dv":"a3f1","type":"I757-M","sn":"0001","name":"Line 1 main controller",
  "groups":[{"id":"in","name":"Inputs"},{"id":"out","name":"Outputs"},{"id":"proc","name":"Process"},{"id":"sys","name":"System"}],
  "modules":[{"slot":1,"type":"H503-16O","name":"Relay expansion","io":16}],
  "points":[
    {"id":"di.0","name":"E-stop","kind":"bool","rw":"ro","group":"in","alarm":{"eq":0,"sev":"critical"}},
    {"id":"t.return","name":"Return water temp","kind":"num","rw":"ro","unit":"℃","group":"proc","hist":true,"alarm":{"hi":80,"hihi":90,"sev":"alarm"}},
    {"id":"ao.0","name":"Pump opening","kind":"num","rw":"rw","unit":"%","min":0,"max":100,"step":1,"widget":"slider","group":"proc","role":"operator"},
    {"id":"mode","name":"Mode","kind":"enum","rw":"rw","enum":[{"v":0,"label":"Auto"},{"v":1,"label":"Manual"},{"v":2,"label":"Stop"}],"group":"proc"},
    {"id":"m1.do","name":"16-channel relay","kind":"bits","width":16,"rw":"rw","group":"out","role":"operator"},
    {"id":"sys.bank","name":"Running bank","kind":"num","rw":"ro","group":"sys"},
    {"id":"sys.ota","name":"OTA status","kind":"str","rw":"ro","group":"sys"},
    {"id":"sys.uptime","name":"Online duration","kind":"num","rw":"ro","unit":"s","group":"sys"}
  ],
  "commands":[
    {"id":"start","name":"Start","confirm":"soft","role":"operator","args":[]},
    {"id":"stop","name":"Stop","confirm":"soft","role":"operator"},
    {"id":"reboot","name":"Reboot","confirm":"strong","danger":true,"role":"admin"},
    {"id":"ota","name":"Firmware upgrade","confirm":"strong","danger":true,"role":"admin"},
    {"id":"broker.set","name":"Switch Broker","role":"admin","args":[{"id":"b","kind":"enum","enum":[{"v":0,"label":"VPS"},{"v":2,"label":"AWS"}],"required":true}]}
  ] }
```

**status (delta)**: `{"pv":1,"seq":51,"dv":"a3f1","f":0,"p":{"t.return":72.4,"ao.0":65}}`
**set**: `{"pv":1,"rid":"r-1","sets":{"ao.0":65}}` → **ack**: `{"pv":1,"rid":"r-1","ok":true,"results":{"ao.0":{"ok":true,"applied":65}}}`
**event**: `{"pv":1,"id":"al.t.return.hi","src":"t.return","sev":"alarm","state":"active","msg":"Return water temp too high 82.1℃"}`

---

## 23. MVP Minimal Subset (implement these first, leave interfaces for the rest)

First get "see the existing + control the existing" working, without waiting for full I/O:

1. **Topics**: migrate `dev/I757-M/0001/...`; bring up `up/state` (LWT) + `up/desc` + `up/status` + `dn/cmd` + `up/ack`.
2. **desc**: treat **the existing** as points and commands — points: `sys.bank`/`sys.ota`/`sys.broker`/`sys.heap`/`sys.uptime`/`sys.clone`; commands: `reboot`/`broker.set`/`ota`.
3. **status**: change the existing heartbeat fields into the `p{}` format (full, no delta/deadband yet).
4. **ack**: add acknowledgment to `reboot`/`broker.set`.
5. **Dashboard**: device list (state/LWT) + device page (generic rendering of 5 points + 3 commands) + SSE real-time + login with three roles + dangerous-command confirmation.
6. Add later: delta reporting, real DI/DO/AI/AO points, event/alarm, trend history, slave proxy.

Leave the interfaces ready (pv/dv/rid/chunking/deadband/role/confirm fields go into the messages now); the MVP doesn't implement them but doesn't change the protocol.
