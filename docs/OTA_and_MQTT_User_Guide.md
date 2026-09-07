# Industry_757 — OTA and MQTT User Manual

> Applicable firmware: A3P series.
> This document is an operations-and-principles manual; for the build / pitfall checklist see `software/README.md`, and the single source of truth for endpoint/credentials configuration is `CM7/App/app_cfg.h`.

---

## 1. The System in One Diagram

```
┌─────────────┐  publish heartbeat/OTA status  ┌──────────────────┐   subscribe    ┌─────────────┐
│ Controller  │ ──────────────────────────────▶ │   MQTT broker    │ ─────────────▶ │  Ops shell   │
│ board       │ ◀────────────────────────────── │ (public VPS relay)│ ◀───────────── │ (anywhere)   │
│(customer LAN)│  subscribe commands/fw blocks  └──────────────────┘    publish     └─────────────┘
└─────────────┘
```

- MQTT is a **publish/subscribe** model: the board and the ops end never connect directly; each actively dials out one long-lived TCP connection to the broker, and the broker forwards by "topic."
- The device **actively dials outward**, so the customer LAN needs no public IP and no inbound firewall port opened — this is why MQTT became the IoT standard.
- Whichever broker the board is on is where commands must be sent (it can't hear the other side).

## 2. Broker and Identity

| | Public (production form) | LAN (development/debug) |
|---|---|---|
| Address:port | `<YOUR_BROKER_HOST>:18884` (your cloud broker, **mTLS**) | `192.168.137.1:1883` (dev PC, plaintext) |
| Authentication | **Mutual certificates**: the board holds a device certificate (CN=`i757-0001`) and verifies the server certificate | Anonymous (local only) |
| Server side | mosquitto TLS listener + `require_certificate` + `use_identity_as_username` | mosquitto Windows service |

**Certificate system (self-hosted CA, all ECC P-256, validity 9999 years — "never brick" comes first, security is ensured by rotatability)**:
- The CA private key `keys/ca/ca.key` = the signing authority (core secret, backed up with the F: drive);
- The board's three-piece set (CA cert + device cert + device private key) is injected at build time from `keys/ca/` (`app_tls.cmake` generates `app_certs.c`, and **the private key never enters the source tree/git**);
- The trust anchor "follows the deployment": if the customer moves to their own cloud/AWS/Azure, only the CA in `app_certs` is swapped (can be delivered via OTA), and the device identity stays fixed for life.
- Plaintext port 18883 has been retired; the username/password mechanism is decommissioned. TLS encryption + CRC integrity + certificate authenticity provide triple protection.

**Dual-identity model and the console command family** (2026-07-26): the ATECC608 holds two identities — **slot 0 = the factory identity** (generated on-chip at production and slot-locked, vendor-CA-signed; anti-clone + vendor-side broker) and **`CFG_SE_CUSTOMER_SLOT` (default slot 2) = your identity** (generated and certified by you, for your own cloud; full walkthrough = `Connect_Your_Own_AWS_IoT.md`). USB console:

| Command | Purpose |
|---|---|
| `id` / `id2` | Show the factory / customer identity (source, CN, length) |
| `id-pub` / `id2-pub` | Read that slot's P-256 public key (128 hex chars) |
| `id2-gen yes` | **Generate a new key pair inside the customer slot** (destroys the old key, hence the confirm word; slot 0 refuses regeneration at chip level) |
| `id-cert <hexDER>` / `id2-cert <hexDER>` | Store a certificate (the board verifies the cert's public key matches this board's slot key — a board mix-up cannot slip through) |
| `id-cert-show` / `id2-cert-show` | Export the certificate as PEM (for cloud registration / backup; public data, copy-paste to a file) |
| `id-clear yes` / `id2-clear yes` | Remove the identity file (takes effect on reboot) |

TLS identity selection: on AWS connections the board automatically presents the customer identity when provisioned and signs the handshake with the customer slot; vendor-side broker connections always use the factory identity. Identity files live in littlefs (`identity.der`/`identity2.der`) and are always recoverable: certificates are public documents, re-writable any time with `id-cert`/`id2-cert`; the private keys live in silicon and cannot be lost with the file system.

The tooling has mTLS built in: `ota_push.ps1 -Public` automatically uses `keys/ca/device-0001.*`; on the command line use `mosquitto_pub -p 18884 --cafile ca.crt --cert device-0001.crt --key device-0001.key`.

**Cross-deployment verification (already run end-to-end on real hardware)**: the same board, the same device certificate — send `broker-aws` to switch it to **AWS IoT Core (`<YOUR_AWS_IOT_ENDPOINT>:8883`)** — uplink heartbeat, downlink commands, both directions fully working, and AWS logs it as Success. The only things that change are the trust anchor (`keys/aws/AmazonRootCA1.pem` = Amazon Root CA 1) + endpoint + SNI + DNS; **the device identity is unchanged to the byte**, proving that "device identity follows the device, the trust anchor follows the deployment." AWS-side provisioning (registering the certificate/policy/Thing) was done with the AWS CLI. The same applies if the customer moves to their own cloud/Azure. ⚠️**Key pitfall**: for mbedTLS to send the SNI extension you must enable `MBEDTLS_SSL_SERVER_NAME_INDICATION` — calling `mbedtls_ssl_set_hostname` alone is not enough (that only sets the certificate-validation host name); without SNI the AWS front end cannot route, and it silently rejects after the TLS handshake completes (only located via packet capture).

## 3. The Board's Network Behavior (No Manual Intervention Needed)

1. On power-up it does DHCP first (8 seconds); if it gets nothing it falls back to the static `192.168.137.2/24` (gateway 137.1, see `app_cfg.h`).
2. **Automatic broker side-switching**: by default it connects to the public one first; after 2 consecutive failures it automatically switches to the LAN one, and vice versa; a single connection attempt stuck for 20 seconds (no SYN response) is force-aborted and counted as one failure. → It lands on whichever side is alive; if a broker goes down the device finds its own way out.
3. Half-open connection self-heal: connected but unable to send a heartbeat for 15 seconds → force reconnect (publish watchdog).
4. Which side it's on now is shown by the heartbeat `bkr` field; the serial (115200) prints `IP READY: <ip> first-broker=<address:port>` at boot.

## 4. MQTT Topic Overview (per-device namespace, since 2026-07-24)

Topics follow the Device Cloud Protocol §3 per-device tree: `dev/<type>/<sn>/...`. `<type>` = board type code (757 main controller = `I757-M`); **`<sn>` = the CN of the device certificate** (parsed from the embedded certificate at startup; it also serves as the MQTT client-id, so multiple boards on the same broker never kick each other off).

| Topic | Direction | Content |
|---|---|---|
| `dev/I757-M/<sn>/up/state` | board→out | Online/offline `{"online":true/false}` (retain; offline is published by the broker via the LWT will — second-level presence) |
| `dev/I757-M/<sn>/up/desc` | board→out | Device self-description (retain, published once per connection) |
| `dev/I757-M/<sn>/up/status` | board→out | Heartbeat JSON, one every 5 seconds once connected |
| `dev/I757-M/<sn>/dn/cmd` | out→board | Text commands (see §5) |
| `dev/I757-M/<sn>/dn/fw` | out→board | OTA firmware binary blocks (see §6.3) |

The board subscribes only once to the `dev/I757-M/<sn>/dn/#` wildcard (a second concurrent subscribe fails silently — a measured pitfall of the LwIP client).
The old global `i757/*` topics were retired on 2026-07-24 (clean cut; the tools' `-LegacyNs` switch exists only for the final push that migrates a board still running pre-migration firmware).

### Heartbeat JSON Fields

```json
{"ver":"A3P-05:17:13","bkr":"pub","bank":1,"ota":"idle","recv":0,"tick":80,"rxirq":398,"heap":20400,"pub_ok":6}
```

| Field | Meaning | How to use |
|---|---|---|
| `ver` | Firmware version (prefix + compile date + time, from the image's 0x400 identity tag) | Compare before/after upgrade to confirm the swap took |
| `bkr` | Current broker: `pub`=public `lan`=LAN | Tell which side the board is on |
| `bank` | Currently running bank (1/2) | Direct evidence that the swap succeeded |
| `ota` | OTA state machine (see §6.4); `trial`=just swapped bank, awaiting cloud confirmation (§6.5) | Watch this during the upgrade flow |
| `evt` | Most recent OTA event: `confirmed` / `rollback:boot-loop` / `rollback:no-cloud` / `rollback:manual` / `refused:*` / `swapped` / `signed:edgron` / `signed:customer` (§6.5; stored in SRAM4, not lost across reset, reported by the old firmware after a rollback) | The reason a swap was refused / an auto-rollback occurred; dash `OTA Last Event` shares this source |
| `fws` | **Who signed the firmware now running** (read from its own bank footer): `edgron` / `customer` (customer key registered on the board, §6.2bis) / `legacy` (footer written before 2026-09-07) / `-` (no footer = flashed over SWD) | Evidence that your self-signed firmware is live; dash `Firmware Signer` shares this source |
| `fwk2` | Fingerprint of the customer signing public key registered on the board (first 4 bytes of SHA256, 8 hex); `-` = none | Compare with the `fwkey2` console echo to confirm which key is registered |
| `recv` | Bytes received so far in this OTA | Transfer progress |
| `tick` | Boot tick (1 tick=0.5s) | Tell whether it just rebooted or has run a long time |
| `rxirq` | Cumulative Ethernet RX interrupts | Network liveness |
| `heap` | FreeRTOS free heap | Memory-leak early warning (stable ≈20400) |
| `pub_ok` | Cumulative successful publishes | Stalled = half-open connection (the watchdog will self-heal) |

## 5. Command Reference (`dev/I757-M/<sn>/dn/cmd`, plain text)

### Everyday Safe Commands (Send at Will)

| Command | Effect | Time to take effect |
|---|---|---|
| `broker-pub` | Force-switch to the public broker (our VPS) and reconnect | ~10s |
| `broker-lan` | Force-switch to the LAN broker and reconnect | ~10s |
| `broker-aws` | Force-switch to AWS IoT Core (Sydney) and reconnect | ~10-20s |
| `ota-apply` | Swap bank + reset (upgrade takes effect) | ~15s until heartbeat returns |
| `ota-revert` | Same as above — the swap is an involution, swap once more = rollback | ~15s until heartbeat returns |

`ota-apply`/`ota-revert` are two names for the same action: each of the two banks holds a complete system, and swapping switches the version. **This is the life-saving mechanism for a one-command global rollback.**

**Brick-proof + anti-counterfeit gate v3 — footer + firmware signature**:
- Each CM7 image carries an **identity tag** at fixed offset `0x400` (magic `I757` + version + compile date-time);
- **Firmware signature**: the PC uses the signing private key (`keys/fwsign/`) to produce an ECDSA-P256 signature over `SHA256(cm7‖cm4)`; the board embeds the public key and, on receiving `ota-sign <hex>`, **verifies the signature** (mbedTLS) — only a genuine article continues, a forged/tampered one → `sig-fail`;
- After verification passes, the firmware writes a **footer** into the last 160B of the spare bank: magic `OTF2` + both segment sizes/CRCs + version + **signature** + the footer's own CRC — **not lost on power loss**;
- **Key point**: the footer **is written only after signature verification passes**, so "footer exists + CRC valid" = "the signature was verified" (an attacker without the private key cannot forge a signature and cannot write flash directly → cannot produce a legitimate footer);
- On receiving `ota-apply`/`ota-revert`: **always** read the footer → admit only if the magic + self-CRC + both segments' content CRC all match; no footer / bad / mismatched → `SWAP REFUSED`, the board stays put.
- **Never use "CRC already verified" as an admit shortcut** — CRC-correct ≠ signature-correct (firmware with a tampered signature still has a correct CRC), which would bypass the anti-counterfeiting.
- Triple defense: TLS (the channel is genuine) + signature (the firmware is genuine, guards against malicious firmware) + footer CRC (integrity, guards against bricking). Even if an attacker steals the device certificate and connects to the broker, they cannot push in firmware that can be swapped.

### Flow Commands (Sent by ota_push.ps1, Do Not Send by Hand)

| Command | Effect | Risk |
|---|---|---|
| `ota-begin <offset hex> <byte count dec> <crc32 hex>` | Prepare to receive a segment; offset 0 triggers a **whole-bank erase** (~8s) | ⚠️ Erasing the spare bank = erasing the rollback insurance |
| `ota-provision` | One-time programming of the BOOT_CM4_ADD0 option byte | Already done, no need to send again |

### Command-Sending Templates (PowerShell)

```powershell
# Over the public network (mTLS port 18884, using the device certificate; <sn> = your board's certificate CN)
& 'mosquitto_pub.exe' -h <YOUR_BROKER_HOST> -p 18884 --cafile ca.crt --cert device.crt --key device.key -t 'dev/I757-M/<sn>/dn/cmd' -m 'ota-revert'
# Over the LAN (when the board is on the lan side)
& 'mosquitto_pub.exe' -h 127.0.0.1 -t 'dev/I757-M/<sn>/dn/cmd' -m 'broker-pub'
# Watch the heartbeat (drop -C 2 to keep it scrolling; subscribe dev/+/+/up/status to watch every board)
& 'mosquitto_sub.exe' -h <YOUR_BROKER_HOST> -p 18884 --cafile ca.crt --cert device.crt --key device.key -t 'dev/I757-M/<sn>/up/status' -C 2
```

## 6. OTA Mechanism

> This section is a quick overview from the operations perspective; the **full treatment of the mechanism and brick-proofing principles** (the four gates / the trial-period design starting point / the full failure-mode panorama / residual risks) has its own document: `OTA_Update_and_Brick_Proofing.md`.

### 6.1 Dual-Bank Layout

```
Flash 2MB = Bank1(1MB) + Bank2(1MB), hardware SWAP bit toggles the address mapping in one shot
Each bank is a complete system: [CM7 image @ +0x00000, 512K] + [CM4 image @ +0x80000, 512K]
Running bank = the 0x08000000 window; spare bank = the 0x08100000 window (erase/write target)
Upgrade = write the new system into the spare bank → SWAP → reset; Rollback = SWAP once more
```

### 6.2 The Pipeline of One Full Upgrade (Executed Automatically by `ota_push.ps1`)

```
1. Receive heartbeat  Confirm online, note the current bank/ver
2. objcopy            ELF → pure binary
3. CRC32              Compute the verification fingerprint
4. ota-begin          Board erases the spare bank, heartbeat ota: erasing → ready
5. Push blocks        34×4KB binary blocks → dev/I757-M/<sn>/dn/fw, heartbeat ota: recv, recv field rises
6. Verify             Board reads back flash and computes CRC, heartbeat ota: verify-ok (or verify-fail)
7. CM4 image          Repeat 4-6 (offset 0x80000, no more erase)
8. ota-sign           PC signs cm7‖cm4, board verifies -> pass writes footer, heartbeat ota: signed (forged=sig-fail)
9. ota-apply          SWAP + reset (the gate checks the footer)
10. Receive new heartbeat  bank changed, tick reset to zero, ver is new = success
```

Usage: `.\ota_push.ps1 -Public` (public) / no arguments (LAN); `-RevertOnly` rolls back only, no push.

### 6.2bis Signing and Pushing With Your Own Key (customers, 2026-09-07)

The board can register a **second** verification public key (mechanism: *OTA Update and Brick-Proofing*, gate ②bis). From then on, firmware you compile yourself can be pushed once it is signed with **your own private key** — Edgron's signing key is never needed. The board still accepts the Edgron key as well, so Edgron releases keep working.

1. **Generate a key pair** (once; guard the private key — if it is lost, register a new one):
   ```
   openssl ecparam -name prime256v1 -genkey -noout -out my_fwsign.key
   openssl ec -in my_fwsign.key -pubout -out my_fwsign.pub
   openssl ec -in my_fwsign.key -pubout -outform DER | tail -c 65 | xxd -p | tr -d '\n'    # 130 hex chars = raw public point
   ```
2. **Register it on the board — over the USB console only; this is deliberately impossible from the cloud**:
   ```
   > fwkey2 set 04....(130 hex)
   fwkey2: OK fp=6D0F2C46 (effective immediately for ota-sign)
   > fwkey2                  # show;  fwkey2 clear yes = revoke
   ```
   The heartbeat field `fwk2` shows the same fingerprint from then on.
3. **Sign + push** (either way):
   - `ota_push.ps1 -Public -Sn <your sn> -SignKey my_fwsign.key -ClientCert cust-<yourcompany>`: signs with your key and connects with the broker operator certificate Edgron issued to you (its ACL admits only the serial numbers you own).
   - Sign only, push elsewhere: `fw_sign.ps1 -Cm7Bin cm7.ota.bin -Cm4Bin cm4.ota.bin -Key my_fwsign.key` (or `fw_sign.sh`) prints the hex that `ota-sign` takes — this is what the dashboard's firmware-upload page asks for (your private key never touches any server).
4. **Verify**: after the swap the heartbeat shows `fws:"customer"` and `evt:"swapped"`; the 10-minute trial commit / rollback applies as usual.

### 6.3 Block Protocol (For Understanding Only, Wrapped by the Script)

- Each block = an 8-byte header `['O','T','A','1', u32LE intra-segment offset]` + ≤4KB data.
- **Idempotent by offset**: a dropped block needs no resume-from-checkpoint, just re-push the whole round; the board deduplicates via a bitmap (reprogramming the same flash word between two erases is forbidden — the H7's ECC will record it as bad, a lesson learned in testing).
- On the board, the tcpip thread only enters the ring buffer, and defaultTask consumes and writes flash (network and flash operations are decoupled to prevent interrupts fighting each other).

### 6.4 OTA State Machine (Heartbeat `ota` Field)

`idle` → (`ota-begin`) → `erasing` → `ready` → (receiving blocks) → `recv` → `verify-ok` / `verify-fail` → (`ota-sign`) → `footer` → `signed` / `sig-fail` → (`ota-apply`) → `applying` → reset
Also: `provisioning` = programming the CM4 boot-address option byte. `sig-fail` = signature verification failed (firmware not signed by us); the footer is not written and the swap is guaranteed to be rejected. The first boot after a swap shows `ota` as `trial` (trial period, see §6.5), returning to `idle` after commit.

### 6.5 Auto-Rollback

**Problem**: if the new firmware crashes / breaks the network, the dash's manual revert button can never reach it. **Solution = a trial-period marker**, not relying on "roll back if no confirmation within N seconds of boot" (which would falsely kill a good firmware on an ordinary power-up that happens to have no network).

- **Confirm word**: one flash word (`0xFFF40`) below each bank's footer. A whole-bank erase naturally resets it to all-FF — "footer legitimate + confirm word FF" = this bank just arrived via OTA (**trial period**); already-programmed = established veteran, which **never enters rollback logic on an ordinary power-up / power loss / network loss**.
- **The three trial-period verdicts** (exist only during the trial period):
  1. **Repeated crashing**: increment the counter by 1 on each boot (SRAM4 `0x38000100`, not lost across reset); a hang is bitten into a reset by the IWDG (32s); **after 3 tries → flip SWAP_BANK and go home before the RTOS comes up**, heartbeat `evt:"rollback:boot-loop"`.
  2. **Cloud confirmation**: MQTT connected + ≥3 heartbeats sent + CM4 ping-pong normal → program the confirm word to **commit**, `evt:"confirmed"`.
  3. **Timeout**: not meeting the criteria within 10 minutes (120s in test builds) → roll back, `evt:"rollback:no-cloud"`. Why we dare use "can't reach it" as a criterion: the firmware was pushed over the network just minutes ago (the network was necessarily up before the swap), and the prime suspect for it going down right after the swap = the new firmware itself; the cost of a false kill = returning to the old version that was still working minutes ago, and just pushing once more.
- **A swap during the trial period (auto / manual revert) does not go through the footer gate**: the rollback target = the old firmware that was running just moments ago, which may be a footer-less image flashed via SWD (the gate would reject it); the normal manual revert gate is unchanged.
- **The trial period rejects `ota-begin`** (`evt:"refused:in-trial"`): the spare bank holds the only escape route, and erasing it then rolling back = jumping into a half-erased bank = a real brick. Wait for commit (≤10min), then push.
- Still **no separate bootloader needed**: image integrity is guaranteed by the footer gate before the swap, and "can't reach main" is excluded; the remaining logic crash is caught by the IWDG + boot counter.
- Test builds: `cmake -B build -DOTA_TEST=HANG|NONET` (the version string is forced to carry a `TEST-*` marker to guard against impersonation; to restore use `-U OTA_TEST`, as the cmake cache remembers it).

## 6bis. USB Field Upgrade (no-network sites, est. 2026-07-25)

When a site has neither internet nor LAN, the USB-C service port doubles as the upgrade port (firmware side = `CM7/App/app_usb_fw.c`, PC side = `tools/usb_fw_push.ps1`). **No new update machinery is introduced**: it feeds the exact same entry points as the cloud path (`ota_cmd` text commands + the dn/fw chunk sinks), so CRC precheck, signature verification, bank swap, 3-strike anti-brick and trial rollback all apply unchanged.

```powershell
# Upgrade a backplane slave module such as the H503 boards (~half a minute): repo -> backplane stream
.\usb_fw_push.ps1 -Port COM8 [-Type 1] [-Addr 2] [-Elf <h503.elf>]
# Upgrade the 757 itself (dual-core + signing, ~2-5 min; the COM port re-enumerates at apply, the script reconnects)
.\usb_fw_push.ps1 -Port COM8 -Self
```

- **Protocol**: console text commands pass through verbatim (the whole `ota-*`/`mota-*`/`broker-*` set works); `fwchunk <n>` followed by n raw bytes is the equivalent of one dn/fw message; the board answers `go`/`ok`.
- **Confirmation without cloud**: after a self-upgrade bank swap the firmware is in its trial period and rolls back in 10 minutes if the cloud is unreachable — once the technician verifies the site, issue **`ota-confirm yes`** (CM4 ping-pong health is still required): the on-site manual promotion. Module upgrades have no trial concept and need nothing.
- **Discipline**: never run a cloud OTA concurrently (the engines are single-session); on an interrupted transfer simply rerun the script (chunks are offset-idempotent, mota-begin resets); self-upgrades still require a signing private key on the operator machine (keys/fwsign, or the customer key registered on the board, §6.2bis — the anti-counterfeit chain is not relaxed just because the bytes travel over USB).

## 7. Quick Troubleshooting

| Symptom | Check first | Most likely |
|---|---|---|
| No heartbeat on the public network | sub from the LAN for a moment | The board landed on the lan side, send `broker-pub` |
| No heartbeat on either side | Serial `serial_watch.ps1` | Cable/power/firmware died, the serial has the truth |
| `verify-fail` | Re-run the push | Dropped block, an idempotent re-push round usually clears it |
| Heartbeat slow after swap | Wait 90s | Half-open connection, the watchdog self-heals + reconnects in 15s |
| Serial won't open | Who's holding COM6 | serial_watch and the automation tool are mutually exclusive, close one |
| mosquitto_pub reports not authorised | -u/-P parameters | On the public net you forgot the account / wrong password |
| Board doesn't react to apply/revert | Heartbeat `evt` field (refused:*) | The spare bank was touched but not verified, the brick-proof gate rejected it — swap after a complete push |
| Push rejected `refused:in-trial` | Whether heartbeat `ota` is `trial` | The last upgrade hasn't committed yet, wait ≤10min (`evt:"confirmed"`) then push |
| Board swaps itself back to the old version ~10min after upgrade | `evt`: `rollback:no-cloud`/`boot-loop` | The new firmware failed the trial period, auto-rollback for safety — investigate the new firmware's problem |
| Many dropped blocks / board freeze on public-net push | Inter-block interval | Public-net TCP bursts overwhelm RX (rooted out in C4), `-Public` defaults to 100ms, don't make it faster |

## 8. A Paragraph for Customers (Ready to Use)

> The device actively connects to the cloud broker via MQTT, penetrating the customer LAN with no network configuration required; firmware uses a dual-partition (A/B) design, an upgrade is written into the spare partition and switched over only after verification passes, a failure automatically retains the original version, and a single command can remotely roll back at any time; the upgrade channel carries account authentication (TLS encryption in the production version), and firmware integrity is guaranteed by CRC verification.
