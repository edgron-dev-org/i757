# Industry_757 — OTA Upgrade and Brick-Proofing Principles

> This document covers the **mechanism and principles** (why bricking is impossible); for daily operations/commands/tool usage see `OTA_and_MQTT_User_Guide.md`.
> Implementation code = `CM7/App/app_ota.c` (all logic) + `app_init.c` (watchdog / boot hooks).

## 0. Architecture in One Sentence

**There is no separate bootloader.** Leveraging the STM32H7 hardware dual-bank swap, OTA is fully integrated into the application framework: new firmware is always written into "the half of flash that is not running," so a corrupt write does not affect the running system; the swap action is guarded by four gates; and after switching over there is a "trial period" as a safety net — the new firmware must prove its own health, otherwise it is automatically swapped back.

```
Flash 2MB = Bank A(1MB) + Bank B(1MB), hardware SWAP bit toggles the address mapping in one shot
Each bank holds a complete system:
  +0x00000  CM7 image(512K, identity tag@+0x400: magic 'I757' + version string)
  +0x80000  CM4 image(512K max, actually a few tens of K)
  +0xFFF40  confirm word(1 flash word, 32B)   ← trial-period marker
  +0xFFF60  footer(160B: size/CRC/version/signature)  ← credential for the swap gate
Running bank is always mapped to 0x08000000; spare bank is always mapped to 0x08100000 (erase/write target)
Upgrade = fill the spare bank → flip the SWAP bit → reset; Rollback = flip it again (an involution)
```

## 1. Why Dual-Bank Is Inherently Brick-Proof (Layer 0)

The fatal flaw of the traditional single-region upgrade: you must erase the old firmware before you can write the new one, and a power loss mid-write = both gone = brick. Dual-bank removes this flaw at the root:

- **What you write is always the spare bank** — power loss / dropped blocks / a bad write during the push leaves the running system untouched; just push again;
- **The H7 supports RWW** (read-while-write, across banks): while writing the spare bank the CPU keeps fetching instructions from the running bank, so the business never stops during the entire upgrade;
- **The swap is a pure hardware action** (a single option-byte bit) — there is no intermediate "half-transferred" state;
- **The swap is an involution**: if you don't like where you swapped to, swap once more and you're back — the old system stays intact the whole time in the other half.

The cost: half the flash only stores the "previous version." For a 2MB H7 with a single-core image of ~230KB, there is plenty to spare.

## 2. The Upgrade Pipeline and the Four Gates

```
 PC(ota_push.ps1)                    Board(app_ota.c)
 ────────────────                    ──────────────────────────────
 1 ota-begin 0 <size> <crc>    →     erase the entire spare bank (8 sectors, ~8s)
 2 push blocks: 8B header[OTA1+offset]+1KB   →     tcpip thread→ring buffer→defaultTask writes flash
 3 (board, automatic)                segment complete → read back flash, recompute CRC ──── Gate ① integrity
 4 CM4 segment repeats 2-3 (offset 0x80000)
 5 ota-sign <ECDSA signature hex>     →     verify SHA256(cm7‖cm4) ─────── Gate ② authenticity
                                     pass → write footer (160B, incl. signature)
 6 ota-apply                   →     re-read footer + recompute content CRC ─────── Gate ③ swap gate
                                     all correct → flip SWAP bit → reset
 7 (new firmware's first boot)       trial period: must prove its own health ───────── Gate ④ trial period
                                     meets criteria → program confirm word to commit; fails → auto swap back
```

### Gate ①: Transmission Integrity (CRC + Idempotent Re-push)

- After each segment (CM7/CM4) is fully received, the board **reads back flash** and recomputes the CRC32 to compare against the pushing side — it verifies "what actually landed on disk," not transient data passing through memory.
- Blocks are idempotent by offset, deduplicated via a bitmap (1KB granularity): a dropped block needs no resume-from-checkpoint, just re-push the whole round; **reprogramming the same flash word between two erases is forbidden** (the H7's ECC will record it as bad, and read-back yields a bus error — a lesson learned in testing; the dedup bitmap exists precisely for this).
- Thread discipline: the MQTT callback (tcpip thread) only pushes data into an SPSC ring buffer; **all flash operations run in defaultTask** — the CPU stall during flash programming would fight with the Ethernet RX path, so they must be decoupled.

### Gate ②: Anti-Counterfeiting (Firmware Signature)

- On the PC side, the signing private key (`keys/fwsign/`, never leaves the dev machine) produces an **ECDSA-P256** signature over `SHA256(cm7 image‖cm4 image)`; the board verifies it with an embedded public key.
- A failed verification = `sig-fail`, and **the footer is not written** — the downstream swap gate is then guaranteed to reject.
- A triple chain of trust: TLS mTLS (the channel is genuine) + signature (the firmware is genuine) + CRC (the content is intact). Even if an attacker steals the device certificate and connects to the broker, they cannot push in firmware that can be swapped.

### Gate ②bis: Customer Signing Key (added 2026-09-07)

A customer must be able to **push firmware they compiled themselves, to their own boards, through the Edgron cloud** — without ever holding Edgron's signing private key. So the board can register a **second verification public key**:

- **Two keys, either one admits**: `ota-sign` verification tries the embedded Edgron public key first, then the customer public key registered on the board (raw 65-byte P-256 point, littlefs `fwkey2.bin`, a slot shaped like identity2). The cloud only carries bytes; **trust is decided by the board**.
- **Registration requires physical presence**: only the USB console command `fwkey2 set <hex>` (or factory pre-provisioning). **There is no cloud route to it** — in code, `app_identity_cli` hangs off app_cli.c (USB) alone; MQTT → `ota_cmd` has no entry. Stealing a device certificate does not let anyone register a key.
- **The footer records the signer**: one byte of the footer's reserved area becomes `signer` ('E' = embedded Edgron key / 'C' = customer key; footers written before 2026-09-07 read 0xFF = `legacy`). After a swap the new firmware reads its own bank's footer at boot and reports in heartbeat field `fws` *who signed the firmware now running*; field `fwk2` reports the customer key fingerprint (first 4 bytes of SHA256, `-` = none registered). At verification time the board also emits `evt:"signed:edgron|customer"`.
- **Every other gate is unchanged**: CRC, footer and trial-period rollback treat customer-signed firmware exactly like ours — if a customer build crashes, it still auto-rolls-back within 10 minutes.
- **Revoke / replace**: `fwkey2 clear yes` (USB) immediately reverts to Edgron-key-only; a new `set` overwrites the old key.
- Honest boundary: the consequences of a lost or leaked customer private key are the customer's (whether Edgron keeps an escrow copy after hand-over is agreed per customer); the board keeps no key-rotation history.

### Gate ③: Swap Gate (Footer = Signature Credential)

- **The footer is written only after signature verification passes** ⇒ "footer exists + its self-CRC is valid" transitively proves "the signature was verified." An attacker without the private key cannot forge a signature, and can only get in via the single MQTT path (cannot write arbitrary flash), so they **cannot produce a legitimate footer**.
- On receiving apply/revert, the board **always** re-verifies on the spot: footer magic → footer self-CRC → sanity of both segment sizes → **recompute the content CRC of both segments**. Any failure = `SWAP REFUSED`, the board stays put, and the reason for refusal is reported in the heartbeat `evt` field (`refused:no-footer/footer-crc/size/content-crc`).
- **Never use "the CRC was already verified this session" as a fast path** — CRC-correct ≠ signature-correct (firmware with a tampered signature still has a correct CRC), which would bypass the anti-counterfeiting.

### Gate ④: Trial-Period Auto-Rollback

The first three gates guarantee "the swapped-in image is intact and genuine," but they cannot cover a **logic fault**: the new firmware crashes, spins in a dead loop, or breaks the network stack — the firmware is "alive" yet permanently unreachable, and the manual rollback button on the dash can't reach it. That is the target of the fourth gate; see §3.

## 3. Trial Period: The Marker Is the Core

### 3.1 Design Starting Point (a Question That Must Be Answered First)

The naive approach "roll back if no confirmation within N seconds of boot" has a fatal counterexample: **an ordinary power-loss reboot that happens to occur when the network is down would falsely kill a good firmware**. So the criterion cannot be "can't reach the network"; it must first be able to distinguish two kinds of boot:

| Boot type | Characteristic | Behavior |
|---|---|---|
| The first (few) boot(s) right after an OTA swap | Trial period | Must prove health, otherwise roll back |
| An ordinary power-up (power loss / reset / network loss all count) | Established veteran | **Enters no rollback logic at all** |

### 3.2 Marker = Confirm Word (Exploiting the Physical Property of Flash)

Below each bank's footer, one flash word (`0xFFF40`) is reserved as a **confirm word**:

- **Erasing the whole bank naturally resets it to all-FF** — nobody touches it when OTA writes in the new firmware;
- "footer legitimate + confirm word all-FF" = this bank just arrived via OTA → **trial period**;
- After the trial period criteria are met, it is **programmed once** (writing `OTACONF1`) → permanently committed, after which this bank is admitted directly on any power-up;
- Flash's "can only be programmed once after erase" property matches exactly the semantics of "committing is one-way and irreversible" — survives power loss, needs no erase, requires zero extra sectors.

The companion boot counter lives in SRAM4 (`0x38000100`, with a magic value to guard against random power-up values): retained across reset, lost on power loss — if lost it just recounts, which is harmless, because **whether it is the trial period is determined by flash alone**.

### 3.3 The Three Verdicts of the Trial Period

Within the trial period (and only within it) there are three paths:

1. **Repeated crashing → fast kill**. On each trial-period boot, `ota_boot_guard()` (which runs **before** the RTOS/network come up) increments the counter by 1; **after 3 tries it flips the SWAP bit and goes home directly**. How does a hang turn into "the next boot"? Via the **IWDG independent watchdog** (32s, fed every 500ms by the main loop): a dead loop, a HardFault stuck in while(1), a starved task — within 32 seconds it will be bitten into a reset. Three rounds ≈ 2 minutes, entirely hands-off.
2. **Cloud confirmation → commit**. MQTT connected (mTLS, identity genuine too) + at least 3 heartbeats actually sent this boot + CM4 ping-pong normal (both cores alive) → program the confirm word, `evt:"confirmed"`. Measured commit occurs ~35 seconds after the swap.
3. **Timeout → slow kill**. If the criteria are not met within 10 minutes of this boot (120s in test builds) → roll back, `evt:"rollback:no-cloud"`.

**Why we dare use "can't reach the cloud" as a criterion** (this is precisely the answer to the §3.1 question): the firmware was pushed **over the network** just minutes ago — the network was necessarily up the moment before the swap. If it goes down right after, the prime suspect is the new firmware itself (it broke LwIP/MQTT/a driver). And this is exactly the scenario that manual rollback can never reach — without self-rescue it becomes a "live brick." The cost of a false kill is capped: return to the old version that was still working minutes ago, and just push once more. **An ordinary power-up with no network never hits this logic** — the confirm word is already programmed, and the guard admits it based on flash before network init.

### 3.4 Two Corner Cases That Must Be Plugged (Derived by Reasoning, Both in the Code)

- **The trial period rejects `ota-begin`** (`evt:"refused:in-trial"`): the spare bank holds the only rollback escape route; erasing it now means a later rollback would jump into a half-erased bank — that would be a real brick. Wait for commit (≤10min), then push.
- **A swap during the trial period (auto-rollback / manual revert) does not go through Gate ③**: the rollback target = the old firmware that was running just moments ago, which may be a **footer-less** image flashed via SWD, and would be rejected by the gate. The trial-period amnesty is well-justified — "it was running normally minutes ago" is itself a harder credential than a footer. The normal manual revert gate is unchanged (so after an SWD flash you must push one round of OTA, otherwise revert is rejected — this is the old rule).

### 3.5 Why a Separate Bootloader Is Still Unnecessary

- "The image simply won't boot" is excluded before the swap by Gates ①②③ (only intact + genuine content is allowed to swap);
- The remaining "it boots but the logic is broken" is caught by the trial period (the guard runs at the very earliest stage of the app, ahead of any complex init) + the IWDG;
- The rollback action is merely flipping an option-byte bit, which the app can do itself — no third-party referee needed.
- Benefits: one less bootloader to maintain, one less jump link, and what the customer gets is "write your own business in the application framework" — OTA is a built-in platform capability.

## 4. Complete Failure-Mode Table

| Failure scenario | Where it's blocked | Result |
|---|---|---|
| Power loss / network loss / dropped block during push | Layer 0 (writing the spare bank) | Running system unaffected, just re-push |
| Corrupted data landed on disk | Gate ① (read-back CRC) | verify-fail, does not enter a swappable state |
| Forged/tampered firmware (even with a stolen device certificate) | Gate ② (signature) + ③ (footer credential) | sig-fail / SWAP REFUSED, reason reported |
| apply mistakenly issued when the spare bank holds an incomplete/stale image | Gate ③ | refused:no-footer etc., board doesn't move |
| New firmware crashes on boot / dead loop | Gate ④ fast kill (IWDG + counter) | Auto rollback in ~2 minutes, evt:"rollback:boot-loop" |
| New firmware alive but can't reach the cloud (network stack broken) | Gate ④ slow kill (10min timeout) | Auto rollback, evt:"rollback:no-cloud" |
| A good firmware happens to have no network on an ordinary power-up | Confirm word already programmed, does not enter trial period | **No rollback** (measured: 17min cold-start with no network, no issue) |
| A new firmware pushed again during the trial period (erasing the escape route) | Corner case ① | refused:in-trial, rejected |
| Rollback target is an SWD footer-less image | Corner case ② (trial-period amnesty) | Swaps back anyway |
| Want to see "why did it just roll back" after swapping back | evt stored in SRAM4, not lost across reset | The old firmware's heartbeat reports the reason on behalf of the fallen |

## 4bis. The verified SWAP-flip engine (rebuilt 2026-07-26)

The old implementation trusted HAL return codes, and HAL can lie: once an OPT unlock fails,
**OPTLOCK stays set until the next reset**, after which OPTSR_PRG writes are ignored, OPTSTART
is ignored and OPT_BUSY never rises — the whole chain returns HAL_OK with zero effect (field
symptom: the board reboots on the same bank while claiming evt=swapped). The new engine
(`ota_swap_ob_try`) has three layers:

1. **Verified attempts ×3**: clear OPTCHANGEERR → unlock → OBProgram → Launch → **read
   FLASH_OPTSR_CUR back and compare** (the only judge that cannot lie; CUR takes the new value
   when the option change completes, while the mapping engages at the next system reset —
   RM0399 bank swapping).
2. Still not flipped → **photograph the registers into evt** (`swapfail cr=%08x sr=%08x`; cr
   bit0 = OPTLOCK) → pend a flag in reset-persistent SRAM → reset → **one-shot retry at the
   very top of `ota_boot_guard`, pre-RTOS, where the OPT engine is still virgin**; success
   reports `swapped-earlyboot` and resets again to engage the mapping.
3. Even that failing boots the old bank with the evidence in the heartbeat evt — the firmware
   **never falsely reports "swapped"**.

The push tool's own "did the bank actually flip" check remains as a final belt; it is not
expected to fire again.

## 5. Residual Risks (Honest Boundaries)

- **Power loss at the instant of flipping the SWAP bit**: option bytes have a CUR/PRG dual-register mechanism, and a failed write retains the old value — the worst case is "the swap didn't happen," not a brick. The window is sub-millisecond; accepted.
- **The confirm-word programming writes to the currently running bank**: the instruction-fetch stall in the same bank is about a hundred µs, delaying the ETH interrupt by at most one frame (TCP auto-retransmits). A bank is written only once in its lifetime; accepted.
- **Flash erase/write endurance**: the H7 is nominally rated at ~10,000 cycles/sector. Each upgrade round erases 8 sectors of the spare bank once — at a normal cadence (weekly/monthly) you'll never exhaust it in a lifetime; but **automated soak testing must be restrained** (e.g. one round every 3 minutes = 480/day would burn through the budget in ~20 days).
- **The collateral discipline of a permanently-on IWDG**: any blocking in defaultTask must be <32s (the current longest = a whole-bank erase ~8s, a 4× margin). Before adding a new long-blocking operation, think of the watchdog first.
- **Cloud confirmation depends on "our broker being alive"**: upgrading during a VPS outage means the new firmware will roll back after 10 minutes because it cannot be confirmed — conservative but safe behavior (upgrades should be done while the cloud is healthy anyway).

## 6. Measured Verification Records

| # | Scenario | Method | Result |
|---|---|---|---|
| 1 | Normal upgrade + commit | Push the production version | swap→`trial`→~35s `confirmed`, each bank went through once |
| 2 | Crash rollback | TEST-HANG build (dead loop 10s after boot) | Serial attempts=1/2/3 across three IWDG resets, flipped back before the RTOS on the 4th boot, ~3min, `rollback:boot-loop` |
| 3 | Lost-contact rollback | TEST-NONET build (doesn't connect to cloud, timeout shortened to 120s) | Rolled back right on 120s, `rollback:no-cloud` |
| 4 | Anti-counterfeit rejection | ota_push -TamperSig (1 byte of the signature altered) | `sig-fail`→forced apply→`refused:no-footer` reported to dash |
| 5 | Can push again after commit | Two consecutive push rounds | begin not rejected, the second round proceeds as usual trial→confirmed |
| 6 | **No false kill on a cold start with no network** | Unplug the cable → power cycle → just wait | Banner `boot: normal attempts=0`, network down 17min with monotonic tick and zero resets (well past the 10min line), plug the cable back and the heartbeat returns as before |

Historical foundation: the dual-bank swap path had previously soak-tested **178 rounds, all PASS, zero failures**, with heap unchanged to the byte after 160+ erase/write reboot cycles.

## 7. Observability (the Whole Upgrade/Rollback Is Traceable)

- Heartbeat `ota` field = the state machine (`idle/erasing/recv/verify-ok/signed/trial/...`);
- Heartbeat `evt` field = the most recent event (`confirmed / rollback:* / refused:* / swapped`), stored in SRAM4 and not lost across reset — **after an auto-rollback the swapped-back old firmware reports the reason on behalf of the "fallen"**;
- The dash (https://dash.edgron.com) automatically displays both of the above (desc-driven, zero panel changes when firmware adds a point);
- The serial (115200) boot banner `[OTA] boot: bank=N TRIAL|normal attempts=N evt="..."` is the verdict record for each boot.

## 8. A Paragraph for Customers (Ready to Quote)

> Firmware upgrades use a dual-partition (A/B) design: new firmware is written into the spare partition, and only after passing CRC integrity checks and ECDSA signature verification is it allowed to switch over; after switching, the new firmware enters a "trial period" and must pass self-tests and reconnect to the cloud within a time limit to be committed, otherwise the device **automatically rolls back** to the previous version — the worst outcome of a failed upgrade is "staying on the current version," and there is no way to brick it. A manual remote rollback is also possible with a single command at any time. The entire mechanism does not rely on a separate bootloader, and application developers need only focus on their business code.
