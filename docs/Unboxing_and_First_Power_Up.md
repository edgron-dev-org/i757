# I757-M — Unboxing and First Power-Up

> Read this once, before the first power-up. It takes you from a boxed controller to "I can see it
> on the dashboard and on the USB console" in about 15 minutes, and explains the things a new
> board does on its own that can look like faults. Wiring details are in `i757_Controller_Manual.md`
> (§3); building your own firmware starts in `../CLAUDE.md` and `Software_Manual_Application_Development.md`.

## 1. What you need

| Item | Notes |
|---|---|
| 24 V DC supply, **at least 0.5 A** | Range 9–36 V DC. A bench supply with a 100 mA current limit will collapse the board's power domain (480 MHz MCU + Ethernet + relay transients) and look like a dead board. |
| Ethernet cable to a **router or switch with DHCP and Internet access** | The board's first job is to reach the cloud broker. A direct cable to a laptop works for the console test but gives no Internet (see §4.2). |
| USB-C cable to a PC | The service port is a virtual COM port. **It does not power the board**, the 24 V supply must be on. |
| Your dashboard login and your board's serial number | Both come from Edgron separately, never from this repository. The serial number has the form `i757-XXXX` and is the CN of the board's device certificate. |

Modules that were shipped **plugged into the controller** stay plugged in. Never plug or unplug an
expansion module with the 24 V on (Manual §3.1).

## 2. Step 1 — power only

Wire the 5.0 mm power block: **V+**, **GND**, **PE** (Manual §3.3). PE must be connected.

Within a few seconds the green **run LED** starts blinking at about 1 Hz. That is the firmware
heartbeat: the CM7 main loop is alive. Expansion modules light their own status LEDs.

Nothing else is expected at this point. No network, no beep.

## 3. Step 2 — USB console (do this before the network test)

The console tells you what the board is doing, so connect it first.

1. Plug a USB-C cable from the service port into a PC. Windows shows a **USB Serial Device (COMx)**;
   Linux `/dev/ttyACMx`. No driver install is needed on Windows 10/11.
2. Open it with any terminal (PuTTY, Tera Term, `screen`, VS Code Serial Monitor). The baud rate
   does not matter (USB CDC).
3. Type `stat` and press Enter:

```
uptime 42s  heap 30904  temp 41C
net: ip=0.0.0.0 link=DOWN cloud=---
ota: bank=1 state=idle evt="confirmed"
time: not synced yet
```

| Field | Meaning |
|---|---|
| `link` | Ethernet cable / PHY link. `DOWN` with a cable plugged in = cable or switch port. |
| `ip` | Address in use. `0.0.0.0` = still waiting for DHCP (first 8 s). |
| `cloud` | `ok` = connected to the broker over mTLS. `---` = not yet. |
| `bank` / `evt` | Which flash bank is running and the last OTA event (`confirmed` is the normal resting state). |

Other commands worth knowing on day one (full list: type `help`; diagnostics: Software Manual §5.9):

```
netcfg        show the DHCP-timeout fallback address (see §4.2) and the address actually in use
tasks         both cores: task list, stack headroom, heap, last reset cause
ota           bank / OTA state / trial flag
mbus          backplane scheduler status: which module addresses are online
flow off      silence the greenhouse flow alarm permanently (see §6)
reboot        software reset
```

## 4. Step 3 — network

### 4.1 Recommended: router or switch with DHCP and Internet

Plug the cable in. Within about a minute the console prints the address and the broker connect:

```
[CM7] IP READY: 192.168.1.57 (broker=...)
```

and `stat` shows `link=up`, a real `ip`, and `cloud=ok`. Time syncs from NTP a few seconds later.

Outbound traffic the board needs through your firewall: **TCP 18884** (MQTT over mutual TLS to the
broker), **UDP 123** (NTP), **UDP 53** (DNS). Nothing inbound.

### 4.2 Alternative: direct cable to a laptop

This is fine for a console-only test, but there is no Internet on that cable, so the cloud will not
come up and the dashboard will stay offline. What happens:

1. The board waits **8 s** for DHCP, then falls back to a **static address**.
2. The factory default is `192.168.137.2/24` (gateway `.1`), **but it can have been changed on the
   bench** with `netcfg`. Always read the real value from the console before you ping:
   ```
   > netcfg
   fallback 192.168.1.250 255.255.255.0 gw=192.168.1.1 dns=192.168.1.1 (net.cfg); current ip=192.168.1.250
   ```
   `(builtin)` instead of `(net.cfg)` means the compile-time default is in force.
3. Give the laptop an address in the same subnet (e.g. `192.168.1.10/24` for the example above)
   and ping the board's address. If you want the factory default back: `netcfg default`.
4. While on the fallback address the board retries DHCP every 60 s and switches over automatically
   when a lease arrives, so moving the cable to a router later needs no reboot.

Windows does not answer pings from an unknown subnet by default; if the *board* cannot ping *you*
that is the laptop firewall, not the board.

## 5. Step 4 — dashboard

1. Open **https://dash.edgron.com** and log in with the account Edgron gave you.
2. Pick your board in the **Board** selector (its serial number). The dot next to the title turns
   green and the heartbeat age resets every 5 s. That is "online".
3. Everything on the page is rendered from the board's own self-description, nothing is configured
   on the server: the **Firmware** card (version, bank, signer, last OTA event), the on-board
   inputs, and one card per expansion module the master discovered (`m1`, `m2`, … = backplane
   address). A module that is plugged in but missing from the page is not answering on the
   backplane (check its DIP address, Manual §6).
4. **History** shows the logged curves, **Firmware** is the upload page you will use once you sign
   your own builds (`../CLAUDE.md` §5).

One dashboard account can see, and control, every board it is permitted to. Treat the login like a
key to the outputs.

## 6. Things a fresh board does that are not faults

**Short beeps every 10 s, starting 2–3 minutes after power-up.** The controller ships with Edgron's
greenhouse *reference application* loaded (`CM7/App/app_user.c`, see `../CLAUDE.md` §3). Part of it
is a feed-flow monitor that expects four flow meters on the fast inputs HSDI4–7; with nothing
connected it reads 0 L/min on all four lines for 2 minutes and raises a "feed flow low on all
gutters" alarm: a FAULT entry in the event log, an alarm on the dashboard, and a 300 ms buzzer chirp
every 10 s. It is harmless. Silence it once and for all from the console (the setting is saved):

```
> flow off
```

`flow ack` only silences the buzzer until the alarm clears; `flow off` disables the monitor. From
the dashboard the same command can be sent once the board is online. When you build your own
application the monitor is simply not compiled in (§3 of `../CLAUDE.md`).

**Relay outputs may move.** The same reference application contains roof-vent and aeration logic
that drives relay coils **O9** and **O14–O16** on an EX-16DO module from the PH_EC temperature
readings. Do not wire loads to the EX-16DO until you have replaced the application. Dosing pumps
(O1–O4) boot in OFF and never move by themselves.

**"cloud=---" for the first minute** is normal. **A reboot about 10 minutes after an OTA** means
the new firmware failed its trial period and was rolled back (`evt` says why); see
`OTA_and_MQTT_User_Guide.md` §6.5.

## 7. Next steps

1. Register your own firmware-signing key on the board over the USB console (`fwkey2 set`,
   `OTA_and_MQTT_User_Guide.md` §6.2bis). Only then can the board accept firmware you compile.
2. Open the repository root with an AI coding assistant or read `../CLAUDE.md`: it is the fast
   path into the code (where to write, how to build, how to flash, what not to touch). For a
   debug probe, the SWD header is under the small left cover: **1 = GND, 2 = SWCLK, 3 = SWDIO**.
3. Write your application in `platform_757/CM7/App/app_user.c` (Software Manual §4).

## 8. Quick troubleshooting

| Symptom | Check |
|---|---|
| No run LED | Supply polarity and ≥0.5 A current capability; 9–36 V at the terminals. |
| No COM port appears | The USB port does not power the board: the 24 V must be on. Try another cable (some are charge-only). |
| `link=DOWN` with cable plugged in | Cable, switch port, PHY LEDs on the RJ45. |
| `ip` assigned but `cloud=---` for minutes | Firewall: outbound TCP 18884, UDP 123, UDP 53. `stat` shows a `mqtt:` autopsy line with the last failure reason. |
| Console says cloud=ok, dashboard says offline | Wrong board selected, or the account is not permitted for this serial number. |
| Beeps every 10 s | §6, `flow off`. |
| Module plugged in, no card on the dashboard | DIP address unique and ≠ another module's; `mbus` on the console prints `online=` with the addresses the master sees. |
| Board reboots ~10 min after an OTA | Trial rollback; `ota` / `evt` give the reason. |

---
Revision: v1.0 (2026-09-29) first issue.
