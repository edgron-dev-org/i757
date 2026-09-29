# I757-M SDK — orientation for an AI coding assistant (and for humans in a hurry)

This file is read automatically by Claude Code when the repository root is opened. It is the fast
path into the SDK: what the hardware is, where code goes, how to build, how to get an image onto the
board, and what must not be touched. Everything here is a summary; the documents it points to are
authoritative when they disagree.

## 1. What you are working on

**I757-M** is a DIN-rail industrial controller built on an **STM32H757** (dual core: Cortex-M7 at
480 MHz + Cortex-M4). On board: Ethernet, six RS-485 ports, two CAN, eight fast digital inputs
(HSDI0–7), USB-C service console, 32 MB SPI flash (littlefs), ATECC608A secure element, buzzer.
Expansion modules (EX-16DO relays, EX-16DI inputs, PH_EC water-quality transmitter, …) plug onto an
internal **backplane bus** that is plain Modbus RTU at 1 Mbps; the master auto-discovers them.

Division of labour between the cores (fixed by the platform, do not fight it):

| CM7 | CM4 |
|---|---|
| Network, TLS, MQTT, OTA, filesystem, console, cloud self-description, **your application** (`CM7/App/app_user.c`) | Real-time I/O: scans every Modbus port and the backplane cyclically into a shared **process image**, HSDI counters/encoders, optional user logic (`CM4/App/app_user_cm4.c`) |

The two cores talk over RPMsg; you never touch that directly, the platform API hides it.

## 2. Read in this order

1. `docs/Unboxing_and_First_Power_Up.md` — what a fresh board does and how to see it on the console and dashboard.
2. `docs/Software_Manual_Application_Development.md` §4–§5 — programming model (your own FreeRTOS tasks), the platform C API, a complete worked example.
3. The contract for whatever interface you are about to touch (§7 of the Software Manual is the index): `Device_Cloud_Protocol.md`, `Process_Image_and_IO_Mapping.md`, `Universal_Modbus_Port_Config.md`, `HSDI_Configuration_and_Counting.md`, `Backplane_Bus_Protocol.md`, `Inter_Core_RPMsg_Protocol.md`, `CubeMX_Code_Boundary_Rules.md`.
4. `docs/OTA_and_MQTT_User_Guide.md` when you need to deploy.

## 3. Repository map — where to write, what never to edit

```
platform_757/
  CM7/App/            ← ALL your CM7 code. app_user.c is the entry point (app_user_init).
  CM7/App/examples/   ← manual examples, compiled but not run (entry names namespaced)
  CM7/Core/           ← CubeMX-generated. NEVER edit (regenerated). Exceptions are logged in
                        docs/CubeMX_Code_Boundary_Rules.md — if you must, log it there.
  CM7/Middlewares/    ← mbedTLS / LwIP / FreeRTOS / littlefs / FatFs, as imported
  CM4/App/            ← your CM4 code: app_user_cm4.c (app_user_cm4_init), app_platform_cm4.h
  CM4/Core/           ← CubeMX-generated. NEVER edit.
  Common/             ← shared between cores: app_user_points.h (point names), process-image layout
  CMakePresets.json, CM7/CMakeLists.txt, CM4/CMakeLists.txt
modbus/               ← transport-agnostic Modbus core (Apache-2.0), used by both cores and the modules
uartdrv/              ← multi-instance UART driver (Apache-2.0)
docs/                 ← manuals + interface contracts (authoritative)
tools/                ← ota_push.ps1 / mota_push.ps1 (cloud OTA), fw_sign.ps1|.sh (signing), vps_mqtt_setup.sh
```

Files in `CM7/App/` that are **platform** (keep, do not rewrite): `app_mqtt.c` (cloud + main
service loop), `app_ota.c`, `app_cli.c` (USB console), `app_mbport.c` / `app_mbcfg.c` / `app_mbtcp.c`
(Modbus ports), `app_pimage_cm7.c` (process image), `app_hsdi.c`, `app_time.c`, `app_identity.c`,
`app_se_*.c` (608A), `app_lfs.c` / `app_qflash.c` (storage), `app_datalog.c`, `app_netcfg.c`,
`app_usb_fw.c`, `app_cdc_console.c`, `app_leds_beep.c`, `app_rpc.c`, `app_init.c`, `app_p0.c`,
`app_p1_stubs.c`, `app_p5_test.c`, `app_anticlone.c`. `app_platform.h` is the one header you include.

**As shipped, `app_user.c` runs Edgron's greenhouse reference application**, not a blank template:
a scan table for three RS-485 field devices on port 485A (9600 8N1, master) plus a relay module on the
backplane, port 485B as a Modbus slave (address 10) exposing the same points, a 20 ms `control_task`,
and four greenhouse features each in its own file: `app_vent.c` (roof vents on relay coils 13–15),
`app_dose.c` (dosing pumps on coils 0–3, boots OFF), `app_flowmon.c` (feed-flow alarm on HSDI4–7,
**beeps on a board with no flow meters**, `flow off` on the console silences it), `app_aer.c` (air pump
on coil 8). To make it yours:

1. Keep §1 configuration tables and §3 startup of `app_user.c` as the pattern; replace the rows with your devices.
2. Delete the four `app_vent_init()` … `app_aer_init()` calls in `app_user_init()`. Without `init` none
   of them creates a task, so they do nothing. **Keep the four files in the build for now**: the platform's
   console (`app_cli.c`), heartbeat (`app_mqtt.c`) and data log (`app_datalog.c`) still link against their
   command and status hooks (`app_*_cli`, `app_*_cmd`, `app_vent_stage`, …). Removing them from
   `target_sources` gives undefined references; a build switch that stubs them is on Edgron's list.
3. Write your logic as tasks (Software Manual §4.3). Stack sizes are in **words**; verify with `tasks` on the console.

## 4. Build

Toolchain: `arm-none-eabi-gcc` + CMake + Ninja (all three come with **STM32CubeCLT**; put its `bin`
directories on PATH). No `keys/` directory is needed, the build substitutes stub certificates.

```
cd platform_757/CM7 && cmake --preset Debug && cmake --build build/Debug    # -> build/Debug/platform_757_CM7.elf
cd platform_757/CM4 && cmake --preset Debug && cmake --build build/Debug    # -> build/Debug/platform_757_CM4.elf
```

Always build **both** cores and deploy them together: one OTA image = CM7 image ‖ CM4 image, and the
CM4 image lives at +0x80000 inside the bank. Cloud can be compiled out with `APP_ENABLE_CLOUD 0` in
`CM7/App/app_cfg.h` (standalone controller; no MQTT, no broker, still Modbus/I/O/storage/console).

## 5. Getting an image onto the board

The board only runs firmware whose signature verifies against the Edgron key **or a customer key
registered on the board**. Register yours once, over the USB console only (there is deliberately no
cloud route): `fwkey2 set <130-hex raw P-256 public point>` — key generation and the exact commands are
in `docs/OTA_and_MQTT_User_Guide.md` §6.2bis. The heartbeat field `fwk2` then shows the fingerprint.

| Path | When | How |
|---|---|---|
| **Dashboard upload page** (recommended) | Board online, you have a dashboard login | `tools/fw_sign.ps1 -Cm7Elf <CM7.elf> -Cm4Elf <CM4.elf> -Key my_fwsign.key` writes `*.ota.bin` next to the ELFs and prints the signature hex. On the dashboard open **Firmware**, pick the board, upload the two `.ota.bin`, paste the hex. |
| `tools/ota_push.ps1 -Public -Sn <sn> -SignKey my_fwsign.key -ClientCert cust-<company> -Cm7Elf … -Cm4Elf …` | Scripted pushes from your PC to the broker | Needs the broker client certificate bundle Edgron issues per customer. Pass the ELF paths explicitly; the script defaults point elsewhere. |
| **SWD** with a CMSIS-DAP / ST-LINK probe + OpenOCD | Bench, no network, bring-up of your own firmware | See below. |
| `tools/usb_fw_push.ps1 -Port COMx -Self -Cm7Elf … -Cm4Elf … -SignKey my_fwsign.key` | No network at all: the USB console doubles as the upgrade port (guide §6bis) | Same CRC / signature / bank-swap / trial machinery as the cloud path; the script finishes with `ota-confirm yes` because there is no cloud to confirm against. Without `-Self` it pushes an expansion-module image (`-Type -Addr -Elf`). |

**Trial period.** After a bank swap the new firmware has ~10 minutes to prove itself (cloud connected
and both cores healthy; only core health when `APP_ENABLE_CLOUD=0`) or the board rolls back to the old
bank by itself. `ota-confirm yes` on the console promotes it early; the heartbeat `evt` says what
happened (`confirmed`, `rollback:no-cloud`, `rollback:boot-loop`, `refused:*`). Break it freely, it
recovers. The independent watchdog is always on: **the platform's `defaultTask` must never block
more than 32 s**; your own tasks may block as long as they like.

**SWD notes.** The header is a 3-pin 2.54 mm row under the small left cover: **1 = GND, 2 = SWCLK,
3 = SWDIO** (no reset line, hence `reset_config none`). H757 needs variables fed to ST's script or it
dies with "can't read AP_NUM":

```
openocd -f interface/cmsis-dap.cfg  (or interface/stlink.cfg) \
  -c "transport select swd; adapter speed 1000; reset_config none" \
  -c "set AP_NUM 0; set CORE_RESET 0; set CONNECT_UNDER_RESET 0; set DUAL_BANK 1" \
  -f target/stm32h7x.cfg \
  -c "init; reset halt; program {platform_757_CM7.elf} verify; program {platform_757_CM4.elf} verify; reset run; shutdown"
```

- `program 0x08000000` writes the **physical** bank 1. If the board is currently running from bank 2
  (`stat` → `bank=2`), that is the standby bank and your flash "does nothing": run `revert yes` first, or
  use OTA. The safe bench habit is to flash the same image into both banks (standby window `0x08100000`
  / `0x08180000` for CM4) so a swap changes nothing.
- An SWD-flashed image carries no signature footer: `fws:"-"` in the heartbeat, no trial period. The next
  real OTA rebuilds the footer chain.
- To halt for more than ~4 s under a debugger set `set STOP_WATCHDOG 1` before sourcing the target
  config, otherwise the IWDG resets the board mid-session.

## 6. Platform API you will actually use (`CM7/App/app_platform.h`, mirrored by `CM4/App/app_platform_cm4.h`)

```c
/* Modbus ports: 485A..485F, tcp; role master/slave */
int  app_mbport_configure(uint8_t port, uint8_t role, uint32_t baud, char parity, uint8_t stop, uint8_t slave_addr);
int  app_mb_read_holding (uint8_t port, uint8_t addr, uint16_t reg, uint16_t n, uint16_t *out);   /* FC03, blocking */
int  app_mb_read_input   (uint8_t port, uint8_t addr, uint16_t reg, uint16_t n, uint16_t *out);   /* FC04 */
int  app_mb_write_single (uint8_t port, uint8_t addr, uint16_t reg, uint16_t v);                  /* FC06 */
int  app_mb_write_multi  (uint8_t port, uint8_t addr, uint16_t reg, uint16_t n, const uint16_t *v);/* FC16 */
int  app_mb_read_coils / app_mb_write_coil / app_mb_write_coils (...)                            /* FC01/05/15 */
/* process image: cyclic, non-blocking — THE way to do field I/O; port APP_PORT_BACKPLANE for modules */
int      app_io_setup (const app_io_point_t *points, uint16_t n, uint16_t base_tick_ms);
uint8_t  app_io_di    (uint16_t pt, uint16_t ch);   uint16_t app_io_ai(uint16_t pt, uint16_t ch);
void     app_io_do_set(uint16_t pt, uint16_t ch, uint8_t on);   void app_io_hr_set(uint16_t pt, uint16_t ch, uint16_t v);
int      app_io_ok    (uint16_t pt);                /* 1 = last scan ok */
/* onboard fast inputs */
int      app_hsdi_setup(const app_hsdi_ch_t cfg[8]);  uint8_t app_hsdi_level(uint8_t ch);  uint32_t app_hsdi_count(uint8_t ch);
/* misc */
uint32_t app_time_now(void);  int app_temp_read(void);  void app_beep_set(uint8_t on);
void     app_log_event(const char *type, const char *fmt, ...);        /* on-board event journal, mirrored to cloud */
/* power-fail retained variables: declare with PF_RETAIN (app_pwrfail.h is the contract) */
```

Point names (`PT_DI`, `PT_RELAY`, …) are the enum in `Common/app_user_points.h`, shared by both
cores so the same function body compiles on either. Cloud publishing of your own points goes through
the self-description in `app_mqtt.c` (`Device_Cloud_Protocol.md`); the dashboard renders whatever the
board describes, the server has no per-board configuration.

## 7. USB console (any terminal, any baud; `help` lists everything)

```
stat        ip / link / cloud / bank / OTA event / time        tasks     both cores: tasks, stack headroom, heap, faults
netcfg      DHCP-timeout fallback address (persisted)          mbus      backplane scheduler: which module addresses are online
mbcfg       show/set Modbus ports (mbcfg 485a slave 9600 8N1 addr=1)
mbr/mbw     read/write a backplane module register             mbpoll    one-shot Modbus master transaction on any port
pio dump    external address of every process-image point      ota / ota-confirm yes / revert yes
fwkey2      customer signing key                               id2       your own cloud identity (Connect_Your_Own_AWS_IoT.md)
beep 1|0    reboot
```

Every cloud command (`dev/I757-M/<sn>/dn/cmd`, plain text) also works verbatim on the console.

## 8. Rules that are enforced by experience, not by the compiler

- **Contracts win.** Change an interface → update its `docs/*.md` contract in the same change. If code and doc disagree, fix the code.
- **Your code lives in `App/`.** At most one line of interface call inside a generated file, and it gets logged in `CubeMX_Code_Boundary_Rules.md`.
- **Interrupt callbacks only raise flags.** Never restart a peripheral or re-enter a HAL flow from an ISR; recover in a task.
- **Use standard protocols** (Modbus, MQTT) before inventing frames; the backplane is Modbus for that reason.
- **CM4 is small**: ~8 KB of free heap on a stock build. `osThreadNew` returning NULL is silent; check `tasks` after adding a thread.
- **Configuration survives reflashing.** littlefs on the external SPI flash keeps `net.cfg` (fallback IP), `flow.cfg`, `dose.cfg`, Modbus port config, `fwkey2.bin`, the device identity, the data log. Flashing new firmware does not reset them; use the console commands (`netcfg default`, `flow off`, `mbcfg …`) or delete on purpose.
- **Never plug or unplug an expansion module under power.**
- **No secrets in the repository**: `keys/` stays outside; the mTLS private key lives inside the 608A and cannot be copied. Comments and identifiers in English; new files carry an SPDX header (`LicenseRef-Edgron-Source-Available`).

## 9. When something is odd

| Symptom | First look |
|---|---|
| Board reboots ~10 min after your OTA | Trial rollback. `ota` / heartbeat `evt`. Usually: cloud not reached (`stat` → `cloud`) or CM4 not alive (`tasks`). |
| Your task never runs | `tasks`: is it listed? If not, `xTaskCreate` failed (heap). Stack in words, not bytes. |
| Hard fault after adding code | `tasks` on the next boot prints the recorded stack overflow / failed allocation; `stack_free` under 100 words is the usual cause. |
| Module not on the dashboard | `mbus` → `online=`; DIP address = value + 1; unique per bus. |
| SWD flash "did nothing" | Running on bank 2 (§5). |
| Beeps every 10 s | Greenhouse flow monitor still initialised (§3). `flow off`. |
| Ping to 192.168.137.2 fails on a direct cable | The fallback address may have been changed: `netcfg`. Unboxing guide §4.2. |

Questions and bugs: GitHub issues on this repository.
