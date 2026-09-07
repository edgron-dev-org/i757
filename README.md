# I757-M Controller SDK

Firmware SDK for the **I757-M** modular industrial controller (STM32H757 dual-core).
Build your own application on top of the platform, or integrate over the cloud API.

## What's here

```
platform_757/        Full buildable dual-core firmware project
├── CM7/             Cortex-M7: network / TLS / MQTT / OTA / storage / application
│   ├── App/         Platform services + YOUR code (write here)
│   ├── Core/        CubeMX-generated (clocks / peripherals / vectors) — don't edit
│   └── Middlewares/ mbedTLS / LwIP / FreeRTOS / littlefs / FatFs
└── CM4/             Cortex-M4: real-time I/O / Modbus / backplane bus master
modbus/              Transport-agnostic Modbus core (RTU + TCP share it)
uartdrv/             Multi-instance UART driver
docs/                Manuals + interface contracts (the authoritative specs)
tools/               OTA push / broker setup scripts
```

## Start here

1. **Hardware & installation** — `docs/i757_Controller_Manual.md` (controller: specs, wiring, operation);
   expansion modules have their own manuals: `docs/EX_16DO_Module_Manual.md`, `docs/PH_EC_Transmitter_User_Manual.md`
2. **Software / how to develop your app** — `docs/Software_Manual_Application_Development.md`
   (where to write code, how to call platform services, a worked example)
3. **Interface contracts** — the `docs/*.md` specs (cloud protocol, Modbus, backplane, RPMsg, security, …)

## Build

```
cd platform_757/CM7 && cmake --preset Debug && cmake --build build/Debug
cd platform_757/CM4 && cmake --preset Debug && cmake --build build/Debug
```
Toolchain: arm-none-eabi-gcc + CMake + Ninja (STM32CubeCLT provides all three).
**No `keys/` folder is needed to build** — see "Configure your broker" below.

## Configure your broker

- **No cloud?** Set `APP_ENABLE_CLOUD 0` in `platform_757/CM7/App/app_cfg.h` for a standalone
  controller (Modbus / I/O / storage / your app tasks, no MQTT). Builds with no broker and no `keys/`.
- **With cloud:** set `CFG_MQTT_PUB_HOST` in `app_cfg.h` to your MQTT broker, and drop your CA +
  device certs into a `keys/ca/` directory (kept OUTSIDE this repo; injected at build time; the
  per-device mTLS private key lives inside the ATECC608A chip). Without `keys/`, the build still
  succeeds with stub certificates — the cloud just stays offline until you add them.

## Security note

This SDK contains **no secrets**: no broker passwords, no private keys, no device
certificates (those are build-generated from your own `keys/`). Each device's mTLS
private key is generated inside and never leaves the ATECC608A — it cannot be copied.

## License

**Source-available, not restricted for personal use.** Edgron-authored code and documents are
under the [Edgron Source-Available License](LICENSE):

- read, study, modify and redistribute the source freely;
- run it on anything for personal, research, educational or evaluation use;
- **commercial use (selling devices or services that run it) is permitted only on genuine
  Edgron hardware** — building or selling other hardware with this firmware is not;
- your own application code stays yours, no disclosure obligation.

Two pure-software components carry no hardware condition and are plain **Apache-2.0**:
[`modbus/`](modbus/LICENSE) (transport-agnostic Modbus core) and [`uartdrv/`](uartdrv/LICENSE)
(multi-instance UART driver).

Third-party components keep their own licenses: see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md), and include those notices with your
product documentation when you ship. "Edgron" and "i757" are trademarks of Edgron; the license
grants no rights to them beyond truthful reference.

## Contributing

Issues and pull requests are welcome. By submitting a contribution you agree that it is licensed
under the same license as the file it modifies.
