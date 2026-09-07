# I757-M Software Manual · Application Development Guide v1.0

> Audience: developers building their own application on the I757-M platform.
> Goal: **where to write your code, how to call platform services, how to build & deploy.**
> See also: hardware & installation in `i757_Controller_Manual.md`; each interface's authoritative definition is in
> the corresponding contract doc under `docs/` (see the index at the end).

---

## 1. Get started in one sentence

> **Write your code in `App/app_user.c`, create your own FreeRTOS task(s) in `app_user_init()`,
> and call the platform services (Modbus, cloud publish, power-fail persistence, storage, …)
> through `app_platform.h`. You almost never touch the CubeMX-generated code.**

---

## 2. Architecture overview

### 2.1 Dual-core split
| Core | Responsibility | Main loop |
|---|---|---|
| **CM7** @480 MHz | Networking (Ethernet/TLS/MQTT), OTA, storage, cloud, **your application tasks** | platform: `mqtt_app_task()` ~500 ms; **your tasks run at any period** |
| **CM4** @240 MHz | Real-time field I/O, Modbus master/slave, backplane bus master | `app_p0_cm4_task()`, 1 ms tick |

**Most business logic goes on CM7** (it has the network, big RAM, runs the application).
Logic that must react without waiting behind the network stack can instead go on CM4 — it has its
own application file and the **same API names on the same process image**, so a piece of logic moves
between cores unchanged. See §4.6. (The two cores exchange *data* through the process image in
shared memory, not through RPMsg; RPMsg only carries control messages.)

### 2.2 Code layers (important)
```
platform_757/CM7/
├── Core/         ← CubeMX-generated (clocks/peripheral init/vectors)   [rarely touch]
├── App/          ← platform + YOUR code                                [write here]
│   ├── app_mqtt.c      platform: network/cloud/main loop
│   ├── app_pwrfail.h   platform: power-fail persistence API ← you call
│   ├── app_mbcfg.c     platform: Modbus port config
│   ├── app_lfs.c       platform: littlefs storage
│   ├── app_platform.h  platform service API  ← include this from your code
│   └── app_user.c      ← YOUR application entry point (app_user_init)   [write here]
├── Middlewares/  ← mbedTLS/LwIP/FreeRTOS/littlefs                       [don't touch]
└── CMakeLists.txt ← add your source file here
```

### 2.3 Golden rule: generated code vs user code
This project follows one discipline (`CubeMX_Code_Boundary_Rules.md`):
- **Your code always goes in `App/` user files**; generated files (Core/) hold at most one line of interface call.
- If you must edit a generated file, do it only inside `/* USER CODE BEGIN */ ... /* USER CODE END */`
  blocks (preserved across regeneration), and log it in the boundary-rules doc.
- Benefit: regenerating from CubeMX won't wipe your code; porting the whole app = copy `App/`.

---

## 3. Two integration modes

### Mode A — Cloud API (external app, no firmware)
Your program (mobile app / server / another controller) connects to the cloud over
**MQTT + TLS + JSON**, subscribes to the device self-description, renders and controls it.
No firmware development. Good for: your other devices, upstream software, cross-device orchestration.
See §5.1 and `Device_Cloud_Protocol.md` (device-cloud protocol).

### Mode B — On-board firmware (business logic on the board) ★ main line of this guide
Business logic (scheduling, dosing programs, config) runs on the controller; the board is the
single source of truth. It keeps running offline. Good for: a self-sufficient controller.
**Everything below is Mode B.**

---

## 4. Developing your application on the board

### 4.1 Toolchain & build
- **Toolchain**: arm-none-eabi-gcc + CMake + Ninja (pre-configured).
- **Build** (CM7 example):
  ```
  cd platform_757/CM7
  cmake --build build
  ```
  Output: `build/platform_757_CM7.elf/.hex`. Same for CM4.
- **Debug**: PW-LINK2 (DAPLink) + OpenOCD, or watch printf logs on the USB-C console.

### 4.2 Where your code goes
Your application lives in **`CM7/App/app_user.c`** (already in the build). The platform calls
`app_user_init()` **once at startup**, after every platform service is up (network, cloud if
enabled, Modbus, time, CLI). You create your own FreeRTOS task(s) there. As your project grows,
add more `app_*.c` files — one line each in `target_sources(...)` of `CM7/CMakeLists.txt`.

Include **`app_platform.h`** from your files — it is the single header for every platform service
(Modbus, backplane I/O, time, temperature, buzzer, power-fail storage).

### 4.3 The programming model — your own tasks
The platform runs its networking / OTA / watchdog on its **own** thread. **Your code runs in tasks
you create** — real FreeRTOS threads that may block, sleep, and run at any period. There is **no
500 ms limit and no "don't block" rule** on your tasks (that restriction is gone — it only ever
applied to the old main-loop hook model).

```c
/* app_user.c */
#include "app_user.h"
#include "app_platform.h"
#include "FreeRTOS.h"
#include "task.h"

static void control_task(void *arg)
{
    (void)arg;
    for (;;)
    {
        /* ... your logic; block freely ... */
        vTaskDelay(pdMS_TO_TICKS(100));      /* your own period */
    }
}

void app_user_init(void)                     /* the platform calls this once at startup */
{
    xTaskCreate(control_task, "ctrl", 1024, NULL, APP_TASK_PRIO_NORMAL, NULL);
    /* create as many tasks as you need — one per concern */
}
```
- **Create as many tasks as you like** — e.g. one for sensors, one for the control loop, one for comms.
- **Blocking is fine**: `vTaskDelay()`, queue / semaphore waits, etc.
- **Priorities**: use `APP_TASK_PRIO_LOW / NORMAL / HIGH` (from `app_user.h`). They sit **below** the
  platform network task, so the watchdog is always fed. If you raise a task above them, make sure it
  still yields (`vTaskDelay`) so it never starves the platform.
- **Stack** is in words: `1024` = 4 KB. Give any task that calls the Modbus helpers ≥ 4 KB.
- Talk to the platform only through the thread-safe APIs in `app_platform.h` (§5) — don't poke
  platform internals from your thread.

### 4.4 A complete example — fertilizer dosing (in a task)

This example is not just prose: it lives at **`CM7/App/examples/example_dosing.c`** and is
**compiled with every firmware build** (entry points namespaced so it never collides with your
`app_user.c` and never runs by itself). If the platform API ever drifts from this code, the build
breaks — the example cannot silently rot. To run it, copy the body into your `app_user.c`.

```c
/* app_user.c — dose a target volume, keep the cumulative total across power loss.
 * Reads a flow meter over Modbus (485B) and drives a pump relay on a backplane relay module. */
#include "app_user.h"
#include "app_platform.h"
#include "FreeRTOS.h"
#include "task.h"

PF_RETAIN static uint32_t g_total_dosed_ml;   /* survives power loss (battery-backed, app_pwrfail.h) */
static volatile uint32_t  s_target_ml;
static volatile uint8_t   s_running;

static void dosing_task(void *arg)
{
    (void)arg;
    if (!app_pf_retain_valid()) { g_total_dosed_ml = 0; }   /* first boot / battery replaced */
    for (;;)
    {
        if (s_running)
        {
            uint16_t flow[1];
            static uint16_t last_raw;
            static uint8_t  have_raw;
            /* read the flow meter's RUNNING TOTAL (holding reg 0, slave addr 1 on 485B) and
             * accumulate the DELTA between polls — adding the meter's total on every poll
             * would count the same volume again and again. u16 subtraction survives the
             * meter's register wrapping. */
            if (app_mb_read_holding(APP_PORT_485B, 1, 0x0000, 1, flow) == 0)
            {
                uint16_t delta = (uint16_t)(flow[0] - last_raw);
                last_raw = flow[0];
                if (!have_raw) { have_raw = 1; delta = 0; }   /* first read: baseline only */
                g_total_dosed_ml += delta;          /* cumulative total, auto-persisted */
                if (g_total_dosed_ml >= s_target_ml)
                {
                    /* stop: pump relay off — coil 0 of a relay module on the backplane at DIP addr 2 */
                    app_mb_write_coil(APP_PORT_BACKPLANE, 2, 0, 0);
                    s_running = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));             /* poll 5×/s — your choice */
    }
}

/* call this from the CLI, a cloud command, or another task */
void app_start_dosing(uint32_t target_ml)
{
    s_target_ml = target_ml;
    app_mb_write_coil(APP_PORT_BACKPLANE, 2, 0, 1);  /* start: pump relay on */
    s_running = 1;
}

void app_user_init(void)
{
    app_mbport_configure(APP_PORT_485B, APP_MB_MASTER, 9600, 'N', 1, 0);   /* flow-meter port */
    xTaskCreate(dosing_task, "dose", 1024, NULL, APP_TASK_PRIO_NORMAL, NULL);
}
```

### 4.5 Don't need the cloud? Build standalone
If your product has no cloud, set one flag in `CM7/App/app_cfg.h`:
```c
#define APP_ENABLE_CLOUD 0
```
This compiles out MQTT, TLS, cloud OTA and SNTP (image ~54 KB smaller). **Everything else keeps
working**: Modbus master/slave on all 6 front ports, Modbus TCP, backplane modules, USB CLI, RTC
time, power-fail retention, and your app tasks. Firmware is then flashed over SWD only, and the
board needs **no broker, no certificates, and no `keys/` folder** to build.
Leave it at `1` (default) to use the cloud — then set your broker in `app_cfg.h` and drop your own
CA/device certs into `keys/ca/` (see §5.1 and `OTA_and_MQTT_User_Guide.md`).


### 4.6 Putting logic on CM4 (the real-time core)

CM4 has its own application file, **`CM4/App/app_user_cm4.c`**, and it works exactly like the CM7
one: the platform calls `app_user_cm4_init()` once before the CM4 main loop starts, and you create
your own FreeRTOS thread(s) there. Include **`app_platform_cm4.h`**, which mirrors `app_platform.h`:

```c
uint8_t  app_io_di    (uint16_t pt, uint16_t ch);       // same names, same meaning, same image
uint16_t app_io_ai    (uint16_t pt, uint16_t ch);
void     app_io_do_set(uint16_t pt, uint16_t ch, uint8_t on);
uint8_t  app_io_do_get(uint16_t pt, uint16_t ch);
void     app_io_hr_set(uint16_t pt, uint16_t ch, uint16_t v);
int      app_io_ok    (uint16_t pt);
uint8_t  app_hsdi_level(uint8_t ch);                    // onboard high-speed inputs
uint32_t app_hsdi_count(uint8_t ch);
```

Because the names match, **the same function body compiles on either core**. Point names come from
the shared `Common/app_user_points.h` enum, so neither side keeps hand-written indexes that could
drift apart.

**Which core?**

| Put it on CM4 | Put it on CM7 |
|---|---|
| Reacts in microseconds: interlocks, pulse-driven logic, anything that must not queue behind the network stack | Cloud, dashboard, storage, recipes, scheduling, UI |
| Reads the field buses and the onboard high-speed inputs — the data is freshest here, since CM4 is the core that scans them | Anything that needs the network, large RAM, or the filesystem |

The shipped example (`app_user_cm4.c`) is a deliberate demonstration of the move: an interlock that
used to live in the CM7 control task now sits on CM4, flipping a relay on an HSDI rising edge every
5 ms, while the remaining inputs still drive their relays from CM7. **Both cores command the same
relay module at the same time** — that is safe because writes to the output image go through the
platform's cross-core lock; you do not add locking of your own for these calls.

Add further CM4 files with one line each in `target_sources(...)` of `CM4/CMakeLists.txt`.

**Watch the CM4 budget.** Thread stacks come out of the FreeRTOS heap, and CM4 runs with far less
free heap than CM7 in practice (~8 KB spare against ~31 KB on a stock build, since CM4 already runs a
thread per bus). If `osThreadNew` returns NULL your thread silently never runs — it does not fail
loudly — so after adding a thread confirm it with `tasks` (§5.9), which lists every live task, the
free heap, and each stack's worst-ever headroom. Raise `configTOTAL_HEAP_SIZE` in
`CM4/Core/Inc/FreeRTOSConfig.h` if you need more.

---

## 5. Platform services you can call (API reference)

### 5.1 Publish to cloud/dashboard + receive control  *(cloud mode only — skip this if `APP_ENABLE_CLOUD=0`)*
The cloud dashboard is **data-driven**: the device publishes a self-description (`desc`) plus a
periodic heartbeat (`status`); the dashboard renders from those, so adding a value needs no
dashboard change.

**To show a value on the dashboard**, edit two spots (both in `app_mqtt.c`):
1. Add a point to `DESC_JSON[]`:
   ```c
   "{\"id\":\"fert.total\",\"name\":\"Total Dosed\",\"kind\":\"num\",\"rw\":\"ro\",\"unit\":\"ml\",\"group\":\"sys\",\"f\":\"ftot\"},"
   ```
   (`kind` = bool/num/enum/bits/str; `rw` = ro/rw; `f` = the field name in the heartbeat)
2. Add the field `"ftot":%lu` to the heartbeat JSON (`snprintf(s_pub_buf,...)`) with argument `app_total_dosed()` (a small getter you expose that returns `g_total_dosed_ml`).
   ⚠️ Keep the heartbeat buffer `s_pub_buf` and the MQTT ring buffer `MQTT_OUTPUT_RINGBUF_SIZE`
   large enough (re-check after adding fields; the `desc` must also fit the ring buffer).

**To receive control commands from the dashboard/cloud**: the cloud publishes `dn/cmd`, dispatched
by `ota_cmd()` in `app_ota.c`. Add your command (an else-if branch):
```c
else if (strncmp(c, "dose ", 5) == 0) {
    extern void app_start_dosing(uint32_t);
    app_start_dosing((uint32_t)strtoul(c + 5, 0, 10));
}
```
If a `desc` point is `rw`, the dashboard sends `set fert.target <value>` — handle it the same way in `ota_cmd`.

Contract: `Device_Cloud_Protocol.md` (topics/JSON/self-description).

### 5.2 Modbus — configure ports and talk to devices (C API)
All of these are in `app_platform.h` and are **thread-safe** — call them from your tasks. Ports are
the six front connectors `APP_PORT_485A … APP_PORT_485F` **and the internal backplane expansion bus
`APP_PORT_BACKPLANE`** — all plain Modbus RTU masters reached through the same calls. Roles (for the
front ports) are `APP_MB_OFF / APP_MB_SLAVE / APP_MB_MASTER`.

**Configure a front 485 port** (persists to flash, applies immediately):
```c
app_mbport_configure(APP_PORT_485B, APP_MB_MASTER, 9600, 'N', 1, 0);  /* master, 9600 8N1 */
app_mbport_configure(APP_PORT_485A, APP_MB_SLAVE, 19200, 'E', 1, 5);  /* this board = slave, addr 5 */
```
(You can also configure from the CLI or cloud: `mbcfg 485b master 9600 8N1`.)

**As a master, read / write a device.** Return 0 on success, `<0` on error (a Modbus exception `E`
comes back as `-(100+E)`):
```c
uint16_t regs[4];
app_mb_read_holding (APP_PORT_485B, 1, 0x0000, 4, regs);   /* FC03: 4 holding regs from slave 1 */
app_mb_read_input   (APP_PORT_485B, 1, 0x0010, 2, regs);   /* FC04: 2 input regs */
app_mb_write_single (APP_PORT_485B, 1, 0x0002, 1234);      /* FC06: write one reg */
uint16_t vals[3] = { 1, 2, 3 };
app_mb_write_multi  (APP_PORT_485B, 1, 0x0000, 3, vals);   /* FC16: write three regs */
```
**Drive relays / read digital I/O (coils).** A relay module maps its outputs to coils `0..N-1`;
`bits`/`out` are packed byte arrays, LSB = first coil:
```c
app_mb_write_coil (APP_PORT_485B, 1, 3, 1);              /* FC05: one coil (coil 3) ON  */
uint8_t pattern[2] = { 0xFF, 0x00 };                     /* coils 0..7 ON, 8..15 OFF     */
app_mb_write_coils(APP_PORT_485B, 1, 0, 16, pattern);    /* FC15: 16 coils at once        */
uint8_t state[2];
app_mb_read_coils (APP_PORT_485B, 1, 0, 16, state);      /* FC01: read 16 coils back       */
```

**Backplane expansion modules** use the *exact same* calls — pass `APP_PORT_BACKPLANE` as the port
and the module's DIP address (1..16) as the slave address. There is nothing module-specific: the
backplane is a standard Modbus RTU bus, so any present or future module at any address is reachable.
```c
/* 16-channel relay board on the backplane at DIP address 2 — all 16 relays ON */
uint8_t all_on[2] = { 0xFF, 0xFF };
app_mb_write_coils(APP_PORT_BACKPLANE, 2, 0, 16, all_on);
app_mb_write_coil (APP_PORT_BACKPLANE, 2, 5, 0);         /* just relay 5 OFF               */
/* read a module's identity registers (FC04, see Backplane_Bus_Protocol.md §3.3) */
uint16_t id[4];
app_mb_read_input (APP_PORT_BACKPLANE, 2, 0x0000, 4, id);
```
For any other function code, build the request PDU yourself and call `app_mbport_master()` with any
port (front or `APP_PORT_BACKPLANE`).

**As a slave**, this board answers a Modbus master (e.g. a touch panel wired to a 485 port) using a
uniform register map; the areas `0x2000+` are reserved for **your** scheduling / program / config
data. **Modbus TCP** (server + client) is available over Ethernet. See
`Universal_Modbus_Port_Config.md` for the full register map and TCP details.

Contracts: `Universal_Modbus_Port_Config.md` + `Backplane_Bus_Protocol.md`.

### 5.3 Process image — cyclic field I/O without blocking ★ the recommended way

The §5.2 helpers are **on-demand**: each call blocks on one bus round-trip, which ties your
control loop to bus timing. For I/O you read or write **every cycle**, register a table of points
once and let the CM4 scanner poll them in the background; your code then touches memory instead
of the wire. Contract: `Process_Image_and_IO_Mapping.md`.

**Declare your field devices** (edit this table in `app_user.c` — that is the whole setup):
```c
/* Point names live in ONE shared header so the CM4 core uses the same handles (§4.6):
 *   Common/app_user_points.h:  enum { PT_DI = 0, PT_AI, PT_DO, PT_AO, PT_COUNT };
 * The table below and that enum must stay in row-for-row sync — the _Static_assert
 * makes a mismatch a compile error instead of silent index shift. */
#include "app_user_points.h"

static const app_io_point_t s_io_table[] = {
  /* port,          addr, access,             flags, start, count, period(x base tick)
   * Pick periods the wire can carry: one FC02/FC04 transaction at 9600 baud costs ~20 ms
   * round-trip, so period 20 (= 20 ms here) is a floor, not a suggestion — a shorter
   * period cannot poll faster than the bus, it only saturates it and counts overruns. */
  { APP_PORT_485A,  1,    APP_IO_IN_DISCRETE, 0,     0,     12,    20  },  /* 12 DI  <- FC02 */
  { APP_PORT_485A,  2,    APP_IO_IN_INPUT_REG,0,     0,     8,     100 },  /*  8 AI  <- FC04 */
  { APP_PORT_485A,  1,    APP_IO_OUT_COILS,   0,     0,     8,     20  },  /*  8 DO  -> FC15 */
  { APP_PORT_485A,  3,    APP_IO_OUT_HOLDING, 0,     0,     8,     100 },  /*  8 AO  -> FC16 */
};
_Static_assert(sizeof(s_io_table) / sizeof(s_io_table[0]) == PT_COUNT,
               "s_io_table and app_user_points.h are out of sync");

void app_user_init(void)
{
    app_mbport_configure(APP_PORT_485A, APP_MB_MASTER, 9600, 'N', 1, 0);
    if (app_io_setup(s_io_table, PT_COUNT, 1) != 0)   /* 1 = base tick in ms */
    {
        /* <0 = bad table; -4 = CM4 did not confirm the reload — table stored, not live */
    }
    xTaskCreate(control_task, "ctrl", 1024, NULL, APP_TASK_PRIO_NORMAL, NULL);
}
```
`start` is the address **inside that device** (from its manual). `period` is the poll interval in
base ticks — fast for digital, slower for analog; it is also how you budget bus bandwidth.
Use `APP_PORT_BACKPLANE` and the module's DIP address for a backplane module — same table.

**Use it — every call is a memory access, nothing blocks:**
```c
static void control_task(void *arg)
{
    (void)arg;
    for (;;)
    {
        uint8_t  start_button = app_io_di(PT_DI, 0);   /* input bit  */
        uint16_t level        = app_io_ai(PT_AI, 0);   /* input word */

        app_io_do_set(PT_DO, 0, (start_button && level < 500) ? 1 : 0);   /* output bit  */
        app_io_hr_set(PT_AO, 0, 4000);                                    /* output word */

        if (app_io_ok(PT_AI) == 0) { /* that device is not answering */ }

        vTaskDelay(pdMS_TO_TICKS(20));   /* your control rate, independent of the scan rate */
    }
}
```

**Let an HMI / SCADA / PLC reach the same data — no extra code.** Set any other front port to
Modbus **slave** and the board becomes a data concentrator; Modbus TCP serves the identical map
at the identical addresses:
```c
app_mbport_configure(APP_PORT_485B, APP_MB_SLAVE, 9600, 'N', 1, 10);  /* address 10 */
```
| External master uses | Reaches |
|---|---|
| FC02 discrete inputs / FC04 input registers | your **input** points (read-only) |
| FC01/05/15 coils / FC03/06/16 holding registers | your **output** points (read **and** write) |

Addresses live in a window based at `0x2000`. Run **`pio dump`** on the USB console to print the
exact address of every point — hand that listing to whoever integrates the HMI:
```
scan: 4 pts base=1ms ver=2 applied=2 run=1
 [0] p0 a1 inDISC st0 n12 per20 ok1 fail0 exc0 | ext FC02 0x2800..0x280B | 0F00
 [1] p0 a2 inIREG st0 n8 per100 ok1 fail0 exc0 | ext FC04 0x2081..0x2088 | 01F2...
 [2] p0 a1 outCOIL st0 n8 per20 ok1 fail0 exc0 | ext FC01/05/15 0x2800..0x2807 | 03
 [3] p0 a3 outHOLD st0 n8 per100 ok1 fail0 exc0 | ext FC03/06/16 0x2081..0x2088 | A00F...
 bus0 online=0000 polls=1234 fails=0 ovr=0 cycle=61200us
```
Platform I/O (onboard HSDI, sub-card slots) owns the low part of the window, so **your point
addresses never move** as that hardware is added. Your application and the external master may
own different channels of the same point safely.

Other `pio` commands: `pio setup [port addr]` (register a demo table), `pio do <pt> <ch> <0|1>`,
`pio hr <pt> <ch> <value>`.

### 5.4 Three-layer power-fail persistence (app_pwrfail.h) ★ used by the dosing total
```c
#include "app_pwrfail.h"
PF_RETAIN static uint32_t my_var;         // Layer 0: modifier = retained across power loss (4KB battery domain)
app_pf_retain_valid();                     // 0 = first boot / battery replaced, platform cleared it, you re-init
app_pf_hook_register(fn);                  // Layer 1: last-gasp callback (memory writes only)
app_pf_blackbox_set(buf, len);             // Layer 2: register a ≤8KB buffer, auto-flushed to flash at last gasp
app_pf_blackbox_last(dst, cap);            // read back the most recent record on boot
app_pf_battery_ok();                       // battery-present self-check
```
Measured: warning window 31~39 ms; Layer 0/1 = 4 KB @50 µs, Layer 2 = flash ≥8 KB. Recommended
budget: use BKPSRAM 4 KB freely + flash ≥8 KB.

**Output behavior after power-up / reset (important, ruled 2026-07-25)**: **after ANY reset
(power-on, power-loss recovery, watchdog, OTA) all outputs are cleared** — outputs are a
function of logic, never a memory, matching industrial PLC cold/warm-restart semantics; the
platform will never restore outputs for you (loads silently re-starting after a mains dip is an
industrial-safety taboo). If you need "restore the pre-outage state", the standard pattern is
explicit: keep your process state in `PF_RETAIN` variables, check `app_pf_retain_valid()` on
your task's first scan, and let **your program** decide which outputs may be re-driven and in
what order (adding interlocks / operator confirmation where appropriate).

### 5.5 Storage
- **On-board NOR / LittleFS** (high-frequency writes, critical state, power-fail-safe):
  ```c
  #include "lfs.h"
  extern lfs_t *app_lfs(void);   // returns the mounted littlefs; use standard lfs_file_* API
  ```
- **SD card / FatFs** (removable bulk, PC-readable, log/data export): platform mounts it; the
  config file `i757.cfg` is a sample.
- Split: config/critical state → NOR; logs/data export → SD.

**MB-scale RAM buffers — the 32 MB external SDRAM:**
```c
APP_SDRAM static uint16_t history[1000000];   /* 2 MB of samples, costs no on-chip RAM */
```
Not loaded and not zeroed at startup (the SDRAM controller starts on the first main-loop pass),
so initialise it yourself and only touch it from tasks — tasks created in `app_user_init` start
late enough. It is cached write-back: do cache maintenance before giving SDRAM buffers to DMA.

**Where files actually live:** littlefs sits on the first 30 MB of the 32 MB QSPI flash; the last
2 MB is the raw power-fail save area used by `app_pf_blackbox_*` (§5.4) — 170 rotating 12 KB
slots, kept pre-erased so the dying write is pure page programming. Layout source:
`App/app_qflash.h`; map: `Hardware_Resource_Allocation.md`.

### 5.6 Inter-core communication (CM7 ↔ CM4)
When you need CM4 to do real-time work, CM7 uses an RPMsg transaction:
```c
#include "app_rpc.h"
uint16_t app_rpc_transact(const uint8_t *req, uint16_t len, uint8_t *rsp, uint16_t cap, uint32_t timeout_ms);
```
The op-code table (time broadcast / Modbus transaction / port config / …) is in `Inter_Core_RPMsg_Protocol.md`.
New ops get a number from that table, and each core adds a branch.

### 5.7 Security identity (608A)
- The platform already uses the 608A private key for mTLS to the cloud (unique hardware identity
  per device, server-verified). You normally don't touch it.
- The anti-clone result is in the heartbeat `sec` field (ok/clone); the 608A also has customer
  slots (slot1~15) free for your own keys.

### 5.8 Other common calls
```c
uint32_t app_time_now(void);        // Unix seconds (RTC/SNTP)
int      app_temp_read(void);       // chip temperature °C
void     app_beep_set(uint8_t on);  // buzzer on/off
// The platform feeds its own watchdog. Your tasks are separate threads — they may block freely.
```

**Network fallback address (netcfg, added 2026-08-31)**: DHCP always comes first; if no lease arrives within ~8 s of boot, the controller falls back to a static address. The factory default comes from compile-time `app_cfg.h` (`CFG_STATIC_IP` etc.) and can be changed per site over the console or a cloud command, persisted on littlefs (`net.cfg`) — one universal image fits every network:

```
netcfg                                                 # show fallback config and the address actually in use
netcfg 192.168.1.250 255.255.255.0 192.168.1.1         # set and persist (optional 4th arg = DNS, defaults to the gateway)
netcfg default                                         # remove the file, back to compile-time defaults
```

Changing the value while stranded in fallback re-applies the address and reconnects immediately; the cloud uses the same syntax over `dn/cmd`, result in the heartbeat `cmdr` field. While in fallback the controller retries DHCP automatically every 60 s (and on a cable replug); as soon as a lease arrives it switches back to the dynamic address and reconnects (2026-08-31).

### 5.9 Diagnostics over the USB console — is anything about to break?

Two console commands report the health of **both cores** in one view. They exist to make the two
silent failure modes loud: a task overrunning its stack, and a heap allocation quietly returning
NULL (which just looks like a thread that never started).

```
tasks      (alias: ps)   task table, stack headroom, heap, reset cause, past faults
runstats                 FreeRTOS's own vTaskGetRunTimeStats() table, both cores
health clear             acknowledge a recorded fault
```

`tasks` output, one row per task on each core:

```
tasks  (cpu% = share since the previous run; stack_free = smallest EVER)
 last reset: software reset
 CM7  heap free 30904 / min-ever 30640 B   uptime 53s
   defaultTask   run    pri24  cpu 14%  stack_free  327 w ( 1308 B)
   IDLE          ready  pri0   cpu 84%  stack_free  115 w (  460 B)
 CM4  heap free 7976 / min-ever 7536 B   uptime 53s
   bus485A       ready  pri16  cpu  4%  stack_free  441 w ( 1764 B)
   IDLE          ready  pri0   cpu 79%  stack_free  116 w (  464 B)
```

**`stack_free` is the number to watch.** It is the FreeRTOS high-water mark — the smallest free
space that stack has *ever* had, in **words** (×4 = bytes), not its free space right now. A stack
overflow corrupts whatever sits below it and surfaces as a hard fault nowhere near the cause, so it
has to be visible while there is still margin. Rows under 100 words are flagged `<-- LOW`; raise
that task's stack size where it is created.

If a stack overflow or a failed allocation ever happened, the offending task's name is written into
a record in SRAM4 that survives the reset, and the next boot prints it:

```
 !! CM4 FAULT ON A PREVIOUS RUN: STACK OVERFLOW in task 'myTask'  ('health clear' to acknowledge)
```

**The two CPU% figures, and why both exist.** The `cpu%` column in `tasks` is the share *since your
previous `tasks` call*, i.e. the load right now. `runstats` prints the kernel's own table, the
average *since boot*, with none of our arithmetic in the path. Keep both and compare them when a
number looks wrong — that is exactly how a false "25% on an idle port" reading was traced to a bug
in the delta computation rather than to the board, and how a wrapping run-time counter on CM7 was
caught reporting `IDLE 423%`.

[2026-07-19] One caveat on `runstats`: its percentages are lifetime averages and are valid within
roughly the **first two days of uptime** — the run-time counter's returned value wraps at about
**49 h on CM7** and **76 h on CM4**. Beyond that, use the delta-based `cpu%` column in `tasks`,
which stays correct indefinitely.

If you add your own run-time counter, it must be **monotonic**: FreeRTOS uses its current value as
the denominator while task totals accumulate across wraps, so a counter that wraps does not just
lose a sample — it collapses the denominator and every percentage explodes.

Contract: `Inter_Core_RPMsg_Protocol.md` op 0x0D (task table) and op 0x0E (run-time statistics).


---

## 6. Build · Flash · OTA deploy

1. **Build** (CM7, then repeat for CM4). Needs `arm-none-eabi-gcc`, CMake and Ninja on PATH:
   ```
   cd platform_757/CM7
   cmake --preset Debug          # configure once
   cmake --build build/Debug     # -> build/Debug/platform_757_CM7.elf
   ```
   No `keys/` folder is needed to build (§4.5). With `APP_ENABLE_CLOUD=1` and no keys you get stub
   certificates plus a warning — drop your `keys/ca/` in to enable the cloud.
2. **First / development flash**: SWD (OpenOCD). ⚠️ On bank 2, use OTA, not SWD (SWAP trap, see Hardware Manual §9).
3. **OTA deploy** (recommended, cloud mode): `tools/ota_push.ps1 -Public` pushes both-core images, with
   automatic dual-bank swap + signature verification.
   - **Fail-safe**: a new image must prove "cloud connected + both cores healthy" (just "both cores
     healthy" when `APP_ENABLE_CLOUD=0`) within a trial period, else it auto-rolls back to the old bank
     — **break it and the board recovers itself, no brick**. Iterate freely during development.
4. **After a SWD flash you must push one OTA** (otherwise revert is refused); ops details in `OTA_and_MQTT_User_Guide.md`.

---

## 7. Contract doc index (single source of truth per interface)

| What you want to do | Contract |
|---|---|
| Cloud publish/control/self-description | `Device_Cloud_Protocol.md` |
| Configure 485 ports master/slave / board register map | `Universal_Modbus_Port_Config.md` |
| Cyclic field I/O, process image, external 0x2000 window | `Process_Image_and_IO_Mapping.md` |
| Onboard 8 high-speed DI: plain DI / counter / encoder | `HSDI_Configuration_and_Counting.md` |
| Attach a backplane expansion module | `Backplane_Bus_Protocol.md` |
| CM7↔CM4 communication | `Inter_Core_RPMsg_Protocol.md` |
| Power-fail persistence | `App/app_pwrfail.h` (the header is the contract) |
| Where code goes / editing generated files | `CubeMX_Code_Boundary_Rules.md` |
| Chip resources (memory/DMA/interrupt/pins) | `Hardware_Resource_Allocation.md` |
| OTA usage | `OTA_and_MQTT_User_Guide.md` |

**Overall discipline**: change an interface → update its contract; new interface → write the
contract first. If code and doc disagree, the doc wins.

---

## 8. Delivery & source conventions

- Private keys / certificates (`keys/`) live **outside** this repo and are never committed; each
  device's mTLS private key is inside the ATECC608A and cannot be copied.
- `build/` intermediates are not committed. Build your own firmware images.
- Production: each board's 608A public key differs → at provisioning, read the public key and have
  your CA sign a per-board device certificate (a factory step); firmware embeds no fixed certificate.

---

For any interface question, check the corresponding contract doc first; if a contract doesn't
cover it, contact the platform team to add the contract before coding.
