# OpenAMP/RPMsg Integration Guide (materials ready 2026-07-08, execute mechanically per this)

> Decisions and architecture see the internal archive plan/2026-07-08 §8. Already in place: middleware `Middlewares/Third_Party/OpenAMP/` (copied from FW_H7_V1.13.0), glue `Common/openamp/` (the H745I-DISCO PingPong example's Common), CM7 `App/app_rpc.c/h`, CM4 `App/app_rpc_cm4.c`—**none integrated into the build; the integration steps follow**.

## 1. SRAM4 partitioning (coexists with existing occupancy, verified non-conflicting)

| Address | Use |
|---|---|
| 0x38000000/04 | IPC ping-pong (active, retired once RPMsg is stable) |
| 0x38000100~13F | OTA trial-period guard (do not touch) |
| **0x38000200 (0x200)** | **resource_table** (example original 0x38000000, must change to here!) |
| **0x38000400 (31K)** | **OPEN_AMP_SHMEM** (vring RX@+0 / TX@+0x400 / buffers@+0x800) |

## 2. Both cores' .ld modification (CM7 and CM4 both; ninja does not track .ld, do a full rebuild after changing)

Append to the MEMORY block:
```
OPENAMP_RSC_TAB (xrw) : ORIGIN = 0x38000200, LENGTH = 0x200
OPEN_AMP_SHMEM (xrw)  : ORIGIN = 0x38000400, LENGTH = 31K
```
Append within SECTIONS (note ABSOLUTE is **0x38000200**; the 0x38000000 copied from the example would step on ping-pong + guard):
```
.resource_table (NOLOAD) : { . = ABSOLUTE(0x38000200); *(.resource_table) } > OPENAMP_RSC_TAB
```
Symbols:
```
__OPENAMP_region_start__ = ORIGIN(OPEN_AMP_SHMEM);
__OPENAMP_region_end__   = ORIGIN(OPEN_AMP_SHMEM) + LENGTH(OPEN_AMP_SHMEM);
```
(rsc_table.c's resource_table_init fills the table at CM4 runtime, NOLOAD is harmless—if the link reports an init-data conflict, remove NOLOAD and retry.)

## 3. Build integration (CM7 = append to app_tls.cmake; CM4 = CMakeLists user region)

Sources (same 17 for both cores):
```
Middlewares/Third_Party/OpenAMP/libmetal/lib/{device,init,io,log,shmem}.c
Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/{condition,irq,time,generic_device,generic_init,generic_io}.c
Middlewares/Third_Party/OpenAMP/libmetal/lib/system/generic/cortexm/sys.c
Middlewares/Third_Party/OpenAMP/open-amp/lib/remoteproc/remoteproc_virtio.c
Middlewares/Third_Party/OpenAMP/open-amp/lib/rpmsg/{rpmsg,rpmsg_virtio}.c
Middlewares/Third_Party/OpenAMP/open-amp/lib/virtio/{virtio,virtqueue}.c
Common/openamp/{openamp,mbox_hsem,rsc_table}.c
+ CM7: App/app_rpc.c ; CM4: App/app_rpc_cm4.c
```
Header paths: `open-amp/lib/include`, `libmetal/lib/include`, `Common/openamp` (including its compat header under openamp/).
Macros (common to both cores): `METAL_INTERNAL  METAL_MAX_DEVICE_REGIONS=2  NO_ATOMIC_64_SUPPORT  RPMSG_BUFFER_SIZE=512` (the example's 100 can't hold a Modbus PDU, enlarged to 512); CM7 adds `VIRTIO_MASTER_ONLY`, CM4 adds `VIRTIO_SLAVE_ONLY`.
⚠️ libmetal may require `libmetal/lib/include/metal/config.h` to exist—if it reports a missing header, find the `metal/config.h` template in the example project or in the CubeMX generated output and copy it to that path.

## 4. Call sites (one line each, conforming to the boundary rules)

- CM7 `app_mqtt.c`: after `app_time_init()` add `app_rpc_init()` (blocks waiting for the CM4 endpoint; if CM4 dies → IWDG bites → trial-period rollback backstop, semantics correct); add `app_rpc_poll();` in the main loop; `app_cli.c` adds command `rpc` → `app_rpc_cli()`.
- CM4 `app_cm4.c`: add `app_rpc_cm4_init();` at the start of the task (after HAL_Init); add `app_rpc_cm4_poll();` in the loop (the osDelay 50ms cadence is enough for bring-up, tighten it after the slave migration).
- The IPC ping-pong is **kept for now** (the heartbeat IPC-OK and the trial-period confirmation both depend on it), retire it after the RPMsg heartbeat replaces it.

## 5. Verification

1. Full rebuild of both cores (.ld changed) → OTA push (both-core images go together).
2. Type `rpc` at the CLI → expect three lines `rpc[i]: pong:ping i`.
3. Regression: `mb`/`can`/`mbx` run as usual, heap/heartbeat no anomalies; existing SRAM4 functions (guard/ping-pong) undisturbed.
4. Past echo → step ②: migrate app_mbport wholesale to CM4 (modbus_core/slave compiled straight into CM4), M7's mbx switched to app_rpc_transact, rerun 8/8.

## 6. Registration reminders

Boundary rules: new middleware line (OpenAMP copied from the FW package) + Common/openamp directory + both cores' new App files + HSEM1/2_IRQHandler ownership + SRAM4 partition-table update; hal_conf if it needs HSEM enabled (already on by default). Archive/memory per convention.


## Appendix: Warm-reset race and its root fix (confirmed 2026-07-08, must read)

**Symptom**: after an OTA bank swap / reboot, occasional `[RPC] init FAIL`, the CM4 main loop alive (feeding IWDG2) but the endpoint never binds, rpc all times out; within the trial period = can't graduate, each reset moves one notch toward the rollback threshold.

**Mechanism** (proven link by link): the resource_table is in SRAM4 (0x38000200), the GCC startup code **does not initialize** this section (ST itself commented it out in rsc_table.c), and **SRAM4 is not cleared on warm reset**. ST's handshake design: CM4 spins waiting for `vring1.da==VRING_RX_ADDRESS` (waiting for CM7 to fill the table) → `rpmsg_virtio_wait_remote_ready` waits for `DRIVER_OK` (waiting for the CM7 host to be ready). After a warm reset the table retains these two values from the previous life → CM4's two checkpoints are both instantly waved through by stale values → the NS announcement is sent into the old vring → then CM7's `MX_OPENAMP_Init` memset + vring rebuild wipes the announcement → CM4 believes it already announced and never sends again = deadlock. Whoever runs there first wins, a pure boot race, hence intermittent.

**Root fix** (app_rpc.c `app_rpc_boot_scrub` + one-line call in CM7 main.c Boot_Mode_Sequence_2): CM4 stops at STOP at boot waiting for an HSEM wake—CM7 wipes the entire resource table to zero **before** waking it (at which point CM4 is asleep, zero race). The two checkpoints regain their designed semantics: CM4 must wait until CM7 is truly ready before announcing, and binding becomes a deterministic event; incidentally fixing cold start (table = random garbage at power-on) and late binding (the announcement can never be lost again). **Verification**: OTA reset + 5 consecutive reboots = 6/6 `init ok (rc=0)`, trial-period boot 1 graduated on the spot.

**Porting reminder**: any dual-core project that "places the resource table / shared memory in RAM not cleared on reset" has this landmine (the H503 slave likewise if it later runs RPMsg); the host-side "wipe shared state before waking the remote core" is the generic solution.
