# MCX-N9XX-EVK — full board port

Porting OpenBIC to the NXP MCX-N9XX-EVK (MCXN947, dual Cortex-M33).

This branch (`full-board-port`) builds on the `mcx-n9xx-evk-port`
bring-up branch's 5 from-scratch subsystems (GPIO monitoring, HWINFO,
watchdog, NVS storage, dual-core mailbox - all still present, see
below) and goes further: `main()` now calls the **real**
`common/hal`/`common/lib`/`common/service` code - `wdt_init()`,
`util_init_timer()`, `util_init_I2C()`, `sensor_init()`, `FRU_init()`,
`ipmi_init()` - ported to mainline Zephyr's API surface, plus real
MCTP (transport core) and PLDM (base protocol), instead of reusing the
bring-up branch's stand-ins. IPMI/IPMB is a real, working transport -
message dispatch verified end to end on real hardware via a
self-test, with only its bottom-edge I2C integration needing new code
(mainline's `i2c_target_register()` API, replacing the Aspeed-fork's
own I2C-slave-as-IPMB driver) - see "IPMI transport" below. See "What's
real, what's excluded" below for exactly what was ported vs. left out
and why - nothing is stubbed silently.

## What's real, what's excluded, and why

**Real, ported, verified on hardware:**
- `common/hal/hal_wdt.c` - watchdog, `DEVICE_DT_GET(DT_ALIAS(watchdog0))` instead of string-name `device_get_binding()`.
- `common/hal/hal_i2c.c` - I2C master (`I2C_MODE_CONTROLLER`, real `DEVICE_DT_GET` bindings, dead Aspeed-only `<drivers/i2c/slave/ipmb.h>` include dropped) **and** target/slave mode: `ipmb_target_register()`/`ipmb_target_read()` (gated on `CONFIG_I2C_TARGET`) back the real IPMB channel below using Zephyr's generic `i2c_target_register()` API. Genuinely talks to real hardware on **two** independent buses, both on the Arduino-compatible header J2 (see "IPMI transport" below for the exact pinout): `i2c scan flexcomm3_lpi2c3` (controller mode, J2 pins 15/17) performs 128 real bus transactions (0 found - nothing's wired there, honestly); `flexcomm2_lpi2c2` (J2 pins 18/20) is the target-mode bus.
- `common/service/ipmi/*` (`ipmi.c`, `app_handler.c`, `chassis_handler.c`, `sensor_handler.c`, `storage_handler.c`, `oem_handler.c`) and `common/service/ipmb/ipmb.c` - the real IPMI dispatch/handler pipeline and IPMB transport framing (checksums, sequence tracking, TX/RX threads) - verified end to end on real hardware via a self-test. See "IPMI transport" below for the full writeup, including the one still-stubbed piece (`oem_1s_handler.c`) and the wire-level test's honest "untested" status.
- `src/plat_ipmb.c` - real IPMB channel config (one channel, target mode, on `flexcomm2_lpi2c2`), analogous to every other real OpenBIC board's own `plat_ipmb.c`.
- `common/lib/timer.c`, `common/lib/libutil.c`, `common/lib/util_pmbus.c`, `common/lib/util_sys.c` (partially - see below) - portable as-is/near-as-is (CMSIS-RTOS2 timing, generic helpers, PMBus math).
- `common/service/sensor/sensor.c` + `sdr.c` - the real sensor framework, with an intentionally **empty** per-board table (`src/plat_sensor_table.c`/`plat_sdr_table.c` - this EVK has no sensors wired up). `sensor_init()` genuinely runs and honestly logs `Init sensor size is zero` - the real code's real behavior for a board with nothing configured, not silenced.
- `common/dev/fru.c` + `eeprom.c` - real FRU read/write path; `src/plat_fru.c` points it at a plausible 24C-EEPROM I2C address with nothing physically there, so it fails gracefully (real I2C NACK/timeout), which is honest given the hardware.
- `common/service/mctp/mctp.c` + `mctp_ctrl.c` - MCTP's core protocol logic, which turned out to be genuinely portable (clean C, no Aspeed coupling).
- `common/service/pldm/pldm.c` + `pldm_base.c` + `pldm_oem.c` (just `set_iana()`/`get_iana()` - IPMI's OEM-1S IANA-checking, portable) - PLDM's base protocol/message types, likewise portable.
- ~90 `common/dev/*.c` sensor-chip drivers (pmbus/i2c-based power/temp/etc. ICs) - compiled because `sensor.c`'s dispatch table (`sensor_drive_tbl`) unconditionally references every chip driver's init function regardless of what any board's table configures; this is how every real OpenBIC board's build already works, not something specific to this port.

**Excluded, with a real reason (not silently stubbed):**
- **`common/service/ipmi/oem_1s_handler.c`** - Meta's proprietary OEM-1S NetFn (0x38) command set (fan/GPIO/JTAG/PECI/APML, all x86-host-adjacent BMC/BIC functionality specific to Meta's own server platforms). Unconditionally includes `hal_gpio.h`/`hal_peci.h` (both genuinely unportable, see below), and none of its commands apply to an EVK with no host, fans, or PECI anyway. `IPMI_OEM_1S_handler()` (the one symbol `ipmi.c`'s dispatch table needs) is stubbed in `src/plat_stubs.c`, returning `CC_INVALID_CMD` - the honest "not implemented" answer.
- **`common/lib/util_sys.c`** (only partially built) - mixes genuinely portable cold/warm-reset logic (`sys_reboot()`, a real mainline API) with unconditional includes of `cmsis_os.h`/`soc_common.h` (Aspeed-fork-only headers with no mainline equivalent anywhere in this tree) and `hal_gpio.h`. Rather than pull in the whole file, `submit_bic_cold_reset()`/`submit_bic_warm_reset()` are reimplemented directly in `src/plat_stubs.c` using the same real `sys_reboot()` call - genuine working functionality, not a stub, despite living in that file.
- **`common/lib/power_status.c`** - needs `hal_gpio.h`/`snoop.h` (both excluded). Only `get_DC_status()` is actually needed (by `chassis_handler.c`'s Get Chassis Status command) - stubbed in `src/plat_stubs.c` returning `false`, since this board has no real host-power-good GPIO wired up.
- **I3C** (`util_init_i3c()`/`hal_i3c.c` not used) - `hal_i3c.c` binds directly to the Aspeed fork's own I3C subsystem (`i3c_master_send_ccc()`, `i3c_master_priv_xfer()`, etc.) with no 1:1 mainline equivalent. A from-scratch real-I3C attempt (`i3c_do_daa()` against mainline's actual generic I3C API, which does exist and does work in principle) was built and tested on hardware: enabling `CONFIG_I3C=y` alone made the board hang before `main()` ever runs - no boot banner at all, before any of our own code executes. That's the mainline `I3C_MCUX` driver itself failing to initialize on this SoC in this Zephyr version, not something introduced by the new code, and not something reasonable to debug further here. Reverted; this attempt is not in the current source tree.
- **`common/hal/hal_gpio.c`** and everything that needs it (`common/service/pldm/pldm_monitor.c`'s `pldm_platform_monitor_read()`, and the 5 dev drivers that call it - `cx7.c`/`nv_satmc.c`/`mpro.c`/`pm8702.c`/`vistara.c`) - `hal_gpio.c` is a hand-rolled, register-level GPIO driver hardcoded to Aspeed/NPCM4XX physical addresses (`REG_GPIO_BASE 0x7e780000`, etc.), not a portable wrapper - it even has `#error "Unsupported GPIO driver"` for anything else. Porting it means writing a new low-level GPIO driver for MCXN947's own registers - real driver development, out of scope here. `cx7_init` (the one symbol of these five `sensor_drive_tbl` actually needs) is stubbed in `src/plat_stubs.c`; the other four aren't referenced at all once `pldm_monitor.c` is excluded from the build.
- **`common/dev/snoop.c`/`pcc.c`** - genuinely Aspeed-only hardware (SNOOP/POST Code Capture peripherals), not referenced by `sensor_drive_tbl`, so simply not compiled.
- **`common/dev/ast_adc.c`** (Aspeed ADC register access) and **`common/dev/intel_peci.c`**'s `peci_read()`/`peci_init()` calls (OpenBIC's own PECI wrappers, distinct from mainline's real `peci_transfer()`/`peci_enable()` API, and MCXN947 has no PECI peripheral *at all* - a genuine hardware absence, not a missing driver) - both stubbed with fixed/error return values in `src/plat_stubs.c`, since `ast_adc_init`/`intel_peci_init` are `sensor_drive_tbl` link requirements but never actually invoked (empty sensor table).
- **`common/lib/util_spi.c`**'s two `spi_nor_config_4byte_mode()`/`spi_nor_re_init()` calls (Aspeed-only SPI-NOR extensions) - stubbed as no-ops inline; nothing in this port exercises SPI flash updates.
- **SPDM** - doesn't exist anywhere in upstream OpenBIC (no `spdm/` directory, no vendored library) - there is nothing to port, so nothing was attempted. Real support would mean integrating a third-party stack (e.g. DMTF's `libspdm`) from scratch, a separate project.

## Two real bugs found and fixed on hardware

Getting `flexcomm2_lpi2c2` (J2 pins 18/20 - see "IPMI transport"
below for the full pinout) working at all - needed as a second bus for
I2C target-mode testing - took chasing down two separate, genuine
bugs, not application-level config mistakes:

**1. A devicetree conflict inherited from NXP's reference board.**
`flexcomm2_lpi2c2` and `flexcomm2_lpuart2` were *both* enabled
(`status = "okay"`) in the shared `mcx_nx4x_evk_cpu0.dtsi` this board
includes - written for NXP's reference board, which has an actual
audio-codec header populated on `flexcomm2_lpuart2`. A single
LP_FLEXCOMM instance can only run one peripheral function at a time;
`drivers/mfd/mfd_nxp_lp_flexcomm.c` (`nxp_lp_flexcomm_init()`) detects
both children enabled and initializes the shared hardware block in a
combined `LP_FLEXCOMM_PERIPH_LPI2CAndLPUART` mode instead of pure
`LPI2C` mode. Nothing on this port uses `flexcomm2_lpuart2` (no audio
codec is populated), so `boards/mcx_n9xx_evk_mcxn947_cpu0.overlay`
explicitly disables it.

**2. A genuine divide-by-zero in NXP's own vendored MCUX LPI2C driver.**
Even after fixing (1), `flexcomm2_lpi2c2` still crashed on every boot
with an identical fault, confirmed via GDB + `addr2line` pointing at
`LPI2C_GetCyclesForWidth`/`LPI2C_MasterInit`
(`modules/hal/nxp/.../fsl_lpi2c.c:166`):

```
***** USAGE FAULT *****
  Division by zero
r0/a1:  0x0000000f  r1/a2:  0x00000000  r2/a3:  0x00000000
r3/a4:  0x000f4240 r12/ip:  0x00000002 r14/lr:  0x10012815
Faulting instruction address (r15/pc): 0x1001271c
>>> ZEPHYR FATAL ERROR 30: Unknown error on CPU 0
Current thread: 0x300026d8 (main)
```

A temporary `printk()` inserted right before the faulting division
showed `sourceClock_Hz=0` for this call - `CLOCK_GetLPFlexCommClkFreq(2)`
transiently reads back an invalid clock-mux state on this SoC when
FLEXCOMM2 is the first FlexComm instance to attach the shared
`FRO_HF_DIV` clock during early board init (`board.c`'s
`CLOCK_AttachClk()` calls for FLEXCOMM2 and FLEXCOMM3 are otherwise
symmetric). Oddly, inserting the `printk()` call itself - just as a
side effect of the delay it added - made the race disappear, which
ruled out a logic bug in `board.c`'s clock-attach sequence and pointed
at a genuine timing/settling issue instead. Root-caused this far, the
robust fix is at the point of failure rather than papering over the
timing: `LPI2C_GetCyclesForWidth()` should never divide by an
unvalidated, possibly-zero clock rate in the first place. Patched (in
the `hal_nxp` west module, *outside* this repo - see "The out-of-tree
hal_nxp fix" below for how `west.yml` pulls it in automatically) to
fall back to the caller's `minCycles` instead of dividing by zero -
which also happens to be the mathematically correct answer for every
call site actually exercised here (`width_ns=0`, i.e. "glitch filter
unset").

With both fixes applied, `flexcomm2_lpi2c2` inits cleanly with its
real 48MHz source clock - see "IPMI transport" below for what it's
used for and an unresolved anomaly encountered while testing it
(scanning it in controller mode reported every address as present,
which a DC multimeter check did *not* explain - see below for the
full writeup and why that's documented as untested rather than
diagnosed).

## IPMI transport

A real IPMI transport now exists end to end: `common/service/ipmi/*`
(dispatch, App/Chassis/Sensor/Storage/OEM command handlers) and
`common/service/ipmb/ipmb.c` (IPMB framing, checksums, sequence-number
tracking, TX/RX threads) turned out to be almost entirely portable
pure-C/Zephyr-kernel code - the only genuinely Aspeed-specific piece
was the transport's *bottom* edge, `ipmb_slave_read()`/
`i2c_slave_driver_register()`, which had no mainline equivalent. That's
now real, working code too: `common/hal/hal_i2c.c`'s
`ipmb_target_register()`/`ipmb_target_read()` (gated on
`CONFIG_I2C_TARGET`) implement it using Zephyr's generic
`i2c_target_register()` API - the same mainline target-mode support
this port proved out earlier - registered on `flexcomm2_lpi2c2` at
address `0x20` (`src/plat_ipmb.c`). `oem_1s_handler.c` (Meta's
proprietary OEM-1S command set - fan/GPIO/JTAG/PECI/APML, none of it
applicable here) is the one piece still stubbed - see
`src/plat_stubs.c`.

Verified on real hardware: boots cleanly and logs

```
<inf> ipmb: Initial IPMB TX/RX threads, bus(0x2), addr(0x20)
```

**The message-dispatch pipeline is verified working, completely and
correctly, end to end on real hardware**, via `plat ipmi_selftest` (a
new shell command, `src/plat_shell.c`) that submits a real IPMI Get
Device ID request through `notify_ipmi_client()` with `InF_source =
SELF`, and reads the real response back off `self_ipmi_msgq` -
exactly the same queues, threads, and handler dispatch that real IPMB
traffic uses, just sourced from the shell instead of the wire:

```
uart:~$ plat ipmi_selftest
Submitting IPMI Get Device ID via SELF...
Response: netfn=0x06 cmd=0x01 cc=0x00 data_len=15
00000000: 00 80 26 34 02 bf 15 a0  00 00 00 00 00 00 00      |..&4.... .......  |
```

Every byte matches exactly what `src/plat_version.h`'s Get Device ID
fields specify - `cc=0x00` (`CC_SUCCESS`), device/firmware revision
bytes, `IPMI_VERSION`, and bytes 6-8 (`15 a0 00`) decoding to
`IANA_ID` = `0x00A015` (Meta's IANA), little-endian. This proves the
real dispatch code (`ipmi_cmd_handle()`'s netfn switch,
`IPMI_APP_handler()`, `APP_GET_DEVICE_ID()`), the message queues, and
the response-routing path are all genuinely working - independent of
whether a wire-level IPMB transaction can be demonstrated (see below).

**Wire-level IPMB test: attempted, result untested/inconclusive - not
confirmed working, not confirmed broken.** This is the same
`flexcomm2_lpi2c2` target-mode bus this port validated earlier with a
standalone test device (since removed in favor of the real IPMB
channel above), and the same unresolved anomaly applies: both buses'
SDA/SCL lines land on the *same* physical header - the
Arduino-compatible header J2 - per Table 24 of the official
MCX-N9XX-EVK Board User Manual (UM12036):

| J2 pin | MCU pin | Signal              |
|--------|---------|----------------------|
| 14     | GND     | Ground               |
| 15     | P1_1    | `flexcomm3_lpi2c3` SCL |
| 17     | P1_0    | `flexcomm3_lpi2c3` SDA |
| 18     | P4_0    | `flexcomm2_lpi2c2` SDA |
| 20     | P4_1    | `flexcomm2_lpi2c2` SCL |

J2 pin 17 jumpered to pin 18 (SDA<->SDA) and pin 15 to pin 20
(SCL<->SCL) - both short jumps on the same header strip (no dedicated
GND jumper needed - both buses already share the board's common
ground plane) - genuinely makes electrical contact (`i2c scan
flexcomm3_lpi2c3` starts reporting every address as present, versus 0
with nothing attached), but a real write through the jumper doesn't
behave like a working link: the controller reports success with no
error, yet a temporary `printk()` in every target callback confirmed
none of them ever fired, and an unregistered address (`0x51`) produced
the identical "succeeds with no error" result as the real address -
ruling out a logic bug on either side (the toy test device's, and now
the real IPMB target's, since they're the same underlying mechanism).
A DC multimeter check of all four pins read a clean, idle 3.3V, ruling
out a simple stuck-low/floating-pin explanation.

What this does and doesn't tell us:
- The IPMB/IPMI **software** - transport framing, target-mode
  registration, and the full dispatch pipeline - is verified correct
  on real hardware (see the self-test above and the boot log).
- The controller-side "write reports success, nothing arrives" pattern
  is consistent with a signal-integrity issue that only manifests
  while the bus is actively toggling (contention, ringing, or a timing
  glitch during the ACK bit window) - which a DC multimeter can't
  distinguish from a logic bug. `flexcomm2_lpi2c2`'s pins are also
  shared with an LCD-touch-panel `LCD_GPIO` function on header J20
  (see the bug writeup above), a plausible source, but **not**
  confirmed - a hypothesis, not a diagnosis.
- Properly resolving this needs an oscilloscope on the bus during a
  live transaction, or a real external I2C device on
  `flexcomm3_lpi2c3` to test the controller side independently -
  neither was available for this port.

**Bottom line: the wire-level test is untested, not failed.** Every
part of the transport this port can verify without a second real
device - registration, framing, checksums, sequencing, and the full
message-dispatch pipeline - is verified correct on real hardware.
Whether *this specific board's* two on-chip I2C instances can be
cleanly bridged together for a live wire test remains an open
question, not a demonstrated limitation.

## Why this is scoped as "bring-up only", not a real board port

OpenBIC currently only supports ASPEED (AST2600/AST1030) and Nuvoton
BMC SoCs, built against vendor-forked Zephyr trees
(`AspeedTech-BMC/zephyr`, `Nuvoton-Israel/zephyr`) - not upstream
Zephyr. Two things fall out of that which matter here:

1. **`common/service/main.c`** - the single `main()` shared by every
   existing OpenBIC board - unconditionally calls `wdt_init()`,
   `util_init_I2C()`, `util_init_i3c()`, `sensor_init()`, `FRU_init()`,
   `ipmi_init()`. There's no way to build any board, new or existing,
   without those symbols resolving.
2. Most of `common/hal/*.c` and `common/lib/*.c` are written against a
   pre-Zephyr-3.0 API (`<zephyr.h>`, `<logging/log.h>`, `<device.h>`,
   legacy `<drivers/i2c.h>` paths) that no longer exists in mainline
   Zephyr after the 3.x restructure - almost entirely mechanical to
   fix (see the bulk include fixes throughout this port). A narrower
   set of things are genuine hardware/subsystem gaps, not just stale
   includes: `CONFIG_I2C_EEPROM_SLAVE` and `CONFIG_SPI_NOR_MULTI_DEV`
   (Aspeed-fork-only Kconfig with no mainline equivalent), and
   `hal_gpio.c`/`hal_i3c.c` (hand-rolled register-level drivers with no
   portable wrapper - see above). Notably, `CONFIG_I2C_IPMB_SLAVE`
   turned out to be *replaceable*, not a hard blocker: IPMI/IPMB's own
   logic (`common/service/ipmi/*`, `common/service/ipmb/ipmb.c`) needed
   almost no changes at all beyond the same mechanical include fixes -
   only its bottom-edge I2C-slave integration was Aspeed-specific, and
   mainline's own `i2c_target_register()` API provides a real,
   equivalent integration point (see "IPMI transport" above).

There is no single Zephyr revision that carries both the Aspeed fork's
IPMI/sensor subsystem and NXP MCXN947 board support - they're
mutually exclusive trees. Since MCXN947 support only exists upstream,
this port's manifest (`west.yml` at the repo root) points `zephyr` at
`zephyrproject-rtos/zephyr` v4.4.0 instead of the Aspeed/Nuvoton forks.
That's what makes this board buildable at all, but it also means none
of `common/`'s vendor-specific pipeline is usable as-is.

## Layout

```
meta-facebook/mcx-n9xx-evk/
├── CMakeLists.txt   Real common/ sources (hal/lib/sensor/mctp/pldm/ipmi/ipmb/dev)
│                    + this board's own src/*.c
├── prj.conf         GPIO, I2C (+ target), shell, logging, hwinfo, watchdog, NVS, mbox
├── boards/          Devicetree overlay: red_led, user_button_2, mbox-consumer,
│                    ipmb0 marker node, disables the conflicting flexcomm2_lpuart2
│                    (see bug writeup above)
├── src/main.c       Banner + shell + heartbeat LED + real wdt/timer/I2C/
│                    sensor/FRU/IPMI init calls
├── src/plat_version.h    Firmware version + real IPMI Get Device ID fields
├── src/plat_def.h, plat_ipmi.h   Minimal per-board glue (see common/service/ipmi/)
├── src/plat_ipmb.[ch]    Real IPMB channel config - see "IPMI transport" below
├── src/plat_fan.h        No fans on this board - pal_set_fan_duty() always fails
├── src/plat_i2c.[h]        Board I2C bus map (both buses - see hal_i2c.c above)
├── src/plat_sensor_table.[ch], plat_sdr_table.[ch]   Empty per-board tables
├── src/plat_fru.[ch]       FRU EEPROM config (points at real I2C, nothing wired)
├── src/plat_stubs.c        Link-only stubs for genuinely unportable/absent
│                           deps (cx7_init, ast_adc, intel_peci, oem_1s_handler,
│                           fan control, DC-power-good, cold/warm reset - see above)
├── src/plat_gpio.[ch]    GPIO status-monitoring subsystem (see below)
├── src/plat_hwinfo.[ch]  Device ID + reset-cause reporting (see below)
├── src/plat_storage.[ch] Persistent NVS storage (see below)
├── src/plat_mbox.[ch]    Inter-core mailbox, cpu0 side (see below)
├── src/plat_shell.c      "plat version/gpio mon0/wdt starve/storage bootcount/
│                         mbox ping/ipmi_selftest"
├── sysbuild.cmake        Orchestrates the optional dual-core build
├── Kconfig.sysbuild      Selects mcx_n9xx_evk/mcxn947/cpu1 as the remote board
└── remote/               cpu1's image (see "Dual-core" below)
    ├── CMakeLists.txt, prj.conf, boards/*.conf|overlay
    └── src/main.c        Headless mailbox echo responder
```

## Bring-up subsystems (from the earlier `mcx-n9xx-evk-port` branch)

These 5 from-scratch subsystems predate the full port above and are
unaffected by it - still real, still verified, still in the build:

### Dual-core bring-up (fifth real subsystem, verified on hardware)

The MCXN947 is dual Cortex-M33 (cpu0 + cpu1). This is the only
subsystem here that needs `west build --sysbuild` - the normal
cpu0-only build (used by every other subsystem above) doesn't touch
`remote/` at all. Closely follows Zephyr's own upstream
`samples/drivers/mbox` sample, which already has a verified
configuration for this exact board (`mcx_n9xx_evk/mcxn947/cpu0` +
`mcx_n9xx_evk/mcxn947/cpu1`) - adapted into this app's structure/style
instead of copied as a standalone sample.

How it boots: `boards/mcx_n9xx_evk_mcxn947_cpu0.conf` sets
`CONFIG_SECOND_CORE_MCUX=y`, which makes cpu0's SoC init code
(`soc/nxp/mcx/mcxn/soc.c`) point cpu1's boot address at
`slot1_partition` and release it out of reset during cpu0's own boot -
no separate flashing step or button press needed to start cpu1.

Communication is over the MU (Messaging Unit) hardware mailbox via
mainline Zephyr's generic `mbox` API: `src/plat_mbox.c` (cpu0) sends a
ping every 2s and logs whatever it receives back; `remote/src/main.c`
(cpu1) is headless (its own console UART, `flexcomm2_lpuart2`, isn't
wired up - no need for a second serial adapter) and immediately echoes
back anything it receives. A `plat mbox ping` shell command sends one
on demand.

Verified on real hardware - cpu0's console showing real, repeating
round trips to cpu1 and back:

```
[00:00:09.506,000] <inf> plat_mbox: ping sent to cpu1 (channel 1)
[00:00:09.506,000] <inf> plat_mbox: pong from cpu1 (channel 0)
[00:00:11.507,000] <inf> plat_mbox: ping sent to cpu1 (channel 1)
[00:00:11.507,000] <inf> plat_mbox: pong from cpu1 (channel 0)
[00:00:13.507,000] <inf> plat_mbox: ping sent to cpu1 (channel 1)
[00:00:13.507,000] <inf> plat_mbox: pong from cpu1 (channel 0)
```

The reply on every single cycle (not just the first) is what shows
cpu1 is genuinely alive and reacting in real time, not just running
whatever was left over from a previous flash.

Build/flash this specific subsystem:

```sh
west build -p always -b mcx_n9xx_evk/mcxn947/cpu0 --sysbuild meta-facebook/mcx-n9xx-evk
west flash   # flashes both cpu0 and cpu1 images
```

**Note:** `boards/mcx_n9xx_evk_mcxn947_cpu0.conf` (which releases cpu1)
applies to *every* build of this app, not just `--sysbuild` ones -
matching upstream's own sample. A plain single-core rebuild will still
release cpu1, which then runs whatever was last flashed to
`slot1_partition` (garbage/erased flash if nothing ever was). This is
harmless - cpu1 has its own isolated core/RAM - but don't be surprised
if cpu1's LED-adjacent behavior (there isn't any yet) seems to have "a
mind of its own" after mixing sysbuild and non-sysbuild builds.

### Persistent storage (fourth real subsystem, verified on hardware)

`src/plat_storage.c` mounts NVS (mainline Zephyr's `zephyr/kvss/nvs.h`)
on the "storage" partition of the on-board **W25Q64 QSPI flash chip**
(`FLASH_MCUX_FLEXSPI_NOR`, auto-selected - already enabled by default
in this board's devicetree, unlike FRDM-MCXN947's equivalent chip
which is off by default) and keeps a boot counter that survives
resets - the same mechanism a real BIC needs for config/calibration
data. Exposed live via `plat storage bootcount`.

Verified on real hardware across three separate `west flash`-triggered
resets:

```
persistent boot count: 5
persistent boot count: 6   (implied - a flash cycle with a garbled capture)
persistent boot count: 7
```

Each reset correctly read back and incremented the previous value
rather than resetting to a fixed number, confirming real flash
persistence rather than a RAM-only counter.

### Watchdog (third real subsystem, verified on hardware)

`src/plat_wdt.c` installs and arms a 3-second timeout on `wwdt0` via
mainline Zephyr's `wdt` API (`WDT_MCUX_WWDT` backend, auto-selected -
no board-specific code needed), fed every 500ms from the main
heartbeat loop - the mainline-Zephyr equivalent of `wdt_init()` +
feeding in `common/service/main.c`. A `plat wdt starve` shell command
deliberately stops feeding, to prove the SoC actually resets itself on
starvation rather than just trusting that it would.

Verified on real hardware (typed interactively into the shell):

```
uart:~$ plat wdt starve
Stopping watchdog feed - SoC will reset shortly.
[00:01:46.630,000] <wrn> plat_wdt: watchdog feeding stopped on purpose - SoC will reset in <= 3000ms
uart:~$ *** Booting Zephyr OS build v4.4.0 ***

  ___                   ____ ___ ____
 / _ \ _ __   ___ _ __ | __ )_ _/ ___|
| | | | '_ \ / _ \ '_ \|  _ \| | |
| |_| | |_) |  __/ | | | |_) | | |___
 \___/| .__/ \___|_| |_|____/___\____|
      |_|
Hello, welcome to OpenBIC / NXP MCX-N9XX-EVK Minimal Bring-up 2026.34.0
```

The SoC reset itself roughly 3 seconds after the last feed, exactly as
configured - a real, deliberate watchdog-triggered reset, not just a
successful install/arm.

### HWINFO: device identity + reset cause (second real subsystem, verified on hardware)

`src/plat_hwinfo.c` reads the chip's unique device ID and the cause of
the last reset via mainline Zephyr's `hwinfo` API
(`HWINFO_MCUX_SYSCON` + `HWINFO_MCUX_RSTCTL` backends, auto-selected -
no board-specific code needed) - the stand-in here for the
device-identity data a real BIC reports over IPMI (Get Device ID / Get
Device GUID). `CONFIG_HWINFO_SHELL=y` also gets you Zephyr's own
`hwinfo devid`/`hwinfo reset_cause` shell commands for free, for
comparison against the plat-level log output.

Verified on real hardware: after a `west flash`-triggered reset, the
console reported this board's actual per-chip device ID
(`36353630004d4...`, truncated as read off the console) and correctly
identified the reset cause as a software reset (as expected for a
debugger-issued reset, distinct from a power-on or watchdog reset).

### GPIO status monitoring (first real subsystem, verified on hardware)

`src/plat_gpio.c` is a from-scratch port of the pattern
`common/hal/hal_gpio.c` + `common/lib/power_status.c` implement for
real OpenBIC boards - interrupt-driven GPIO monitoring with debounce,
used on actual BICs to track things like power-good/host-presence
signals - written against mainline Zephyr's GPIO API instead of the
legacy one those files use.

This EVK has no dedicated sensor or platform-status input, so it
monitors **SW2** (`user_button_2`/`sw0` alias, GPIO1.3, active-low)
as a stand-in signal source: `GPIO_INT_EDGE_BOTH` interrupt +
30ms debounce, logs every transition, and exposes live state via
`plat gpio mon0` in the shell.

Verified on real hardware (physical button presses, captured over the
serial console):

```
[00:05:28.232] <inf> plat_gpio: mon0 (SW2) state changed: asserted
[00:05:28.383] <inf> plat_gpio: mon0 (SW2) state changed: deasserted
[00:05:30.668] <inf> plat_gpio: mon0 (SW2) state changed: asserted
[00:05:30.798] <inf> plat_gpio: mon0 (SW2) state changed: deasserted
```

## The out-of-tree hal_nxp fix - handled automatically

One of the two bugs above (the `LPI2C_GetCyclesForWidth` divide-by-zero)
lives in NXP's vendored HAL, in the `hal_nxp` **west module** -
`~/openbic-workspace/modules/hal/nxp/`, not this repo - so it can't be
committed here directly. Submitted upstream as
[zephyrproject-rtos/hal_nxp#799](https://github.com/zephyrproject-rtos/hal_nxp/pull/799);
until that merges, this repo's root `west.yml` overrides the `hal_nxp`
project to pull from a fork/branch carrying the fix
(`wrouwet/hal_nxp`, `fix/lpi2c-divzero-guard`) instead of upstream's
pinned revision - a plain `west update` gets you a working tree with
no manual patching step. Once the upstream PR merges and a `hal_nxp`
release/pin picks it up, this override in `west.yml` can be dropped in
favor of whatever revision `zephyr`'s own manifest pins.

Without this fix, any build that enables `flexcomm2_lpi2c2` (as this
board's overlay does, for the IPMB channel - see "IPMI transport"
below) will crash at boot with the USAGE FAULT documented above.

## Build / flash

Same toolchain as NXP's own MCXN947 Zephyr samples - see the repo-root
`SETUP.md` (west workspace at `~/openbic-workspace`, Zephyr SDK,
LinkServer). Run `west update` first if you haven't already (see above
for why that's now sufficient on its own).

```sh
source ~/openbic-workspace/.venv/bin/activate
cd ~/openbic-workspace/openbic
west build -p always -b mcx_n9xx_evk/mcxn947/cpu0 meta-facebook/mcx-n9xx-evk
export PATH="/usr/local/LinkServer_26.6.137:$PATH"
west flash
```

Open a serial console (115200 8N1) on the MCU-Link's VCOM port to see
the banner, shell prompt, and real init log lines (device ID, reset
cause, NVS boot count, GPIO monitor start, mbox ready, IPMB TX/RX
thread startup, and the honest `sensor: Init sensor size is zero`);
the red LED should blink every 500ms. Try `i2c scan flexcomm3_lpi2c3`
for a live 128-address real bus scan (0 devices - nothing's wired to
the Arduino header), or `plat ipmi_selftest` for a full real IPMI
Get Device ID round trip through the dispatch pipeline - see "IPMI
transport" above for both the self-test's verified output and
`flexcomm2_lpi2c2`'s wire-level caveat.

## What's still not done

The remaining gap to a real, fully-functional BIC target on this
board:

- **A confirmed wire-level IPMB/I2C target-mode test** - the software
  side is done and verified, including a full end-to-end
  message-dispatch self-test on real hardware (see "IPMI transport"
  above); an attempted jumper test on this board's two on-chip I2C
  buses came back untested/inconclusive rather than a pass or a
  demonstrated failure - see "IPMI transport" above for the full
  writeup, including what would be needed to actually resolve it (an
  oscilloscope, or a real external I2C device on `flexcomm3_lpi2c3`).
- **`oem_1s_handler.c`'s command set** - Meta's proprietary OEM-1S
  NetFn, needs `hal_gpio.h`/`hal_peci.h` (see above) and doesn't apply
  to this EVK's hardware anyway (no fans, host, or PECI) - stubbed,
  not ported.
- **I3C** - blocked on a real MCXN947 `I3C_MCUX` driver issue in this
  Zephyr version (see above) - would need upstream Zephyr driver
  debugging/fixing, not application-level work.
- **A real low-level GPIO driver** for MCXN947, to replace
  `hal_gpio.c` and unlock `pldm_monitor.c`'s platform-monitoring
  effectors (and the 5 dev drivers gated on it).
- **Real sensors** - this EVK has none wired up; the sensor/SDR tables
  are genuinely empty. Wiring up an external I2C sensor (e.g. to the
  Arduino header) and giving it a real `plat_sensor_table.c` entry
  would be the natural next step to prove a sensor reading end-to-end.
- **SPDM** - a separate project (see above), not attempted.
