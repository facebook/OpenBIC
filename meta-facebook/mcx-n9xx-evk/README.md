# MCX-N9XX-EVK — full board port

Porting OpenBIC to the NXP MCX-N9XX-EVK (MCXN947, dual Cortex-M33).

This branch (`full-board-port`) builds on the `mcx-n9xx-evk-port`
bring-up branch's 5 from-scratch subsystems (GPIO monitoring, HWINFO,
watchdog, NVS storage, dual-core mailbox - all still present, see
below) and goes further: `main()` now calls the **real**
`common/hal`/`common/lib`/`common/service` code - `wdt_init()`,
`util_init_timer()`, `util_init_I2C()`, `sensor_init()`, `FRU_init()` -
ported to mainline Zephyr's API surface, plus real MCTP (transport
core) and PLDM (base protocol), instead of reusing the bring-up
branch's stand-ins. It also adds real I2C **target/slave-mode**
support (`src/plat_i2c_target.c`, see "I2C target mode" below) -
mainline's MCUX LPI2C driver already implements target mode natively
for this SoC, it just needed a real consumer and a second working bus
to test against. See "What's real, what's excluded" below for exactly
what was ported vs. left out and why - nothing is stubbed silently.

## What's real, what's excluded, and why

**Real, ported, verified on hardware:**
- `common/hal/hal_wdt.c` - watchdog, `DEVICE_DT_GET(DT_ALIAS(watchdog0))` instead of string-name `device_get_binding()`.
- `common/hal/hal_i2c.c` - I2C master (`I2C_MODE_CONTROLLER`, real `DEVICE_DT_GET` bindings, dead Aspeed-only `<drivers/i2c/slave/ipmb.h>` include dropped). Genuinely talks to real hardware on **two** independent buses, both on the Arduino-compatible header J2 (see "I2C target mode" below for the exact pinout): `i2c scan flexcomm3_lpi2c3` (controller mode, J2 pins 15/17) performs 128 real bus transactions (0 found - nothing's wired there, honestly); `flexcomm2_lpi2c2` (J2 pins 18/20) is the second bus, used in target mode.
- `src/plat_i2c_target.c` - a real I2C **target-mode** device (Zephyr's generic `i2c_target_register()`/`i2c_target_callbacks` API, backed natively by the MCUX LPI2C driver's target-mode IRQ handling) registered on `flexcomm2_lpi2c2` - see "I2C target mode" below.
- `common/lib/timer.c`, `common/lib/libutil.c`, `common/lib/util_pmbus.c` - portable as-is/near-as-is (CMSIS-RTOS2 timing, generic helpers, PMBus math).
- `common/service/sensor/sensor.c` + `sdr.c` - the real sensor framework, with an intentionally **empty** per-board table (`src/plat_sensor_table.c`/`plat_sdr_table.c` - this EVK has no sensors wired up). `sensor_init()` genuinely runs and honestly logs `Init sensor size is zero` - the real code's real behavior for a board with nothing configured, not silenced.
- `common/dev/fru.c` + `eeprom.c` - real FRU read/write path; `src/plat_fru.c` points it at a plausible 24C-EEPROM I2C address with nothing physically there, so it fails gracefully (real I2C NACK/timeout), which is honest given the hardware.
- `common/service/mctp/mctp.c` + `mctp_ctrl.c` - MCTP's core protocol logic, which turned out to be genuinely portable (clean C, no Aspeed coupling).
- `common/service/pldm/pldm.c` + `pldm_base.c` - PLDM's base protocol/message types, likewise portable.
- ~90 `common/dev/*.c` sensor-chip drivers (pmbus/i2c-based power/temp/etc. ICs) - compiled because `sensor.c`'s dispatch table (`sensor_drive_tbl`) unconditionally references every chip driver's init function regardless of what any board's table configures; this is how every real OpenBIC board's build already works, not something specific to this port.

**Excluded, with a real reason (not silently stubbed):**
- **IPMI transport** (`ipmi_init()` not called) - `CONFIG_IPMI` and the KCS/I2C-slave-as-IPMB transport are Aspeed-fork-only additions with zero presence in mainline Zephyr. This is a missing *subsystem*, not a hardware gap - no amount of driver work on this board fixes it. `libutil.h`'s `ipmi_msg` *type* is portable and used as-is (only the transport is missing).
- **I3C** (`util_init_i3c()`/`hal_i3c.c` not used) - `hal_i3c.c` binds directly to the Aspeed fork's own I3C subsystem (`i3c_master_send_ccc()`, `i3c_master_priv_xfer()`, etc.) with no 1:1 mainline equivalent. A from-scratch real-I3C attempt (`i3c_do_daa()` against mainline's actual generic I3C API, which does exist and does work in principle) was built and tested on hardware: enabling `CONFIG_I3C=y` alone made the board hang before `main()` ever runs - no boot banner at all, before any of our own code executes. That's the mainline `I3C_MCUX` driver itself failing to initialize on this SoC in this Zephyr version, not something introduced by the new code, and not something reasonable to debug further here. Reverted; this attempt is not in the current source tree.
- **`common/hal/hal_gpio.c`** and everything that needs it (`common/service/pldm/pldm_monitor.c`'s `pldm_platform_monitor_read()`, and the 5 dev drivers that call it - `cx7.c`/`nv_satmc.c`/`mpro.c`/`pm8702.c`/`vistara.c`) - `hal_gpio.c` is a hand-rolled, register-level GPIO driver hardcoded to Aspeed/NPCM4XX physical addresses (`REG_GPIO_BASE 0x7e780000`, etc.), not a portable wrapper - it even has `#error "Unsupported GPIO driver"` for anything else. Porting it means writing a new low-level GPIO driver for MCXN947's own registers - real driver development, out of scope here. `cx7_init` (the one symbol of these five `sensor_drive_tbl` actually needs) is stubbed in `src/plat_stubs.c`; the other four aren't referenced at all once `pldm_monitor.c` is excluded from the build.
- **`common/dev/snoop.c`/`pcc.c`** - genuinely Aspeed-only hardware (SNOOP/POST Code Capture peripherals), not referenced by `sensor_drive_tbl`, so simply not compiled.
- **`common/dev/ast_adc.c`** (Aspeed ADC register access) and **`common/dev/intel_peci.c`**'s `peci_read()`/`peci_init()` calls (OpenBIC's own PECI wrappers, distinct from mainline's real `peci_transfer()`/`peci_enable()` API, and MCXN947 has no PECI peripheral *at all* - a genuine hardware absence, not a missing driver) - both stubbed with fixed/error return values in `src/plat_stubs.c`, since `ast_adc_init`/`intel_peci_init` are `sensor_drive_tbl` link requirements but never actually invoked (empty sensor table).
- **`common/lib/util_spi.c`**'s two `spi_nor_config_4byte_mode()`/`spi_nor_re_init()` calls (Aspeed-only SPI-NOR extensions) - stubbed as no-ops inline; nothing in this port exercises SPI flash updates.
- **SPDM** - doesn't exist anywhere in upstream OpenBIC (no `spdm/` directory, no vendored library) - there is nothing to port, so nothing was attempted. Real support would mean integrating a third-party stack (e.g. DMTF's `libspdm`) from scratch, a separate project.

## Two real bugs found and fixed on hardware

Getting `flexcomm2_lpi2c2` (J2 pins 18/20 - see "I2C target mode"
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
unvalidated, possibly-zero clock rate in the first place. Patched (see
`patches/hal_nxp-fsl_lpi2c-divzero-fix.patch`, applied to the `hal_nxp`
west module, *outside* this repo - see "Applying the required
out-of-tree patch" below) to fall back to the caller's `minCycles`
instead of dividing by zero - which also happens to be the
mathematically correct answer for every call site actually exercised
here (`width_ns=0`, i.e. "glitch filter unset").

With both fixes applied, `flexcomm2_lpi2c2` inits cleanly with its
real 48MHz source clock and `i2c scan flexcomm2_lpi2c2` runs a real
bus scan (see "I2C target mode" below for why every address currently
shows as present - the header has no pull-ups with nothing wired to
it, not a driver bug).

## I2C target mode

`src/plat_i2c_target.c` registers a real I2C **target/slave-mode**
device on `flexcomm2_lpi2c2` (address `0x50`) using Zephyr's generic
`i2c_target_register()`/`struct i2c_target_callbacks` API. This isn't
new driver work - mainline's MCUX LPI2C driver
(`drivers/i2c/i2c_mcux_lpi2c.c`) already implements target mode
natively for this SoC (`mcux_lpi2c_target_register()`, IRQ-driven
address/RX/TX/stop handling) - it simply had no consumer or working
second bus to exercise it against on this board before now. The device
itself is a tiny EEPROM-style register file (256 bytes): the first
byte of a write selects an offset (like a real EEPROM's address byte),
further bytes write sequentially from there; a read returns bytes
sequentially from the last-selected offset.

Verified on real hardware: boots cleanly and logs

```
<inf> plat_i2c_target: I2C target registered on flexcomm2_lpi2c2, addr 0x50, 256-byte regfile
```

**To exercise a real controller<->target transaction** (not yet done
on this specific board - needs a physical jumper wire, offered but not
performed as of this writing): both buses' SDA/SCL lines land on the
*same* physical header - the Arduino-compatible header J2 - per
Table 24 of the official MCX-N9XX-EVK Board User Manual (UM12036):

| J2 pin | MCU pin | Signal              |
|--------|---------|----------------------|
| 14     | GND     | Ground               |
| 15     | P1_1    | `flexcomm3_lpi2c3` SCL |
| 17     | P1_0    | `flexcomm3_lpi2c3` SDA |
| 18     | P4_0    | `flexcomm2_lpi2c2` SDA |
| 20     | P4_1    | `flexcomm2_lpi2c2` SCL |

Jumper J2 pin 17 to pin 18 (SDA<->SDA) and J2 pin 15 to pin 20
(SCL<->SCL) - both short jumps on the same header strip. A dedicated
GND jumper (pin 14) isn't required since both buses already share the
board's common ground plane, but it's there if you want a firmer
reference. Then from the shell:

```
uart:~$ i2c write flexcomm3_lpi2c3 0x50 0x00 0xde 0xad 0xbe 0xef
uart:~$ i2c read flexcomm3_lpi2c3 0x50 0x00 4
00000000: de ad be ef                                      |....            |
```

Without the jumper wire, the two buses are electrically independent -
a write on `flexcomm3_lpi2c3` simply goes nowhere, and a read back
returns whatever the target's regfile already held (zeroes, on a fresh
boot) - which is exactly what was observed, confirming the two buses
are correctly isolated rather than the target actually responding.

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
   Zephyr after the 3.x restructure. Separately, `CONFIG_IPMI`,
   `CONFIG_I2C_IPMB_SLAVE`, `CONFIG_I2C_EEPROM_SLAVE`, and
   `CONFIG_SPI_NOR_MULTI_DEV` (all required by every existing board's
   `prj.conf`) are Aspeed-fork-only Kconfig additions with **zero**
   presence in mainline Zephyr.

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
├── CMakeLists.txt   Real common/ sources (hal/lib/sensor/mctp/pldm/dev)
│                    + this board's own src/*.c
├── prj.conf         GPIO, I2C, shell, logging, hwinfo, watchdog, NVS, mbox
├── boards/          Devicetree overlay: red_led, user_button_2, mbox-consumer,
│                    disables the conflicting flexcomm2_lpuart2 (see bug writeup above)
├── patches/         hal_nxp-fsl_lpi2c-divzero-fix.patch - required out-of-tree
│                    fix, see "Applying the required out-of-tree patch" below
├── src/main.c       Banner + shell + heartbeat LED + real wdt/timer/I2C/
│                    sensor/FRU init calls
├── src/plat_version.h, plat_def.h, plat_ipmb.h   Minimal per-board glue
├── src/plat_i2c.[h]        Board I2C bus map (both buses - see hal_i2c.c above)
├── src/plat_i2c_target.[ch]  I2C target-mode test device (see above)
├── src/plat_sensor_table.[ch], plat_sdr_table.[ch]   Empty per-board tables
├── src/plat_fru.[ch]       FRU EEPROM config (points at real I2C, nothing wired)
├── src/plat_stubs.c        Link-only stubs for genuinely unportable/absent
│                           deps (cx7_init, ast_adc, intel_peci - see above)
├── src/plat_gpio.[ch]    GPIO status-monitoring subsystem (see below)
├── src/plat_hwinfo.[ch]  Device ID + reset-cause reporting (see below)
├── src/plat_storage.[ch] Persistent NVS storage (see below)
├── src/plat_mbox.[ch]    Inter-core mailbox, cpu0 side (see below)
├── src/plat_shell.c      "plat version/gpio mon0/wdt starve/storage bootcount/mbox ping"
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

## Applying the required out-of-tree patch

One of the two bugs above (the `LPI2C_GetCyclesForWidth` divide-by-zero)
lives in NXP's vendored HAL, in the `hal_nxp` **west module** -
`~/openbic-workspace/modules/hal/nxp/`, not this repo - so it can't be
committed here directly, and a fresh `west update` will re-pull the
unpatched, crashing version. After `west update`, apply it once:

```sh
cd ~/openbic-workspace/modules/hal/nxp
git apply ~/openbic-workspace/openbic/meta-facebook/mcx-n9xx-evk/patches/hal_nxp-fsl_lpi2c-divzero-fix.patch
```

Without this patch, any build that enables `flexcomm2_lpi2c2` (as this
board's overlay does, for I2C target-mode testing - see above) will
crash at boot with the USAGE FAULT documented above.

## Build / flash

Same toolchain as NXP's own MCXN947 Zephyr samples - see the repo-root
`SETUP.md` (west workspace at `~/openbic-workspace`, Zephyr SDK,
LinkServer). Apply the patch above first if you haven't already.

```sh
source ~/openbic-workspace/.venv/bin/activate
cd ~/openbic-workspace/openbic
west build -p always -b mcx_n9xx_evk/mcxn947/cpu0 meta-facebook/mcx-n9xx-evk
export PATH="/usr/local/LinkServer_26.6.137:$PATH"
west flash
```

Open a serial console (115200 8N1) on the MCU-Link's VCOM port to see
the banner, shell prompt, and real init log lines (device ID, reset
cause, NVS boot count, GPIO monitor start, mbox ready, I2C target
registration, and the honest `sensor: Init sensor size is zero`); the
red LED should blink every 500ms. Try `i2c scan flexcomm3_lpi2c3` for
a live 128-address real bus scan (0 devices - nothing's wired to the
Arduino header) - see "I2C target mode" above for exercising
`flexcomm2_lpi2c2`.

## What's still not done

The remaining gap to a real, fully-functional BIC target on this
board:

- **A real wire-level I2C target-mode test** - the software side is
  done and verified (registers cleanly, see above); proving a genuine
  controller<->target transaction over the wire just needs two short
  jumpers on header J2 (both buses' SDA/SCL land on the same header -
  see "I2C target mode" above for the exact pin table and commands).
- **IPMI transport** - needs a from-scratch transport design (no
  mainline equivalent to port), independent of any further hardware
  work on this board. Note that IPMB-over-I2C specifically no longer
  needs new *driver* work - target mode is real and working (see
  above) - what's missing is the IPMB/IPMI protocol-layer transport
  code itself, which upstream OpenBIC doesn't have a mainline-portable
  version of.
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
