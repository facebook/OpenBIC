# MCX-N9XX-EVK — bring-up

Porting OpenBIC to the NXP MCX-N9XX-EVK (MCXN947, dual Cortex-M33).
Phase 1 got this board booting to an interactive shell over OpenBIC's
own repo/build/west pipeline, on mainline Zephyr. Phase 2 adds real
subsystems one at a time, ported to mainline Zephyr's API surface
instead of reusing `common/` as-is: GPIO status monitoring, HWINFO,
watchdog, persistent NVS storage, and dual-core cpu0↔cpu1 mailbox IPC
(all below, each verified on real hardware). It does **not** yet run
OpenBIC's IPMI/FRU/sensor-table pipeline as a whole.

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

## What this target actually does

`src/main.c` is a from-scratch `main()` (not `common/service/main.c`)
that prints an explicit "OpenBIC" ASCII banner, brings up the shell,
and toggles the on-board red LED as a heartbeat - proving the build,
flash, and console/shell all work end-to-end via OpenBIC's own
CMakeLists.txt/west setup, on real MCX-N9XX-EVK hardware.

```
meta-facebook/mcx-n9xx-evk/
├── CMakeLists.txt   Only builds src/*.c - no common/ globs
├── prj.conf         GPIO + shell + logging + hwinfo + watchdog only
├── boards/          Devicetree overlay: enables red_led + user_button_2
├── src/main.c       Banner + shell + heartbeat LED
├── src/plat_version.h
├── src/plat_gpio.[ch]    GPIO status-monitoring subsystem (see below)
├── src/plat_hwinfo.[ch]  Device ID + reset-cause reporting (see below)
├── src/plat_wdt.[ch]     Watchdog subsystem (see below)
├── src/plat_storage.[ch] Persistent NVS storage (see below)
├── src/plat_mbox.[ch]    Inter-core mailbox, cpu0 side (see below)
├── src/plat_shell.c      "plat version/gpio mon0/wdt starve/storage bootcount/mbox ping"
├── sysbuild.cmake        Orchestrates the optional dual-core build
├── Kconfig.sysbuild      Selects mcx_n9xx_evk/mcxn947/cpu1 as the remote board
└── remote/               cpu1's image (see "Dual-core" below)
    ├── CMakeLists.txt, prj.conf, boards/*.conf|overlay
    └── src/main.c        Headless mailbox echo responder
```

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

## Build / flash

Same toolchain as NXP's own MCXN947 Zephyr samples - see the repo-root
`SETUP.md` (west workspace at `~/openbic-workspace`, Zephyr SDK,
LinkServer):

```sh
source ~/openbic-workspace/.venv/bin/activate
cd ~/openbic-workspace/openbic
west build -p always -b mcx_n9xx_evk/mcxn947/cpu0 meta-facebook/mcx-n9xx-evk
export PATH="/usr/local/LinkServer_26.6.137:$PATH"
west flash
```

Open a serial console (115200 8N1) on the MCU-Link's VCOM port to see
the banner and shell prompt; the red LED should blink every 500ms.

## Beyond this: full IPMI/sensor/FRU bring-up (not started here)

Making this a real BIC target means porting `common/` to mainline
Zephyr's API surface, which is substantial, open-ended work (153 `.c`
files / ~57K LOC across `common/`):

- Mechanical pass: fix legacy include paths repo-wide
  (`<zephyr.h>` → `<zephyr/kernel.h>`, etc.) - large diff, low risk.
- Real porting: modernize driver-level API calls (GPIO v1→v2, I2C,
  `k_work`, etc.) file by file as each subsystem gets wired up for
  this board.
- Replace or reimplement the Aspeed-only subsystems this board will
  actually need against stock Zephyr APIs: IPMI transport, I2C-slave-
  as-IPMB, EEPROM-emulation-over-I2C. None of these have a drop-in
  mainline equivalent - each is its own design decision.
- Decide, board by board, which of `common/service`'s init calls
  (sensor, FRU, IPMI, USB) actually apply to this platform's role
  before wiring them back into `main()`.
