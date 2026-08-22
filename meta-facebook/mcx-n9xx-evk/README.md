# MCX-N9XX-EVK — bring-up

Porting OpenBIC to the NXP MCX-N9XX-EVK (MCXN947, dual Cortex-M33).
Phase 1 got this board booting to an interactive shell over OpenBIC's
own repo/build/west pipeline, on mainline Zephyr. Phase 2 (in progress)
is adding real subsystems one at a time, ported to mainline Zephyr's
API surface instead of reusing `common/` as-is - starting with GPIO
status monitoring (below). It does **not** yet run OpenBIC's IPMI/FRU/
sensor-table pipeline as a whole.

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
that prints an OpenBIC-style banner, brings up the shell, and toggles
the on-board red LED as a heartbeat - proving the build, flash, and
console/shell all work end-to-end via OpenBIC's own CMakeLists.txt/
west setup, on real MCX-N9XX-EVK hardware.

```
meta-facebook/mcx-n9xx-evk/
├── CMakeLists.txt   Only builds src/*.c - no common/ globs
├── prj.conf         GPIO + shell + logging only
├── boards/          Devicetree overlay: enables red_led + user_button_2
├── src/main.c       Banner + shell + heartbeat LED
├── src/plat_version.h
├── src/plat_gpio.[ch]   GPIO status-monitoring subsystem (see below)
└── src/plat_shell.c     "plat gpio mon0" shell command
```

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

## Phase 2: full bring-up (tracked separately, not started here)

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
