# From-scratch environment setup — OpenBIC on MCX-N9XX-EVK

This target reuses the same Zephyr SDK / NXP LinkServer toolchain as any
other MCXN947 Zephyr project on this machine (see e.g.
`~/zephyr-workspace/MCXN9XX_EVK_baseline/SETUP.md` for the from-scratch
apt/Python/SDK/LinkServer install steps - not repeated here). This doc
only covers what's specific to standing up *this* west workspace.

## 1. West workspace

```sh
mkdir -p ~/openbic-workspace
python3.12 -m venv ~/openbic-workspace/.venv
source ~/openbic-workspace/.venv/bin/activate
pip install --upgrade pip
pip install west

cd ~/openbic-workspace
git clone <this-fork-url> openbic
cd openbic && git checkout mcx-n9xx-evk-port
cd ~/openbic-workspace
west init -l openbic
west update            # pulls zephyr v4.4.0 + cmsis/cmsis_6/hal_nxp - a few GB
pip install -r zephyr/scripts/requirements.txt
```

If you already have another Zephyr v4.4.0 workspace on this machine
(e.g. `~/zephyr-workspace`), you can skip most of the download by
pre-seeding local reference clones before `west update` - it will just
verify the revision instead of fetching:

```sh
cd ~/openbic-workspace
git clone --reference ~/zephyr-workspace/zephyr --dissociate \
  https://github.com/zephyrproject-rtos/zephyr.git zephyr
(cd zephyr && git checkout -f v4.4.0)

mkdir -p modules/hal
for m in nxp:hal_nxp cmsis:cmsis cmsis_6:cmsis_6; do
  src=${m%%:*}; name=${m##*:}
  # match paths/revisions from ~/zephyr-workspace/zephyr/west.yml
done
# concretely (revisions pinned in zephyr v4.4.0's own west.yml):
git clone --reference ~/zephyr-workspace/modules/hal/nxp --dissociate \
  https://github.com/zephyrproject-rtos/hal_nxp.git modules/hal/nxp
(cd modules/hal/nxp && git checkout -f 2c2f28ac333e995d7279777409817c4b4f92c1ec)

git clone --reference ~/zephyr-workspace/modules/hal/cmsis --dissociate \
  https://github.com/zephyrproject-rtos/cmsis.git modules/hal/cmsis
(cd modules/hal/cmsis && git checkout -f 512cc7e895e8491696b61f7ba8066b4a182569b8)

git clone --reference ~/zephyr-workspace/modules/hal/cmsis_6 --dissociate \
  https://github.com/zephyrproject-rtos/cmsis_6.git modules/hal/cmsis_6
(cd modules/hal/cmsis_6 && git checkout -f 30a859f44ef8ab4dc8f84b03ed586fd16ccf9d74)

west update   # now just verifies revisions, no full re-clone
```

### Gotcha: `west.yml`'s import allowlist needs `cmsis_6`, not just `cmsis`

This repo's root `west.yml` imports a curated subset of mainline
Zephyr's module list via `name-allowlist`. `cmsis` alone sets
`CONFIG_HAS_CMSIS_CORE` but **not** `CONFIG_HAS_CMSIS_CORE_M` - on
Cortex-M targets (like MCXN947) that symbol is what actually pulls in
`cmsis_core.h`, and it's only set by the separate `cmsis_6` module
(`modules/hal/cmsis_6`, not `modules/hal/cmsis`). Missing it fails at
the very first compile (`offsets.c`) with `fatal error: cmsis_core.h:
No such file or directory`, deep inside `zephyr/include/zephyr/arch/
arm/asm_inline_gcc.h` - not an obviously board-specific error. If you
add other Cortex-M board targets to this manifest, `cmsis_6` needs to
stay in the allowlist.

## 2. Build / flash / debug

See `README.md` in this directory.

## 3. Verified boot log

```
*** Booting Zephyr OS build v4.4.0 ***
Hello, welcome to NXP MCX-N9XX-EVK Minimal Bring-up 2026.34.0
Minimal bring-up: no sensor/IPMI/FRU services in this build.
uart:~$
```

Captured over the MCU-Link's VCOM serial port (115200 8N1) after
`west flash` on real MCX-N9XX-EVK hardware; the on-board red LED
blinks as a heartbeat.
