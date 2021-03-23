# I3C master deriver demonstration
## enable I3C driver
- kconfig source: `config DEVICE_I3C` in `driver/Kconfig`
- `make menuconfig`
	- `Devices`: select `I3C device and driver`

## enable I3C demo APP
- kconfig source: `config DEMO_I3C` in `app/kconfig`
- `make menuconfig`
	- `Demo APPs`:  select `demo I3C0 access slave device`
- the I3C demo APP demonstrates how to use I3C master driver to access IMX3102 device registers.
	- print IMX3102 device ID in a task.
  ```
  minibmc>[37]:read slave0 addr 00: 31
  [37]:read slave0 addr 01: 02
  [37]:read slave0 addr 00: 31
  [37]:read slave0 addr 01: 02
  [37]:read slave0 addr 00: 31
  [37]:read slave0 addr 01: 02
  ```

## board setup
- use `Aspeed AST2600-DDR4 EVB v1.4` with `Aspeed I3C daughter card`
	- `Aspeed I3C daughter card`: Renesas IDT3102 cascade with SPD5118