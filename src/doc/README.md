# miniBMC SDK
This codebase is designed for the ARM Cortex-M series miniBMC SDK.  
Including:
- AST2600 secondary service processor (SSP)
- AST1030 bridge IC (BIC) project

## overview
CMSIS core
  - Keil CMSIS source code are used for CPU configuration
  - downloads from https://github.com/ARM-software/CMSIS_5
  - version : CMSIS_5-5.6.0

FreeRTOS kernel
	- version 10.3.1
## toolchain
- for cross-compiling and providing libraries
- only support GCC toolchains
- downloads from https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads
- lastest verified version is ```gcc-arm-none-eabi-9-2019-q4-major```
- python 2.7

	```shell=
	> apt install python libpython2.7
	```

### install `gcc-arm-none-eabi` on MAC OS
```shell=
> xcode-select --install
> xcode-select --reset
> brew tap ArmMbed/homebrew-formulae
> brew install arm-none-eabi-gcc
```

## build
- environment setup
  - support Linux and MAC OS build
  - `source setenv.sh` to setup global environment variables
    - may need to modify `setenv.sh` for your local environment.
- `make xxxx_defconfig`:  to specify the target: 
  - see `./configs` for all defconfig files
- `make menuconfig`: modify the Kconfig options if necessary
- `make`: to build
  - `bin/ast2600.elf`: AXF for GDB debugging
  - `bin/ast2600_ssp.bin`: target binary
- example:

  ```shell=
  source setenv.sh
  make evb-ast2600a1_defconfig
  make
  ```

## run AST1030 FPGA
TBD

## run AST2600 SSP
### load from u-boot
- CA7 side
	- make sure DRAM is stable-initialized
  	- configure CM3 memory: IMEM, DMEM and cache
    	- run the script below in u-boot
        	- ast2600 ssp    : `tools/console_scripts/ast2600_ssp.minicom`
            - ast1030 bic: `tools/console_scripts/ast1030_bic.minicom`

#### reload binary from GDB
- CM3/4 side
```
source debug_ast2600.gdb
reload
```
- CA7 side
```
mw 1e6e2a00 3
mw 1e6e2a00 1
```

- CM3/4 side
```
c
ctrl + c
```


### command line interface
- default STDIO port
    - `UART11` for AST2600 SSP 
	
	![](doc/uart11_connect_0.jpg)
	![](doc/uart11_connect_1.jpg)
    - `UART5` for AST1030 BIC

### debugging
#### GNU GDB
1. install `PyCortexMDebug` for parsing CPU memory-mapped registers
   ```
   cd tools/PyCortexMDebug
   python setup.py install
   ```
2. execute `arm-none-eabi-gdb-py -x debug.gdb`

### Segger Ozone
open `minibmc.jdebug`

## directories
- os/: FreeRTOS kernel 10.3.1 downloaded from https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/V10.3.1-kernel-only
- arch/arm/
	- aspeed/ast2600/: ast2600 startup code and linker scripts
	- aspeed/ast2600/include/: memory map of the peripheral devices, cpu related API
	- cmsis_core/:  CMSIS core API
- drivers/:
	- include/: driver API header files
- libraries/
	- aspeed/: aspeed built-in libs.
	- clib/: simplified standard C lib downloaded or clone from 3rd parties
	- freertos_plus/: FreeRTOS_Plus libs
- tools/
	- Kconfiglib/: Kconfiglib source code
- toolchain/:
	- gcc-arm-none-eabi-9-2019-q4-major: newest verified cross-tool
## how to add an new driver
- add device ID in `arch/arm/aspeed/<target>/device_id.h`
- create a file named `<IP>_<SOC>.c` under driver/
- check if `<IP>_api.h` exists in drivers/include
	- exist: implement the API body
	- not exist: create `<IP>_api.h`
- add device/driver data structure in drivers/include/objects.h

## todos
- add HAL and LL header files
- SCU register defines
- TCP software stack
- USB software stack
- drivers	
	- FMC/SPI master
	- OTP
	- USB
	- I2C
	- I3C
	- GPIO
	- timer
	- watchdog
	- LPC
	- ADC
	- PWM
	- Fan Tachometer
	- PECI
	- JTAG master
