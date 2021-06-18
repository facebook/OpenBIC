# OpenBIC

OpenBIC is an open software framework to build a complete firmware image for a BridgeIC.

| Platform | Status | Description |
|-------|--------|-------------|
Yosemite v3.5 | N/A | Yosemite Compute

## Contents

This repo currently contains the following directories 
* Code common to all platforms under the `common` directory.
* Platform specific code in the `meta-facebook` directory.
* The ASPEED OpenBIC SDK under the `src` directory.
  * **app** - An example application to test the various drivers
  * **arch** - Supporting files for the board architecture (ARM) and CMSIS.
  * **board** - Definitions for the various supported boards.
  * **configs** - configuration files used for building the supported board types
  * **Drivers** - Currently implemented drivers.
  * **libraries** - Supporting libraries such as getopt and FreeRTOS+
  * **os** - Supported RTOSs (currently only FreeRTOS)
  * **toolchain** - The included cross compilation toolchain used to build OpenBIC
  * **tools** - Misc tools

### Currently supported RTOS
OpenBIC functionality is written using the CMSIS RTOS API as an RTOS wrapper. This allows for RTOS agnostic design and feature implementations.

The currently supported RTOSs are:
* FreeRTOS

### Currently supported Boards
The board configuration files can be found in the `configs/` directory.

The currently supported boards are:
* ASPEED AST2600 EVB
* ASPEED AST1030 EVB

## Build Instructions

### Dependencies

| Tool | Version |
|------|---------|
GNU make | v4.1 or later
Python | v2.7

The cross compilation tool chain is currently packaged with the SDK: \
`toolchain/gcc-arm-none-eabi-9-2019-q4-major`

### Build Steps

The first time the repo is cloned the submodules will need to be initialized.
1. run `git submodule init`
2. run `git submodule update`

1. Navigate to the src Directory: \
`cd src`
2. Source the enviroment file: \
`source setenv.sh`
3. (OPTIONAL) Change configurations through kconfig: \
`make menuconfig`
4. Return to the top level directory: \
`cd ..`
5. Call make for the desired platform: \
`make yv35_cl`

The build ouput can be found in the `bin` directory.

## License

OpenBIC is [Apache 2.0 licensed](https://github.com/facebookincubator/OpenBIC/blob/master/LICENSE)
