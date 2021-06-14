# OpenBIC

OpenBIC is an open software framework to build a complete firmware image for a BridgeIC.

| Platform | Status | Description |
|-------|--------|-------------|
TBD | TBD | TBD

## Contents

This repo currently contains the ASPEED OpenBIC SDK under the source directory.

Within that directory the subdirectories contain:
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

## Build Instructions

### After Cloning Repo
You must initialize and setup submodule using the following commands.

* git submodule init
* git submodule update

### Dependencies

| Tool | Version |
|------|---------|
GNU make | v4.1 or later
Python | v2.7

The cross compilation tool chain is currently packaged with the SDK: \
`toolchain/gcc-arm-none-eabi-9-2019-q4-major`

### Currently supported Boards
The board configuration files can be found in the `configs/` directory.

The currently supported boards are:
* ASPEED AST2600 EVB
* ~~ASPEED AST1030 EVB~~ (Incomplete)

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
4. Call make for the desired board: \
`make evb-ast2600_defconfig`
5. Call make for to compile the image: \
`make`

The build ouput can be found in the `bin` folder in the `src` directory.

## License

Copyright (c) Facebook, Inc. and its affiliates.

Licensed under the Apache License, Version 2.0 (the "License");
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
