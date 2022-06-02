# OpenBIC

OpenBIC is an open software framework to build a complete firmware image for a BridgeIC.

| Platform | Status | Description |
|-------|--------|-------------|
obgt-cc | ![badge](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/goldenbug/62fb115c4fa43a02acad226534e10932/raw/gt-cc.json) | Grand Teton Clear Creek
oby35-bb | ![badge](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/goldenbug/62fb115c4fa43a02acad226534e10932/raw/yv35-bb.json) | Yosemite v3.5 Baseboard
oby35-cl | ![badge](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/goldenbug/62fb115c4fa43a02acad226534e10932/raw/yv35-cl.json) | Yosemite v3.5 Crater Lake Board
oby35-rf | ![badge](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/goldenbug/62fb115c4fa43a02acad226534e10932/raw/yv35-rf.json) | Yosemite v3.5 Rainbow Falls Board

## Contents

Documentation: https://facebook.github.io/OpenBIC/

### Currently supported RTOS
OpenBIC functionality is written using the CMSIS RTOS API as an RTOS wrapper. This allows for RTOS agnostic design and feature implementations.

The currently supported RTOSs are:
* Zephyr 

### Currently supported Boards
The board configuration files can be found in the `configs/` directory.

The currently supported boards are:
* ASPEED AST2600 EVB
* ASPEED AST1030 EVB

## Build Instructions

### Dependencies

The SDK develop environment is based on Ubuntu 18.04 LTS – 64bits and bash shell.
The following tools must be installed on the host machine.
- Reference url: https://docs.zephyrproject.org/2.6.0/getting_started/index.html
- Reference url: https://github.com/AspeedTech-BMC/zephyr

Dependencies minimum required: 
```
Cmake: 3.20.0
Python: 3.6
Devicetree compiler: 1.4.6
```

Install dependencies:
```
sudo apt install --no-install-recommends git cmake ninja-build gperf \
ccache dfu-util device-tree-compiler wget \
python3-dev python3-pip python3-setuptools python3-tk python3-wheel xzutils file \
make gcc gcc-multilib g++-multilib libsdl2-dev
```

Download Zephyr package:
Install west, and make sure ~/.local/bin is on your PATH environment variable:
```
pip3 install --user -U west
echo 'export PATH=~/.local/bin:"$PATH"' >> ~/.bashrc
source ~/.bashrc
```

Get the application and Zephyr code base from openbic GitHub.
> **Note:** First step will only work after west.yml being merged **
```
west init -m https://github.com/facebook/OpenBIC zephyrproject
cd zephyrproject
west update
```

Install the toolchain:
```
cd ~
wget https://github.com/zephyrproject-rtos/sdkng/releases/download/v0.12.4/zephyr-sdk-0.12.4-x86_64-linux-setup.run
```
Run the installer, installing the SDK in ~/zephyr-sdk-0.12.4:
```
chmod +x zephyr-sdk-0.12.4-x86_64-linux-setup.run
./zephyr-sdk-0.12.4-x86_64-linux-setup.run -- -d ~/zephyr-sdk-0.12.4
```

### Build Steps

Clean build application and Zephyr code
```
cd $zephyrproject/openbic.odm
touch meta-facebook/yv35-cl/CMakeLists.txt
west build -p auto -b ast1030_evb meta-facebook/yv35-cl/
```

## License

OpenBIC is [Apache 2.0 licensed](https://github.com/facebookincubator/OpenBIC/blob/main/LICENSE)
