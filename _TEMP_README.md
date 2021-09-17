/* Reference from Aspeed Zephyr SDK user guide and zephyr offcial user guide */

The SDK develop environment is based on Ubuntu 18.04 LTS – 64bits and bash shell.
Also, the following tools must be installed on the host machine.
Reference url: https://docs.zephyrproject.org/2.6.0/getting_started/index.html
Reference url: https://github.com/AspeedTech-BMC/zephyr

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
> **Note:** First step might be effective only after west.yml being merged **
```
west init -m https://github.com/facebook/OpenBIC zephyrproject
cd zephyrproject
west update
```


Install a toolchain:
```
cd ~
wget https://github.com/zephyrproject-rtos/sdkng/releases/download/v0.12.4/zephyr-sdk-0.12.4-x86_64-linux-setup.run
```
Run the installer, installing the SDK in ~/zephyr-sdk-0.12.4:
```
chmod +x zephyr-sdk-0.12.4-x86_64-linux-setup.run
./zephyr-sdk-0.12.4-x86_64-linux-setup.run -- -d ~/zephyr-sdk-0.12.4
```

Clean build application and Zephyr code
```
cd $zephyrproject/openbic.odm
touch meta-facebook/yv35-cl/CMakeLists.txt
west build -p auto -b ast1030_evb meta-facebook/yv35-cl/
```
