#!/bin/bash

(cd os/FreeRTOS-Kernel/ && ../FreeRTOS-Kernel-patches/patch_files.sh)

if [ "$(uname -s)" = "Darwin" ]; then
	TOOLCHAIN=/usr/local/Cellar/arm-none-eabi-gcc/9-2019-q4-major/gcc
else
	TOOLCHAIN=$PWD/toolchain/gcc-arm-none-eabi-9-2019-q4-major
fi
export TOOLCHAIN

CROSS_COMPILE=arm-none-eabi-
export CROSS_COMPILE

LIB_PATH=$TOOLCHAIN/arm-none-eabi/lib
export LIB_PATH

LIB_GCC_PATH=$TOOLCHAIN/lib/gcc/arm-none-eabi/9.2.1
export LIB_GCC_PATH

INC_PATH=$TOOLCHAIN/arm-none-eabi/include
export INC_PATH

if ! [[ $PATH =~ "$TOOLCHAIN/bin" ]];
then
	PATH="$PATH:$TOOLCHAIN/bin"
fi
export path
