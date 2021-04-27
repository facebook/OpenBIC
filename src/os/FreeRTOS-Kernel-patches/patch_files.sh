#!/bin/bash

cp ../FreeRTOS-Kernel/include/portable.h ../FreeRTOS-Kernel-patches/include/portable.h.orig
patch --strip=1 < ../FreeRTOS-Kernel-patches/include/portable.h.patch

cp tasks.c ../FreeRTOS-Kernel-patches/tasks.c.orig
patch --strip=1 < ../FreeRTOS-Kernel-patches/tasks.c.patch

patch --strip=1 < ../FreeRTOS-Kernel-patches/Makefile.patch
patch --strip=1 < ../FreeRTOS-Kernel-patches/portable/GCC/ARM_CM3/Makefile.patch 
patch --strip=1 < ../FreeRTOS-Kernel-patches/portable/GCC/ARM_CM4F/Makefile.patch
patch --strip=1 < ../FreeRTOS-Kernel-patches/portable/MemMang/Makefile.patch
patch --strip=1 < ../FreeRTOS-Kernel-patches/portable/MemMang/heap_4_nc.c.patch

