#ifndef PLAT_VERSION_H
#define PLAT_VERSION_H

#include "version.h"

#define PLATFORM_NAME "Yosemite 3.5"
#define PROJECT_NAME "Half Dome"
#define PROJECT_STAGE POC

#define BOARD_ID 0x01
#define DEVICE_ID 0x00
#define DEVICE_REVISION 0x80

#define FIRMWARE_REVISION_1 0xff
#define FIRMWARE_REVISION_2 0xff

#define IPMI_VERSION 0x02
#define ADDITIONAL_DEVICE_SUPPORT 0xBF
#define PRODUCT_ID 0x0000
#define AUXILIARY_FW_REVISION 0x00000000

#define BIC_FW_YEAR_MSB 0xff
#define BIC_FW_YEAR_LSB 0xff
#define BIC_FW_WEEK 0xff
#define BIC_FW_VER 0xff
#define BIC_FW_platform_0 0x68 // char: h
#define BIC_FW_platform_1 0x64 // char: d
#define BIC_FW_platform_2 0x00 // char: '\0'

#endif
