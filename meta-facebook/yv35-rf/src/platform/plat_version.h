#ifndef PLAT_VERSION_H
#define PLAT_VERSION_H

#define PLATFORM_NAME "yv35"
#define PROJECT_NAME "Rainbow Falls"
#define DEVICE_ID 0x00
#define DEVICE_REVISION 0x80
// BIT 0:3  1: CraterLake 2: Baseboard 3: Rainbow falls
// BIT 4:7  0: POC 1: EVT 2: DVT
#define FIRMWARE_REVISION_1 0x03
#define FIRMWARE_REVISION_2 0x01
#define IPMI_VERSION 0x02
#define ADDITIONAL_DEVICE_SUPPORT 0xBF
#define PRODUCT_ID 0x0000
#define AUXILIARY_FW_REVISION 0x00000000

#define BIC_FW_YEAR_MSB 0x20
#define BIC_FW_YEAR_LSB 0x22
#define BIC_FW_WEEK 0x22
#define BIC_FW_VER 0x01
#define BIC_FW_platform_0 0x72 // char: r
#define BIC_FW_platform_1 0x66 // char: f
#define BIC_FW_platform_2 0x00 // char: '\0'

#endif
