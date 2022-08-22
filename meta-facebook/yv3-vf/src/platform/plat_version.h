#ifndef PLAT_VERSION_H
#define PLAT_VERSION_H

#define PLATFORM_NAME "yv3"
#define PROJECT_NAME "vernalfall"
#define IANA_ID 0x009C9C // same as ti
#define DEVICE_ID 0x00
#define DEVICE_REVISION 0x80
/*
 *  FIRMWARE REVISION_1 
 *  [bit 0-3] 
 *    7: Vernal falls
 *  [bit 4-7]
 *    0: POC 
 *    1: EVT 
 *    2: DVT
 *    3: PVT
 *    4: MP
 *  FIRMWARE_REVISION_2
 *    Count of release firmware at each stage.
 */
#define FIRMWARE_REVISION_1 0x47
#define FIRMWARE_REVISION_2 0x03
#define IPMI_VERSION 0x02
#define ADDITIONAL_DEVICE_SUPPORT 0xBF
#define PRODUCT_ID 0x0000
#define AUXILIARY_FW_REVISION 0x00000000

#define BIC_FW_YEAR_MSB 0x20
#define BIC_FW_YEAR_LSB 0x22
#define BIC_FW_WEEK 0x32
#define BIC_FW_VER 0x01
#define BIC_FW_platform_0 0x76 // char: v
#define BIC_FW_platform_1 0x66 // char: f
#define BIC_FW_platform_2 0x00 // char: '\0'

#endif
