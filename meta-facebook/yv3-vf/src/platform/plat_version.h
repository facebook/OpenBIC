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
 *    1: CraterLake 
 *    2: Baseboard 
 *    3: Rainbow falls
 *    4. Vernal falls
 *  [bit 4-7]
 *    0: POC 
 *    1: EVT 
 *    2: DVT
 *  FIRMWARE_REVISION_2
 *    Count of release firmware at each stage.
 */
#define FIRMWARE_REVISION_1 0xFF
#define FIRMWARE_REVISION_2 0xFF
#define IPMI_VERSION 0x02
#define ADDITIONAL_DEVICE_SUPPORT 0xBF
#define PRODUCT_ID 0x0000
#define AUXILIARY_FW_REVISION 0x00000000

#define BIC_FW_YEAR_MSB 0xFF
#define BIC_FW_YEAR_LSB 0xFF
#define BIC_FW_WEEK 0xFF
#define BIC_FW_VER 0xFF
#define BIC_FW_platform_0 0x76 // char: v
#define BIC_FW_platform_1 0x66 // char: f
#define BIC_FW_platform_2 0x00 // char: '\0'

#endif
