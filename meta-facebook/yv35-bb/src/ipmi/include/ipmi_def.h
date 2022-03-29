#ifndef IPMI_DEF_H
#define IPMI_DEF_H

#define DEVICE_ID 0x00
#define DEVICE_REVISION 0x80
// Bit 0 boade: 01h CL  02h BB
// Bit 1 stage: 00h POC 01h EVT
#define FIRMWARE_REVISION_1 0x12
#define FIRMWARE_REVISION_2 0x02
#define IPMI_VERSION 0x02
#define ADDITIONAL_DEVICE_SUPPORT 0xBF
#define PRODUCT_ID 0x0000
#define AUXILIARY_FW_REVISION 0x00000000

#define BIC_FW_YEAR_MSB 0x20
#define BIC_FW_YEAR_LSB 0x22
#define BIC_FW_WEEK 0x13
#define BIC_FW_VER 0x01
#define BIC_FW_platform_0 0x62 // char: b
#define BIC_FW_platform_1 0x62 // char: b
#define BIC_FW_platform_2 0x00 // char: '\0'

// firmware update interface
#define CPLD_UPDATE (1 << 0)
#define BIC_UPDATE (1 << 1)
#define UPDATE_EN (1 << 7)
#define GET_TEST_RESULT 0

#endif
