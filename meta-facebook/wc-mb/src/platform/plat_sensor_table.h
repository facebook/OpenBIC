#ifndef PLAT_SENSOR_TABLE_H
#define PLAT_SENSOR_TABLE_H

#include <stdint.h>

/*  define config for sensors  */
#define VR_PU14_SRC0_ADDR (0xC0 >> 1)
#define VR_PU14_SRC1_ADDR (0xC0 >> 1)
#define VR_PU5_SRC0_ADDR (0xC2 >> 1)
#define VR_PU5_SRC1_ADDR (0xC8 >> 1)
#define VR_PU35_SRC0_ADDR (0xC6 >> 1)
#define VR_PU35_SRC1_ADDR (0xC4 >> 1)
#define PVCCIN_ADDR VR_PU14_SRC0_ADDR
#define PVCCFA_EHV_FIVRA_ADDR VR_PU14_SRC0_ADDR
#define PVCCINFAON_ADDR VR_PU5_SRC0_ADDR
#define PVCCFA_EHV_ADDR VR_PU5_SRC0_ADDR
#define PVCCD_HV_ADDR VR_PU35_SRC0_ADDR

/*  threshold sensor number, 1 based  */
#define SENSOR_NUM_VOL_BAT3V 0x21
#define SENSOR_NUM_PWR_HSCIN 0x61

#define SENSOR_NUM_SYSTEM_STATUS 0x10
#define SENSOR_NUM_POWER_ERROR 0x56
#define SENSOR_NUM_PROC_FAIL 0x65
#define SENSOR_NUM_VR_HOT 0xB2
#define SENSOR_NUM_CPUDIMM_HOT 0xB3
#define SENSOR_NUM_CATERR 0xEB
#define SENSOR_NUM_RMCA 0xEC //TBD: BMC should know this

uint8_t plat_get_config_size();
void load_sensor_config(void);

#endif
