#ifndef OEM_1S_HANDLER_H
#define OEM_1S_HANDLER_H

#include "ipmi.h"

enum FIRMWARE_COMPONENT {
	COMPNT_CPLD = 1,
	COMPNT_BIC,
	COMPNT_ME,
	COMPNT_BIOS,
	COMPNT_PVCCIN,
	COMPNT_PVCCFA_EHV_FIVRA,
	COMPNT_PVCCD_HV,
	COMPNT_PVCCINFAON,
	COMPNT_PVCCFA_EHV
};

#define IS_SECTOR_END_MASK 0x80
enum FIRWARE_UPDATE_TARGET {
	BIOS_UPDATE = 0,
	CPLD_UPDATE,
	BIC_UPDATE,
};

enum GET_SET_GPIO_OPTIONS {
	GET_GPIO_OUTPUT_STATUS = 0,
	SET_GPIO_OUTPUT_STATUS,
	GET_GPIO_DIRECTION_STATUS,
	SET_GPIO_DIRECTION_STATUS,
};
void OEM_1S_MSG_OUT(ipmi_msg *msg);
void OEM_1S_GET_GPIO(ipmi_msg *msg);
void OEM_1S_FW_UPDATE(ipmi_msg *msg);
void OEM_1S_GET_FW_VERSION(ipmi_msg *msg);
void OEM_1S_RESET_BMC(ipmi_msg *msg);
void OEM_1S_SENSOR_POLL_EN(ipmi_msg *msg);
void OEM_1S_ACCURACY_SENSOR(ipmi_msg *msg);
void OEM_1S_GET_SET_GPIO(ipmi_msg *msg);
void OEM_1S_I2C_DEV_SCAN(ipmi_msg *msg);
void OEM_1S_GET_BIC_STATUS(ipmi_msg *msg);
void OEM_1S_RESET_BIC(ipmi_msg *msg);
void OEM_1S_12V_CYCLE_SLOT(ipmi_msg *msg);

#ifdef CONFIG_IPMI_KCS_ASPEED
void OEM_1S_GET_POST_CODE(ipmi_msg *msg);
#endif

#ifdef CONFIG_PECI
void OEM_1S_PECI_ACCESS(ipmi_msg *msg);
#endif

#ifdef CONFIG_JTAG
void OEM_1S_SET_JTAG_TAP_STA(ipmi_msg *msg);
void OEM_1S_JTAG_DATA_SHIFT(ipmi_msg *msg);

#ifdef ENABLE_ASD
void OEM_1S_ASD_INIT(ipmi_msg *msg);
#endif
#endif

void IPMI_OEM_1S_handler(ipmi_msg *msg);

#endif
