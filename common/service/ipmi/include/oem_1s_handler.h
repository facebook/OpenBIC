#ifndef OEM_1S_HANDLER_H
#define OEM_1S_HANDLER_H

#include "ipmi.h"

#define SENSOR_EVENT_MESSAGES_ENABLE (1 << 7)
#define SENSOR_SCANNING_ENABLE (1 << 6)
#define SENSOR_READING_STATE_UNAVAILABLE (1 << 5)

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
	CXL_UPDATE,
};

enum GET_SET_GPIO_OPTIONS {
	GET_GPIO_OUTPUT_STATUS = 0,
	SET_GPIO_OUTPUT_STATUS,
	GET_GPIO_DIRECTION_STATUS,
	SET_GPIO_DIRECTION_STATUS,
};
typedef struct _ACCURACY_SENSOR_READING_REQ {
	uint8_t sensor_num;
	uint8_t read_option;
} ACCURACY_SENSOR_READING_REQ;

typedef struct _ACCURACY_SENSOR_READING_RES {
	uint16_t decimal;
	uint16_t fraction;
	uint8_t status;
} ACCURACY_SENSOR_READING_RES;

void OEM_1S_MSG_OUT(ipmi_msg *msg);
void OEM_1S_GET_GPIO(ipmi_msg *msg);
void OEM_1S_FW_UPDATE(ipmi_msg *msg);
void OEM_1S_GET_FW_VERSION(ipmi_msg *msg);
void OEM_1S_SET_VR_MONITOR_STATUS(ipmi_msg *msg);
void OEM_1S_GET_VR_MONITOR_STATUS(ipmi_msg *msg);
void OEM_1S_RESET_BMC(ipmi_msg *msg);
void OEM_1S_SENSOR_POLL_EN(ipmi_msg *msg);
void OEM_1S_ACCURACY_SENSOR_READING(ipmi_msg *msg);
void OEM_1S_GET_SET_GPIO(ipmi_msg *msg);
void OEM_1S_I2C_DEV_SCAN(ipmi_msg *msg);
void OEM_1S_GET_BIC_STATUS(ipmi_msg *msg);
void OEM_1S_RESET_BIC(ipmi_msg *msg);
void OEM_1S_12V_CYCLE_SLOT(ipmi_msg *msg);
void OEM_1S_READ_BIC_REGISTER(ipmi_msg *msg);
void OEM_1S_WRITE_BIC_REGISTER(ipmi_msg *msg);
void OEM_1S_INFORM_PEER_SLED_CYCLE(ipmi_msg *msg);
void OEM_1S_PEX_FLASH_READ(ipmi_msg *msg);
void OEM_1S_GET_FPGA_USER_CODE(ipmi_msg *msg);
void OEM_1S_GET_CARD_TYPE(ipmi_msg *msg);

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

#ifdef ENABLE_FAN
void OEM_1S_SET_FAN_DUTY_AUTO(ipmi_msg *msg);
void OEM_1S_GET_FAN_DUTY(ipmi_msg *msg);
void OEM_1S_GET_FAN_RPM(ipmi_msg *msg);
#endif

void IPMI_OEM_1S_handler(ipmi_msg *msg);

#endif
