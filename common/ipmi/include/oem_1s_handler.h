#ifndef OEM_1S_HANDLER_H
#define OEM_1S_HANDLER_H

#include "ipmi.h"

enum {
	CPNT_CPLD = 1,
	CPNT_BIC,
	CPNT_ME,
	CPNT_PVCCIN = 5,
	CPNT_PVCCFA_EHV_FIVRA,
	CPNT_PVCCD_HV,
	CPNT_PVCCINFAON,
	CPNT_PVCCFA_EHV
};

// firmware update interface
enum {
	BIOS_UPDATE,
	CPLD_UPDATE,
	BIC_UPDATE,
	UPDATE_EN = 0x80,
};

void OEM_1S_MSG_OUT(ipmi_msg *msg);
void OEM_1S_GET_GPIO(ipmi_msg *msg);
void OEM_1S_FW_UPDATE(ipmi_msg *msg);
void OEM_1S_GET_FW_VERSION(ipmi_msg *msg);
void OEM_1S_RESET_BMC(ipmi_msg *msg);
void OEM_1S_SENSOR_POLL_EN(ipmi_msg *msg);
void OEM_1S_ACCURACY_SENSNR(ipmi_msg *msg);
void OEM_1S_GET_SET_GPIO(ipmi_msg *msg);
void OEM_1S_I2C_DEV_SCAN(ipmi_msg *msg);
void OEM_1S_GET_BIC_STATUS(ipmi_msg *msg);
void OEM_1S_RESET_BIC(ipmi_msg *msg);
void OEM_1S_12V_CYCLE_SLOT(ipmi_msg *msg);

#ifdef CONFIG_ESPI
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

void send_gpio_interrupt(uint8_t gpio_num);

void IPMI_OEM_1S_handler(ipmi_msg *msg);

#endif
