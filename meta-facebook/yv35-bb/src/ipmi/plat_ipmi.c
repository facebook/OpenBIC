#include <stdio.h>
#include <stdlib.h>
#include "libipmi.h"
#include "libutil.h"
#include "ipmi.h"
#include "ipmb.h"
#include "plat_ipmi.h"
#include "plat_ipmb.h"
#include "plat_gpio.h"
#include "plat_isr.h"

void OEM_CABLE_DETECTION(ipmi_msg *msg)
{
	if (msg == NULL) {
		printf("%s failed due to parameter *msg is NULL\n", __func__);
		return;
	}

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->completion_code = CC_SUCCESS;

	common_addsel_msg_t *sel_msg = (common_addsel_msg_t *)malloc(sizeof(common_addsel_msg_t));
	sel_msg->sensor_type = IPMI_OEM_SENSOR_TYPE_OEM;
	sel_msg->event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
	sel_msg->sensor_number = CMD_OEM_CABLE_DETECTION;
	sel_msg->event_data2 = 0xFF;
	sel_msg->event_data3 = 0xFF;

	if (msg->InF_source == SLOT1_BIC) {
		ISR_SLOT3_PRESENT();
		if (((gpio_get(SLOT1_ID1_DETECT_BIC_N) << 1) |
		     (gpio_get(SLOT1_ID0_DETECT_BIC_N) << 0)) != BB_CABLE_MATCH_SLOT1) {
			sel_msg->event_data1 = IPMI_OEM_EVENT_OFFSET_SLOT3_INSERT_SLOT1;
			sel_msg->InF_target = SLOT1_BIC;
			common_add_sel_evt_record(sel_msg);
		}
	} else if (msg->InF_source == SLOT3_BIC) {
		ISR_SLOT1_PRESENT();
		if (((gpio_get(SLOT3_ID1_DETECT_BIC_N) << 1) |
		     (gpio_get(SLOT3_ID0_DETECT_BIC_N) << 0)) != BB_CABLE_MATCH_SLOT3) {
			sel_msg->event_data1 = IPMI_OEM_EVENT_OFFSET_SLOT1_INSERT_SLOT3;
			sel_msg->InF_target = SLOT3_BIC;
			common_add_sel_evt_record(sel_msg);
		}
	} else {
		printf("%s Unknown request", __func__);
	}

	SAFE_FREE(sel_msg);
	return;
}
