#include "plat_ipmi.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "libutil.h"
#include "ipmi.h"
#include "plat_ipmb.h"
#include "hal_gpio.h"

#define GET_BIT_VAL_1(val, n) ((val & BIT(n)) >> (n))
#define REG_GPIO_BASE 0x7e780000

enum { CMD_GPIO_GET, CMD_GPIO_SET };

static uint32_t GPIO_GROUP_REG_ACCESS[GPIO_GROUP_NUM] = {
	REG_GPIO_BASE + 0x00, /* GPIO_A/B/C/D Data Value Register */
	REG_GPIO_BASE + 0x20, /* GPIO_E/F/G/H Data Value Register */
	REG_GPIO_BASE + 0x70, /* GPIO_I/J/K/L Data Value Register */
	REG_GPIO_BASE + 0x78, /* GPIO_M/N/O/P Data Value Register */
	REG_GPIO_BASE + 0x80, /* GPIO_Q/R/S/T Data Value Register */
	REG_GPIO_BASE + 0x88, /* GPIO_U Data Value Register */
};

bool add_sel_evt_record(addsel_msg_t *sel_msg)
{
	ipmb_error status;
	ipmi_msg *msg;
	uint8_t system_event_record = 0x02; // IPMI spec definition
	uint8_t evt_msg_version = 0x04; // IPMI spec definition
	static uint16_t record_id = 0x1;

	if (sel_msg == NULL) {
		printf("sel_msg was passed in as NULL\n");
		return false;
	}

	// According to IPMI spec, record id 0h and FFFFh is reserved for special usage
	if ((record_id == 0) || (record_id == 0xFFFF)) {
		record_id = 0x1;
	}

	msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (msg == NULL) {
		printf("add_sel_evt_record malloc fail\n");
		return false;
	}
	memset(msg, 0, sizeof(ipmi_msg));

	msg->data_len = 16;
	msg->InF_source = SELF;
	msg->InF_target = BMC_IPMB;
	msg->netfn = NETFN_STORAGE_REQ;
	msg->cmd = CMD_STORAGE_ADD_SEL;

	msg->data[0] = (record_id & 0xFF); // record id byte 0, lsb
	msg->data[1] = ((record_id >> 8) & 0xFF); // record id byte 1
	msg->data[2] = system_event_record; // record type
	msg->data[3] = 0x00; // timestamp, bmc would fill up for bic
	msg->data[4] = 0x00; // timestamp, bmc would fill up for bic
	msg->data[5] = 0x00; // timestamp, bmc would fill up for bic
	msg->data[6] = 0x00; // timestamp, bmc would fill up for bic
	msg->data[7] = (SELF_I2C_ADDRESS << 1); // generator id
	msg->data[8] = 0x00; // generator id
	msg->data[9] = evt_msg_version; // event message format version
	msg->data[10] = sel_msg->sensor_type; // sensor type, TBD
	msg->data[11] = sel_msg->sensor_number; // sensor number
	msg->data[12] = sel_msg->event_type; // event dir/event type
	msg->data[13] = sel_msg->event_data1; // sensor data 1
	msg->data[14] = sel_msg->event_data2; // sensor data 2
	msg->data[15] = sel_msg->event_data3; // sensor data 3
	record_id++;

	status = ipmb_read(msg, IPMB_inf_index_map[msg->InF_target]);
	SAFE_FREE(msg);

	switch (status) {
	case IPMB_ERROR_FAILURE:
		printf("Fail to post msg to txqueue for addsel\n");
		return false;
	case IPMB_ERROR_GET_MESSAGE_QUEUE:
		printf("No response from bmc for addsel\n");
		return false;
	default:
		return true;
	}
}

/*
  - Name: OEM_1S_GET_SET_GPIO
  - Description: OEM command NetFn:0x38 Cmd:0x41
  - Request:
		Byte 1:3 - MFG ID
		Byte 4 - option
			0x00 - get gpio status
			0x01 - set gpio status
		Byte 5 - gpio number
		Byte 6 - (if Byte4 = 0x01)set value
  - Response:
		Byte 1 - Completion Code
		Byte 2:4 - MFG ID
		Byte 5 - gpio number
		Byte 6 - gpio direction
		Byte 7 - gpio value
*/
void OEM_1S_GET_SET_GPIO(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	uint8_t completion_code = CC_INVALID_LENGTH;

	if (msg->data_len < 2 || msg->data_len > 3) {
		msg->completion_code = completion_code;
		return;
	}

	if (msg->data[0] == CMD_GPIO_GET) {
		if (msg->data_len != 2) {
			msg->completion_code = CC_INVALID_LENGTH;
			return;
		}
	} else if (msg->data[0] == CMD_GPIO_SET) {
		if (msg->data_len != 3) {
			msg->completion_code = CC_INVALID_LENGTH;
			return;
		}
	}

	uint8_t gpio_idx = msg->data[1];

	if (gpio_idx >= GPIO_CFG_SIZE) {
		completion_code = CC_PARAM_OUT_OF_RANGE;
		goto exit;
	}

	if (gpio_cfg[gpio_idx].is_init == DISABLE) {
		completion_code = CC_INVALID_DATA_FIELD;
		goto exit;
	}

	uint8_t val = 0xFF;
	uint8_t dir = 0xFF;
	uint32_t g_val = sys_read32(GPIO_GROUP_REG_ACCESS[gpio_idx / 32]);
	uint32_t g_dir = sys_read32(GPIO_GROUP_REG_ACCESS[gpio_idx / 32] + 0x4);

	val = GET_BIT_VAL_1(g_val, gpio_idx % 32);
	if (g_dir & BIT(gpio_idx % 32))
		dir = 0x01;
	else
		dir = 0x00;

	switch (msg->data[0]) {
	case CMD_GPIO_GET:
		if (msg->data_len == 2) {
			msg->data[0] = gpio_idx;
			msg->data[1] = dir;
			msg->data[2] = val;
			completion_code = CC_SUCCESS;
		}
		break;
	case CMD_GPIO_SET:
		if (msg->data_len == 3) {
			msg->data[0] = gpio_idx;
			gpio_set(gpio_idx, msg->data[2]);
			g_dir = sys_read32(GPIO_GROUP_REG_ACCESS[gpio_idx / 32] + 0x4);
			msg->data[1] = (g_dir & BIT(gpio_idx % 32)) >> (gpio_idx % 32);
			msg->data[2] = gpio_get(gpio_idx);
			completion_code = CC_SUCCESS;
		}
		break;
	default:
		printf("[%s] Unknown options(0x%x)", __func__, msg->data[0]);
		return;
	}

exit:
	if (completion_code != CC_SUCCESS) {
		msg->data_len = 0;
	} else {
		msg->data_len = 3;
	}
	msg->completion_code = completion_code;
	return;
}
