#include "plat_ipmi.h"

#include <stdio.h>
#include <stdlib.h>

#include "hal_gpio.h"
#include "libutil.h"
#include "ipmi.h"
#include "oem_1s_handler.h"
#include "plat_class.h"
#include "plat_ipmb.h"
#include "plat_gpio.h"
#include "pmbus.h"
#include "plat_sensor_table.h"
#include "util_sys.h"
#include "isl69254iraz_t.h"
#include "xdpe12284c.h"

void OEM_1S_GET_FW_VERSION(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t component = msg->data[0];
	ipmb_error status;
	ipmi_msg *bridge_msg;
	I2C_MSG i2c_msg;
	uint8_t retry = 3;

	switch (component) {
	case DL_COMPNT_CPLD:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	case DL_COMPNT_BIC:
		msg->data[0] = FIRMWARE_REVISION_1;
		msg->data[1] = FIRMWARE_REVISION_2;
		msg->data_len = 2;
		msg->completion_code = CC_SUCCESS;
		break;
	case DL_COMPNT_ME:
		bridge_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
		if (bridge_msg == NULL) {
			msg->completion_code = CC_OUT_OF_SPACE;
			return;
		}
		bridge_msg->data_len = 0;
		bridge_msg->seq_source = 0xff;
		bridge_msg->InF_source = SELF;
		bridge_msg->InF_target = ME_IPMB;
		bridge_msg->netfn = NETFN_APP_REQ;
		bridge_msg->cmd = CMD_APP_GET_DEVICE_ID;

		status = ipmb_read(bridge_msg, IPMB_inf_index_map[bridge_msg->InF_target]);
		if (status != IPMB_ERROR_SUCCESS) {
			printf("ipmb read fail status: %x", status);
			SAFE_FREE(bridge_msg);
			msg->completion_code = CC_BRIDGE_MSG_ERR;
			return;
		} else {
			msg->data[0] = bridge_msg->data[2] & 0x7F;
			msg->data[1] = bridge_msg->data[3] >> 4;
			msg->data[2] = bridge_msg->data[3] & 0x0F;
			msg->data[3] = bridge_msg->data[12];
			msg->data[4] = bridge_msg->data[13] >> 4;
			msg->data_len = 5;
			msg->completion_code = CC_SUCCESS;
			SAFE_FREE(bridge_msg);
		}
		break;
	case DL_COMPNT_PVCCIN:
	case DL_COMPNT_PVCCSA:
	case DL_COMPNT_PVCCIO:
	case DL_COMPNT_P3V3_STBY:
	case DL_COMPNT_PVDDR_ABC:
	case DL_COMPNT_PVDDR_DEF:
		if ((component == DL_COMPNT_PVCCIN) || (component == DL_COMPNT_PVCCSA)) {
			i2c_msg.target_addr = VCCIN_VCCSA_ADDR;
		}
		if ((component == DL_COMPNT_PVCCIO) || (component == DL_COMPNT_P3V3_STBY)) {
			i2c_msg.target_addr = VCCIO_P3V3_STBY_ADDR;
		}
		if (component == DL_COMPNT_PVDDR_ABC) {
			i2c_msg.target_addr = VDDQ_ABC_ADDR;
		}
		if (component == DL_COMPNT_PVDDR_DEF) {
			i2c_msg.target_addr = VDDQ_DEF_ADDR;
		}

		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 7;
		i2c_msg.bus = I2C_BUS8;
		i2c_msg.data[0] = PMBUS_IC_DEVICE_ID;

		if (i2c_master_read(&i2c_msg, retry)) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		if (memcmp(i2c_msg.data, ISL69254_DEVICE_ID, sizeof(ISL69254_DEVICE_ID)) == 0) {
			/* Renesas isl69254 */

			if (isl69254iraz_t_get_checksum(i2c_msg.bus, i2c_msg.target_addr,
							&(msg->data[0])) == false) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			if (isl69254iraz_t_get_remaining_write(i2c_msg.bus, i2c_msg.target_addr,
							       &(msg->data[4])) == false) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			msg->data[5] = VENDOR_RENESAS;
			msg->data_len = 6;
			msg->completion_code = CC_SUCCESS;

		} else if (memcmp(i2c_msg.data, XDPE12284C_DEVICE_ID,
				  sizeof(XDPE12284C_DEVICE_ID)) == 0) {
			/* Infineon xdpe12284c */

			if (k_mutex_lock(&vr_page_mutex, K_MSEC(VR_PAGE_MUTEX_TIMEOUT_MS))) {
				printf("[%s] Failed to lock vr page\n", __func__);
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			if (xdpe12284c_get_checksum(i2c_msg.bus, i2c_msg.target_addr,
						    &(msg->data[0])) == false) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				goto unlock_exit;
			}

			if (xdpe12284c_get_remaining_write(i2c_msg.bus, i2c_msg.target_addr,
							   &(msg->data[4])) == false) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				goto unlock_exit;
			}
			msg->data[5] = VENDOR_INFINEON;
			msg->data_len = 6;
			msg->completion_code = CC_SUCCESS;

			goto unlock_exit;

		} else {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
		}

		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	return;

unlock_exit:
	if (k_mutex_unlock(&vr_page_mutex)) {
		printf("[%s] Failed to unlock vr page\n", __func__);
	}

	return;
}

void OEM_1S_GET_GPIO(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	// only input enable status
	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t eight_bit_value = 0, gpio_value = 0;
	// Bump up the gpio_align_table_length to multiple of 8.
	uint8_t gpio_cnt = gpio_align_table_length + (8 - (gpio_align_table_length % 8));
	uint8_t data_len = gpio_cnt / 8;
	msg->data_len = data_len;
	for (uint8_t i = 0; i < gpio_cnt; i++) {
		if ((gpio_align_t[i] == PVCCIO_CPU) || (gpio_align_t[i] == BMC_HEARTBEAT_LED_R) ||
		    (gpio_align_t[i] == FM_FORCE_ADR_N_R)) {
			//pass dummy data to bmc, because AST bic do not have this gpio
			gpio_value = 0;
		} else {
			//if i large than length, fill 0 into the last byte
			gpio_value = (i >= gpio_align_table_length) ? 0 : gpio_get(gpio_align_t[i]);
		}

		// clear temporary variable to avoid return wrong GPIO value
		if (i % 8 == 0) {
			eight_bit_value = 0;
		}

		eight_bit_value = eight_bit_value | (gpio_value << (i % 8));
		msg->data[i / 8] = eight_bit_value;
	}
	msg->completion_code = CC_SUCCESS;

	return;
}

uint8_t gpio_idx_exchange(ipmi_msg *msg)
{
	if (msg == NULL)
		return 1;
	if (msg->data_len < 2)
		return 1;

	int need_change = 1;
	switch (msg->data[0]) {
	case GET_GPIO_STATUS:
	case GET_GPIO_DIRECTION_STATUS:
		if (msg->data_len == 3) {
			if (msg->data[2] == GLOBAL_GPIO_IDX_KEY) {
				need_change = 0;
			}
			// Ignore last data byte if provided.
			msg->data_len--;
		}
		break;
	case SET_GPIO_OUTPUT_STATUS:
	case SET_GPIO_DIRECTION_STATUS:
		if (msg->data_len == 4) {
			if (msg->data[3] == GLOBAL_GPIO_IDX_KEY) {
				need_change = 0;
			}
			// Ignore last data byte if provided.
			msg->data_len--;
		}
		break;
	default:
		break;
	}

	if (need_change)
		msg->data[1] = gpio_align_t[msg->data[1]];
	return 0;
}

void OEM_1S_GET_SET_GPIO(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	// whether need to change gpio index type
	if (gpio_idx_exchange(msg)) {
		msg->data_len = 0;
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	uint8_t gpio_num = msg->data[1];
	if (gpio_num >= TOTAL_GPIO_NUM) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (gpio_cfg[gpio_num].is_init == DISABLE) {
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	if ((gpio_num == PVCCIO_CPU) || (gpio_num == BMC_HEARTBEAT_LED_R) ||
	    (gpio_num == FM_FORCE_ADR_N_R)) {
		msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
		msg->data_len = 0;
		return;
	}

	uint8_t completion_code = CC_INVALID_LENGTH;
	switch (msg->data[0]) {
	case GET_GPIO_STATUS:
		if (msg->data_len == 2) {
			msg->data[0] = gpio_num;
			msg->data[1] = gpio_get(gpio_num);
			completion_code = CC_SUCCESS;
		}
		break;
	case SET_GPIO_OUTPUT_STATUS:
		if (msg->data_len == 3) {
			msg->data[0] = gpio_num;
			gpio_set(gpio_num, msg->data[2]);
			msg->data[1] = gpio_get(gpio_num);
			completion_code = CC_SUCCESS;
		}
		break;
	case GET_GPIO_DIRECTION_STATUS:
		if (msg->data_len == 2) {
			msg->data[0] = gpio_num;
			msg->data[1] = gpio_get_direction(gpio_num);
			completion_code = CC_SUCCESS;
		}
		break;
	case SET_GPIO_DIRECTION_STATUS:
		if (msg->data_len == 3) {
			if (msg->data[2]) {
				gpio_conf(gpio_num, GPIO_OUTPUT);
			} else {
				gpio_conf(gpio_num, GPIO_INPUT);
			}
			msg->data[0] = gpio_num;
			msg->data[1] = msg->data[2];
			completion_code = CC_SUCCESS;
		}
		break;
	default:
		printf("[%s] Unknown options(0x%x)", __func__, msg->data[0]);
		return;
	}

	if (completion_code != CC_SUCCESS) {
		msg->data_len = 0;
	} else {
		msg->data_len = 2; // Return GPIO number, status
	}
	msg->completion_code = completion_code;
	return;
}
