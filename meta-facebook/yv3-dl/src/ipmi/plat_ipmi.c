/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "plat_ipmi.h"

#include <stdio.h>
#include <stdlib.h>
#include <logging/log.h>

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

LOG_MODULE_DECLARE(ipmi);

#define I2C_MAX_RETRY 3

#define FEXP_BIC_I2C_WRITE_IF 0x20
#define FEXP_BIC_I2C_READ_IF 0x21
#define FEXP_BIC_I2C_UPDATE_IF 0x22
#define FEXP_BIC_IPMI_I2C_SW_IF 0x23

#define BIC_COMMAND_STATUS 0x23

#define SNOWFLAKE_BIC_CMD_STATUS_SIZE 3
#define SNOWFLAKE_BIC_BOOTLOADER_RESPONSE_SIZE 5 // ack 2 + response 3
#define SNOWFLAKE_BIC_BOOTLOADER_ACK_SIZE 2
#define EXTEND_MSG_OUT_CC_OFFSET 4

static void fexp_bic_i2c_read(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data[EXTEND_MSG_OUT_CC_OFFSET] = 0;

	I2C_MSG i2c_msg = { .bus = IPMB_EXP1_BUS, .target_addr = BIC1_I2C_ADDRESS };
	i2c_msg.rx_len = SNOWFLAKE_BIC_BOOTLOADER_ACK_SIZE;
	if (i2c_master_read(&i2c_msg, I2C_MAX_RETRY)) {
		LOG_ERR("read response from snowflake bootloader failed");
		msg->data[EXTEND_MSG_OUT_CC_OFFSET] = 1;
	}

	if (i2c_msg.data[0] != 0x00 || i2c_msg.data[1] != 0xCC)
		msg->data[EXTEND_MSG_OUT_CC_OFFSET] = 1;

	return;
}

static void fexp_bic_i2c_write(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	I2C_MSG i2c_msg = { .bus = IPMB_EXP1_BUS, .target_addr = BIC1_I2C_ADDRESS };
	i2c_msg.tx_len = msg->data_len;
	memcpy(i2c_msg.data, msg->data, msg->data_len);
	if (i2c_master_write(&i2c_msg, I2C_MAX_RETRY)) {
		LOG_ERR("write to snowflake bootloader failed");
		msg->data[EXTEND_MSG_OUT_CC_OFFSET] = 1;
	} else {
		msg->data[EXTEND_MSG_OUT_CC_OFFSET] = 0;
	}

	return;
}

static void fexp_bic_update(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	I2C_MSG i2c_msg = { .bus = IPMB_EXP1_BUS, .target_addr = BIC1_I2C_ADDRESS };

	// step 1. send "get status" command
	i2c_msg.tx_len = SNOWFLAKE_BIC_CMD_STATUS_SIZE;
	i2c_msg.data[0] = SNOWFLAKE_BIC_CMD_STATUS_SIZE;
	i2c_msg.data[1] = BIC_COMMAND_STATUS; //checksum same as data
	i2c_msg.data[2] = BIC_COMMAND_STATUS;
	if (i2c_master_write(&i2c_msg, I2C_MAX_RETRY)) {
		LOG_ERR("write command to snowflake bootloader failed");
		return;
	}

	// step 2. read response data of "get status"
	memset(i2c_msg.data, 0, sizeof(i2c_msg.data));
	i2c_msg.tx_len = 0;
	i2c_msg.rx_len = SNOWFLAKE_BIC_BOOTLOADER_RESPONSE_SIZE;
	if (i2c_master_read(&i2c_msg, I2C_MAX_RETRY)) {
		LOG_ERR("read response from snowflake bootloader failed");
		return;
	}

	// step 3. send ack
	memset(i2c_msg.data, 0, sizeof(i2c_msg.data));
	i2c_msg.rx_len = 0;
	i2c_msg.tx_len = 1;
	i2c_msg.data[0] = 0xCC;
	if (i2c_master_write(&i2c_msg, I2C_MAX_RETRY)) {
		LOG_ERR("write ack to snowflake bootloader failed");
		return;
	}

	// step 4. brdige message to bootloader
	i2c_msg.tx_len = msg->data_len;
	memcpy(i2c_msg.data, msg->data, msg->data_len);
	if (i2c_master_write(&i2c_msg, I2C_MAX_RETRY)) {
		LOG_ERR("bridge message to snowflake bootloader failed");
		return;
	}

	// step 5. read the ack for previous step
	memset(i2c_msg.data, 0xFF, sizeof(i2c_msg.data));
	i2c_msg.tx_len = 0;
	i2c_msg.rx_len = SNOWFLAKE_BIC_BOOTLOADER_ACK_SIZE;
	if (i2c_master_read(&i2c_msg, I2C_MAX_RETRY)) {
		LOG_ERR("read response from snowflake bootloader failed");
		return;
	}

	msg->data[EXTEND_MSG_OUT_CC_OFFSET] = 1;
}

int pal_extend_msg_out_interface_handler(ipmi_msg *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, 0);

	static uint8_t fexp_i2c_freq = I2C_SPEED_FAST_PLUS;
	const uint8_t target_IF = msg->data[0];

	// remove target_IF
	memmove(msg->data, msg->data + 1, msg->data_len - 1);
	msg->data_len -= 1;

	switch (target_IF) {
	case FEXP_BIC_I2C_WRITE_IF:
		fexp_bic_i2c_write(msg);
		break;

	case FEXP_BIC_I2C_READ_IF:
		fexp_bic_i2c_read(msg);
		break;

	case FEXP_BIC_I2C_UPDATE_IF:
		fexp_bic_update(msg);
		break;

	case FEXP_BIC_IPMI_I2C_SW_IF:
		msg->data[EXTEND_MSG_OUT_CC_OFFSET] = 1;

		if ((msg->data[0] == 0) && (fexp_i2c_freq == I2C_SPEED_FAST_PLUS)) {
			// start to update 1ou expander board TI BIC,
			// switch frequency to 100KHz and disable ipmb tx function
			fexp_i2c_freq = I2C_SPEED_STANDARD;
			ipmb_tx_suspend(IPMB_inf_index_map[EXP1_IPMB]);
		} else if ((msg->data[0] == 1) && (fexp_i2c_freq == I2C_SPEED_STANDARD)) {
			// 1ou expander board TI BIC update is completed,
			// switch frequency to 1MHz and enable ipmb tx function
			fexp_i2c_freq = I2C_SPEED_FAST_PLUS;
			ipmb_tx_resume(IPMB_inf_index_map[EXP1_IPMB]);
		} else {
			msg->data[EXTEND_MSG_OUT_CC_OFFSET] = 0;
		}

		if (msg->data[EXTEND_MSG_OUT_CC_OFFSET])
			i2c_freq_set(IPMB_EXP1_BUS, fexp_i2c_freq, 1);
		break;

	default:
		LOG_ERR("unsupported target interface %x", target_IF);
		break;
	}

	msg->completion_code = CC_SUCCESS;
	msg->data_len = 5;
	msg->data[0] = IANA_ID & 0xFF;
	msg->data[1] = (IANA_ID >> 8) & 0xFF;
	msg->data[2] = (IANA_ID >> 16) & 0xFF;
	msg->data[3] = target_IF;

	ipmb_error status = ipmb_send_response(msg, IPMB_inf_index_map[msg->InF_source]);
	if (status != IPMB_ERROR_SUCCESS)
		LOG_ERR("OEM_MSG_OUT send IPMB resp fail status: %x", status);
	return 0;
}

void OEM_1S_GET_CARD_TYPE(ipmi_msg *msg)
{
	if (msg == NULL) {
		LOG_ERR("Failed due to parameter *msg is NULL");
		return;
	}

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	CARD_STATUS _1ou_status = get_1ou_status();
	CARD_STATUS _2ou_status = get_2ou_status();
	switch (msg->data[0]) {
	case GET_1OU_CARD_TYPE:
		msg->data_len = 1;
		msg->completion_code = CC_SUCCESS;
		if (_1ou_status.present) {
			msg->data[0] = _1ou_status.card_type;
		} else {
			msg->data[0] = TYPE_1OU_ABSENT;
		}
		break;
	case GET_2OU_CARD_TYPE:
		msg->data_len = 1;
		msg->completion_code = CC_SUCCESS;
		if (_2ou_status.present) {
			msg->data[0] = _2ou_status.card_type;
		} else {
			msg->data[0] = TYPE_2OU_ABSENT;
		}
		break;
	default:
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	}

	return;
}

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
			LOG_ERR("IPMB read fail status: %x", status);
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
				LOG_ERR("Failed to lock vr page");
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			if (xdpe12284c_get_checksum(i2c_msg.bus, i2c_msg.target_addr,
						    &(msg->data[0])) == false) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				goto unlock_exit;
			}

			if (xdpe12284c_get_remaining_write(i2c_msg.bus, i2c_msg.target_addr,
							   (uint16_t *)&(msg->data[4])) == false) {
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
		LOG_ERR("Failed to unlock vr page");
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
		    (gpio_align_t[i] == FM_FORCE_ADR_N_R) ||
		    (gpio_align_t[i] == JTAG_BMC_NTRST_R_N)) {
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

void OEM_1S_GET_GPIO_CONFIG(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len == 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t idx = 0;
	uint8_t *gpio_mask = (uint8_t *)malloc(msg->data_len * sizeof(uint8_t));
	if (gpio_mask == NULL) {
		return;
	}
	memcpy(gpio_mask, &msg->data[0], msg->data_len);
	for (uint8_t i = 0; i < gpio_align_table_length; i++) {
		//check enough data
		if (i / 8 == msg->data_len) {
			break;
		}

		if (gpio_mask[i / 8] & (1 << (i % 8))) {
			if ((gpio_align_t[i] == PVCCIO_CPU) ||
			    (gpio_align_t[i] == BMC_HEARTBEAT_LED_R) ||
			    (gpio_align_t[i] == FM_FORCE_ADR_N_R) ||
			    (gpio_align_t[i] == JTAG_BMC_NTRST_R_N)) {
				//pass dummy data to bmc, because AST bic do not have this gpio
				msg->data[idx] = 0;
			} else {
				msg->data[idx] = gpio_get_direction(gpio_align_t[i]);
				msg->data[idx] |= gpio_get_reg_value(gpio_align_t[i],
								     REG_INTERRUPT_ENABLE_OFFSET)
						  << GPIO_CONF_SET_INT;
				msg->data[idx] |= gpio_get_reg_value(gpio_align_t[i],
								     REG_INTERRUPT_TYPE1_OFFSET)
						  << GPIO_CONF_SET_TRG_TYPE;
				if (gpio_get_reg_value(gpio_align_t[i],
						       REG_INTERRUPT_TYPE2_OFFSET)) {
					msg->data[idx] |= 1 << GPIO_CONF_SET_TRG_BOTH;
				} else {
					msg->data[idx] |=
						gpio_get_reg_value(gpio_align_t[i],
								   REG_INTERRUPT_TYPE0_OFFSET)
						<< GPIO_CONF_SET_TRG_EDGE;
				}
			}
			idx++;
		}
	}
	free(gpio_mask);
	msg->data_len = idx;
	msg->completion_code = CC_SUCCESS;

	return;
}

#ifdef ENABLE_GPIO_SET_CONFG
void OEM_1S_SET_GPIO_CONFIG(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	uint8_t gpio_bytes_len = (gpio_align_table_length + 7) / 8;

	if (msg->data_len < gpio_bytes_len + 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t idx = gpio_bytes_len;
	for (uint8_t i = 0; i < gpio_align_table_length; i++) {
		//check mask
		if (msg->data[i / 8] & (1 << (i % 8))) {
			if ((gpio_align_t[i] == PVCCIO_CPU) ||
			    (gpio_align_t[i] == BMC_HEARTBEAT_LED_R) ||
			    (gpio_align_t[i] == FM_FORCE_ADR_N_R) ||
			    (gpio_align_t[i] == JTAG_BMC_NTRST_R_N)) {
				//AST bic do not have this gpio
			} else {
				//setting direction
				if (msg->data[idx] & BIT(GPIO_CONF_SET_DIR)) {
					gpio_conf(gpio_align_t[i], GPIO_OUTPUT);
				} else {
					gpio_conf(gpio_align_t[i], GPIO_INPUT);
				}
				//setting interrupt
				if (msg->data[idx] & BIT(GPIO_CONF_SET_INT)) {
					if (msg->data[idx] & BIT(GPIO_CONF_SET_TRG_BOTH)) {
						gpio_interrupt_conf(gpio_align_t[i],
								    GPIO_INT_EDGE_BOTH);
					} else {
						if (msg->data[idx] & BIT(GPIO_CONF_SET_TRG_TYPE)) {
							if (msg->data[idx] &
							    BIT(GPIO_CONF_SET_TRG_EDGE)) {
								gpio_interrupt_conf(
									gpio_align_t[i],
									GPIO_INT_LEVEL_HIGH);
							} else {
								gpio_interrupt_conf(
									gpio_align_t[i],
									GPIO_INT_LEVEL_LOW);
							}
						} else {
							if (msg->data[idx] &
							    BIT(GPIO_CONF_SET_TRG_EDGE)) {
								gpio_interrupt_conf(
									gpio_align_t[i],
									GPIO_INT_EDGE_RISING);
							} else {
								gpio_interrupt_conf(
									gpio_align_t[i],
									GPIO_INT_EDGE_FALLING);
							}
						}
					}
				} else {
					gpio_interrupt_conf(gpio_align_t[i], GPIO_INT_DISABLE);
				}
			}
			idx++;
		}
	}
	msg->data_len = idx;
	msg->completion_code = CC_SUCCESS;

	return;
}
#endif

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
	    (gpio_num == FM_FORCE_ADR_N_R) || (gpio_num == JTAG_BMC_NTRST_R_N)) {
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
		LOG_ERR("Unknown options(0x%x)", msg->data[0]);
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
