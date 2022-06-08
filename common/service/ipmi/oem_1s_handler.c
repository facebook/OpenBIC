#include "oem_1s_handler.h"
#include <stdlib.h>
#include <drivers/peci.h>
#include "libutil.h"
#include "ipmb.h"
#include "sensor.h"
#include "snoop.h"
#include "hal_gpio.h"
#include "hal_i2c.h"
#include "hal_jtag.h"
#include "hal_peci.h"
#include "plat_def.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_ipmi.h"
#include "plat_sensor_table.h"
#include "plat_sys.h"
#ifdef ENABLE_FAN
#include "plat_fan.h"
#endif
#include "plat_ipmb.h"
#include "power_status.h"
#include "pmbus.h"
#include "altera.h"
#include "util_spi.h"
#include "util_sys.h"

#define BIOS_UPDATE_MAX_OFFSET 0x4000000
#define BIC_UPDATE_MAX_OFFSET 0x50000

__weak void OEM_1S_MSG_OUT(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}
#if MAX_IPMB_IDX
	uint8_t target_IF;
	ipmb_error status;
	ipmi_msg *bridge_msg = NULL;

	// If command is valid the default cc is CC_INVALID_CMD, set cc to CC_SUCCESS before we execute the command.
	// If the default cc is CC_INVALID_IANA, call ipmb_send_response for this invalid command.
	if (msg->completion_code != CC_INVALID_IANA) {
		msg->completion_code = CC_SUCCESS;
	}

	// Should input target, netfn, cmd
	if (msg->data_len <= 2) {
		msg->completion_code = CC_INVALID_LENGTH;
	}

	target_IF = msg->data[0];

	if (target_IF == PEER_BMC_IPMB) {
		if (msg->InF_source == SLOT1_BIC) {
			target_IF = msg->data[0] = SLOT3_BIC;
		} else if (msg->InF_source == SLOT3_BIC) {
			target_IF = msg->data[0] = SLOT1_BIC;
		} else {
			msg->completion_code = CC_INVALID_DATA_FIELD;
		}

		if (msg->data[1] == NETFN_STORAGE_REQ) {
			msg->data[1] = msg->data[1] << 2;
		}
	}

	// Bridge to invalid or disabled interface
	if ((IPMB_config_table[IPMB_inf_index_map[target_IF]].interface == RESERVED_IF) ||
	    (IPMB_config_table[IPMB_inf_index_map[target_IF]].enable_status == DISABLE)) {
		printf("OEM_MSG_OUT: Invalid bridge interface: %x\n", target_IF);
		msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
	}

	// only send to target while msg is valid
	if (msg->completion_code == CC_SUCCESS) {
		bridge_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
		if (bridge_msg == NULL) {
			msg->completion_code = CC_OUT_OF_SPACE;
		} else {
			memset(bridge_msg, 0, sizeof(ipmi_msg));

			if (DEBUG_IPMI) {
				printf("bridge targetIf %x, len %d, netfn %x, cmd %x\n", target_IF,
				       msg->data_len, msg->data[1] >> 2, msg->data[2]);
			}

			bridge_msg->data_len = msg->data_len - 3;
			bridge_msg->seq_source = msg->seq_source;
			bridge_msg->InF_target = msg->data[0];
			bridge_msg->InF_source = msg->InF_source;
			bridge_msg->netfn = msg->data[1] >> 2;
			bridge_msg->cmd = msg->data[2];

			if (bridge_msg->data_len != 0) {
				memcpy(&bridge_msg->data[0], &msg->data[3],
				       bridge_msg->data_len * sizeof(msg->data[0]));
			}

			status = ipmb_send_request(bridge_msg, IPMB_inf_index_map[target_IF]);

			if (status != IPMB_ERROR_SUCCESS) {
				printf("OEM_MSG_OUT send IPMB req fail status: %x", status);
				msg->completion_code = CC_BRIDGE_MSG_ERR;
			}
			SAFE_FREE(bridge_msg);
		}
	}

	// Return to source while data is invalid or sending req to Tx task fail
	if (msg->completion_code != CC_SUCCESS) {
		msg->data_len = 0;
		status = ipmb_send_response(msg, IPMB_inf_index_map[msg->InF_source]);
		if (status != IPMB_ERROR_SUCCESS) {
			printf("OEM_MSG_OUT send IPMB resp fail status: %x", status);
		}
	}
#else
	msg->completion_code = CC_UNSPECIFIED_ERROR;
#endif
	return;
}

__weak void OEM_1S_GET_GPIO(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	// only input enable status
	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t eight_bit_value = 0, gpio_value, gpio_cnt, data_len;
	// Bump up the gpio_ind_to_num_table_cnt to multiple of 8.
	gpio_cnt = gpio_ind_to_num_table_cnt + (8 - (gpio_ind_to_num_table_cnt % 8));
	data_len = gpio_cnt / 8;
	msg->data_len = data_len;
	for (uint8_t i = 0; i < gpio_cnt; i++) {
		gpio_value =
			(i >= gpio_ind_to_num_table_cnt) ? 0 : gpio_get(gpio_ind_to_num_table[i]);

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

__weak void OEM_1S_FW_UPDATE(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	/*********************************
	* Request Data
	*
	* Byte   0: [6:0] fw update target, [7] indicate last packet
	* Byte 1-4: offset, lsb first
	* Byte 5-6: length, lsb first
	* Byte 7-N: data
	***********************************/
	if (msg->data_len < 8) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t target = msg->data[0];
	uint8_t status = -1;
	uint32_t offset =
		((msg->data[4] << 24) | (msg->data[3] << 16) | (msg->data[2] << 8) | msg->data[1]);
	uint16_t length = ((msg->data[6] << 8) | msg->data[5]);

	if ((length == 0) || (length != msg->data_len - 7)) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (target == BIOS_UPDATE) {
		// BIOS size maximum 64M bytes
		if (offset > BIOS_UPDATE_MAX_OFFSET) {
			msg->completion_code = CC_PARAM_OUT_OF_RANGE;
			return;
		}
		int pos = pal_get_bios_flash_position();
		if (pos == -1) {
			msg->completion_code = CC_INVALID_PARAM;
			return;
		}

		// Switch GPIO(BIOS SPI Selection Pin) to BIC
		bool ret = pal_switch_bios_spi_mux(GPIO_HIGH);
		if (!ret) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		status = fw_update(offset, length, &msg->data[7], (target & IS_SECTOR_END_MASK),
				   pos);

		// Switch GPIO(BIOS SPI Selection Pin) to PCH
		ret = pal_switch_bios_spi_mux(GPIO_LOW);
		if (!ret) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

	} else if ((target == BIC_UPDATE) || (target == (BIC_UPDATE | IS_SECTOR_END_MASK))) {
		// Expect BIC firmware size not bigger than 320k
		if (offset > BIC_UPDATE_MAX_OFFSET) {
			msg->completion_code = CC_PARAM_OUT_OF_RANGE;
			return;
		}
		status = fw_update(offset, length, &msg->data[7], (target & IS_SECTOR_END_MASK),
				   DEVSPI_FMC_CS0);

	} else if ((target == CPLD_UPDATE) || (target == (CPLD_UPDATE | IS_SECTOR_END_MASK))) {
		status = cpld_altera_max10_fw_update(offset, length, &msg->data[7]);

	} else {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	msg->data_len = 0;

	switch (status) {
	case FWUPDATE_SUCCESS:
		msg->completion_code = CC_SUCCESS;
		break;
	case FWUPDATE_OUT_OF_HEAP:
		msg->completion_code = CC_LENGTH_EXCEEDED;
		break;
	case FWUPDATE_OVER_LENGTH:
		msg->completion_code = CC_OUT_OF_SPACE;
		break;
	case FWUPDATE_REPEATED_UPDATED:
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	case FWUPDATE_UPDATE_FAIL:
		msg->completion_code = CC_TIMEOUT;
		break;
	case FWUPDATE_ERROR_OFFSET:
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	if (status != FWUPDATE_SUCCESS) {
		printf("firmware (0x%02X) update failed cc: %x\n", target, msg->completion_code);
	}

	return;
}

__weak void OEM_1S_GET_FW_VERSION(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t component;
	component = msg->data[0];
#if MAX_IPMB_IDX
	ipmb_error status;
	ipmi_msg *bridge_msg;
#endif

#ifdef ENABLE_ISL69260
	I2C_MSG i2c_msg;
	uint8_t retry = 3;
#endif

	switch (component) {
	case COMPNT_CPLD:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	case COMPNT_BIC:
		msg->data[0] = BIC_FW_YEAR_MSB;
		msg->data[1] = BIC_FW_YEAR_LSB;
		msg->data[2] = BIC_FW_WEEK;
		msg->data[3] = BIC_FW_VER;
		msg->data[4] = BIC_FW_platform_0;
		msg->data[5] = BIC_FW_platform_1;
		msg->data[6] = BIC_FW_platform_2;
		msg->data_len = 7;
		msg->completion_code = CC_SUCCESS;
		break;
#if MAX_IPMB_IDX
	case COMPNT_ME:
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
#endif
#ifdef ENABLE_ISL69260
	case COMPNT_PVCCIN:
	case COMPNT_PVCCFA_EHV_FIVRA:
	case COMPNT_PVCCD_HV:
	case COMPNT_PVCCINFAON:
	case COMPNT_PVCCFA_EHV:
		if ((component == COMPNT_PVCCIN) || (component == COMPNT_PVCCFA_EHV_FIVRA)) {
			i2c_msg.target_addr = PVCCIN_ADDR;
		}
		if (component == COMPNT_PVCCD_HV) {
			i2c_msg.target_addr = PVCCD_HV_ADDR;
		}
		if ((component == COMPNT_PVCCINFAON) || (component == COMPNT_PVCCFA_EHV)) {
			i2c_msg.target_addr = PVCCFA_EHV_ADDR;
		}
		i2c_msg.tx_len = 1;
		i2c_msg.rx_len = 7;
		i2c_msg.bus = I2C_BUS5;
		i2c_msg.data[0] = PMBUS_IC_DEVICE_ID;

		if (i2c_master_read(&i2c_msg, retry)) {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}

		if (i2c_msg.data[0] == 0x04 && i2c_msg.data[1] == 0x00 && i2c_msg.data[2] == 0x81 &&
		    i2c_msg.data[3] == 0xD2 && i2c_msg.data[4] == 0x49) {
			/* Renesas isl69259 */
			i2c_msg.tx_len = 3;
			i2c_msg.data[0] = 0xC7;
			i2c_msg.data[1] = 0x94;
			i2c_msg.data[2] = 0x00;

			if (i2c_master_write(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			i2c_msg.tx_len = 1;
			i2c_msg.rx_len = 4;
			i2c_msg.data[0] = 0xC5;

			if (i2c_master_read(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			msg->data[0] = i2c_msg.data[3];
			msg->data[1] = i2c_msg.data[2];
			msg->data[2] = i2c_msg.data[1];
			msg->data[3] = i2c_msg.data[0];
			msg->data_len = 4;
			msg->completion_code = CC_SUCCESS;

		} else if (i2c_msg.data[0] == 0x06 && i2c_msg.data[1] == 0x54 &&
			   i2c_msg.data[2] == 0x49 && i2c_msg.data[3] == 0x53 &&
			   i2c_msg.data[4] == 0x68 && i2c_msg.data[5] == 0x90 &&
			   i2c_msg.data[6] == 0x00) {
			/* TI tps53689 */
			i2c_msg.tx_len = 1;
			i2c_msg.rx_len = 2;
			i2c_msg.data[0] = 0xF4;

			if (i2c_master_read(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			msg->data[0] = i2c_msg.data[1];
			msg->data[1] = i2c_msg.data[0];
			msg->data_len = 2;
			msg->completion_code = CC_SUCCESS;

		} else if (i2c_msg.data[0] == 0x02 && i2c_msg.data[2] == 0x8A) {
			/* Infineon xdpe15284 */
			i2c_msg.tx_len = 6;
			i2c_msg.data[0] = 0xFD;
			i2c_msg.data[1] = 0x04;
			i2c_msg.data[2] = 0x00;
			i2c_msg.data[3] = 0x00;
			i2c_msg.data[4] = 0x00;
			i2c_msg.data[5] = 0x00;

			if (i2c_master_write(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			i2c_msg.tx_len = 2;
			i2c_msg.data[0] = 0xFE;
			i2c_msg.data[1] = 0x2D;

			if (i2c_master_write(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			i2c_msg.tx_len = 1;
			i2c_msg.rx_len = 5;
			i2c_msg.data[0] = 0xFD;

			if (i2c_master_read(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			msg->data[0] = i2c_msg.data[4];
			msg->data[1] = i2c_msg.data[3];
			msg->data[2] = i2c_msg.data[2];
			msg->data[3] = i2c_msg.data[1];
			msg->data_len = 4;
			msg->completion_code = CC_SUCCESS;
		} else {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
		}
		break;
#endif
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	return;
}

__weak void OEM_1S_RESET_BMC(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	int ret = pal_submit_bmc_cold_reset();
	if (ret == -1) {
		msg->completion_code = CC_INVALID_CMD;
	} else {
		msg->completion_code = CC_SUCCESS;
	}

	msg->data_len = 0;
	return;
}

#ifdef CONFIG_IPMI_KCS_ASPEED
__weak void OEM_1S_GET_POST_CODE(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	int postcode_num = snoop_read_num;
	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (postcode_num) {
		uint8_t offset = 0;
		if (snoop_read_num > SNOOP_MAX_LEN) {
			postcode_num = SNOOP_MAX_LEN;
			offset = snoop_read_num % SNOOP_MAX_LEN;
		}
		copy_snoop_read_buffer(offset, postcode_num, msg->data, COPY_ALL_POSTCODE);
	}

	for (int i = 0; i < (postcode_num / 2); ++i) {
		uint8_t tmp = msg->data[i];
		msg->data[i] = msg->data[postcode_num - i - 1];
		msg->data[postcode_num - i - 1] = tmp;
	}

	msg->data_len = postcode_num;
	msg->completion_code = CC_SUCCESS;
	return;
}
#endif

#ifdef CONFIG_PECI
__weak void OEM_1S_PECI_ACCESS(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	uint8_t addr, cmd, *writeBuf, *readBuf;
	uint8_t writeLen, readLen;
	int ret;

	if (msg->data_len < 3) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	addr = msg->data[0];
	writeLen = msg->data[1];
	readLen = msg->data[2];
	cmd = msg->data[3];

	// PECI driver would add 1 byte to check that data writing to host is correct
	// so input data len would one less then input writelen in write command.
	if ((msg->data_len - 3 != writeLen) && (msg->data_len - 3 != writeLen - 1)) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (msg->data_len == 3) {
		if ((writeLen == 0) && (readLen == 0)) {
			ret = peci_ping(addr);
			msg->data[0] = ret;
			msg->data_len = 1;
			msg->completion_code = CC_SUCCESS;
			return;
		} else {
			msg->completion_code = CC_INVALID_LENGTH;
			return;
		}
	} else {
		if ((cmd == PECI_GET_DIB_CMD) || (cmd == PECI_GET_TEMP0_CMD)) {
			readBuf = (uint8_t *)malloc(sizeof(uint8_t) * (readLen + 1));
		} else {
			readBuf = (uint8_t *)malloc(sizeof(uint8_t) * readLen);
		}
		writeBuf = (uint8_t *)malloc(sizeof(uint8_t) * writeLen);
		if ((readBuf == NULL) || (writeBuf == NULL)) {
			printf("PECI access util buffer alloc fail\n");
			SAFE_FREE(writeBuf);
			SAFE_FREE(readBuf);
			msg->completion_code = CC_OUT_OF_SPACE;
			return;
		}
		memcpy(&writeBuf[0], &msg->data[4], writeLen);

		ret = peci_write(cmd, addr, readLen, readBuf, writeLen, writeBuf);
		if (ret) {
			SAFE_FREE(writeBuf);
			SAFE_FREE(readBuf);
			msg->completion_code = CC_UNSPECIFIED_ERROR;
			return;
		}
		memcpy(&msg->data[0], &readBuf[0], readLen);

		SAFE_FREE(writeBuf);
		SAFE_FREE(readBuf);
		msg->data_len = readLen;
		msg->completion_code = CC_SUCCESS;
		return;
	}
}
#endif

#ifdef CONFIG_JTAG
__weak void OEM_1S_SET_JTAG_TAP_STA(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len != 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}
	uint8_t tapbitlen, tapdata;

	tapbitlen = msg->data[0];
	tapdata = msg->data[1];
	jtag_set_tap(tapdata, tapbitlen);

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_JTAG_DATA_SHIFT(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	uint8_t lastidx;
	uint16_t writebitlen, readbitlen, readbyte, databyte;

	writebitlen = (msg->data[1] << 8) | msg->data[0];
	databyte = (writebitlen + 7) >> 3;
	readbitlen = (msg->data[3 + databyte] << 8) | msg->data[2 + databyte];
	readbyte = (readbitlen + 7) >> 3;
	lastidx = msg->data[4 + databyte];

	if (msg->data_len != (5 + databyte)) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t shiftdata[databyte], receivedata[readbyte];
	memset(shiftdata, 0, databyte);
	memset(receivedata, 0, readbyte);
	memcpy(shiftdata, &msg->data[2], databyte);
	jtag_shift_data(writebitlen, shiftdata, readbitlen, receivedata, lastidx);

	memcpy(&msg->data[0], &receivedata[0], readbyte);
	msg->data_len = readbyte;
	msg->completion_code = CC_SUCCESS;
	return;
}

#ifdef ENABLE_ASD
__weak void OEM_1S_ASD_INIT(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (msg->data[0] == 0x01) {
		enable_PRDY_interrupt();
	} else if (msg->data[0] == 0xff) {
		disable_PRDY_interrupt();
	} else {
		disable_PRDY_interrupt();
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}
#endif
#endif

__weak void OEM_1S_SENSOR_POLL_EN(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (msg->data[0] == 1) {
		enable_sensor_poll();
	} else if (msg->data[0] == 0) {
		disable_sensor_poll();
	} else {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_ACCURACY_SENSOR_READING(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	ACCURACY_SENSOR_READING_REQ *req = (ACCURACY_SENSOR_READING_REQ *)msg->data;
	ACCURACY_SENSOR_READING_RES *res = (ACCURACY_SENSOR_READING_RES *)msg->data;
	uint8_t status = -1, sensor_report_status;
	int reading;
	if (msg->data_len != 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	// following IPMI sensor status response
	if (enable_sensor_poll_thread) {
		sensor_report_status = SENSOR_EVENT_MESSAGES_ENABLE | SENSOR_SCANNING_ENABLE;
	} else {
		sensor_report_status = SENSOR_EVENT_MESSAGES_ENABLE;
	}

	if (req->read_option == GET_FROM_CACHE) {
		if (enable_sensor_poll_thread) {
			status = get_sensor_reading(req->sensor_num, &reading, GET_FROM_CACHE);
		} else {
			status = SENSOR_POLLING_DISABLE;
		}
	} else if (req->read_option == GET_FROM_SENSOR) {
		status = get_sensor_reading(req->sensor_num, &reading, GET_FROM_SENSOR);
	} else {
		printf("Error: read_option was not either GET_FROM_CACHE or GET_FROM_SENSOR.\n");
		status = SENSOR_UNSPECIFIED_ERROR;
	}
	switch (status) {
	case SENSOR_READ_SUCCESS:
	case SENSOR_READ_ACUR_SUCCESS:
		res->decimal = (int16_t)reading;
		if (reading < 0) {
			res->fraction = (uint16_t)((res->decimal - reading + 0.0005) * 1000);
		} else {
			res->fraction = (uint16_t)((reading - res->decimal + 0.0005) * 1000);
		}
		msg->data[3] = sensor_report_status;
		msg->data_len = 4;
		msg->completion_code = CC_SUCCESS;
		break;
	case SENSOR_READ_4BYTE_ACUR_SUCCESS:
		memcpy(msg->data, &reading, sizeof(reading));
		msg->data[4] = sensor_report_status;
		msg->data_len = 5;
		msg->completion_code = CC_SUCCESS;
		break;
	case SENSOR_NOT_ACCESSIBLE:
	case SENSOR_INIT_STATUS:
		res->decimal = 0;
		res->fraction = 0;
		// notice BMC about sensor temporary in not accessible status
		msg->data[4] = (sensor_report_status | SENSOR_READING_STATE_UNAVAILABLE);
		msg->data_len = 5;
		msg->completion_code = CC_SUCCESS;
		break;
	case SENSOR_POLLING_DISABLE:
		// getting sensor cache while sensor polling disable
		msg->completion_code = CC_SENSOR_NOT_PRESENT;
		break;
	case SENSOR_FAIL_TO_ACCESS:
		// transaction error
		msg->completion_code = CC_NODE_BUSY;
		break;
	case SENSOR_NOT_FOUND:
		// request sensor number not found
		msg->completion_code = CC_INVALID_DATA_FIELD;
		break;
	case SENSOR_UNSPECIFIED_ERROR:
	default:
		// unknown error
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	return;
}

__weak void OEM_1S_GET_SET_GPIO(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	uint8_t completion_code = CC_INVALID_LENGTH;
	uint8_t gpio_num = gpio_ind_to_num_table[msg->data[1]];

	switch (msg->data[0]) {
	case GET_GPIO_OUTPUT_STATUS:
		if (msg->data_len == 2) {
			msg->data[0] = gpio_num;
			msg->data[1] = gpio_get(gpio_num);
			completion_code = CC_SUCCESS;
		}
		break;
	case SET_GPIO_OUTPUT_STATUS:
		if (msg->data_len == 3) {
			msg->data[0] = gpio_num;
			gpio_conf(gpio_num, GPIO_OUTPUT);
			gpio_set(gpio_num, msg->data[2]);
			msg->data[1] = gpio_get(gpio_num);
			completion_code = CC_SUCCESS;
		}
		break;
	case GET_GPIO_DIRECTION_STATUS:
		completion_code = CC_NOT_SUPP_IN_CURR_STATE;
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

__weak void OEM_1S_I2C_DEV_SCAN(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len != 1) { // only input scan bus
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t bus = msg->data[0];

	i2c_scan(bus, &msg->data[0], (uint8_t *)&msg->data_len);

	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_GET_BIC_STATUS(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->data[0] = FIRMWARE_REVISION_1;
	msg->data[1] = FIRMWARE_REVISION_2;

	msg->data_len = 2;
	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_SET_VR_MONITOR_STATUS(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	if (msg->data[0] == 0) {
		set_vr_monitor_status(false);
	} else if (msg->data[0] == 1) {
		set_vr_monitor_status(true);
	} else {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_GET_VR_MONITOR_STATUS(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->data[0] = (uint8_t)get_vr_monitor_status();
	msg->data_len = 1;
	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_RESET_BIC(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	submit_bic_warm_reset();

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_12V_CYCLE_SLOT(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	int ret = pal_submit_12v_cycle_slot(msg);
	switch (ret) {
	case SUCCESS_12V_CYCLE_SLOT:
		msg->completion_code = CC_SUCCESS;
		break;
	case NOT_SUPPORT_12V_CYCLE_SLOT:
		msg->completion_code = CC_INVALID_CMD;
		break;
	case SLOT_OFF_FAILED:
	case SLOT_ON_FAILED:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}

	return;
}

__weak void OEM_1S_READ_BIC_REGISTER(ipmi_msg *msg)
{
	/*********************************
	* data 0~3: start of register address to read, LSB first
	* data 4  : bytes to read
	***********************************/
	if (!msg) {
		printf("pal_OEM_1S_READ_BIC_REGISTER: parameter msg is NULL\n");
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}
	if (msg->data_len != 5) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}
	uint32_t *addr = (uint32_t *)(msg->data[0] | (msg->data[1] << 8) | (msg->data[2] << 16) |
				      (msg->data[3] << 24));
	uint8_t read_len = msg->data[4];
	memcpy(&msg->data[0], (uint8_t *)addr, read_len);

	msg->data_len = read_len;
	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void OEM_1S_WRITE_BIC_REGISTER(ipmi_msg *msg)
{
	/*********************************
 	* data 0~3: start of register address to write, LSB first
 	* data 4  : bytes to write, <= 4 bytes
	* data 5~N: date to write, LSB first
	*
	* NOTE: The register address must be a multiple of 4
	***********************************/
	if (!msg) {
		printf("pal_OEM_1S_WRITE_BIC_REGISTER: parameter msg is NULL\n");
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}
	if (msg->data[4] < 1 || msg->data[4] > 4 || (msg->data_len != 5 + msg->data[4])) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint32_t addr =
		msg->data[0] | (msg->data[1] << 8) | (msg->data[2] << 16) | (msg->data[3] << 24);
	if (addr % 4) {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	uint8_t write_len = msg->data[4];
	uint32_t reg_data = *(uint32_t *)addr;
	/* replace write_len bytes */
	reg_data &= ~BIT_MASK(write_len * 8);
	for (uint8_t i = 0; i < write_len; i++) {
		reg_data |= msg->data[i + 5] << (i * 8);
	}
	*(uint32_t *)addr = reg_data;

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;
	return;
}

#ifdef ENABLE_FAN
__weak void OEM_1S_SET_FAN_DUTY_AUTO(ipmi_msg *msg)
{
	/*********************************
	Request -
	data 0: Fan pwm index
	data 1: Duty value
	Response -
	data 0: Completion code
	***********************************/
	if (msg == NULL) {
		printf("%s failed due to parameter *msg is NULL\n", __func__);
		return;
	}

	if (msg->data_len != 2) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t pwm_id = msg->data[0];
	uint8_t duty = msg->data[1];
	uint8_t current_fan_mode = FAN_AUTO_MODE, slot_index = 0;
	int ret = 0;

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;

	if (msg->InF_source == SLOT1_BIC) {
		slot_index = INDEX_SLOT1;
	} else if (msg->InF_source == SLOT3_BIC) {
		slot_index = INDEX_SLOT3;
	} else {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	if (pwm_id >= MAX_FAN_PWM_INDEX_COUNT) {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	ret = pal_get_fan_ctrl_mode(&current_fan_mode);
	if (ret < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		return;
	}

	if (current_fan_mode != FAN_AUTO_MODE) {
		printf("%s() is called when it's not at auto mode\n", __func__);
		return;
	}

	if (duty > MAX_FAN_DUTY_VALUE) {
		duty = MAX_FAN_DUTY_VALUE;
	}

	ret = pal_set_fan_duty(pwm_id, duty, slot_index);
	if (ret < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
	}

	return;
}

__weak void OEM_1S_GET_FAN_DUTY(ipmi_msg *msg)
{
	/*********************************
	Request -
	data 0: Fan pwm index
	Response -
	data 0: Completion code
	data 1: current fan duty
	***********************************/
	if (msg == NULL) {
		printf("%s failed due to parameter *msg is NULL\n", __func__);
		return;
	}

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t pwm_id = msg->data[0];
	uint8_t duty = 0, slot_index = 0;
	int ret = 0;

	msg->data_len = 0;
	if (msg->InF_source == SLOT1_BIC) {
		slot_index = INDEX_SLOT1;
	} else if (msg->InF_source == SLOT3_BIC) {
		slot_index = INDEX_SLOT3;
	} else {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	if (pwm_id >= MAX_FAN_PWM_INDEX_COUNT) {
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	ret = pal_get_fan_duty(pwm_id, &duty, slot_index);
	if (ret < 0) {
		msg->completion_code = CC_UNSPECIFIED_ERROR;
	} else {
		msg->data[0] = duty;
		msg->data_len = 1;
		msg->completion_code = CC_SUCCESS;
	}

	return;
}

__weak void OEM_1S_GET_FAN_RPM(ipmi_msg *msg)
{
	/*********************************
	Request -
	data 0: Fan index
	Response -
	data 0: Completion code
	data 1: Current fan rpm
	***********************************/
	if (msg == NULL) {
		printf("%s failed due to parameter *msg is NULL\n", __func__);
		return;
	}

	if (msg->data_len != 1) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t fan_id = msg->data[0];
	uint16_t data = 0;
	int ret = 0;

	ret = pal_get_fan_rpm(fan_id, &data);
	if (ret < 0) {
		msg->data_len = 0;
		msg->completion_code = CC_UNSPECIFIED_ERROR;

	} else {
		msg->data[0] = (data >> 8) & 0xFF;
		msg->data[1] = data & 0xFF;
		msg->data_len = 2;
		msg->completion_code = CC_SUCCESS;
	}

	return;
}
#endif

__weak void OEM_1S_INFORM_PEER_SLED_CYCLE(ipmi_msg *msg)
{
	if (msg == NULL) {
		printf("%s failed due to parameter *msg is NULL\n", __func__);
		return;
	}

	msg->data_len = 0;
	msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
	return;
}

__weak void OEM_1S_PEX_FLASH_READ(ipmi_msg *msg)
{
	return;
}

__weak void OEM_1S_GET_FPGA_USER_CODE(ipmi_msg *msg)
{
	return;
}

__weak void OEM_1S_GET_CARD_TYPE(ipmi_msg *msg)
{
	if (msg == NULL) {
		printf("%s failed due to parameter *msg is NULL\n", __func__);
		return;
	}

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

void IPMI_OEM_1S_handler(ipmi_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	switch (msg->cmd) {
	case CMD_OEM_1S_MSG_IN:
		break;
	case CMD_OEM_1S_MSG_OUT:
		OEM_1S_MSG_OUT(msg);
		break;
	case CMD_OEM_1S_GET_GPIO:
		OEM_1S_GET_GPIO(msg);
		break;
	case CMD_OEM_1S_SET_GPIO:
		break;
	case CMD_OEM_1S_FW_UPDATE:
		OEM_1S_FW_UPDATE(msg);
		break;
	case CMD_OEM_1S_GET_FW_VERSION:
		OEM_1S_GET_FW_VERSION(msg);
		break;
	case CMD_OEM_1S_RESET_BMC:
		OEM_1S_RESET_BMC(msg);
		break;
	case CMD_OEM_1S_SENSOR_POLL_EN: // debug command
		OEM_1S_SENSOR_POLL_EN(msg);
		break;
	case CMD_OEM_1S_ACCURACY_SENSOR_READING:
		OEM_1S_ACCURACY_SENSOR_READING(msg);
		break;
	case CMD_OEM_1S_GET_SET_GPIO:
		OEM_1S_GET_SET_GPIO(msg);
		break;
	case CMD_OEM_1S_I2C_DEV_SCAN: // debug command
		OEM_1S_I2C_DEV_SCAN(msg);
		break;
	case CMD_OEM_1S_GET_BIC_STATUS:
		OEM_1S_GET_BIC_STATUS(msg);
		break;
	case CMD_OEM_1S_SET_VR_MONITOR_STATUS:
		OEM_1S_SET_VR_MONITOR_STATUS(msg);
		break;
	case CMD_OEM_1S_GET_VR_MONITOR_STATUS:
		OEM_1S_GET_VR_MONITOR_STATUS(msg);
		break;
	case CMD_OEM_1S_RESET_BIC:
		OEM_1S_RESET_BIC(msg);
		break;
	case CMD_OEM_1S_12V_CYCLE_SLOT:
		OEM_1S_12V_CYCLE_SLOT(msg);
		break;
	case CMD_OEM_1S_READ_BIC_REGISTER:
		OEM_1S_READ_BIC_REGISTER(msg);
		break;
	case CMD_OEM_1S_WRITE_BIC_REGISTER:
		OEM_1S_WRITE_BIC_REGISTER(msg);
		break;
#ifdef CONFIG_IPMI_KCS_ASPEED
	case CMD_OEM_1S_GET_POST_CODE:
		OEM_1S_GET_POST_CODE(msg);
		break;
#endif
#ifdef CONFIG_PECI
	case CMD_OEM_1S_PECI_ACCESS:
		OEM_1S_PECI_ACCESS(msg);
		break;
#endif
#ifdef CONFIG_JTAG
	case CMD_OEM_1S_SET_JTAG_TAP_STA:
		OEM_1S_SET_JTAG_TAP_STA(msg);
		break;
	case CMD_OEM_1S_JTAG_DATA_SHIFT:
		OEM_1S_JTAG_DATA_SHIFT(msg);
		break;
#ifdef ENABLE_ASD
	case CMD_OEM_1S_ASD_INIT:
		OEM_1S_ASD_INIT(msg);
		break;
#endif
#endif
#ifdef ENABLE_FAN
	case CMD_OEM_1S_SET_FAN_DUTY_AUTO:
		OEM_1S_SET_FAN_DUTY_AUTO(msg);
		break;
	case CMD_OEM_1S_GET_FAN_DUTY:
		OEM_1S_GET_FAN_DUTY(msg);
		break;
	case CMD_OEM_1S_GET_FAN_RPM:
		OEM_1S_GET_FAN_RPM(msg);
		break;
#endif
	case CMD_OEM_1S_INFORM_PEER_SLED_CYCLE:
		OEM_1S_INFORM_PEER_SLED_CYCLE(msg);
		break;
	case CMD_OEM_1S_PEX_FLASH_READ:
		OEM_1S_PEX_FLASH_READ(msg);
		break;
	case CMD_OEM_1S_GET_FPGA_USER_CODE:
		OEM_1S_GET_FPGA_USER_CODE(msg);
		break;
	case CMD_OEM_1S_GET_CARD_TYPE:
		OEM_1S_GET_CARD_TYPE(msg);
		break;
	default:
		printf("Invalid OEM message, netfn(0x%x) cmd(0x%x)\n", msg->netfn, msg->cmd);
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_CMD;
		break;
	}
	return;
}
