#include "plat_ipmi.h"

#include <stdio.h>
#include <stdlib.h>

#include "libutil.h"
#include "ipmi.h"
#include "plat_class.h"
#include "plat_ipmb.h"
#include "pmbus.h"
#include "plat_sensor_table.h"
#include "util_sys.h"

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
			i2c_msg.tx_len = 3;
			i2c_msg.data[0] = 0xC7; //command code
			i2c_msg.data[1] = 0x3F; //register
			i2c_msg.data[2] = 0x00; //dummy data

			if (i2c_master_write(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			i2c_msg.tx_len = 1;
			i2c_msg.rx_len = 4;
			i2c_msg.data[0] = 0xC5; //command code

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

		} else if (memcmp(i2c_msg.data, XDPE12284C_DEVICE_ID,
				  sizeof(XDPE12284C_DEVICE_ID)) == 0) {
			/* Infineon xdpe12284c */
			i2c_msg.tx_len = 2;
			i2c_msg.data[0] = 0x00;
			i2c_msg.data[1] = 0x62; //set page to 0x62

			if (i2c_master_write(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			//Read lower word for the 32bit checksum value
			i2c_msg.tx_len = 1;
			i2c_msg.rx_len = 2;
			i2c_msg.data[0] = 0x43;
			if (i2c_master_read(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			msg->data[0] = i2c_msg.data[1];
			msg->data[1] = i2c_msg.data[0];

			i2c_msg.tx_len = 2;
			i2c_msg.data[0] = 0x00;
			i2c_msg.data[1] = 0x62; //set page to 0x62

			if (i2c_master_write(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			//Read higher word for the 32bit checksum value
			i2c_msg.tx_len = 1;
			i2c_msg.rx_len = 2;
			i2c_msg.data[0] = 0x42;
			if (i2c_master_read(&i2c_msg, retry)) {
				msg->completion_code = CC_UNSPECIFIED_ERROR;
				return;
			}

			msg->data[2] = i2c_msg.data[1];
			msg->data[3] = i2c_msg.data[0];
			msg->data_len = 4;
			msg->completion_code = CC_SUCCESS;
		} else {
			msg->completion_code = CC_UNSPECIFIED_ERROR;
		}
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}
	return;
}
