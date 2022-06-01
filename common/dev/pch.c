#include <stdio.h>
#include <stdlib.h>
#include "sensor.h"
#include "libutil.h"
#include "ipmi.h"
#include "plat_ipmb.h"

#define VALID_READING 0x40

uint8_t pch_read(uint8_t sensor_num, int *reading)
{
	if (!reading || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}
#if MAX_IPMB_IDX
	ipmb_error status;
	ipmi_msg *bridge_msg;
	bridge_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (bridge_msg == NULL) {
		printf("pch_read bridge message alloc fail\n");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	/* read sensor from ME */
	bridge_msg->seq_source = 0xff;
	bridge_msg->netfn = NETFN_SENSOR_REQ;
	bridge_msg->cmd = CMD_SENSOR_GET_SENSOR_READING;
	bridge_msg->InF_source = SELF;
	bridge_msg->InF_target = ME_IPMB;
	bridge_msg->data_len = 1;
	/* parameter offset is the sensor number to read from pch */
	bridge_msg->data[0] = sensor_config[sensor_config_index_map[sensor_num]].offset;

	uint8_t pch_retry_num = 0;
	for (pch_retry_num = 0; pch_retry_num < 4; pch_retry_num++) {
		status = ipmb_read(bridge_msg, IPMB_inf_index_map[bridge_msg->InF_target]);
		if (status != IPMB_ERROR_SUCCESS) {
			printf("pch_read ipmb read fail, ret %d\n", status);
			SAFE_FREE(bridge_msg);
			return SENSOR_FAIL_TO_ACCESS;
		}

		/* BIC reads the PCH sensors from ME through IPMI command.
		 * The completion code, the scanning and reading state should be checked.
		 * If the completion code is not successfully or the scanning and reading state are disabled or unavailable,
		 * BIC returns the unspecified error completion code.
		 * Follow the IPMI spec table 35 - get sensor reading command,
		 * the byte 3 of response data is
		 * bit-6: 0b means sensor scanning disabled
		 * bit-5: 1b means reading state unavailable
		 */
		if ((bridge_msg->completion_code == CC_SUCCESS) &&
		    ((bridge_msg->data[1] & (BIT(5) | BIT(6))) == VALID_READING)) {
			sensor_val *sval = (sensor_val *)reading;
			memset(sval, 0, sizeof(sensor_val));
			sval->integer = bridge_msg->data[0];
			SAFE_FREE(bridge_msg);
			return SENSOR_READ_SUCCESS;
		} else if (bridge_msg->completion_code == CC_NODE_BUSY) {
			continue;
		} else {
			SAFE_FREE(bridge_msg);
			return SENSOR_UNSPECIFIED_ERROR;
		}
	}

	printf("pch_read retry read fail\n");
	SAFE_FREE(bridge_msg);
#endif
	return SENSOR_UNSPECIFIED_ERROR;
}

uint8_t pch_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = pch_read;
	return SENSOR_INIT_SUCCESS;
}
