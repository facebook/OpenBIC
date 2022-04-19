#include <stdio.h>
#include <stdlib.h>
#include "sensor.h"
#include "libutil.h"
#include "ipmi.h"

uint8_t pch_read(uint8_t sensor_num, int *reading)
{
	if (!reading || (sensor_num > SENSOR_NUM_MAX)) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

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
	status = ipmb_read(bridge_msg, IPMB_inf_index_map[bridge_msg->InF_target]);

	if (status != IPMB_ERROR_SUCCESS) {
		printf("pch_read ipmb read fail, ret %d\n", status);
		SAFE_FREE(bridge_msg);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	sensor_val *sval = (sensor_val *)reading;
	memset(sval, 0, sizeof(sensor_val));
	sval->integer = bridge_msg->data[0];

	SAFE_FREE(bridge_msg);
	return SENSOR_READ_SUCCESS;
}

uint8_t pch_init(uint8_t sensor_num)
{
	if (sensor_num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	sensor_config[sensor_config_index_map[sensor_num]].read = pch_read;
	return SENSOR_INIT_SUCCESS;
}
