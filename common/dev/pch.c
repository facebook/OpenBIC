#include "pch.h"

#include <stdio.h>
#include <stdlib.h>

#include "ipmi.h"
#include "sensor.h"
#include "plat_sensor_table.h"

bool pch_read(uint8_t sensor_num, float *reading)
{
	snr_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	ipmb_error status;
	ipmi_msg *bridge_msg;
	bridge_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (bridge_msg == NULL) {
		printf("PCH bridge message alloc fail\n");
		return false;
	}
	bridge_msg->seq_source = 0xff;
	bridge_msg->netfn = NETFN_SENSOR_REQ;
	bridge_msg->cmd = CMD_SENSOR_GET_SENSOR_READING;
	bridge_msg->InF_source = SELF;
	bridge_msg->InF_target = ME_IPMB;
	bridge_msg->data_len = 1;
	// Offset is sensor number to read from ME
	bridge_msg->data[0] = cfg->offset;

	for (uint8_t pch_retry_num = 0; pch_retry_num < 4; pch_retry_num++) {
		status = ipmb_read(bridge_msg, IPMB_inf_index_map[bridge_msg->InF_target]);
		if (status != ipmb_error_success) {
			printf("ipmb read fail status: %x\n", status);
			cfg->cache_status = SNR_FAIL_TO_ACCESS;
			free(bridge_msg);
			return false;
		}

		if (bridge_msg->completion_code == CC_SUCCESS) {
			*reading = convert_MBR_to_reading(sensor_num, bridge_msg->data[0]);
			cfg->cache = *reading;
			cfg->cache_status = SNR_READ_SUCCESS;
			free(bridge_msg);
			return true;
		} else if (bridge_msg->completion_code == CC_NODE_BUSY) {
			continue;
		} else {
			free(bridge_msg);
			cfg->cache_status = SNR_UNSPECIFIED_ERROR;
			return false;
		}
	}

	printf("PCH retry read fail\n");
	cfg->cache_status = SNR_UNSPECIFIED_ERROR;
	free(bridge_msg);
	return false;
}
