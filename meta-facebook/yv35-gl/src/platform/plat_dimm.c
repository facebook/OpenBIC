#include "plat_dimm.h"

#include <stdio.h>
#include <stdlib.h>
#include <zephyr.h>
#include <logging/log.h>
#include <errno.h>
#include "plat_sensor_table.h"
#include "ipmb.h"
#include "ipmi.h"
#include "kcs.h"

LOG_MODULE_REGISTER(plat_dimm);

bool dimm_presence[MAX_COUNT_DIMM];
static bool dimm_inited = false;

bool is_dimm_inited()
{
	return dimm_inited;
}

void init_dimm_status()
{
	int index;
	for (index = DIMM_ID_A; index < MAX_COUNT_DIMM; index++) {
		ipmi_msg msg = { 0 };

		msg.InF_source = SELF;
		msg.InF_target = BMC_IPMB;
		msg.netfn = NETFN_OEM_Q_REQ;
		msg.cmd = CMD_OEM_Q_GET_DIMM_INFO;
		msg.data_len = 5;
		msg.data[0] = IANA_ID & 0xFF;
		msg.data[1] = (IANA_ID >> 8) & 0xFF;
		msg.data[2] = (IANA_ID >> 16) & 0xFF;
		msg.data[3] = (uint8_t)index;
		msg.data[4] = CMD_DIMM_LOCATION;

		ipmb_error ret = ipmb_read(&msg, IPMB_inf_index_map[msg.InF_target]);
		if (ret != IPMB_ERROR_SUCCESS) {
			LOG_ERR("Failed to get DIMM status, ret %d", ret);
			return;
		}

		uint8_t status = msg.data[0];
		if (status == DIMM_PRESENT) {
			dimm_presence[index] = true;
		} else {
			dimm_presence[index] = false;
		}
	}

	dimm_inited = true;
}

bool get_dimm_presence_status(uint8_t dimm_id)
{
	return dimm_presence[dimm_id];
}

void set_dimm_presence_status(uint8_t index, uint8_t status)
{
	if (status == DIMM_PRESENT) {
		dimm_presence[index] = true;
	} else {
		dimm_presence[index] = false;
	}
}

uint8_t sensor_num_map_dimm_id(uint8_t sensor_num)
{
	uint8_t dimm_id = DIMM_ID_UNKNOWN;

	switch (sensor_num) {
	case SENSOR_NUM_MB_DIMMA_TEMP_C:
		dimm_id = DIMM_ID_A;
		break;
	case SENSOR_NUM_MB_DIMMB_TEMP_C:
		dimm_id = DIMM_ID_B;
		break;
	case SENSOR_NUM_MB_DIMMC_TEMP_C:
		dimm_id = DIMM_ID_C;
		break;
	case SENSOR_NUM_MB_DIMMD_TEMP_C:
		dimm_id = DIMM_ID_D;
		break;
	case SENSOR_NUM_MB_DIMME_TEMP_C:
		dimm_id = DIMM_ID_E;
		break;
	case SENSOR_NUM_MB_DIMMF_TEMP_C:
		dimm_id = DIMM_ID_F;
		break;
	case SENSOR_NUM_MB_DIMMG_TEMP_C:
		dimm_id = DIMM_ID_G;
		break;
	case SENSOR_NUM_MB_DIMMH_TEMP_C:
		dimm_id = DIMM_ID_H;
		break;
	default:
		dimm_id = DIMM_ID_UNKNOWN;
		break;
	}

	return dimm_id;
}
