#include "plat_ipmi.h"

#include <stdio.h>
#include <stdlib.h>

#include "libutil.h"
#include "ipmi.h"
#include "plat_ipmb.h"

bool add_sel_evt_record(addsel_msg_t *sel_msg)
{
	ipmb_error status;
	ipmi_msg *msg;
	uint8_t system_event_record = 0x02; // IPMI spec definition
	uint8_t evt_msg_version = 0x04; // IPMI spec definition
	static uint16_t record_id = 0x1;

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
	if (status == IPMB_ERROR_FAILURE) {
		printf("Fail to post msg to txqueue for addsel\n");
		return false;
	} else if (status == IPMB_ERROR_GET_MESSAGE_QUEUE) {
		printf("No response from bmc for addsel\n");
		return false;
	}

	return true;
}
