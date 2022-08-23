#ifndef STORAGE_HANDLER_H
#define STORAGE_HANDLER_H

#include "ipmi.h"

struct sel_event_record {
	uint16_t record_id;
	uint8_t record_type;
	uint32_t timestamp;
	uint8_t gen_id[2];
	uint8_t evm_rev;
	uint8_t sensor_type;
	uint8_t sensor_num;

	union {
		struct {
			uint8_t event_type : 7;
			uint8_t event_dir : 1;
		};
		uint8_t event_dir_type;
	};
	uint8_t event_data[3];
} __attribute__((__packed__));

struct ipmi_storage_add_sel_req {
	struct sel_event_record event;
} __attribute__((__packed__));

struct ipmi_storage_add_sel_resp {
	uint8_t record_id[2];
} __attribute__((__packed__));

void STORAGE_GET_FRUID_INFO(ipmi_msg *msg);
void STORAGE_READ_FRUID_DATA(ipmi_msg *msg);
void STORAGE_WRITE_FRUID_DATA(ipmi_msg *msg);
void STORAGE_RSV_SDR(ipmi_msg *msg);
void STORAGE_GET_SDR(ipmi_msg *msg);
void STORAGE_ADD_SEL(ipmi_msg *msg);

void IPMI_Storage_handler(ipmi_msg *msg);

#endif
