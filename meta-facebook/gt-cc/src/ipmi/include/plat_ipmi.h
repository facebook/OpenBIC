#ifndef PLAT_IPMI_H
#define PLAT_IPMI_H

#include <stdbool.h>
#include <stdint.h>

#define IS_SECTOR_END_MASK 0x80
#define WITHOUT_SENCTOR_END_MASK 0x7F
#define BIC_UPDATE_MAX_OFFSET 0x50000

typedef struct addsel_msg_t {
	uint8_t sensor_type;
	uint8_t sensor_number;
	uint8_t event_type;
	uint8_t event_data1;
	uint8_t event_data2;
	uint8_t event_data3;
} addsel_msg_t;

enum GT_FIRWARE_UPDATE_TARGET {
	SWB_BIC_UPDATE = 2,
	PEX0_UPDATE = 3,
	PEX1_UPDATE = 4,
	PEX2_UPDATE = 5,
	PEX3_UPDATE = 6,
};

bool add_sel_evt_record(addsel_msg_t *sel_msg);

#endif
