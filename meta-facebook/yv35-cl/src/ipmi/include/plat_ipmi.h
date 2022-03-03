#ifndef PLAT_IPMI_H
#define PLAT_IPMI_H

#include <stdbool.h>
#include <stdint.h>

typedef struct addsel_msg_t {
	uint8_t snr_type;
	uint8_t snr_number;
	uint8_t evt_type;
	uint8_t evt_data1;
	uint8_t evt_data2;
	uint8_t evt_data3;
} addsel_msg_t;

bool add_sel_evt_record(addsel_msg_t *sel_msg);

#endif
