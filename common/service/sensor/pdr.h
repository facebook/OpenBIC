#ifndef PDR_H
#define PDR_H

#include <stdbool.h>
#include <stdint.h>

#define TIMESTAMP104_SIZE 13//A binary datetime type formatted as a series of 13 bytes
#define NUMERIC_PDR_SIZE 108

typedef float real32_t;

uint8_t pdr_init(void);

enum pdr_repository_state {
	PDR_STATE_AVAILABLE = 0x00,
	PDR_STATE_UPDATE_IN_PROGRESS = 0x01,
	PDR_STATE_FAILED = 0x02,
};

typedef struct __attribute__((packed)) {
	uint32_t record_handle;
	uint8_t PDR_header_version;
	uint8_t PDR_type;
	uint16_t record_change_number;
	uint16_t data_length;
} PDR_common_header;

typedef struct __attribute__((packed)) {
	/*** PDR common header***/
	PDR_common_header pdr_common_header;

	/***numeric sensor format***/
	uint16_t PLDM_terminus_handle;
	uint16_t sensor_id;
	uint16_t entity_type;
	uint16_t entity_instance_number;
	uint16_t container_id;
	uint8_t sensor_init;
	uint8_t sensor_auxiliary_names_pdr;
	uint8_t base_unit;
	int8_t unit_modifier;
	uint8_t rate_unit;
	uint8_t base_oem_unit_handle;
	uint8_t aux_unit;
	int8_t aux_unit_modifier;
	uint8_t auxrate_unit;
	uint8_t rel;
	uint8_t aux_oem_unit_handle;
	uint8_t is_linear;
	uint8_t sensor_data_size;
	real32_t resolution;
	real32_t offset;
	uint16_t accuracy;
	uint8_t plus_tolerance;
	uint8_t minus_tolerance;
	uint32_t hysteresis;
	uint8_t supported_thresholds;
	uint8_t threshold_and_hysteresis_volatility;
	real32_t state_transition_interval;
	real32_t update_interval;
	uint32_t max_readable;
	uint32_t min_readable;
	uint8_t range_field_format;
	uint8_t range_field_support;
	uint32_t nominal_value;
	uint32_t normal_max;
	uint32_t normal_min;
	uint32_t warning_high;
	uint32_t warning_low;
	uint32_t critical_high;
	uint32_t critical_low;
	uint32_t fatal_high;
	uint32_t fatal_low;
} PDR_numeric_sensor;

typedef struct __attribute__((packed)) {
	uint8_t repository_state;
	uint8_t update_time[TIMESTAMP104_SIZE];
	uint8_t oem_update_time[TIMESTAMP104_SIZE];
	uint32_t record_count;
	uint32_t repository_size;
	uint32_t largest_record_size;
	uint8_t data_transfer_handle_timeout;
} PDR_INFO;

PDR_INFO* get_pdr_info();
PDR_numeric_sensor* get_pdr_table();
uint16_t get_pdr_size();
uint16_t plat_get_pdr_size();
void plat_load_pdr_table(PDR_numeric_sensor* numeric_sensor_table);

#endif
