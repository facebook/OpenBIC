#ifndef PDR_H
#define PDR_H

#include <stdbool.h>
#include <stdint.h>
#include <sys/byteorder.h>

#define TIMESTAMP104_SIZE 13 //A binary datetime type formatted as a series of 13 bytes
#define NUMERIC_PDR_SIZE 108
#define MAX_LANGUAGE_TAG_LEN 3 //name language tag: "en"
#define MAX_AUX_SENSOR_NAME_LEN 40

typedef float real32_t;
typedef uint_least16_t char16_t;

int pdr_init(void);

enum pdr_repository_state {
	PDR_STATE_AVAILABLE = 0x00,
	PDR_STATE_UPDATE_IN_PROGRESS = 0x01,
	PDR_STATE_FAILED = 0x02,
};

enum PDR_SENSOR_INIT_STATE {
	PDR_SENSOR_NO_INIT = 0,
	PDR_SENSOR_USEINIT_PDR,
	PDR_SENSOR_ENABLE,
	PDR_SENSOR_DISABLE,
};

enum pldm_pdr_types {
	PLDM_TERMINUS_LOCATOR_PDR = 1,
	PLDM_NUMERIC_SENSOR_PDR = 2,
	PLDM_NUMERIC_SENSOR_INITIALIZATION_PDR = 3,
	PLDM_STATE_SENSOR_PDR = 4,
	PLDM_STATE_SENSOR_INITIALIZATION_PDR = 5,
	PLDM_SENSOR_AUXILIARY_NAMES_PDR = 6,
	PLDM_OEM_UNIT_PDR = 7,
	PLDM_OEM_STATE_SET_PDR = 8,
	PLDM_NUMERIC_EFFECTER_PDR = 9,
	PLDM_NUMERIC_EFFECTER_INITIALIZATION_PDR = 10,
	PLDM_STATE_EFFECTER_PDR = 11,
	PLDM_STATE_EFFECTER_INITIALIZATION_PDR = 12,
	PLDM_EFFECTER_AUXILIARY_NAMES_PDR = 13,
	PLDM_EFFECTER_OEM_SEMANTIC_PDR = 14,
	PLDM_PDR_ENTITY_ASSOCIATION = 15,
	PLDM_ENTITY_AUXILIARY_NAMES_PDR = 16,
	PLDM_OEM_ENTITY_ID_PDR = 17,
	PLDM_INTERRUPT_ASSOCIATION_PDR = 18,
	PLDM_EVENT_LOG_PDR = 19,
	PLDM_PDR_FRU_RECORD_SET = 20,
	PLDM_COMPACT_NUMERIC_SENSOR_PDR = 21,
	PLDM_OEM_DEVICE_PDR = 126,
	PLDM_OEM_PDR = 127,
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
	int32_t warning_high;
	int32_t warning_low;
	int32_t critical_high;
	int32_t critical_low;
	int32_t fatal_high;
	int32_t fatal_low;
} PDR_numeric_sensor;

typedef struct __attribute__((packed)) {
	PDR_common_header pdr_common_header;
	uint16_t terminus_handle;
	uint16_t sensor_id;
	uint8_t sensor_count;
	uint8_t nameStringCount;
	char nameLanguageTag[MAX_LANGUAGE_TAG_LEN];
	char16_t sensorName[MAX_AUX_SENSOR_NAME_LEN];
} PDR_sensor_auxiliary_names;

typedef struct __attribute__((packed)) {
	uint8_t repository_state;
	uint8_t update_time[TIMESTAMP104_SIZE];
	uint8_t oem_update_time[TIMESTAMP104_SIZE];
	uint32_t record_count;
	uint32_t repository_size;
	uint32_t largest_record_size;
	uint8_t data_transfer_handle_timeout;
} PDR_INFO;

PDR_INFO *get_pdr_info();
uint32_t get_record_count();
uint32_t plat_get_pdr_size(uint8_t pdr_type);
void plat_load_numeric_sensor_pdr_table(PDR_numeric_sensor *numeric_sensor_table);
void plat_load_aux_sensor_names_pdr_table(PDR_sensor_auxiliary_names *aux_sensor_name_table);
int get_pdr_table_via_record_handle(uint8_t *record_data, uint32_t record_handle);

#endif
