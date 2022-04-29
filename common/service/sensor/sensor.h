#ifndef SENSOR_H
#define SENSOR_H

#include <stdbool.h>
#include <stdint.h>
#include <kernel.h>

#include "plat_def.h"
#include "sdr.h"

#define SENSOR_POLL_STACK_SIZE 2048
#define NONE 0

#define GET_FROM_CACHE 0x00
#define GET_FROM_SENSOR 0x01

#define MAX_SENSOR_SIZE 60

#define SENSOR_NULL 0xFF
#define SENSOR_FAIL 0xFF
#define SENSOR_NUM_MAX 0xFF

#define DEBUG_SENSOR 0

#define SAMPLE_COUNT_DEFAULT 1

enum LTC4282_OFFSET {
	LTC4282_ADJUST_OFFSET = 0x11,
	LTC4282_VSENSE_OFFSET = 0x40,
	LTC4282_POWER_OFFSET = 0x46,
	LTC4282_VSOURCE_OFFSET = 0x3A,
};

enum ADM1278_OFFSET {
	ADM1278_PEAK_IOUT_OFFSET = 0xD0,
	ADM1278_PEAK_PIN_OFFSET = 0xDA,
	ADM1278_EIN_EXT_OFFSET = 0xDC,
};

enum SENSOR_DEV {
	sensor_dev_tmp75 = 0,
	sensor_dev_ast_adc = 0x01,
	sensor_dev_intel_peci = 0x02,
	sensor_dev_isl69259 = 0x03,
	sensor_dev_adm1278 = 0x04,
	sensor_dev_nvme = 0x05,
	sensor_dev_pch = 0x06,
	sensor_dev_mp5990 = 0x10,
	sensor_dev_isl28022 = 0x11,
	sensor_dev_pex89000 = 0x12,
	sensor_dev_tps53689 = 0x13,
	sensor_dev_xdpe15284 = 0x14,
	sensor_dev_ltc4282 = 0x15,
	sensor_dev_ast_fan = 0x16,
	sensor_dev_tmp431 = 0x18,
	sensor_dev_max
};

typedef struct _sensor_val {
	int16_t integer;
	int16_t fraction;
} sensor_val;

struct tca9548 {
	uint8_t addr;
	uint8_t chan;
};

static inline int calculate_accurate_MBR(uint8_t sensor_num, int val)
{ // for better accuracy, enlarge SDR to two byte scale
	if (SDR_M(sensor_num) == 0) {
		return ((val << 8) * SDR_Rexp(sensor_num));
	}
	return ((val << 8) / SDR_M(sensor_num) * SDR_Rexp(sensor_num));
}

static inline int calculate_MBR(uint8_t sensor_num, int val)
{
	if (SDR_M(sensor_num) == 0) {
		return (val * SDR_Rexp(sensor_num) + round_add(sensor_num, val));
	}
	return (val * SDR_Rexp(sensor_num) / SDR_M(sensor_num) + round_add(sensor_num, val));
}

static inline float convert_MBR_to_reading(uint8_t sensor_num, uint8_t val)
{
	if (SDR_M(sensor_num) == 0) {
		return (val - round_add(sensor_num, val)) / SDR_Rexp(sensor_num);
	}

	return (val - round_add(sensor_num, val)) * SDR_M(sensor_num) / SDR_Rexp(sensor_num);
}

enum {
	SENSOR_READ_SUCCESS,
	SENSOR_READ_ACUR_SUCCESS,
	SENSOR_NOT_FOUND,
	SENSOR_NOT_ACCESSIBLE,
	SENSOR_FAIL_TO_ACCESS,
	SENSOR_INIT_STATUS,
	SENSOR_UNSPECIFIED_ERROR,
	SENSOR_POLLING_DISABLE,
	SENSOR_PRE_READ_ERROR,
	SENSOR_POST_READ_ERROR,
	SENSOR_READ_API_UNREGISTER,
	SENSOR_READ_4BYTE_ACUR_SUCCESS
};

enum { SENSOR_INIT_SUCCESS, SENSOR_INIT_UNSPECIFIED_ERROR };

typedef struct _sensor_cfg__ {
	uint8_t num;
	uint8_t type;
	uint8_t port; // port, bus, channel, etc.
	uint8_t target_addr;
	uint16_t offset;
	bool (*access_checker)(uint8_t);
	int arg0;
	int arg1;
	int sample_count;
	int cache;
	uint8_t cache_status;
	bool (*pre_sensor_read_hook)(uint8_t, void *);
	void *pre_sensor_read_args;
	bool (*post_sensor_read_hook)(uint8_t, void *, int *);
	void *post_sensor_read_args;
	void *init_args;

	/* if there is new parameter should be added, please add on above */
	uint8_t retry;
	uint8_t (*init)(uint8_t, int *);
	uint8_t (*read)(uint8_t, int *);
} sensor_cfg;

/* INIT arg */
typedef struct _isl28022_init_arg {
	/* value to set configuration register */
	union {
		uint16_t value;
		struct {
			uint16_t MODE : 3;
			uint16_t SADC : 4;
			uint16_t BADC : 4;
			uint16_t PG : 2;
			uint16_t BRNG : 2;
			uint16_t RST : 1;
		} fields;
	} config;
	/* R_shunt valus, unit: milliohm */
	uint32_t r_shunt;

	/* Initialize function will set following arguments, no need to give value */
	bool is_init;
	/* used when read current/power */
	float current_LSB;
} isl28022_init_arg;

typedef struct _adc_asd_init_arg {
	bool is_init;
} adc_asd_init_arg;

typedef struct _adm1278_init_arg {
	/* value to set configuration register */
	union {
		uint16_t value;
		struct {
			uint16_t RSV1 : 1;
			uint16_t VOUT_EN : 1;
			uint16_t VIN_EN : 1;
			uint16_t TEMP1_EN : 1;
			uint16_t PMON_MODE : 1;
			uint16_t RSV2 : 3;
			uint16_t VI_AVG : 3;
			uint16_t PWR_AVG : 3;
			uint16_t SIMULTANEOUS : 1;
			uint16_t TSFILT : 1;
		} fields;
	} config;
	/* Rsense valus, unit: milliohm */
	float r_sense;

	/* Initialize function will set following arguments, no need to give value */
	bool is_init;

} adm1278_init_arg;

typedef struct _pex89000_init_arg {
	uint8_t idx;
	struct k_mutex brcm_pciesw;

	/* Initialize function will set following arguments, no need to give value */
	bool is_init;

} pex89000_init_arg;

typedef struct _ltc4282_init_arg {
	float r_sense;

} ltc4282_init_arg;

typedef struct _mp5990_init_arg {
	/* value to sets the gain for output current reporting */
	uint16_t iout_cal_gain;
	/* value to sets the IMON based total over current fault limit */
	uint16_t iout_oc_fault_limit;

	/* Initialize function will set following arguments, no need to give value */
	bool is_init;

} mp5990_init_arg;

extern bool enable_sensor_poll_thread;
extern uint8_t SDR_NUM;
extern sensor_cfg *sensor_config;
// Mapping sensor number to sensor config index
extern uint8_t sensor_config_index_map[SENSOR_NUM_MAX];
extern uint8_t sensor_config_num;

void clear_unaccessible_sensor_cache(uint8_t sensor_num);
uint8_t get_sensor_reading(uint8_t sensor_num, int *reading, uint8_t read_mode);
void pal_set_sensor_poll_interval(int *interval_ms);
bool stby_access(uint8_t sensor_num);
bool dc_access(uint8_t sensor_num);
bool post_access(uint8_t sensor_num);
bool me_access(uint8_t sensor_num);
bool vr_access(uint8_t sensor_num);
bool sensor_init(void);
void disable_sensor_poll();
void enable_sensor_poll();
void pal_fix_sensor_config(void);
bool check_sensor_num_exist(uint8_t sensor_num);
void add_sensor_config(sensor_cfg config);
bool check_is_sensor_ready();

#endif
