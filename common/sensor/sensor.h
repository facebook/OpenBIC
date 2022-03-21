#ifndef SENSOR_H
#define SENSOR_H

#include "sdr.h"
#include "pmbus.h"

#define sensor_poll_stack_size 2048

#define get_from_sensor 0x00
#define get_from_cache 0x01

#define MAX_SENSOR_SIZER 60

#define sensor_null 0xFF
#define sensor_fail 0xFF
#define SENSOR_NUM_MAX 0xFF

#define DEBUG_SENSOR 0

/*  define sensor type  */
#define type_tmp75 0x00
#define type_adc 0x01
#define type_peci 0x02
#define type_vr 0x03
#define type_hsc 0x04
#define type_nvme 0x05
#define type_pch 0x06
#define type_medusa 0x07
#define type_fan 0x08

enum ltc4282_offset {
	LTC4282_ADJUST_OFFSET = 0x11,
	LTC4282_VSENSE_OFFSET = 0x40,
	LTC4282_POWER_OFFSET = 0x46,
	LTC4282_VSOURCE_OFFSET = 0x3A,
};

enum adm1278_offset {
	ADM1278_VSOURCE_OFFSET = 0x88,
	ADM1278_CURRENT_OFFSET = 0x8C,
	ADM1278_TEMP_OFFSET = 0x8D,
	ADM1278_POWER_OFFSET = 0x97,
	ADM1278_PEAK_IOUT_OFFSET = 0xD0,
	ADM1278_PEAK_PIN_OFFSET = 0xDA,
	ADM1278_EIN_EXT_OFFSET = 0xDC,
};

enum sensor_dev {
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

static inline int acur_cal_MBR(uint8_t sensor_num, int val)
{ // for better accuracy, enlarge SDR to two byte scale
	if (SDR_M(sensor_num) == 0) {
		return ((val << 8) * SDR_Rexp(sensor_num));
	}
	return ((val << 8) / SDR_M(sensor_num) * SDR_Rexp(sensor_num));
}

static inline int cal_MBR(uint8_t sensor_num, int val)
{
	if (SDR_M(sensor_num) == 0) {
		return (val * SDR_Rexp(sensor_num) + round_add(sensor_num, val));
	}
	return (val * SDR_Rexp(sensor_num) / SDR_M(sensor_num) + round_add(sensor_num, val));
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
	uint8_t slave_addr;
	uint8_t offset;
	bool (*access_checker)(uint8_t);
	int arg0;
	int arg1;
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

	/* Initailize function will set following arguments, no need to give value */
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

	/* Initailize function will set following arguments, no need to give value */
	bool is_init;

} adm1278_init_arg;

typedef struct _pex89000_init_arg {
	uint8_t idx;
	struct k_mutex brcm_pciesw;

	/* Initailize function will set following arguments, no need to give value */
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

	/* Initailize function will set following arguments, no need to give value */
	bool is_init;

} mp5990_init_arg;

extern bool enable_sensor_poll;
extern uint8_t SDR_NUM;
extern sensor_cfg *sensor_config;
extern uint8_t SensorNum_SensorCfg_map[SENSOR_NUM_MAX];

uint8_t get_sensor_reading(uint8_t sensor_num, int *reading, uint8_t read_mode);
void clear_unaccessible_sensor_cache();
bool sensor_init(void);
void sensor_poll_disable();
void sensor_poll_enable();
void clear_unaccessible_sensor_cache();

/* i2c-mux tca9548 */
bool tca9548_select_chan(uint8_t sensor_num, void *args);
#endif
