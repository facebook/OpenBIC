#ifndef SENSOR_H
#define SENSOR_H

#include <stdbool.h>
#include <stdint.h>
#include <kernel.h>

#include "plat_def.h"
#include "sdr.h"
#include "libutil.h"

#define SENSOR_POLL_STACK_SIZE 2048
#define NONE 0

#define GET_FROM_CACHE 0x00
#define GET_FROM_SENSOR 0x01

#define SENSOR_NULL 0xFF
#define SENSOR_FAIL 0xFF
#define SENSOR_NUM_MAX 0xFF
#define SENSOR_NOT_SUPPORT 0xFF
#define DIMM_NOT_PRESENT 0xFF

#define DEBUG_SENSOR 0

#define SAMPLE_COUNT_DEFAULT 1

#define POLL_TIME_DEFAULT 1

enum LTC4282_OFFSET {
	LTC4282_ILIM_ADJUST_OFFSET = 0x11,
	LTC4282_VSENSE_OFFSET = 0x40,
	LTC4282_POWER_OFFSET = 0x46,
	LTC4282_VSOURCE_OFFSET = 0x3A,
	LTC4282_ADC_CONTROL_OFFSET = 0x1D,
	LTC4282_ENERGY_OFFSET = 0x12,
	LTC4282_STATUS_OFFSET = 0x1F,
};

enum ADM1278_OFFSET {
	ADM1278_PEAK_IOUT_OFFSET = 0xD0,
	ADM1278_PEAK_PIN_OFFSET = 0xDA,
	ADM1278_EIN_EXT_OFFSET = 0xDC,
};

enum NCT7718W_OFFSET {
	NCT7718W_LOCAL_TEMP_OFFSET = 0x00,
	NCT7718W_REMOTE_TEMP_MSB_OFFSET = 0x01,
	NCT7718W_REMOTE_TEMP_LSB_OFFSET = 0x10,
};

enum INA230_OFFSET {
	INA230_CFG_OFFSET = 0x00,
	INA230_VSH_VOL_OFFSET = 0x01,
	INA230_BUS_VOL_OFFSET = 0x02,
	INA230_PWR_OFFSET = 0x03,
	INA230_CUR_OFFSET = 0x04,
	INA230_CAL_OFFSET = 0x05,
	INA230_MSK_OFFSET = 0x06,
	INA230_ALT_OFFSET = 0x07,
};

enum G788P81U_OFFSET {
	G788P81U_LOCAL_TEMP_OFFSET = 0x00,
	G788P81U_REMOTE_TEMP_OFFSET = 0x01,
	G788P81U_REMOTE_TEMP_EXT_OFFSET = 0x10,
};

enum SENSOR_DEV {
	sensor_dev_tmp75 = 0,
	sensor_dev_ast_adc = 0x01,
	sensor_dev_intel_peci = 0x02,
	sensor_dev_isl69259 = 0x03,
	sensor_dev_adm1278 = 0x04,
	sensor_dev_nvme = 0x05,
	sensor_dev_pch = 0x06,
	sensor_dev_mp5990 = 0x07,
	sensor_dev_isl28022 = 0x08,
	sensor_dev_pex89000 = 0x09,
	sensor_dev_tps53689 = 0x0A,
	sensor_dev_xdpe15284 = 0x0B,
	sensor_dev_ltc4282 = 0x0C,
	sensor_dev_ast_fan = 0x0D,
	sensor_dev_tmp431 = 0x0E,
	sensor_dev_pmic = 0x0F,
	sensor_dev_ina233 = 0x10,
	sensor_dev_isl69254iraz_t = 0x11,
	sensor_dev_max16550a = 0x12,
	sensor_dev_ina230 = 0x13,
	sensor_dev_xdpe12284c = 0x14,
	sensor_dev_raa229621 = 0x15,
	sensor_dev_nct7718w = 0x16,
	sensor_dev_ltc4286 = 0x17,
	sensor_dev_amd_tsi = 0x18,
	sensor_dev_apml_mailbox = 0x19,
	sensor_dev_xdpe19283b = 0x1A,
	sensor_dev_g788p81u = 0x1B,
	sensor_dev_mp2856gut = 0x1C,
	sensor_dev_max
};

enum CONTROL_SENSOR_POLLING_OPTION {
	DISABLE_SENSOR_POLLING = false,
	ENABLE_SENSOR_POLLING = true,
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
	SENSOR_NOT_ACCESSIBLE, // Only use to check sensor access fail
	SENSOR_FAIL_TO_ACCESS,
	SENSOR_INIT_STATUS,
	SENSOR_UNSPECIFIED_ERROR,
	SENSOR_POLLING_DISABLE,
	SENSOR_PRE_READ_ERROR,
	SENSOR_POST_READ_ERROR,
	SENSOR_READ_API_UNREGISTER,
	SENSOR_READ_4BYTE_ACUR_SUCCESS,
	SENSOR_NOT_PRESENT
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
	int64_t poll_time; // sec
	bool is_enable_polling;
	int cache;
	uint8_t cache_status;
	bool (*pre_sensor_read_hook)(uint8_t, void *);
	void *pre_sensor_read_args;
	bool (*post_sensor_read_hook)(uint8_t, void *, int *);
	void *post_sensor_read_args;
	void *init_args;

	/* if there is new parameter should be added, please add on above */
	void *priv_data;
	uint8_t retry;
	uint8_t (*init)(uint8_t, int *);
	uint8_t (*read)(uint8_t, int *);
} sensor_cfg;

typedef struct _sensor_poll_time_cfg {
	uint8_t sensor_num;
	int64_t last_access_time;
} sensor_poll_time_cfg;

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
	/* value to get/set ILIM ADJUST register */
	union {
		uint8_t value;
		struct {
			uint8_t _16_bit : 1;
			uint8_t gpio_mode : 1;
			uint8_t vsource_vdd : 1;
			uint8_t foldback_mode : 2;
			uint8_t ilim_adjust : 3;
		} fields;
	} ilim_adjust;

	/* Rsense valus, unit: milliohm */
	float r_sense_mohm;

	/* Initialize function will set following arguments, no need to give value */
	bool is_init;

	/* Initialized chip registers if it's needed
	 * bit 0 - ilim adjust register setting
	 * bit 1 to 7 are reserved
	 */
	uint8_t is_register_setting_needed;
} ltc4282_init_arg;

typedef struct _ltc4286_init_arg {
	/* value to get/set MFR CONFIG 1 register */
	union {
		uint16_t value;
		struct {
			uint16_t vpwr_select : 1;
			uint16_t vrange_select : 1;
			uint16_t reserved_1 : 8; // bit[9:2] are reserved.
			uint16_t ilim : 4;
			uint16_t reserved_2 : 2; // bit[15:14] are reserved.
		} fields;
	} mfr_config_1;
	/* Rsense valus, unit: milliohm */
	float r_sense_mohm;
	/* Initailize function will set following arguments, no need to give value */
	bool is_init;
} ltc4286_init_arg;

typedef struct _mp5990_init_arg {
	/* value to sets the gain for output current reporting */
	uint16_t iout_cal_gain;
	/* value to sets the IMON based total over current fault limit */
	uint16_t iout_oc_fault_limit;

	/* Initialize function will set following arguments, no need to give value */
	bool is_init;

} mp5990_init_arg;

typedef struct _pmic_init_arg {
	bool is_init;
	uint8_t smbus_bus_identifier;
	uint8_t smbus_addr;
} pmic_init_arg;

typedef struct _ina233_init_arg_ {
	bool is_init;
} ina233_init_arg;

typedef struct _max16550a_init_arg_ {
	float r_load;
} max16550a_init_arg;

typedef struct _ina230_init_arg {
	/* value to set configuration register */
	union {
		uint16_t value;
		struct {
			uint16_t MODE : 3;
			uint16_t VSH_CT : 3;
			uint16_t VBUS_CT : 3;
			uint16_t AVG : 3;
			uint16_t reserved : 3;
			uint16_t RST : 1;
		};
	} config;

	/* Shunt resistor value. Unit: Ohm. */
	double r_shunt;

	/* Alert value.
	 * The unit is Volt or Watt, depending on the alert function.
	 */
	double alert_value;

	/* value to set alert register */
	union {
		uint16_t value;
		struct {
			uint16_t LEN : 1;
			uint16_t APOL : 1;
			uint16_t OVF : 1;
			uint16_t CVRF : 1;
			uint16_t AFF : 1;
			uint16_t reserved : 5;
			uint16_t CNVR : 1;
			uint16_t POL : 1; // Lowest priority
			uint16_t BUL : 1;
			uint16_t BOL : 1;
			uint16_t SUL : 1;
			uint16_t SOL : 1; // Highest priority
		};
	} alt_cfg;

	/* Expected maximum current */
	double i_max;

	/* The current/power value per 1 bit.
	 * The unit is Amp or Watt, depending on the respective register.
	 */
	double cur_lsb;
	double pwr_lsb;

	/* Initialize function will set following arguments, no need to give value */
	bool is_init;

} ina230_init_arg;

typedef struct _apml_mailbox_init_arg_ {
	uint32_t data;
	uint8_t retry;
} apml_mailbox_init_arg;

extern bool enable_sensor_poll_thread;
extern sensor_cfg *sensor_config;
// Mapping sensor number to sensor config index
extern uint8_t sensor_config_index_map[SENSOR_NUM_MAX];
extern uint8_t sensor_config_count;

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
void pal_extend_sensor_config(void);
bool check_sensor_num_exist(uint8_t sensor_num);
void add_sensor_config(sensor_cfg config);
bool check_is_sensor_ready();
bool pal_is_time_to_poll(uint8_t sensor_num, int poll_time);
uint8_t plat_get_config_size();
void load_sensor_config(void);
void control_sensor_polling(uint8_t sensor_num, uint8_t optional, uint8_t cache_status);

#endif
