/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <stdbool.h>
#include <stdint.h>
#include <kernel.h>

#include "plat_def.h"
#include "sdr.h"
#include "libutil.h"

#define sensor_name_to_num(x) #x,

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
	LTC4282_STATUS_OFFSET_BYTE1 = 0x1E,
	LTC4282_STATUS_OFFSET_BYTE2 = 0x1F,
};

enum ADM1278_OFFSET {
	ADM1278_PEAK_IOUT_OFFSET = 0xD0,
	ADM1278_PEAK_PIN_OFFSET = 0xDA,
	ADM1278_EIN_EXT_OFFSET = 0xDC,
};

enum NCT7718W_OFFSET {
	NCT7718W_LOCAL_TEMP_OFFSET = 0x00,
	NCT7718W_REMOTE_TEMP_MSB_OFFSET = 0x01,
	NCT7718W_CONFIGURATION_OFFSET = 0x03,
	NCT7718W_RT1_HIGH_ALERT_TEMP_OFFSET = 0x0D,
	NCT7718W_REMOTE_TEMP_LSB_OFFSET = 0x10,
	NCT7718W_ALERT_MASK_OFFSET = 0x16,
	NCT7718W_RT1_CRITICAL_TEMP_OFFSET = 0x19,
	NCT7718W_LT_CRITICAL_TEMP_OFFSET = 0x20,
	NCT7718W_RT_FILTER_ALERT_MODE_OFFSET = 0xBF,
	NCT7718W_CHIP_ID_OFFSET = 0xFD,
	NCT7718W_VENDOR_ID_OFFSET = 0xFE,
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
	G788P81U_CONFIGURATION_OFFSET = 0x03,
	G788P81U_REMOTE_THIGH_LIMIT_OFFSET = 0x0D,
	G788P81U_REMOTE_TEMP_EXT_OFFSET = 0x10,
	G788P81U_ALERT_MASK_OFFSET = 0x16,
	G788P81U_REMOTE_TEMP_THERM_LIMIT_OFFSET = 0x19,
	G788P81U_LOCAL_TEMP_THERM_LIMIT_OFFSET = 0x20,
	G788P81U_ALERT_MODE_OFFSET = 0xBF,
};

enum DIMM_RELATED_OFFSET {
	DIMM_PMIC_SWA_PWR = 0x0C,
	DIMM_SPD_TEMP = 0x31,
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
#ifdef ENABLE_APML
	sensor_dev_amd_tsi = 0x18,
	sensor_dev_apml_mailbox = 0x19,
#endif
	sensor_dev_xdpe19283b = 0x1A,
	sensor_dev_g788p81u = 0x1B,
	sensor_dev_mp2856gut = 0x1C,
	sensor_dev_ddr5_power = 0x1D,
	sensor_dev_ddr5_temp = 0x1E,
	sensor_dev_adm1272 = 0x1F,
	sensor_dev_q50sn120a1 = 0x20,
	sensor_dev_mp2971 = 0x21,
#ifdef ENABLE_PM8702
	sensor_dev_pm8702 = 0x22,
#endif
	sensor_dev_ltc2991 = 0x23,
	sensor_dev_sq52205 = 0x24,
	sensor_dev_emc1412 = 0x25,
	sensor_dev_i3c_dimm = 0x26,
	sensor_dev_pt5161l = 0x27,
	sensor_dev_lm75bd118 = 0x28,
	sensor_dev_tmp461 = 0x29,
	sensor_dev_mp2985 = 0x2A,
	sensor_dev_m88rt51632 = 0x2B,
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
	SENSOR_NOT_PRESENT,
	SENSOR_PEC_ERROR,
	SENSOR_PARAMETER_NOT_VALID,
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

typedef struct _vr_page_cfg {
	uint8_t vr_page;
} vr_page_cfg;

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

	struct bus_volt_threshold_config {
		bool do_config;
		/* The threshold of Volts */
		float min_threshold_limit;
		float max_threshold_limit;
	} bus_volt_threshold_config;

	struct aux_control_config {
		bool do_config;
		union {
			uint16_t value;
			struct {
				/* External Clock Divider bits*/
				uint16_t ExtCLKDiv : 6;
				/* External Clock Enable bit */
				uint16_t ExtClkEn : 1;
				/* Interrupt Enable bit*/
				uint16_t INTREN : 1;
				/* Force Interrupt bit */
				uint16_t FORCEINTR : 1;
				uint16_t resv : 7;
			} fields;
		} config;
	} aux_control_config;

	/* R_shunt valus, unit: milliohm */
	uint32_t r_shunt;
	/* used when read current/power */
	float current_LSB;
	/* Initialize function will set following arguments, no need to give value */
	bool is_init;

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
	/* value to sets the SCREF and OCWREF voltage level. */
	uint16_t ocw_sc_ref;

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
	float current_lsb;
	float r_shunt;
	bool mfr_config_init;
	/* value to get/set MFR CONFIG register */
	union {
		uint16_t value;
		struct {
			uint16_t operating_mode : 3;
			uint16_t shunt_volt_time : 3;
			uint16_t bus_volt_time : 3;
			uint16_t aver_mode : 3;
			uint16_t rsvd : 4; // bit[15:12] are reserved.
		};
	} mfr_config;
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

typedef struct _pm8702_dimm_init_arg {
	bool is_init;
	uint8_t dimm_id;
} pm8702_dimm_init_arg;

typedef struct _ddr5_init_power_arg_ {
	uint8_t HID_code;
	uint8_t port_number;
} ddr5_init_power_arg;

typedef struct _ddr5_init_temp_arg_ {
	uint8_t HID_code;
	uint8_t port_number;
	float ts0_temp;
	float ts1_temp;
} ddr5_init_temp_arg;

typedef struct _adm1272_init_arg {
	union {
		uint16_t value;
		struct {
			uint16_t IRANGE : 1;
			uint16_t VOUT_EN : 1;
			uint16_t VIN_EN : 1;
			uint16_t TEMP1_EN : 1;
			uint16_t PMON_MODE : 1;
			uint16_t VRANGE : 1;
			uint16_t RSV2 : 2;
			uint16_t VI_AVG : 3;
			uint16_t PWR_AVG : 3;
			uint16_t SIMULTANEOUS : 1;
			uint16_t TSFILT : 1;
		} fields;
	} pwr_monitor_cfg;

	/* Initialize function will set following arguments, no need to give value */
	bool is_init;
	bool is_need_set_pwr_cfg;

} adm1272_init_arg;

typedef struct _sq52205_init_arg_ {
	bool is_init;
	float current_lsb;
	float r_shunt;
	union {
		uint16_t value;
		struct {
			uint16_t operating_mode : 3;
			uint16_t shunt_volt_time : 3;
			uint16_t bus_volt_time : 3;
			uint16_t aver_mode : 3;
			uint16_t rsvd : 3; // bit[14:12] are reserved.
			uint16_t reset_bit : 1;
		};
	} config;
} sq52205_init_arg;

typedef struct _ltc2991_init_arg_ {
	bool is_init;
	union {
		int value;
		struct {
			uint8_t V1_V2_DIFFERENTIAL : 1;
			uint8_t V1_V2_TEMPERATURE : 1;
			uint8_t T1_KELVIN : 1;
			uint8_t V1_V2_FILT : 1;
			uint8_t V3_V4_DIFFERENTIAL : 1;
			uint8_t V3_V4_TEMPERATURE : 1;
			uint8_t T2_KELVIN : 1;
			uint8_t V3_V4_FILT : 1;
		} fields;
	} v1_v4_control_operation;
	union {
		int value;
		struct {
			uint8_t V5_V6_DIFFERENTIAL : 1;
			uint8_t V5_V6_TEMPERATURE : 1;
			uint8_t T3_KELVIN : 1;
			uint8_t V5_V6_FILT : 1;
			uint8_t V7_V8_DIFFERENTIAL : 1;
			uint8_t V7_V8_TEMPERATURE : 1;
			uint8_t T4_KELVIN : 1;
			uint8_t V7_V8_FILT : 1;
		} fields;
	} v5_v8_control_operation;
} ltc2991_init_arg;

typedef struct _nct7718w_init_arg_ {
	bool is_init;
	uint8_t rt1_high_alert_temp;
	uint8_t rt_filter_alert_mode;
	uint8_t alert_mask;
	uint8_t configuration;
	uint8_t rt1_critical_temperature;
	uint8_t lt_critical_temperature;
} nct7718w_init_arg;

typedef struct _g788p81u_init_arg_ {
	bool is_init;
	uint8_t remote_T_high_limit;
	uint8_t alert_mode;
	uint8_t alert_mask;
	uint8_t configuration;
	uint8_t remote_temp_therm_limit;
	uint8_t local_temp_therm_limit;
} g788p81u_init_arg;

typedef struct _pt5161l_init_arg_ {
	uint8_t temp_cal_code_pma_a[4]; // temp calibration codes for PMA A
	uint8_t temp_cal_code_pma_b[4]; // temp calibration codes for PMA B
	uint8_t temp_cal_code_avg; // average temp calibration code
	bool is_init;
} pt5161l_init_arg;

typedef struct _mp2985_init_arg {
	bool is_init;
} mp2985_init_arg;

extern bool enable_sensor_poll_thread;
extern sensor_cfg *sensor_config;
// Mapping sensor number to sensor config index
extern uint8_t sensor_config_index_map[SENSOR_NUM_MAX];
extern uint8_t sensor_config_count;
extern const char *const sensor_type_name[];

void clear_unaccessible_sensor_cache(uint8_t sensor_num);
uint8_t get_sensor_reading(uint8_t sensor_num, int *reading, uint8_t read_mode);
void pal_set_sensor_poll_interval(int *interval_ms);
bool stby_access(uint8_t sensor_num);
bool dc_access(uint8_t sensor_num);
bool post_access(uint8_t sensor_num);
bool me_access(uint8_t sensor_num);
bool vr_access(uint8_t sensor_num);
bool vr_stby_access(uint8_t sensor_num);
bool sensor_init(void);
void disable_sensor_poll();
void enable_sensor_poll();
bool get_sensor_poll_enable_flag();
void pal_extend_sensor_config(void);
bool check_sensor_num_exist(uint8_t sensor_num);
void add_sensor_config(sensor_cfg config);
bool check_is_sensor_ready();
bool pal_is_time_to_poll(uint8_t sensor_num, int poll_time);
uint8_t plat_get_config_size();
void load_sensor_config(void);
void control_sensor_polling(uint8_t sensor_num, uint8_t optional, uint8_t cache_status);
bool check_reading_pointer_null_is_allowed(uint8_t sensor_num);
bool init_drive_type_delayed(sensor_cfg *cfg);

#endif
