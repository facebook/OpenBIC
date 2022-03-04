#ifndef SENSOR_H
#define SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "sdr.h"

#define NONE 0

#define sensor_poll_stack_size 2048

#define get_from_sensor 0x00
#define get_from_cache 0x01

#define MAX_SNR_SIZE 60

#define sensor_null 0xFF
#define sensor_fail 0xFF
#define SENSOR_NUM_MAX 0xFF

#define DEBUG_SNR 0

/*  define sensor type  */
#define TYPE_TMP75 0x00
#define TYPE_ADC 0x01
#define TYPE_PECI 0x02
#define TYPE_ISL69260 0x03
#define TYPE_HSC 0x04
#define TYPE_NVME 0x05
#define TYPE_PCH 0x06
#define TYPE_MEDUSA 0x07
#define TYPE_FAN 0x08

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

static inline float convert_MBR_to_reading(uint8_t sensor_num, uint8_t val)
{
	if (SDR_M(sensor_num) == 0) {
		return (val - round_add(sensor_num, val)) / SDR_Rexp(sensor_num);
	}

	return (val - round_add(sensor_num, val)) * SDR_M(sensor_num) / SDR_Rexp(sensor_num);
}

enum {
	SNR_READ_SUCCESS,
	SNR_NOT_FOUND,
	SNR_NOT_ACCESSIBLE,
	SNR_FAIL_TO_ACCESS,
	SNR_INIT_STATUS,
	SNR_UNSPECIFIED_ERROR,
	SNR_POLLING_DISABLE,
};

typedef struct _snr_cfg__ {
	uint8_t num;
	uint8_t type;
	uint8_t port; // port, bus, channel, etc.
	uint8_t slave_addr;
	uint16_t offset;
	bool (*access_checker)(uint8_t);
	int arg0;
	int arg1;
	float cache;
	uint8_t cache_status;
	uint8_t mux_address;
	uint8_t mux_offset;
	int (*pre_sensor_read_fn)(uint8_t sensor_num, void *);
	void *pre_sensor_read_args;
	int (*post_sensor_read_fn)(uint8_t sensor_num, void *);
	void *post_sensor_read_args;
} snr_cfg;

extern bool enable_sensor_poll;
extern uint8_t SDR_NUM;
extern snr_cfg *sensor_config;
// Mapping sensor number to sensor config index
extern uint8_t sensor_config_index_map[SENSOR_NUM_MAX];
extern uint8_t sensor_config_num;

void clear_unaccessible_sensor_cache();
uint8_t get_sensor_reading(uint8_t sensor_num, float *reading, uint8_t read_mode);
void pal_set_sensor_poll_interval(int *interval_ms);
bool stby_access(uint8_t snr_num);
bool DC_access(uint8_t snr_num);
bool post_access(uint8_t snr_num);
bool sensor_init(void);
void disable_snr_poll();
void enable_snr_poll();
void pal_fix_sensor_config(void);
bool check_sensor_num_exist(uint8_t sensor_num);
void add_sensor_config(snr_cfg config);

#endif
