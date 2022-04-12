#ifndef SENSOR_H
#define SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "sdr.h"

#define SENSOR_POLL_STACK_SIZE 2048
#define NONE 0

#define GET_FROM_CACHE 0x00
#define GET_FROM_SENSOR 0x01

#define MAX_SENSOR_SIZE 60

#define SENSOR_NULL 0xFF
#define SENSOR_FAIL 0xFF
#define SENSOR_NUM_MAX 0xFF

#define DEBUG_SNR 0

enum SENSOR_TYPE {
	TYPE_TMP75 = 0,
	TYPE_ADC,
	TYPE_PECI,
	TYPE_ISL69260,
	TYPE_HSC,
	TYPE_NVME,
	TYPE_PCH,
	TYPE_MEDUSA,
	TYPE_FAN
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
	SENSOR_NOT_FOUND,
	SENSOR_NOT_ACCESSIBLE,
	SENSOR_FAIL_TO_ACCESS,
	SENSOR_INIT_STATUS,
	SENSOR_UNSPECIFIED_ERROR,
	SENSOR_POLLING_DISABLE,
};

typedef struct _snr_cfg__ {
	uint8_t num;
	uint8_t type;
	uint8_t port; // port, bus, channel, etc.
	uint8_t target_addr;
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

extern bool enable_sensor_poll_thread;
extern uint8_t SDR_NUM;
extern snr_cfg *sensor_config;
// Mapping sensor number to sensor config index
extern uint8_t sensor_config_index_map[SENSOR_NUM_MAX];
extern uint8_t sensor_config_num;

void clear_unaccessible_sensor_cache();
uint8_t get_sensor_reading(uint8_t sensor_num, float *reading, uint8_t read_mode);
void pal_set_sensor_poll_interval(int *interval_ms);
bool stby_access(uint8_t sensor_num);
bool DC_access(uint8_t sensor_num);
bool post_access(uint8_t sensor_num);
bool sensor_init(void);
void disable_sensor_poll();
void enable_sensor_poll();
void pal_fix_sensor_config(void);
bool check_sensor_num_exist(uint8_t sensor_num);
void add_sensor_config(snr_cfg config);

#endif
