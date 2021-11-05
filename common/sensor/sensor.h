#ifndef SENSOR_H
#define SENSOR_H

#include "sdr.h"

#define sensor_poll_stack_size 1000

#define get_from_sensor 0x00
#define get_from_cache 0x01

#define MAX_SNR_SIZE 60

#define sensor_null 0xFF
#define sensor_fail 0xFF
#define SENSOR_NUM_MAX 0xFF

#define DEBUG_SNR 0

/*  define sensor type  */
#define type_tmp75 0x00
#define type_adc   0x01
#define type_peci  0x02
#define type_vr    0x03
#define type_hsc   0x04
#define type_nvme  0x05
#define type_pch   0x06


static inline int cal_MBR(uint8_t sensor_num, int val){
  if( SDR_M(sensor_num) == 0 ) {
    return ( val * SDR_Rexp(sensor_num) + round_add(sensor_num, val) );
  }
  return ( val * SDR_Rexp(sensor_num) / SDR_M(sensor_num) + round_add(sensor_num, val) ); 
}

enum {
  SNR_READ_SUCCESS,
  SNR_NOT_FOUND,
  SNR_NOT_ACCESSIBLE,
  SNR_FAIL_TO_ACCESS,
  SNR_INIT_STATUS,
  SNR_UNSPECIFIED_ERROR,
};

typedef struct _snr_cfg__ {
  uint8_t num;
  uint8_t type;
  uint8_t port; // port, bus, channel, etc.
  uint8_t slave_addr;
  uint8_t offset;
  bool (*access_checker)(uint8_t);
  int arg0;
  int arg1;
  uint8_t cache;
  uint8_t cache_status;
} snr_cfg;
  
extern bool enable_sensor_poll;
extern uint8_t SDR_NUM;
extern snr_cfg *sensor_config;
extern uint8_t SnrNum_SnrCfg_map[SENSOR_NUM_MAX];

uint8_t get_sensor_reading(uint8_t sensor_num, int *reading, uint8_t read_mode);
bool sensor_init(void);

#endif
