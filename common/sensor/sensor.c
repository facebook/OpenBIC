#include <stdio.h>
#include <string.h>
#include "cmsis_os2.h"
#include "sdr.h"
#include "pal.h"
#include "sensor.h"
#include "sensor_def.h"

struct k_thread sensor_poll;
K_KERNEL_STACK_MEMBER(sensor_poll_stack, sensor_poll_stack_size);

uint8_t SnrNum_SnrCfg_map[SENSOR_NUM_MAX];
uint8_t SnrNum_SDR_map[SENSOR_NUM_MAX];

bool enable_sensor_poll = 1;
static bool snr_poll_eanble_flag = 1;

const int negative_ten_power[16] = {1,1,1,1,1,1,1,1000000000,100000000,10000000,1000000,100000,10000,1000,100,10};

snr_cfg *sensor_config;

static void init_SnrNum(void) {
  for (int i = 0; i < SENSOR_NUM_MAX; i++) {
    SnrNum_SDR_map[i] = 0xFF;
    SnrNum_SnrCfg_map[i] = 0xFF;
  }
}

void map_SnrNum_SDR_CFG(void) {
  uint8_t i,j;

  for (i = 0; i < SENSOR_NUM_MAX; i++) {
    for (j = 0; j < SDR_NUM; j++) {
      if (i == full_sensor_table[j].sensor_num) {
        SnrNum_SDR_map[i] = j;
        break;
      } else if (i == SDR_NUM) {
        SnrNum_SDR_map[i] = sensor_null;
      }
    }
    for (j = 0; j < SDR_NUM; j++) {
      if (i == sensor_config[j].num) {
        SnrNum_SnrCfg_map[i] = j;
        break;
      }
      else if (i == SDR_NUM) {
        SnrNum_SnrCfg_map[i] = sensor_null;
      }
    }
  }
  return ;
}

bool access_check(uint8_t sensor_num) {
  bool (*access_checker)(uint8_t);

  access_checker = sensor_config[SnrNum_SnrCfg_map[sensor_num]].access_checker;
  return (access_checker)(sensor_config[SnrNum_SnrCfg_map[sensor_num]].num);
}

bool sensor_read(uint8_t sensor_num, int *reading) {
  bool status;
  switch(sensor_config[SnrNum_SnrCfg_map[sensor_num]].type){
    case type_tmp75:
      status = pal_tmp75_read(sensor_num, reading);
      if (status)
        return true;
      break;
    case type_adc:
      status = pal_adc_read(sensor_num, reading);
      if (status)
        return true;
      break;
    case type_peci:
      status = pal_peci_read(sensor_num, reading);
      if (status)
        return true;
      break;
    case type_vr:
      status = pal_vr_read(sensor_num, reading);
      if (status)
        return true;
      break;
    case type_pch:
      status = pal_pch_read(sensor_num, reading);
      if (status)
        return true;
      break;
    case type_hsc:
      status = pal_hsc_read(sensor_num, reading);
      if (status)
        return true;
      break;
    case type_nvme:
      status = pal_nvme_read(sensor_num, reading);
      if (status)
        return true;
      break;

    default:
      printf("sensor_read with unexpected sensor type\n");
      return false;
      break;
  }
  return false;
}

uint8_t get_sensor_reading(uint8_t sensor_num, int *reading, uint8_t read_mode) {
  uint8_t status;

  if(SnrNum_SDR_map[sensor_num] == 0xFF) { // look for sensor in SDR table
    return SNR_NOT_FOUND;
  }

  if( !access_check(sensor_num) ) { // sensor not accessable
    return SNR_NOT_ACCESSIBLE;
  }

  if (read_mode == get_from_sensor) {
    status = sensor_read(sensor_num, reading);
    if (status) {
      if( !access_check(sensor_num) ) { // double check access to avoid not accessible read at same moment status change
        return SNR_NOT_ACCESSIBLE;
      }
      return sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status;
    } else {
      printf("sensor[%x] read fail\n",sensor_num);
      return sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status;
    }
  } else if (read_mode == get_from_cache) {
    if (sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status == SNR_READ_SUCCESS
        || sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status == SNR_READ_ACUR_SUCCESS) {
      *reading = sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache;
      if( !access_check(sensor_num) ) { // double check access to avoid not accessible read at same moment status change
        return SNR_NOT_ACCESSIBLE;
      }
      return sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status;
    } else {
      sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache = sensor_fail;
      sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_FAIL_TO_ACCESS;
      printf("sensor[%x] cache read fail\n",sensor_num);
      return sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status;
    }
  }

  return SNR_UNSPECIFIED_ERROR; // should not reach here
}

void disable_snr_poll() {
  snr_poll_eanble_flag = 0;
}

void enable_snr_poll() {
  snr_poll_eanble_flag = 1;
}

void SNR_poll_handler(void *arug0, void *arug1, void *arug2) {
  uint8_t poll_num;
  int reading, SNR_POLL_INTERVEL_ms;
  k_msleep(3000); // delay 3 second to wait for drivers ready before start sensor polling

  pal_set_sensor_poll_interval(&SNR_POLL_INTERVEL_ms);

  while(1) {
    for (poll_num = 0; poll_num < SENSOR_NUM_MAX; poll_num++) {
      if (snr_poll_eanble_flag == 0) { /* skip if disable sesnor poll */
        break;
      }
      if (SnrNum_SnrCfg_map[poll_num] == sensor_null) { // sensor not exist
        continue;
      }
      get_sensor_reading(poll_num, &reading, get_from_sensor);

      k_yield();
    }
    k_msleep(SNR_POLL_INTERVEL_ms);
  }
}

void sensor_poll_init() {
  k_thread_create(&sensor_poll, sensor_poll_stack,
                  K_THREAD_STACK_SIZEOF(sensor_poll_stack),
                  SNR_poll_handler,
                  NULL, NULL, NULL,
                  osPriorityBelowNormal, 0, K_NO_WAIT);
  k_thread_name_set(&sensor_poll, "sensor_poll");
  return;
}

bool sensor_init(void) {
  init_SnrNum();
  SDR_init();

  if( SDR_NUM != 0) {
    sensor_config = malloc(SDR_NUM * sizeof(snr_cfg));
    if(sensor_config != NULL) {
      pal_load_snr_config();
    } else {
      printf("sensor_config alloc fail\n");
      return false;
    }
  } else {
    printf("SDR_NUM == 0\n");
    return false;
  }

  pal_fix_Snrconfig();
  map_SnrNum_SDR_CFG();  
  
  if (DEBUG_SNR) {
    printf("SNR0: %s\n",full_sensor_table[SnrNum_SDR_map[1]].ID_str);
  }

  if (enable_sensor_poll) {
    sensor_poll_init();
  }
  
  return true;
}
