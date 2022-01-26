#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdr.h"
#include "pal.h"
#include "sensor.h"
#include "sensor_def.h"

#define SEN_DRIVE_INIT_DECLARE(name) \
  uint8_t name##_init(uint8_t snr_num)

#define SEN_DRIVE_TYPE_INIT_MAP(name) {sen_dev_##name, name##_init}

#define SEN_READ_RETRY_MAX 3

struct k_thread sensor_poll;
K_KERNEL_STACK_MEMBER(sensor_poll_stack, sensor_poll_stack_size);

uint8_t SnrNum_SnrCfg_map[SENSOR_NUM_MAX];
uint8_t SnrNum_SDR_map[SENSOR_NUM_MAX];

bool enable_sensor_poll = 1;
static bool snr_poll_eanble_flag = 1;

const int negative_ten_power[16] = {1,1,1,1,1,1,1,1000000000,100000000,10000000,1000000,100000,10000,1000,100,10};

snr_cfg *sensor_config;

SEN_DRIVE_INIT_DECLARE(tmp75);
SEN_DRIVE_INIT_DECLARE(ast_adc);
SEN_DRIVE_INIT_DECLARE(isl69259);
SEN_DRIVE_INIT_DECLARE(nvme);
SEN_DRIVE_INIT_DECLARE(mp5990);
SEN_DRIVE_INIT_DECLARE(isl28022);
SEN_DRIVE_INIT_DECLARE(pex89000);
SEN_DRIVE_INIT_DECLARE(intel_peci);
SEN_DRIVE_INIT_DECLARE(pch);
SEN_DRIVE_INIT_DECLARE(adm1278);

struct sen_drive_api {
  enum sen_dev dev;
  uint8_t (*init)(uint8_t);
} sen_drive_tbl[] = {
  SEN_DRIVE_TYPE_INIT_MAP(tmp75),
  SEN_DRIVE_TYPE_INIT_MAP(ast_adc),
  SEN_DRIVE_TYPE_INIT_MAP(isl69259),
  SEN_DRIVE_TYPE_INIT_MAP(nvme),
  SEN_DRIVE_TYPE_INIT_MAP(mp5990),
  SEN_DRIVE_TYPE_INIT_MAP(isl28022),
  SEN_DRIVE_TYPE_INIT_MAP(pex89000),
  SEN_DRIVE_TYPE_INIT_MAP(intel_peci),
  SEN_DRIVE_TYPE_INIT_MAP(pch),
  SEN_DRIVE_TYPE_INIT_MAP(adm1278),
};

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

void clear_unaccessible_sensor_cache() {
  uint8_t poll_num;

  for (poll_num = 0; poll_num < SENSOR_NUM_MAX; poll_num++) {
    if (SnrNum_SnrCfg_map[poll_num] == sensor_null) { // sensor not exist
      continue;
    }

    if ( !access_check(poll_num) ) {
      sensor_config[SnrNum_SnrCfg_map[poll_num]].cache = sensor_fail;
      sensor_config[SnrNum_SnrCfg_map[poll_num]].cache_status = SNR_INIT_STATUS;
    }
  }
}

uint8_t get_sensor_reading(uint8_t sensor_num, int *reading, uint8_t read_mode) {
  if(SnrNum_SDR_map[sensor_num] == 0xFF) { // look for sensor in SDR table
    return SNR_NOT_FOUND;
  }

  if( !access_check(sensor_num) ) { // sensor not accessable
    return SNR_NOT_ACCESSIBLE;
  }

  snr_cfg *cfg = &sensor_config[SnrNum_SnrCfg_map[sensor_num]];
  if (read_mode == get_from_sensor) {
    if (cfg->pre_sen_read_hook) {
      if (cfg->pre_sen_read_hook(sensor_num, cfg->pre_sen_read_args) == false) {
        printf("sen %d pre sensor read failed!\n", sensor_num);
        return SNR_PRE_READ_ERROR;
      }
    }
    
    int status = SNR_READ_API_UNREGISTER;
    if (cfg->read)
      status = cfg->read(sensor_num, reading);

    if (status == SNR_READ_SUCCESS || status == SNR_READ_ACUR_SUCCESS) {
      cfg->retry = 0;
      if( !access_check(sensor_num) ) { // double check access to avoid not accessible read at same moment status change
        return SNR_NOT_ACCESSIBLE;
      }

      if (cfg->post_sen_read_hook) {
        if (cfg->post_sen_read_hook(sensor_num, cfg->post_sen_read_args, reading) == false) {
          printf("sen %d post sensor read failed!\n", sensor_num);
          cfg->cache_status = SNR_POST_READ_ERROR;
          return SNR_POST_READ_ERROR;
        }
      }
      memcpy(&cfg->cache, reading, sizeof(*reading));
      status = SNR_READ_4BYTE_ACUR_SUCCESS;
      cfg->cache_status = status;
      return cfg->cache_status;
    } else {
      /* common retry */
      if (cfg->retry >= SEN_READ_RETRY_MAX)
        cfg->cache_status = status;
      else
        cfg->retry++;

      return cfg->cache_status;
    }
  } else if (read_mode == get_from_cache) {
    if (cfg->cache_status == SNR_READ_SUCCESS
        || cfg->cache_status == SNR_READ_ACUR_SUCCESS
        || cfg->cache_status == SNR_READ_4BYTE_ACUR_SUCCESS) {
      
      if (cfg->cache_status == SNR_READ_4BYTE_ACUR_SUCCESS)
        memcpy(reading, &cfg->cache, sizeof(cfg->cache));
      else
        *reading = cfg->cache;

      if( !access_check(sensor_num) ) { // double check access to avoid not accessible read at same moment status change
        return SNR_NOT_ACCESSIBLE;
      }
      return cfg->cache_status;
    } else if (cfg->cache_status == SNR_INIT_STATUS) {
      cfg->cache = sensor_fail;
      return cfg->cache_status;
    } else {
      cfg->cache = sensor_fail;
      printf("sensor[%x] cache read fail, status %x\n", sensor_num, cfg->cache_status);
      return cfg->cache_status;
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
  int SNR_POLL_INTERVEL_ms;
  k_msleep(1000); // delay 1 second to wait for drivers ready before start sensor polling

  pal_set_sensor_poll_interval(&SNR_POLL_INTERVEL_ms);

  while(1) {
    for (poll_num = 0; poll_num < SENSOR_NUM_MAX; poll_num++) {
      if (snr_poll_eanble_flag == 0) { /* skip if disable sesnor poll */
        break;
      }
      if (SnrNum_SnrCfg_map[poll_num] == sensor_null) { // sensor not exist
        continue;
      }

      int reading = 0;
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
                  CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&sensor_poll, "sensor_poll");
  return;
}

static void drive_init(void)
{
  uint16_t drive_num = ARRAY_SIZE(sen_drive_tbl);
  uint16_t i, j;

  for (i = 0; i < SDR_NUM; i++) {
    snr_cfg *p = sensor_config + i;
    for (j = 0; j < drive_num; j++) {
      if (p->type == sen_drive_tbl[j].dev) {
        sen_drive_tbl[j].init(p->num);
      break;
      }
    }

    if (j == drive_num) {
      printk("sen %d, type = %d is not supported!\n", i, p->type);
      p->read = NULL;
    }
  }
}

bool sensor_init(void) {
  init_SnrNum();
  SDR_init();

  if( SDR_NUM != 0) {
    sensor_config = (snr_cfg *)malloc(SDR_NUM * sizeof(snr_cfg));
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
  
  /* register read api of sensor_config */
  drive_init();

  if (DEBUG_SNR) {
    printf("SNR0: %s\n",full_sensor_table[SnrNum_SDR_map[1]].ID_str);
  }

  if (enable_sensor_poll) {
    sensor_poll_init();
  }
  
  return true;
}
