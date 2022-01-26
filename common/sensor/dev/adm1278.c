#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "sensor_def.h"

#define REG_PWR_MT_CFG 0xD4

uint8_t adm1278_read(uint8_t sensor_num, int *reading)
{
  if((reading == NULL) || (sensor_config[SnrNum_SnrCfg_map[sensor_num]].init_args == NULL)) {
    return SNR_UNSPECIFIED_ERROR;
  }

  adm1278_init_arg *init_arg = (adm1278_init_arg*)sensor_config[SnrNum_SnrCfg_map[sensor_num]].init_args;
  if (init_arg->is_init == false) {
    printk("adm1278_read, device isn't initialized\n");
    return SNR_UNSPECIFIED_ERROR;
  }

  if (!init_arg->r_sense) {
    printk("adm1278_read, Rsense hasn't given\n");
    return SNR_UNSPECIFIED_ERROR;
  }

  float Rsense = init_arg->r_sense;
  float val;
  uint8_t retry = 5;
  I2C_MSG msg;

  msg.bus = sensor_config[SnrNum_SnrCfg_map[sensor_num]].port;
  msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
  msg.tx_len = 1;
  msg.data[0] = sensor_config[SnrNum_SnrCfg_map[sensor_num]].offset;
  msg.rx_len = 2;

  if ( i2c_master_read(&msg, retry) )
    return SNR_FAIL_TO_ACCESS;

  switch (sensor_num) {
  case SENSOR_NUM_VOL_HSCIN:
    // m = +19599, b = 0, R = -2
    val = (float) ( ((msg.data[1] << 8)  | msg.data[0]) * 100 / 19599 );
    break;

  case SENSOR_NUM_CUR_HSCOUT:
    // m = +800 * Rsense(mohm), b = +20475, R = -1
    val = (float) ( ((msg.data[1] << 8)  | msg.data[0]) * 10 - 20475 ) / (800 * Rsense);
    break;

  case SENSOR_NUM_TEMP_HSC:
    // m = +42, b = +31880, R = -1
    val = (float) ( ((msg.data[1] << 8)  | msg.data[0]) * 10 - 31880 ) / 42;
    break;

  case SENSOR_NUM_PWR_HSCIN:
    // m = +6123 * Rsense(mohm), b = 0, R = -2
    val = (float) ( ((msg.data[1] << 8)  | msg.data[0]) * 100 ) / (6123 * Rsense);
    break;

  default:
    printf("Invalid sensor 0x%x\n", sensor_num);
    return false;
  }

  sen_val *sval = (sen_val *)reading;
  sval->integer = (int)val & 0xFFFF;
  sval->fraction = (val - sval->integer) * 1000;

  return SNR_READ_SUCCESS;
}

uint8_t adm1278_init(uint8_t sensor_num)
{
  if ( !sensor_config[SnrNum_SnrCfg_map[sensor_num]].init_args ) {
    printk("<error> ADM1278 init args not provide!\n");
    return false;
  }

  adm1278_init_arg *init_args = (adm1278_init_arg *) sensor_config[SnrNum_SnrCfg_map[sensor_num]].init_args;
  if ( init_args->is_init )
    goto skip_init;

  uint8_t retry = 5;
  I2C_MSG msg;
  msg.bus = sensor_config[SnrNum_SnrCfg_map[sensor_num]].port;
  msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
  msg.tx_len = 3;
  msg.data[0] = REG_PWR_MT_CFG;
  msg.data[1] = init_args->config.value & 0xFF;
  msg.data[2] = (init_args->config.value >> 8) & 0xFF;

  if (i2c_master_write(&msg, retry)) {
    printf("<error> ADM1278 initail failed while i2c writing\n");
    return false;
  }
      
  memset(&msg, 0, sizeof(msg));
  msg.bus = sensor_config[SnrNum_SnrCfg_map[sensor_num]].port;
  msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
  msg.tx_len = 1;
  msg.data[0] = REG_PWR_MT_CFG;
  msg.rx_len = 2;

  if (i2c_master_read(&msg, retry)) {
    printf("<error> ADM1278 initail failed while i2c reading\n");
    return false;
  }

  if ( (msg.data[0] != (init_args->config.value & 0xFF)) || (msg.data[1] != ((init_args->config.value >> 8) & 0xFF)) ) {
    printf("<error> ADM1278 initail failed with wrong reading data\n");
    return false;
  }
  init_args->is_init = 1;

skip_init:
  sensor_config[SnrNum_SnrCfg_map[sensor_num]].read = adm1278_read;
  return true;
}
