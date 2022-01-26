#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"

#define ISL28022_CONFIG_REG           0
#define ISL28022_BUS_VOLTAGE_REG      2
#define ISL28022_POWER_REG            3
#define ISL28022_CURRENT_REG          4
#define ISL28022_CALIBRATION_REG      5


uint8_t isl28022_read(uint8_t sensor_num, int* reading) {
  if((reading == NULL) ||
     (sensor_config[SnrNum_SnrCfg_map[sensor_num]].init_args == NULL)) {
    return SNR_UNSPECIFIED_ERROR;
  }

  isl28022_init_arg *init_arg = (isl28022_init_arg*)sensor_config[SnrNum_SnrCfg_map[sensor_num]].init_args;
  if (init_arg->is_init == false) {
    printk("isl28022_read, device isn't initialized\n");
    return SNR_UNSPECIFIED_ERROR;
  }

  uint8_t retry = 5;
  I2C_MSG msg;
  uint8_t offset = sensor_config[SnrNum_SnrCfg_map[sensor_num]].offset;
  sen_val *snr_val = (sen_val*)reading;
  memset(snr_val, 0, sizeof(sen_val));

  msg.bus = sensor_config[SnrNum_SnrCfg_map[sensor_num]].port;
  msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
  msg.tx_len = 1;
  msg.rx_len = 2;
  msg.data[0] = offset;
  if (i2c_master_read(&msg, retry)) {
    /* read fail */
    return SNR_FAIL_TO_ACCESS;
  }

  if (offset == ISL28022_BUS_VOLTAGE_REG) {
    /* unsigned */
    uint16_t read_mv;

    if ((init_arg->config.fields.BRNG == 0b11) || (init_arg->config.fields.BRNG == 0b10)) {
      read_mv = ((msg.data[0] << 6) | (msg.data[1] >> 2)) * 4;
    } else if (init_arg->config.fields.BRNG == 0b01) {
      read_mv = ((msg.data[0] << 5) | (msg.data[1] >> 3)) * 4;
    } else {
      read_mv = (((msg.data[0] & BIT_MASK(7)) << 5) | (msg.data[1] >> 3)) * 4;
    }
    snr_val->integer = read_mv / 1000;
    snr_val->fraction = read_mv % 1000;
  } else if (offset == ISL28022_CURRENT_REG) {
    /* signed */
    float read_current = ((int16_t)(msg.data[0] << 8) | msg.data[1]) * init_arg->current_LSB;
    snr_val->integer = read_current;
    snr_val->fraction = (read_current - snr_val->integer) * 1000;
  } else if (offset == ISL28022_POWER_REG) {
    /* unsigned */
    float read_power = ((msg.data[0] << 8) | msg.data[1]) * init_arg->current_LSB * 20;
    snr_val->integer = read_power;
    snr_val->fraction = ((read_power - snr_val->integer) * 1000);
  } else {
    return SNR_FAIL_TO_ACCESS;
  }

  return SNR_READ_SUCCESS;
}

uint8_t isl28022_init(uint8_t sensor_num) {
  if (sensor_config[SnrNum_SnrCfg_map[sensor_num]].init_args == NULL) {
    printk("isl28022_init: init_arg is NULL\n");
    return false;
  }

  sensor_config[SnrNum_SnrCfg_map[sensor_num]].read = isl28022_read;
  isl28022_init_arg *init_arg = (isl28022_init_arg*)sensor_config[SnrNum_SnrCfg_map[sensor_num]].init_args;
  if (init_arg->is_init == true) {
    return true;
  }

  I2C_MSG msg;
  uint8_t retry = 5;
  
  /* set configuration register */
  msg.bus = sensor_config[SnrNum_SnrCfg_map[sensor_num]].port;
  msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
  msg.tx_len = 3;
  msg.data[0] = ISL28022_CONFIG_REG;
  msg.data[1] = (init_arg->config.value >> 8) & 0xFF;
  msg.data[2] = init_arg->config.value & 0xFF;
  if (i2c_master_write(&msg, retry)) {
    printk("isl28022_init, set configuration register fail\n");
    return false;
  }

  /* calculate and set calibration */
  uint16_t v_shunt_fs, adc_res, calibration;

  v_shunt_fs = 40 << (init_arg->config.fields.PG);
  if (!(init_arg->config.fields.SADC & BIT(3)) && ((init_arg->config.fields.SADC & BIT_MASK(2)) < 3)) {
    adc_res = 1 << (12 + (init_arg->config.fields.SADC & BIT_MASK(2)));
  } else {
    adc_res = 32768;
  }
  init_arg->current_LSB = (float)v_shunt_fs / (init_arg->r_shunt * adc_res);
  calibration = (40.96 / (init_arg->current_LSB * init_arg->r_shunt));
  calibration = calibration << 1;  /* 15 bits, bit[0] is fix to 0 */

  msg.bus = sensor_config[SnrNum_SnrCfg_map[sensor_num]].port;
  msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
  msg.tx_len = 3;
  msg.data[0] = ISL28022_CALIBRATION_REG;
  msg.data[1] = (calibration >> 8) & 0xFF;
  msg.data[2] = calibration & 0xFF;
  if (i2c_master_write(&msg, retry)) {
    printk("isl28022_init, set calibration register fail\n");
    return false;
  }

  init_arg->is_init = true;
  return true;
}
