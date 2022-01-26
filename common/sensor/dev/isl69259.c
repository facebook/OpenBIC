#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "hal_i2c.h"


uint8_t isl69259_read(uint8_t sensor_num, int* reading) {
  if (reading == NULL) {
    return SNR_UNSPECIFIED_ERROR;
  }

  uint8_t retry = 5;
  sen_val *snr_val = (sen_val*)reading;
  I2C_MSG msg;
  memset(snr_val, 0, sizeof(sen_val));

  msg.bus = sensor_config[SnrNum_SnrCfg_map[sensor_num]].port;
  msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
  msg.tx_len = 1;
  msg.rx_len = 2;
  msg.data[0] = sensor_config[SnrNum_SnrCfg_map[sensor_num]].offset;
  
  if (i2c_master_read(&msg, retry)) {
    /* read fail */
    return SNR_FAIL_TO_ACCESS;
  }

  uint8_t offset = sensor_config[SnrNum_SnrCfg_map[sensor_num]].offset;
  if (offset == PMBUS_READ_VOUT) {
    /* 1 mV/LSB, unsigned integer */
    snr_val->integer = ((msg.data[1] << 8) | msg.data[0]) / 1000;
    snr_val->fraction = ((msg.data[1] << 8) | msg.data[0]) % 1000;
  } else if (offset == PMBUS_READ_IOUT) {
    /* 0.1 A/LSB, 2's complement */
    snr_val->integer = (int16_t)((msg.data[1] << 8) | msg.data[0]) / 10;
    snr_val->fraction = (int16_t)(((msg.data[1] << 8) | msg.data[0]) - (snr_val->integer * 10)) * 100;
  } else if (offset == PMBUS_READ_TEMPERATURE_1) {
    /* 1 Degree C/LSB, 2's complement */
    snr_val->integer = ((msg.data[1] << 8) | msg.data[0]);
  } else if (offset == PMBUS_READ_POUT) {
    /* 1 Watt/LSB, 2's complement */
    snr_val->integer = ((msg.data[1] << 8) | msg.data[0]);
  } else {
    return SNR_FAIL_TO_ACCESS;
  }

  return SNR_READ_SUCCESS;
}

uint8_t isl69259_init(uint8_t sensor_num) {
  sensor_config[SnrNum_SnrCfg_map[sensor_num]].read = isl69259_read;
  return true;
}
