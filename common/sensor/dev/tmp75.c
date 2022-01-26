#include <stdio.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pal.h"

uint8_t tmp75_read(uint8_t sensor_num, int *reading)
{
  if (!reading)
    return SNR_UNSPECIFIED_ERROR;

  uint8_t retry = 5;
  I2C_MSG msg = {0};

  msg.bus = sensor_config[SnrNum_SnrCfg_map[sensor_num]].port;
  msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
  msg.tx_len = 1;
  msg.rx_len = 1;
  msg.data[0] = sensor_config[SnrNum_SnrCfg_map[sensor_num]].offset;

  if ( i2c_master_read(&msg, retry) )
    return SNR_FAIL_TO_ACCESS;
  
  sen_val *sval = (sen_val *)reading;
  sval->integer = msg.data[0];
  return SNR_READ_SUCCESS;
}

uint8_t tmp75_init(uint8_t sensor_num)
{
  sensor_config[SnrNum_SnrCfg_map[sensor_num]].read = tmp75_read;
  return true;
}
