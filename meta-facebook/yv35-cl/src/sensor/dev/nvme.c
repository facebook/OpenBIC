#include <stdio.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pal.h"

#define SSD0_mux_addr (0xE2 >> 1)
#define SSD0_channel 2

bool pal_nvme_read(uint8_t sensor_num, int *reading) {
  uint8_t retry = 5;
  int val;
  I2C_MSG msg;
  msg.bus = sensor_config[SnrNum_SnrCfg_map[sensor_num]].port;
  // Mux
  msg.slave_addr = SSD0_mux_addr;
  msg.data[0] = SSD0_channel;
  msg.tx_len = 1;
  msg.rx_len = 0;

  if ( !i2c_master_write(&msg, retry) ) {
    // SSD0
    msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
    msg.data[0] = sensor_config[SnrNum_SnrCfg_map[sensor_num]].offset;
    msg.tx_len = 1;
    msg.rx_len = 4;
    if ( !i2c_master_read(&msg, retry) ) {
      val = msg.data[3];
    } else {
      sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_FAIL_TO_ACCESS;
      return false;
    }
  } else {
    sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_FAIL_TO_ACCESS;
    return false;
  }

  *reading = cal_MBR(sensor_num,val) & 0xff;
  sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache = *reading;
  sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_READ_SUCCESS; 
  return true;
}
