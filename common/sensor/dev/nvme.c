#include <stdio.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "pal.h"

#define NVMe_NOT_AVAILABLE 0x80
#define NVMe_TMPSNR_FAILURE 0x81

uint8_t nvme_read(uint8_t sensor_num, int *reading)
{
  if (!reading)
    return SNR_UNSPECIFIED_ERROR;

  uint8_t retry = 5;
  int val;
  bool is_drive_ready;
  I2C_MSG msg;

  msg.bus = sensor_config[SnrNum_SnrCfg_map[sensor_num]].port;
  msg.slave_addr = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
  msg.data[0] = sensor_config[SnrNum_SnrCfg_map[sensor_num]].offset;
  msg.tx_len = 1;
  msg.rx_len = 4;

  if ( !i2c_master_read(&msg, retry) ) {
    /* Check SSD drive ready */
    is_drive_ready = ( ( msg.data[1] & 0x40 ) == 0 ? true : false );
    if ( !is_drive_ready )
      return SNR_NOT_ACCESSIBLE;

    /* Check reading value */
    val = msg.data[3];
    if ( val == NVMe_NOT_AVAILABLE )
      return SNR_FAIL_TO_ACCESS;
    else if ( val == NVMe_TMPSNR_FAILURE )
      return SNR_UNSPECIFIED_ERROR;
  } else
    return SNR_FAIL_TO_ACCESS;

  sen_val *sval = (sen_val *)reading;
  sval->integer = val & 0xFF;

  return SNR_READ_SUCCESS;
}

uint8_t nvme_init(uint8_t sensor_num)
{
  sensor_config[SnrNum_SnrCfg_map[sensor_num]].read = nvme_read;
  return true;
}
