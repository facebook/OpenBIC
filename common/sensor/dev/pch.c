#include <stdio.h>
#include <stdlib.h>
#include "sensor.h"
#include "ipmi.h"

uint8_t pch_read(uint8_t sensor_num, int *reading) {
  if (!reading)
    return SNR_UNSPECIFIED_ERROR;

  ipmb_error status;
  ipmi_msg *bridge_msg;
  bridge_msg = (ipmi_msg*)malloc(sizeof(ipmi_msg));
  if (bridge_msg == NULL) {
    printk("pch_read bridge message alloc fail\n");
    return false;
  }

  /* read sensor from ME */
  bridge_msg->seq_source = 0xff;
  bridge_msg->netfn = NETFN_SENSOR_REQ;
  bridge_msg->cmd = CMD_SENSOR_GET_SENSOR_READING;
  bridge_msg->InF_source = Self_IFs;
  bridge_msg->InF_target = ME_IPMB_IFs;
  bridge_msg->data_len = 1;
  /* parameter offset is the sensor number to read from pch */
  bridge_msg->data[0] = sensor_config[SnrNum_SnrCfg_map[sensor_num]].offset;
  status = ipmb_read(bridge_msg, IPMB_inf_index_map[bridge_msg->InF_target]);

  if (status != ipmb_error_success) {
    printk("pch_read ipmb read fail, ret %d\n", status);
    free(bridge_msg);
    return SNR_UNSPECIFIED_ERROR;
  }

  sen_val *sval = (sen_val *)reading;
  memset(sval, 0, sizeof(sen_val));
  sval->integer = bridge_msg->data[0];

  free(bridge_msg);
  return SNR_READ_SUCCESS;
}

uint8_t pch_init(uint8_t sensor_num) {
  sensor_config[SnrNum_SnrCfg_map[sensor_num]].read = pch_read;
  return true;
}
