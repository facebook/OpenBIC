#include <stdio.h>
#include <stdlib.h>
#include "sensor.h"
#include "sensor_def.h"
#include "ipmi.h"

bool pal_pch_read(uint8_t sensor_num, int *reading) {

  ipmb_error status;
  ipmi_msg *bridge_msg;
  bridge_msg = (ipmi_msg*)malloc(sizeof(ipmi_msg));

  bridge_msg->data_len = 1;
  bridge_msg->seq_source = 0xff;
  bridge_msg->InF_source = Self_IFs;
  bridge_msg->InF_target = ME_IPMB_IFs;
  bridge_msg->netfn = NETFN_SENSOR_REQ;
  bridge_msg->cmd = CMD_SENSOR_GET_SENSOR_READING;
  bridge_msg->data[0] = 0x08;

  status = ipmb_read(bridge_msg, IPMB_inf_index_map[bridge_msg->InF_target]);
  if (status != ipmb_error_success) {
    sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_FAIL_TO_ACCESS;
    printf("ipmb read fail status: %x", status);
    free(bridge_msg);
    return false;
  }
  *reading = (cal_MBR(sensor_num, bridge_msg->data[0])) & 0xff;
  sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache = *reading;
  sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_READ_SUCCESS;
  free(bridge_msg);
  return true;
}
