#include <stdio.h>
#include <stdlib.h>
#include "sensor.h"
#include "sensor_def.h"
#include "ipmi.h"

ipmb_error pch_ipmb_read(ipmi_msg *bridge_msg) {
  bridge_msg->seq_source = 0xff;
  bridge_msg->netfn = NETFN_SENSOR_REQ;
  bridge_msg->cmd = CMD_SENSOR_GET_SENSOR_READING;
  bridge_msg->InF_source = Self_IFs;
  bridge_msg->InF_target = ME_IPMB_IFs;
  bridge_msg->data_len = 1;
  bridge_msg->data[0] = 0x08;
  return ipmb_read(bridge_msg, IPMB_inf_index_map[bridge_msg->InF_target]);
}

bool pal_pch_read(uint8_t sensor_num, int *reading) {

  ipmb_error status;
  ipmi_msg *bridge_msg;
  bridge_msg = (ipmi_msg*)malloc(sizeof(ipmi_msg));
  if (bridge_msg == NULL) {
    printk("PCH bridge message alloc fail\n");
  }

  status = pch_ipmb_read(bridge_msg);
  if (status != ipmb_error_success) {
    sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_FAIL_TO_ACCESS;
    printk("ipmb read fail status: %x\n", status);
    if (bridge_msg != NULL) {
      free(bridge_msg);
    }
    return false;
  }
  if (bridge_msg->completion_code == CC_SUCCESS) {
    *reading = (cal_MBR(sensor_num, bridge_msg->data[0])) & 0xff;
    sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache = *reading;
    sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_READ_SUCCESS;
    if (bridge_msg != NULL) {
      free(bridge_msg);
    }
    return true;
  } else if (bridge_msg->completion_code == CC_NODE_BUSY) {
    uint8_t pch_retry_num;
    for (pch_retry_num = 0 ; pch_retry_num < 3 ; pch_retry_num ++) {
       ipmb_error pch_retry_result = pch_ipmb_read(bridge_msg);
      if (pch_retry_result != ipmb_error_success) {
        printk("ipmb read fail status: %x\n", status);
        if (bridge_msg != NULL) {
          free(bridge_msg);
        }
        return false;
      }
      if (bridge_msg->completion_code == CC_SUCCESS) { 
        sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache = cal_MBR(sensor_num, bridge_msg->data[0]) & 0xff;
        sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_READ_SUCCESS;
        if (bridge_msg != NULL) {
          free(bridge_msg);
        }
        return true;
      } else if (bridge_msg->completion_code != CC_NODE_BUSY) {
        if (bridge_msg != NULL) {
          free(bridge_msg);
        }
        sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_UNSPECIFIED_ERROR;
        return false;
      } 
    }
    if (bridge_msg != NULL) {
      free(bridge_msg);
    }
    printk("PCH retry read fail\n");
    sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_UNSPECIFIED_ERROR;
    return false;
  } else {
    if (bridge_msg != NULL) {
      free(bridge_msg);
    }
    sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_UNSPECIFIED_ERROR;
    return false;
  }
}
