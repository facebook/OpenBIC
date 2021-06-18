/* 
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "board_device.h"
#include "objects.h"
#include "kcs.h"
#include "ipmi.h"

extern aspeed_device_t kcs3_dev;
extern IPMB_config IPMB_config_table[];

struct kcs_request {
	uint8_t netfn;
	uint8_t cmd;
	uint8_t data[0];
};

struct kcs_response {
	uint8_t netfn;
	uint8_t cmd;
	uint8_t cmplt_code;
	uint8_t data[0];
};

static osThreadId_t host_kcs_tid;
static osThreadAttr_t host_kcs_tattr;

kcs_t kcs3 = { .device = &kcs3_dev };

static void kcs_read_task(void *arg)
{
	int i, rc;
	uint8_t ibuf[KCS_buff_size];
  ipmi_msg *bridge_msg;
  ipmb_error status;

	struct kcs_request *req;

	aspeed_kcs_init(&kcs3);

	while (1) {
		rc = aspeed_kcs_read(&kcs3, ibuf, sizeof(ibuf));
		if (rc < 0) {
			printf("failed to read KCS data, rc=%d\n", rc);
			continue;
		}

    if ( DEBUG_KCS ) {
		  printf("host KCS read: netfn=0x%02x, cmd=0x%02x, data:\n", ibuf[0], ibuf[1]);
  		for (i = 2; i < rc; ++i) {
  			if (i && (i % 16 == 0))
  				printf("\n");
  			printf("%02x ", ibuf[i]);
  		}
	  	printf("\n");
    }

		req = (struct kcs_request *)ibuf;
    req->netfn = req->netfn >> 2;

    if (1) { // default command for BMC, should add BIC firmware update, BMC reset, real time sensor read in future
      bridge_msg = (ipmi_msg*)pvPortMalloc(sizeof(ipmi_msg));

      bridge_msg->data_len = rc - 2; // exclude netfn, cmd
      bridge_msg->seq_source = 0xff; // No seq for KCS
      bridge_msg->InF_source = HOST_KCS_IFs;
      bridge_msg->InF_target = BMC_IPMB_IFs; // default bypassing IPMI standard command to BMC
      bridge_msg->netfn = req->netfn;
      bridge_msg->cmd = req->cmd;
      if (bridge_msg->data_len != 0) {
        memcpy( &bridge_msg->data[0], &ibuf[2], rc );
      }

      status = ipmb_send_request(bridge_msg, IPMB_inf_index_map[BMC_IPMB_IFs]);
      if (status != ipmb_error_success) {
        printf("kcs_read_task send to BMC fail status: %x", status);
      }
      vPortFree(bridge_msg);
    }
    
		osDelay(10);
	}
}

void host_kcs_init(void)
{
	host_kcs_tattr.name = "kcs_read_thread";
	host_kcs_tattr.priority = osPriorityBelowNormal;
	host_kcs_tattr.stack_size = 4096;

	host_kcs_tid = osThreadNew(kcs_read_task, NULL, &host_kcs_tattr);
}
