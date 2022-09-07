
#ifdef CONFIG_IPMI_KCS_ASPEED

#include <zephyr.h>
#include <string.h>
#include <stdio.h>
#include <device.h>
#include <stdlib.h>
#include "ipmi.h"
#include "kcs.h"
#include "plat_def.h"
#include "libutil.h"

struct k_thread kcs_polling;
K_KERNEL_STACK_MEMBER(KCS_POLL_stack, KCS_POLL_STACK_SIZE);

static const struct device *kcs_dev;
static bool proc_kcs_ok = false;

int kcs_aspeed_read(const struct device *dev, uint8_t *buf, uint32_t buf_sz);

int kcs_aspeed_write(const struct device *dev, uint8_t *buf, uint32_t buf_sz);

void kcs_write(uint8_t *buf, uint32_t buf_sz)
{
	int rc;

	rc = kcs_aspeed_write(kcs_dev, buf, buf_sz);
	if (rc < 0) {
		printf("failed to write KCS data, rc=%d\n", rc);
	}
}

bool get_kcs_ok()
{
	return proc_kcs_ok;
}

void reset_kcs_ok()
{
	proc_kcs_ok = false;
}

void kcs_read(void *arvg0, void *arvg1, void *arvg2)
{
	int i, rc;
	uint8_t ibuf[KCS_BUFF_SIZE];
	ipmi_msg bridge_msg;
	ipmi_msg_cfg current_msg;
	ipmb_error status;

	struct kcs_request *req;

	while (1) {
		k_msleep(KCS_POLLING_INTERVAL);

		rc = kcs_aspeed_read(kcs_dev, ibuf, sizeof(ibuf));
		if (rc < 0) {
			if (rc != -ENODATA)
				printf("failed to read KCS data, rc=%d\n", rc);
			continue;
		}

		if (DEBUG_KCS) {
			printf("host KCS read: netfn=0x%02x, cmd=0x%02x, data:\n", ibuf[0],
			       ibuf[1]);
			for (i = 2; i < rc; ++i) {
				if (i && (i % 16 == 0))
					printf("\n");
				printf("%02x ", ibuf[i]);
			}
			printf("\n");
		}

		proc_kcs_ok = true;
		req = (struct kcs_request *)ibuf;
		req->netfn = req->netfn >> 2;

		if (pal_request_msg_to_BIC_from_KCS(
			    req->netfn, req->cmd)) { // In-band update command, not bridging to bmc
			current_msg.buffer.InF_source = HOST_KCS;
			current_msg.buffer.netfn = req->netfn;
			current_msg.buffer.cmd = req->cmd;
			current_msg.buffer.data_len = rc - 2; // exclude netfn, cmd
			if (current_msg.buffer.data_len != 0) {
				memcpy(current_msg.buffer.data, req->data,
				       current_msg.buffer.data_len);
			}

			if (DEBUG_KCS) {
				printf("kcs to ipmi netfn %x, cmd %x, length %d\n",
				       current_msg.buffer.netfn, current_msg.buffer.cmd,
				       current_msg.buffer.data_len);
			}

			while (k_msgq_put(&ipmi_msgq, &current_msg, K_NO_WAIT) != 0) {
				k_msgq_purge(&ipmi_msgq);
				printf("KCS retrying put ipmi msgq\n");
			}

		} else { // default command for BMC, should add BIC firmware update, BMC reset, real time sensor read in future
			if (pal_immediate_respond_from_KCS(req->netfn, req->cmd)) {
				do { // break if malloc fail.
					uint8_t *kcs_buff;
					kcs_buff = malloc(KCS_BUFF_SIZE * sizeof(uint8_t));
					if (kcs_buff == NULL) {
						printf("[%s] Failed to malloc for kcs_buff\n",
						       __func__);
						break;
					}
					kcs_buff[0] = (req->netfn | BIT(0)) << 2;
					kcs_buff[1] = req->cmd;
					kcs_buff[2] = CC_SUCCESS;

					if (((req->netfn == NETFN_STORAGE_REQ) &&
					     (req->cmd == CMD_STORAGE_ADD_SEL))) {
						kcs_buff[3] = 0x00;
						kcs_buff[4] = 0x00;
						kcs_write(kcs_buff, 5);
					} else {
						kcs_write(kcs_buff, 3);
					}
					SAFE_FREE(kcs_buff);
				} while (0);
			}
			bridge_msg.data_len = rc - 2; // exclude netfn, cmd
			bridge_msg.seq_source = 0xff; // No seq for KCS
			bridge_msg.InF_source = HOST_KCS;
			bridge_msg.InF_target =
				BMC_IPMB; // default bypassing IPMI standard command to BMC
			bridge_msg.netfn = req->netfn;
			bridge_msg.cmd = req->cmd;
			if (bridge_msg.data_len != 0) {
				memcpy(&bridge_msg.data[0], &ibuf[2], rc);
			}

			status = ipmb_send_request(&bridge_msg, IPMB_inf_index_map[BMC_IPMB]);
			if (status != IPMB_ERROR_SUCCESS) {
				printf("kcs_read_task send to BMC fail status: %x", status);
			}
		}
	}
}

void kcs_init(void)
{
	kcs_dev = device_get_binding(DT_LABEL(DT_NODELABEL(HOST_KCS_PORT)));
	if (!kcs_dev) {
		printf("No KCS device found\n");
		return;
	}

	k_thread_create(&kcs_polling, KCS_POLL_stack, K_THREAD_STACK_SIZEOF(KCS_POLL_stack),
			kcs_read, NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&kcs_polling, "kcs_polling");
}

#endif
