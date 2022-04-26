#include <zephyr.h>
#include <kernel.h>
#include <stdio.h>
#include "cmsis_os2.h"
#include "ipmi.h"
#include "kcs.h"
#include "usb.h"
#include <string.h>
#include <stdlib.h>
#include "app_handler.h"
#include "chassis_handler.h"
#include "oem_handler.h"
#include "oem_1s_handler.h"
#include "sensor_handler.h"
#include "storage_handler.h"

#define IPMI_QUEUE_SIZE 5

struct k_thread IPMI_thread;
K_KERNEL_STACK_MEMBER(IPMI_thread_stack, IPMI_THREAD_STACK_SIZE);

char __aligned(4) ipmi_msgq_buffer[IPMI_BUF_LEN * sizeof(struct ipmi_msg_cfg)];
struct k_msgq ipmi_msgq;

__weak bool pal_request_msg_to_BIC_from_KCS(uint8_t netfn, uint8_t cmd)
{
	if (netfn == NETFN_OEM_1S_REQ) {
		if ((cmd == CMD_OEM_1S_FW_UPDATE) || (cmd == CMD_OEM_1S_RESET_BMC) ||
		    (cmd == CMD_OEM_1S_GET_BIC_STATUS) || (cmd == CMD_OEM_1S_RESET_BIC))
			return true;
	}

	return false;
}

__weak bool pal_request_msg_to_BIC_from_ME(uint8_t netfn, uint8_t cmd)
{
	if ((netfn == NETFN_OEM_REQ) && (cmd == CMD_OEM_NM_SENSOR_READ)) {
		return true;
	}

	return false;
}

__weak bool pal_is_not_return_cmd(uint8_t netfn, uint8_t cmd)
{
	if ((netfn == NETFN_OEM_1S_REQ)) {
		if ((cmd == CMD_OEM_1S_MSG_OUT) || (cmd == CMD_OEM_1S_MSG_IN)) {
			return true;
		}
	}

	// Reserve for future commands

	return false;
}

void IPMI_handler(void *arug0, void *arug1, void *arug2)
{
	uint8_t i;
	ipmi_msg_cfg msg_cfg;
	uint8_t *kcs_buff;

	while (1) {
		k_msgq_get(&ipmi_msgq, &msg_cfg, K_FOREVER);

		if (DEBUG_IPMI) {
			printf("IPMI_handler[%d]: netfn: %x\n", msg_cfg.buffer.data_len,
			       msg_cfg.buffer.netfn);
			for (i = 0; i < msg_cfg.buffer.data_len; i++) {
				printf(" 0x%2x", msg_cfg.buffer.data[i]);
			}
			printf("\n");
		}

		msg_cfg.buffer.completion_code = CC_INVALID_CMD;
		switch (msg_cfg.buffer.netfn) {
		case NETFN_CHASSIS_REQ:
			IPMI_CHASSIS_handler(&msg_cfg.buffer);
			break;
		case NETFN_BRIDGE_REQ:
			// IPMI_BRIDGE_handler();
			break;
		case NETFN_SENSOR_REQ:
			IPMI_SENSOR_handler(&msg_cfg.buffer);
			break;
		case NETFN_APP_REQ:
			IPMI_APP_handler(&msg_cfg.buffer);
			break;
		case NETFN_FIRMWARE_REQ:
			break;
		case NETFN_STORAGE_REQ:
			IPMI_Storage_handler(&msg_cfg.buffer);
			break;
		case NETFN_TRANSPORT_REQ:
			break;
		case NETFN_DCMI_REQ:
			break;
		case NETFN_NM_REQ:
			break;
		case NETFN_OEM_REQ:
			IPMI_OEM_handler(&msg_cfg.buffer);
			break;
		case NETFN_OEM_1S_REQ:
			if ((msg_cfg.buffer.data[0] | (msg_cfg.buffer.data[1] << 8) |
			     (msg_cfg.buffer.data[2] << 16)) == IANA_ID) {
				memcpy(&msg_cfg.buffer.data[0], &msg_cfg.buffer.data[3],
				       msg_cfg.buffer.data_len);
				msg_cfg.buffer.data_len -= 3;
				IPMI_OEM_1S_handler(&msg_cfg.buffer);
				break;
			} else if (pal_is_not_return_cmd(msg_cfg.buffer.netfn,
							 msg_cfg.buffer.cmd)) {
				// Due to command not returning to bridge command source,
				// enter command handler and return with other invalid CC
				msg_cfg.buffer.completion_code = CC_INVALID_IANA;
				IPMI_OEM_1S_handler(&msg_cfg.buffer);
				break;
			} else {
				msg_cfg.buffer.completion_code = CC_INVALID_IANA;
				msg_cfg.buffer.data_len = 0;
				break;
			}
		default: // invalid net function
			printf("invalid msg netfn: %x, cmd: %x\n", msg_cfg.buffer.netfn,
			       msg_cfg.buffer.cmd);
			msg_cfg.buffer.data_len = 0;
			break;
		}

		if (pal_is_not_return_cmd(msg_cfg.buffer.netfn, msg_cfg.buffer.cmd)) {
			;
		} else {
			ipmb_error status;

			if (msg_cfg.buffer.completion_code != CC_SUCCESS) {
				msg_cfg.buffer.data_len = 0;
			} else if (msg_cfg.buffer.netfn == NETFN_OEM_1S_REQ) {
				uint8_t copy_data[msg_cfg.buffer.data_len];
				memcpy(&copy_data[0], &msg_cfg.buffer.data[0],
				       msg_cfg.buffer.data_len);
				memcpy(&msg_cfg.buffer.data[3], &copy_data[0],
				       msg_cfg.buffer.data_len);
				msg_cfg.buffer.data_len += 3;
				msg_cfg.buffer.data[0] = IANA_ID & 0xFF;
				msg_cfg.buffer.data[1] = (IANA_ID >> 8) & 0xFF;
				msg_cfg.buffer.data[2] = (IANA_ID >> 16) & 0xFF;
			}

			if (msg_cfg.buffer.InF_source == BMC_USB) {
				usb_write_by_ipmi(&msg_cfg.buffer);
			} else if (msg_cfg.buffer.InF_source == HOST_KCS) {
#ifdef CONFIG_IPMI_KCS_ASPEED
				kcs_buff = malloc(KCS_BUFF_SIZE * sizeof(uint8_t));
				if (kcs_buff == NULL) { // allocate fail, retry allocate
					k_msleep(10);
					kcs_buff = malloc(KCS_BUFF_SIZE * sizeof(uint8_t));
					if (kcs_buff == NULL) {
						printf("IPMI_handler: Fail to malloc for kcs_buff\n");
						continue;
					}
				}
				kcs_buff[0] = (msg_cfg.buffer.netfn + 1)
					      << 2; // ipmi netfn response package
				kcs_buff[1] = msg_cfg.buffer.cmd;
				kcs_buff[2] = msg_cfg.buffer.completion_code;
				if (msg_cfg.buffer.data_len) {
					if (msg_cfg.buffer.data_len <= (KCS_BUFF_SIZE - 3))
						memcpy(&kcs_buff[3], msg_cfg.buffer.data,
						       msg_cfg.buffer.data_len);
					else
						memcpy(&kcs_buff[3], msg_cfg.buffer.data,
						       (KCS_BUFF_SIZE - 3));
				}

				if (DEBUG_KCS) {
					printf("kcs from ipmi netfn %x, cmd %x, length %d, cc %x\n",
					       kcs_buff[0], kcs_buff[1], msg_cfg.buffer.data_len,
					       kcs_buff[2]);
				}

				kcs_write(kcs_buff, msg_cfg.buffer.data_len + 3);

				if (kcs_buff != NULL)
					free(kcs_buff);
#endif

			} else {
				status = ipmb_send_response(
					&msg_cfg.buffer,
					IPMB_inf_index_map[msg_cfg.buffer.InF_source]);
				if (status != IPMB_ERROR_SUCCESS) {
					printf("IPMI_handler send IPMB resp fail status: %x",
					       status);
				}
			}
		}
	}
}

void ipmi_init(void)
{
	printf("ipmi_init\n"); // rain

	k_msgq_init(&ipmi_msgq, ipmi_msgq_buffer, sizeof(struct ipmi_msg_cfg), IPMI_BUF_LEN);

	k_thread_create(&IPMI_thread, IPMI_thread_stack, K_THREAD_STACK_SIZEOF(IPMI_thread_stack),
			IPMI_handler, NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&IPMI_thread, "IPMI_thread");

	ipmb_init();
}
