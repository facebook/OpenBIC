#include <zephyr.h>
#include <kernel.h>
#include <stdio.h>
#include "cmsis_os2.h"
#include "ipmi.h"
#include "kcs.h"
#include <string.h>
#define IPMI_QUEUE_SIZE 5

struct k_thread IPMI_thread;
K_KERNEL_STACK_MEMBER(IPMI_thread_stack, IPMI_THREAD_STACK_SIZE);

char __aligned(4) ipmi_msgq_buffer[ipmi_buf_len * sizeof(struct ipmi_msg_cfg)];
struct k_msgq ipmi_msgq;

__weak bool pal_is_not_return_cmd(uint8_t netfn, uint8_t cmd)
{
	return 0;
}

void IPMI_CHASSIS_handler(ipmi_msg *msg)
{
	switch (msg->cmd) {
	case CMD_CHASSIS_GET_CHASSIS_STATUS:
		pal_CHASSIS_GET_CHASSIS_STATUS(msg);
		break;
	default:
		printf("invalid chassis msg netfn: %x, cmd: %x\n", msg->netfn, msg->cmd);
		msg->data_len = 0;
		break;
	}
	return;
}

void IPMI_SENSOR_handler(ipmi_msg *msg)
{
	switch (msg->cmd) {
	case CMD_SENSOR_GET_SENSOR_READING:
		pal_SENSOR_GET_SENSOR_READING(msg);
		break;
	default:
		printf("invalid sensor msg netfn: %x, cmd: %x\n", msg->netfn, msg->cmd);
		msg->data_len = 0;
		break;
	}
	return;
}

void IPMI_APP_handler(ipmi_msg *msg)
{
	switch (msg->cmd) {
	case CMD_APP_GET_DEVICE_ID:
		pal_APP_GET_DEVICE_ID(msg);
		break;
	case CMD_APP_COLD_RESET:
		break;
	case CMD_APP_WARM_RESET:
		pal_APP_WARM_RESET(msg);
		break;
	case CMD_APP_GET_SELFTEST_RESULTS:
		pal_APP_GET_SELFTEST_RESULTS(msg);
		break;
	case CMD_APP_GET_SYSTEM_GUID:
		pal_APP_GET_SYSTEM_GUID(msg);
		break;
	case CMD_APP_MASTER_WRITE_READ:
		pal_APP_MASTER_WRITE_READ(msg);
		break;
	default:
		printf("invalid APP msg netfn: %x, cmd: %x\n", msg->netfn, msg->cmd);
		msg->data_len = 0;
		break;
	}

	return;
}

void IPMI_Storage_handler(ipmi_msg *msg)
{
	switch (msg->cmd) {
	case CMD_STORAGE_GET_FRUID_INFO:
		pal_STORAGE_GET_FRUID_INFO(msg);
		break;
	case CMD_STORAGE_READ_FRUID_DATA:
		pal_STORAGE_READ_FRUID_DATA(msg);
		break;
	case CMD_STORAGE_WRITE_FRUID_DATA:
		pal_STORAGE_WRITE_FRUID_DATA(msg);
		break;
	case CMD_STORAGE_RSV_SDR:
		pal_STORAGE_RSV_SDR(msg);
		break;
	case CMD_STORAGE_GET_SDR:
		pal_STORAGE_GET_SDR(msg);
		break;
	default:
		printf("invalid Storage msg netfn: %x, cmd: %x\n", msg->netfn, msg->cmd);
		msg->data_len = 0;
		break;
	}
	return;
}


void IPMI_OEM_handler(ipmi_msg *msg)
{
	switch (msg->cmd) {
  case CMD_OEM_SENSOR_READ:
    pal_OEM_SENSOR_READ(msg);
    break;
	case CMD_OEM_SET_SYSTEM_GUID:
		pal_OEM_SET_SYSTEM_GUID(msg);
		break;
	default:
		printf("invalid OEM msg netfn: %x, cmd: %x\n", msg->netfn, msg->cmd);
		msg->data_len = 0;
		break;
	}
	return;
}

void IPMI_OEM_1S_handler(ipmi_msg *msg)
{
	switch (msg->cmd) {
	case CMD_OEM_1S_MSG_IN:
		break;
	case CMD_OEM_1S_MSG_OUT:
		pal_OEM_1S_MSG_OUT(msg);
		break;
	case CMD_OEM_1S_GET_GPIO:
		pal_OEM_1S_GET_GPIO(msg);
		break;
	case CMD_OEM_1S_SET_GPIO:
		pal_OEM_1S_SET_GPIO(msg);
		break;
	case CMD_OEM_1S_SEND_INTERRUPT_TO_BMC:
		pal_OEM_1S_SEND_INTERRUPT_TO_BMC(msg);
		break;
	case CMD_OEM_1S_FW_UPDATE:
		pal_OEM_1S_FW_UPDATE(msg);
		break;
	case CMD_OEM_1S_GET_FW_VERSION:
		pal_OEM_1S_GET_FW_VERSION(msg);
		break;
  case CMD_OEM_1S_PECIaccess:
    pal_OEM_1S_PECIaccess(msg);
    break;
  case CMD_OEM_1S_GET_POST_CODE:
    pal_OEM_1S_GET_POST_CODE(msg);
    break;
	case CMD_OEM_1S_SET_JTAG_TAP_STA:
		pal_OEM_1S_SET_JTAG_TAP_STA(msg);
		break;
	case CMD_OEM_1S_JTAG_DATA_SHIFT:
		pal_OEM_1S_JTAG_DATA_SHIFT(msg);
		break;
	case CMD_OEM_1S_SENSOR_POLL_EN:
		pal_OEM_1S_SENSOR_POLL_EN(msg);
    break;
	case CMD_OEM_1S_ACCURACY_SENSNR:
		pal_OEM_1S_ACCURACY_SENSNR(msg);
		break;
	case CMD_OEM_1S_ASD_INIT:
		pal_OEM_1S_ASD_INIT(msg);
		break;
	case CMD_OEM_1S_GET_SET_GPIO:
		pal_OEM_1S_GET_SET_GPIO(msg);
		break;
	case CMD_OEM_1S_I2C_DEV_SCAN: // debug command
		pal_OEM_1S_I2C_DEV_SCAN(msg);
		break;
	case CMD_OEM_1S_GET_BIC_STATUS:
		pal_OEM_1S_GET_BIC_STATUS(msg);
		break;
	case CMD_OEM_1S_RESET_BIC:
		pal_OEM_1S_RESET_BIC(msg);
		break;
	default:
		printf("invalid OEM msg netfn: %x, cmd: %x\n", msg->netfn, msg->cmd);
		msg->data_len = 0;
		break;

	}
	return;
}

ipmi_error IPMI_handler(void *arug0, void *arug1, void *arug2)
{
  uint8_t i;
  ipmi_msg_cfg msg_cfg;
  uint8_t *kcs_buff;

  while(1) {

    k_msgq_get(&ipmi_msgq, &msg_cfg, K_FOREVER);

    if (DEBUG_IPMI) {
      printf("IPMI_handler[%d]: netfn: %x\n", msg_cfg.buffer.data_len, msg_cfg.buffer.netfn);
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
        if ((msg_cfg.buffer.data[0] | (msg_cfg.buffer.data[1] << 8) | (msg_cfg.buffer.data[2] << 16)) == WW_IANA_ID) {
          memcpy(&msg_cfg.buffer.data[0], &msg_cfg.buffer.data[3], msg_cfg.buffer.data_len);
          msg_cfg.buffer.data_len -= 3;
          IPMI_OEM_1S_handler(&msg_cfg.buffer);
          break;
        } else if (pal_is_not_return_cmd(msg_cfg.buffer.netfn, msg_cfg.buffer.cmd)) {
          msg_cfg.buffer.completion_code = CC_INVALID_IANA;
          IPMI_OEM_1S_handler(&msg_cfg.buffer); // Due to command not returning to bridge command source, enter command handler and return with other invalid CC
          break;
        } else {
          msg_cfg.buffer.completion_code = CC_INVALID_IANA;
          msg_cfg.buffer.data_len = 0;
          break;
        }
      default: // invalid net function
        printf("invalid msg netfn: %x, cmd: %x\n", msg_cfg.buffer.netfn, msg_cfg.buffer.cmd);
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
        memcpy(&copy_data[0], &msg_cfg.buffer.data[0], msg_cfg.buffer.data_len);
        memcpy(&msg_cfg.buffer.data[3], &copy_data[0], msg_cfg.buffer.data_len);
        msg_cfg.buffer.data_len += 3;
        msg_cfg.buffer.data[0] = WW_IANA_ID & 0xFF;
        msg_cfg.buffer.data[1] = (WW_IANA_ID >> 8) & 0xFF;
        msg_cfg.buffer.data[2] = (WW_IANA_ID >> 16) & 0xFF;
      }

      if (msg_cfg.buffer.InF_source == BMC_USB_IFs) {
        USB_write(&msg_cfg.buffer);
      } else if (msg_cfg.buffer.InF_source == HOST_KCS_IFs) {
        kcs_buff = malloc(KCS_buff_size * sizeof(uint8_t));
        if ( kcs_buff == NULL ) { // allocate fail, retry allocate
            k_msleep(10);
            kcs_buff = malloc(KCS_buff_size * sizeof(uint8_t));
            if ( kcs_buff == NULL ) {
                printk("IPMI_handler: Fail to malloc for kcs_buff\n");
                continue;
            }
        }
        kcs_buff[0] = (msg_cfg.buffer.netfn + 1) << 2; // ipmi netfn response package
        kcs_buff[1] = msg_cfg.buffer.cmd;
        kcs_buff[2] = msg_cfg.buffer.completion_code;
        if(msg_cfg.buffer.data_len) {
            if ( msg_cfg.buffer.data_len <= (KCS_buff_size-3))
                memcpy(&kcs_buff[3], msg_cfg.buffer.data, msg_cfg.buffer.data_len);
            else
                memcpy(&kcs_buff[3], msg_cfg.buffer.data, (KCS_buff_size-3));
        }

        if ( DEBUG_KCS ) {
            printk("kcs from ipmi netfn %x, cmd %x, length %d, cc %x\n", kcs_buff[0], kcs_buff[1], msg_cfg.buffer.data_len, kcs_buff[2]);
        }

        kcs_write(kcs_buff, msg_cfg.buffer.data_len + 3);

        if ( kcs_buff != NULL )
            free(kcs_buff);

      } else {
        status = ipmb_send_response(&msg_cfg.buffer, IPMB_inf_index_map[msg_cfg.buffer.InF_source]);
        if (status != ipmb_error_success) {
          printf("IPMI_handler send IPMB resp fail status: %x", status);
        }
      }
    }

  }

}

void ipmi_init(void)
{
  printf("ipmi_init\n"); // rain

  k_msgq_init(&ipmi_msgq, ipmi_msgq_buffer, sizeof(struct ipmi_msg_cfg), ipmi_buf_len);

  k_thread_create(&IPMI_thread, IPMI_thread_stack,
                  K_THREAD_STACK_SIZEOF(IPMI_thread_stack),
                  IPMI_handler,
                  NULL, NULL, NULL,
                  osPriorityBelowNormal, 0, K_NO_WAIT);
  k_thread_name_set(&IPMI_thread, "IPMI_thread");

  ipmb_init();
}

