#include <zephyr.h>
#include <stdlib.h>
#include "ipmi.h"
#include "plat_gpio.h"

#define ME_FW_recovery 0x01
#define ME_FW_restore 0x02

static void set_ME_FW_mode(uint8_t ME_FW_mode)
{
	ipmi_msg *me_msg;
	ipmb_error status;

	me_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (me_msg == NULL) {
		printk("ME restore msg alloc fail\n");
		return false;
	}

	me_msg->seq_source = 0xFF;
	me_msg->netfn = NETFN_NM_REQ;
	me_msg->cmd = 0xDF; // Get Intel ME FW Capabilities
	me_msg->InF_source = Self_IFs;
	me_msg->InF_target = ME_IPMB_IFs;
	me_msg->data_len = 4;
	me_msg->data[0] = 0x57;
	me_msg->data[1] = 0x01;
	me_msg->data[2] = 0x00;
	me_msg->data[3] = ME_FW_mode;
	status = ipmb_read(me_msg, IPMB_inf_index_map[me_msg->InF_target]);

	if (status != ipmb_error_success) {
		printk("set_ME_FW_mode reach ME fail with status: %x\n", status);
	}

	free(me_msg);
	return;
}

static void ME_enter_restore()
{
	set_ME_FW_mode(ME_FW_restore);
	return;
}

static void ME_enter_recovery()
{
	ipmi_msg *me_msg;
	ipmb_error status;
	uint8_t retry = 3;

	for (int i = 0; i < retry; i++) {
		set_ME_FW_mode(ME_FW_recovery);
		k_msleep(2000);

		me_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
		if (me_msg == NULL) {
			printk("ME recovery msg alloc fail\n");
			k_msleep(10);
			continue;
		}

		me_msg->seq_source = 0xFF;
		me_msg->netfn = NETFN_APP_REQ;
		me_msg->cmd = CMD_APP_GET_SELFTEST_RESULTS;
		me_msg->InF_source = Self_IFs;
		me_msg->InF_target = ME_IPMB_IFs;
		me_msg->data_len = 0;
		status = ipmb_read(me_msg, IPMB_inf_index_map[me_msg->InF_target]);
		if (status == ipmb_error_success) {
			if ((me_msg->data_len == 2) && (me_msg->data[0] == 0x81) &&
			    (me_msg->data[1] == 0x02)) {
				free(me_msg);
				break;
			}
		} else {
			printk("ME restore get selftest fail with status: %x\n", status);
		}
		if (me_msg != NULL) {
			free(me_msg);
		}
	}

	return;
}

void set_ME_restore()
{
	ipmi_msg *me_msg;
	ipmb_error status;

	me_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (me_msg == NULL) {
		printk("ME restore msg alloc fail\n");
		return false;
	}

	me_msg->seq_source = 0xFF;
	me_msg->netfn = NETFN_APP_REQ;
	me_msg->cmd = CMD_APP_GET_SELFTEST_RESULTS;
	me_msg->InF_source = Self_IFs;
	me_msg->InF_target = ME_IPMB_IFs;
	me_msg->data_len = 0;
	status = ipmb_read(me_msg, IPMB_inf_index_map[me_msg->InF_target]);
	if (status == ipmb_error_success) {
		if ((me_msg->data_len == 2) && (me_msg->data[0] == 0x81) &&
		    (me_msg->data[1] == 0x02)) {
			ME_enter_restore();
		}
	} else {
		printk("ME restore get selftest fail with status: %x\n", status);
	}
	free(me_msg);
	return;
}

void pal_warm_reset_prepare()
{
	ME_enter_recovery();
}

void pal_cold_reset_prepare()
{
	ME_enter_recovery();
}

/* BMC reset */
void BMC_reset_handler()
{
	gpio_set(RST_BMC_R_N, GPIO_LOW);
	k_msleep(10);
	gpio_set(RST_BMC_R_N, GPIO_HIGH);
}

K_DELAYED_WORK_DEFINE(BMC_reset_work, BMC_reset_handler);
void submit_bmc_warm_reset()
{
	k_work_schedule(&BMC_reset_work, K_MSEC(1000));
}
/* BMC reset */
