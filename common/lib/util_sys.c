#include <zephyr.h>
#include <sys/reboot.h>
#include <stdio.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "hal_gpio.h"
#include "pal.h"
#include "util_sys.h"
#include "ipmi.h"

/* get bic boot source through from SRST */
#define SYS_RST_EVT_LOG_REG 0x7e6e2074
#define SRST_POWER_ON_SET BIT(0)

static bool is_boot_ACon = false;

void set_boot_source()
{
	uint32_t sys_rst_evt;

	sys_rst_evt = sys_read32(SYS_RST_EVT_LOG_REG);
	is_boot_ACon = sys_rst_evt & 0x1;
	sys_write32(SRST_POWER_ON_SET, SYS_RST_EVT_LOG_REG);
}

bool get_boot_source_ACon()
{
	return is_boot_ACon;
}
/* get bic boot source through from SRST */

/* bic warm reset work */
#define bic_warm_reset_delay 100

void bic_warm_reset()
{
	pal_warm_reset_prepare();
	k_msleep(bic_warm_reset_delay);
	sys_reboot(SYS_REBOOT_WARM);
}

K_WORK_DEFINE(bic_warm_reset_work, bic_warm_reset);
void submit_bic_warm_reset()
{
	k_work_submit(&bic_warm_reset_work);
}
/* bic warm reset work */

/* bic cold reset work */
#define bic_cold_reset_delay 100

void bic_cold_reset()
{
	pal_cold_reset_prepare();
	k_msleep(bic_cold_reset_delay);
	sys_reboot(SYS_REBOOT_COLD);
}

K_WORK_DEFINE(bic_cold_reset_work, bic_cold_reset);
void submit_bic_cold_reset()
{
	k_work_submit(&bic_cold_reset_work);
}
/* bic cold reset work */

__weak void pal_warm_reset_prepare()
{
	return;
}

__weak void pal_cold_reset_prepare()
{
	return;
}

__weak int pal_submit_bmc_cold_reset()
{
	return -1;
}

__weak int pal_submit_12v_cycle_slot()
{
	return NOT_SUPPORT_12V_CYCLE_SLOT;
}

#define ME_FW_RECOVERY 0x01
#define ME_FW_RESTORE 0x02

static void set_ME_FW_mode(uint8_t me_fw_mode)
{
	ipmi_msg *me_msg;
	ipmb_error status;

	me_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (me_msg == NULL) {
		printf("[%s] Failed to allocate memory\n", __func__);
		return;
	}

	me_msg->seq_source = 0xFF;
	me_msg->netfn = NETFN_NM_REQ;
	me_msg->cmd = 0xDF; // Get Intel ME FW Capabilities
	me_msg->InF_source = SELF;
	me_msg->InF_target = ME_IPMB;
	me_msg->data_len = 4;
	me_msg->data[0] = 0x57;
	me_msg->data[1] = 0x01;
	me_msg->data[2] = 0x00;
	me_msg->data[3] = me_fw_mode;
	status = ipmb_read(me_msg, IPMB_inf_index_map[me_msg->InF_target]);

	if (status != IPMB_ERROR_SUCCESS) {
		printk("set_ME_FW_mode reach ME fail with status: %x\n", status);
	}

	free(me_msg);
	return;
}

void ME_enter_recovery()
{
	ipmi_msg *me_msg;
	ipmb_error status;
	uint8_t retry = 3;

	for (int i = 0; i < retry; i++) {
		set_ME_FW_mode(ME_FW_RECOVERY);
		k_msleep(2000);

		me_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
		if (me_msg == NULL) {
			printf("ME recovery msg alloc fail\n");
			k_msleep(10);
			continue;
		}

		me_msg->seq_source = 0xFF;
		me_msg->netfn = NETFN_APP_REQ;
		me_msg->cmd = CMD_APP_GET_SELFTEST_RESULTS;
		me_msg->InF_source = SELF;
		me_msg->InF_target = ME_IPMB;
		me_msg->data_len = 0;
		status = ipmb_read(me_msg, IPMB_inf_index_map[me_msg->InF_target]);
		if (status == IPMB_ERROR_SUCCESS) {
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

void ME_enter_restore()
{
	set_ME_FW_mode(ME_FW_RESTORE);
	return;
}

void set_ME_restore()
{
	ipmi_msg *me_msg;
	ipmb_error status;

	me_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (me_msg == NULL) {
		printk("ME restore msg alloc fail\n");
		return;
	}

	me_msg->seq_source = 0xFF;
	me_msg->netfn = NETFN_APP_REQ;
	me_msg->cmd = CMD_APP_GET_SELFTEST_RESULTS;
	me_msg->InF_source = SELF;
	me_msg->InF_target = ME_IPMB;
	me_msg->data_len = 0;
	status = ipmb_read(me_msg, IPMB_inf_index_map[me_msg->InF_target]);
	if (status == IPMB_ERROR_SUCCESS) {
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
