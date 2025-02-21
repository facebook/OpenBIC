/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <zephyr.h>
#include <soc.h>
#include "hal_gpio.h"
#include "hal_peci.h"
#include "power_status.h"
#include "util_sys.h"
#include "plat_gpio.h"
#include "util_worker.h"
#include "plat_mctp.h"
#include "plat_apml.h"
#include <stdio.h>
#include <stdlib.h>
#include "hal_i3c.h"
#include "libutil.h"
#include "mctp_ctrl.h"
#include "rg3mxxb12.h"
#include "plat_mctp.h"
#include "plat_i2c_target.h"
#include "plat_pldm_monitor.h"
#include "plat_class.h"
#include "plat_i3c.h"
#include "plat_isr.h"
#include "plat_dimm.h"
#include "pcc.h"
#include "plat_kcs.h"
#include "plat_pldm_sensor.h"
#include "plat_pmic.h"

SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e6e2610, 0x04020000 },
	{ 0x7e6e2618, 0x00c30000 },
};

void pal_pre_init()
{
	gpio_init(NULL);
	scu_init(scu_cfg, sizeof(scu_cfg) / sizeof(SCU_CFG));
	aspeed_print_sysrst_info();

	/* init i2c target */
	for (int index = 0; index < MAX_TARGET_NUM; index++) {
		if (I2C_TARGET_ENABLE_TABLE[index])
			i2c_target_control(
				index, (struct _i2c_target_config *)&I2C_TARGET_CONFIG_TABLE[index],
				1);
	}

	init_platform_config();

	//TODO: if 1OU card present
	// i3c master initial
	I3C_MSG i3c_msg = { 0 };
	i3c_msg.bus = I3C_BUS_HUB;
	i3c_msg.target_addr = I3C_ADDR_HUB;

	const int rstdaa_count = 2;
	int ret = 0;

	for (int i = 0; i < rstdaa_count; i++) {
		ret = i3c_brocast_ccc(&i3c_msg, I3C_CCC_RSTDAA, I3C_BROADCAST_ADDR);
		if (ret != 0) {
			printf("Error to reset daa. count = %d\n", i);
		}
	}

	ret = i3c_brocast_ccc(&i3c_msg, I3C_CCC_SETAASA, I3C_BROADCAST_ADDR);
	if (ret != 0) {
		printf("Error to set daa\n");
	}

	i3c_attach(&i3c_msg);

	// Initialize I3C HUB
	if (!rg3mxxb12_i3c_mode_only_init(&i3c_msg, LDO_VOLT)) {
		printk("failed to initialize 1ou rg3mxxb12\n");
	}

	init_vr_event_work();
	init_event_work();
	init_pmic_event_work();
	init_plat_worker(K_PRIO_PREEMPT(2)); // work queue for low priority jobs

	plat_init_pldm_sensor_table();
}

void pal_post_init()
{
	plat_mctp_init();
	pcc_init();
	kcs_init();
	pldm_load_state_effecter_table(PLAT_PLDM_MAX_STATE_EFFECTER_IDX);
	pldm_assign_gpio_effecter_id(PLAT_EFFECTER_ID_GPIO_HIGH_BYTE);
	// Create a thread to keep sending AASA
	start_setaasa();
	start_get_dimm_info_thread();
	set_sys_ready_pin(BIC_READY_R);
	reset_usb_hub();

	if (is_ac_lost()) {
		// Clear VR_CPU0 fault bit
		plat_pldm_sensor_clear_vr_fault(ADDR_VR_CPU0, I2C_BUS4, 2);
		// Clear VR_CPU1 fault bit
		plat_pldm_sensor_clear_vr_fault(ADDR_VR_CPU1, I2C_BUS4, 2);
		// Clear VR_PVDD11 fault bit
		plat_pldm_sensor_clear_vr_fault(ADDR_VR_PVDD11, I2C_BUS4, 1);
	}
}

void pal_set_sys_status()
{
	set_DC_status(PWRGD_CPU_LVC3);
	set_DC_on_delayed_status();
	set_post_status(FM_BIOS_POST_CMPLT_BIC_N);
	sync_bmc_ready_pin();
	apml_init();

	if (get_post_status()) {
		apml_recovery();
		set_tsi_threshold();
		disable_mailbox_completion_alert();
		enable_alert_signal();
		read_cpuid();
	}
}

void pal_device_init()
{
	start_monitor_pmic_error_thread();
}
