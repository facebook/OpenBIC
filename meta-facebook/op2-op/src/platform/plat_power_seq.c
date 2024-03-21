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
#include <stdio.h>
#include <stdlib.h>
#include "ipmi.h"
#include "ipmb.h"
#include "libipmi.h"
#include "libutil.h"
#include "util_worker.h"
#include "power_status.h"
#include "plat_gpio.h"
#include "plat_isr.h"
#include "plat_sensor_table.h"
#include "plat_power_seq.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(power_sequence);

K_THREAD_STACK_EXTERN(e1s_power_thread);
K_THREAD_STACK_ARRAY_DEFINE(e1s_power_threads, MAX_E1S_IDX, POWER_SEQ_CTRL_STACK_SIZE);
K_THREAD_STACK_DEFINE(cpu_pcie_reset_thread, POWER_SEQ_CTRL_STACK_SIZE);
K_MUTEX_DEFINE(cpld_e1s_prsnt_reg_mutex);
struct k_thread e1s_power_thread_handler[MAX_E1S_IDX];
k_tid_t e1s_power_tid[MAX_E1S_IDX];
struct k_thread cpu_pcie_reset_thread_handler;
k_tid_t cpu_pcie_reset_tid;

static bool is_e1s_sequence_done[MAX_E1S_IDX] = { false, false, false, false, false };
static bool is_retimer_sequence_done = false;
static uint8_t cpld_e1s_prsnt_reg = 0x1F;

e1s_power_control_gpio opa_e1s_power_control_gpio[] = {
	[0] = { .present = OPA_E1S_0_PRSNT_N,
		.p12v_efuse_enable = OPA_E1S_0_P12V_POWER_EN,
		.p12v_efuse_power_good = OPA_PWRGD_P12V_E1S_0_R,
		.p3v3_efuse_enable = OPA_E1S_0_P3V3_POWER_EN,
		.p3v3_efuse_power_good = OPA_PWRGD_P3V3_E1S_0_R,
		.clkbuf_oe_en = OPA_CLKBUF_E1S_0_OE_N,
		.cpu_pcie_reset = OPA_RST_PCIE_EXP_PERST0_N,
		.e1s_pcie_reset = OPA_PERST_E1S_0_N },
	[1] = { .present = OPA_E1S_1_PRSNT_N,
		.p12v_efuse_enable = OPA_E1S_1_P12V_POWER_EN,
		.p12v_efuse_power_good = OPA_PWRGD_P12V_E1S_1_R,
		.p3v3_efuse_enable = OPA_E1S_1_P3V3_POWER_EN,
		.p3v3_efuse_power_good = OPA_PWRGD_P3V3_E1S_1_R,
		.clkbuf_oe_en = OPA_CLKBUF_E1S_1_OE_N,
		.cpu_pcie_reset = OPA_RST_PCIE_EXP_PERST0_N,
		.e1s_pcie_reset = OPA_PERST_E1S_1_N },
	[2] = { .present = OPA_E1S_2_PRSNT_N,
		.p12v_efuse_enable = OPA_E1S_2_P12V_POWER_EN,
		.p12v_efuse_power_good = OPA_PWRGD_P12V_E1S_2_R,
		.p3v3_efuse_enable = OPA_E1S_2_P3V3_POWER_EN,
		.p3v3_efuse_power_good = OPA_PWRGD_P3V3_E1S_2_R,
		.clkbuf_oe_en = OPA_CLKBUF_E1S_2_OE_N,
		.cpu_pcie_reset = OPA_RST_PCIE_EXP_PERST0_N,
		.e1s_pcie_reset = OPA_PERST_E1S_2_N },
};

e1s_power_control_gpio opb_e1s_power_control_gpio[] = {
	[0] = { .present = OPB_E1S_0_PRSNT_N,
		.p12v_efuse_enable = OPB_P12V_E1S_0_EN_R,
		.p12v_efuse_power_good = OPB_PWRGD_P12V_E1S_0_R,
		.p3v3_efuse_enable = OPB_P3V3_E1S_0_EN_R,
		.p3v3_efuse_power_good = OPB_PWRGD_P3V3_E1S_0_R,
		.clkbuf_oe_en = OPB_CLKBUF_E1S_0_OE_N,
		.cpu_pcie_reset = OPB_RST_CPLD_PERST1_N,
		.e1s_pcie_reset = OPB_RST_E1S_0_PERST },
	[1] = { .present = OPB_E1S_1_PRSNT_N,
		.p12v_efuse_enable = OPB_P12V_E1S_1_EN_R,
		.p12v_efuse_power_good = OPB_PWRGD_P12V_E1S_1_R,
		.p3v3_efuse_enable = OPB_P3V3_E1S_1_EN_R,
		.p3v3_efuse_power_good = OPB_PWRGD_P3V3_E1S_1_R,
		.clkbuf_oe_en = OPB_CLKBUF_E1S_1_OE_N,
		.cpu_pcie_reset = OPB_RST_CPLD_PERST1_N,
		.e1s_pcie_reset = OPB_RST_E1S_1_PERST },
	[2] = { .present = OPB_E1S_2_PRSNT_N,
		.p12v_efuse_enable = OPB_P12V_E1S_2_EN_R,
		.p12v_efuse_power_good = OPB_PWRGD_P12V_E1S_2_R,
		.p3v3_efuse_enable = OPB_P3V3_E1S_2_EN_R,
		.p3v3_efuse_power_good = OPB_PWRGD_P3V3_E1S_2_R,
		.clkbuf_oe_en = OPB_CLKBUF_E1S_2_OE_N,
		.cpu_pcie_reset = OPB_RST_CPLD_PERST1_N,
		.e1s_pcie_reset = OPB_RST_E1S_2_PERST },
	[3] = { .present = OPB_E1S_3_PRSNT_N,
		.p12v_efuse_enable = OPB_P12V_E1S_3_EN_R,
		.p12v_efuse_power_good = OPB_PWRGD_P12V_E1S_3_R,
		.p3v3_efuse_enable = OPB_P3V3_E1S_3_EN_R,
		.p3v3_efuse_power_good = OPB_PWRGD_P3V3_E1S_3_R,
		.clkbuf_oe_en = OPB_CLKBUF_E1S_3_OE_N,
		.cpu_pcie_reset = OPB_RST_CPLD_PERST1_N,
		.e1s_pcie_reset = OPB_RST_E1S_3_PERST },
	[4] = { .present = OPB_E1S_4_PRSNT_N,
		.p12v_efuse_enable = OPB_P12V_E1S_4_EN_R,
		.p12v_efuse_power_good = OPB_PWRGD_P12V_E1S_4_R,
		.p3v3_efuse_enable = OPB_P3V3_E1S_4_EN_R,
		.p3v3_efuse_power_good = OPB_PWRGD_P3V3_E1S_4_R,
		.clkbuf_oe_en = OPB_CLKBUF_E1S_4_OE_N,
		.cpu_pcie_reset = OPB_RST_CPLD_PERST1_N,
		.e1s_pcie_reset = OPB_RST_E1S_4_PERST },
};

bool get_e1s_present(uint8_t index)
{
	uint8_t card_type = get_card_type();
	bool present = false;

	switch (card_type) {
	case CARD_TYPE_OPA:
		if (gpio_get(opa_e1s_power_control_gpio[index].present) == GPIO_LOW) {
			present = true;
		}
		break;
	case CARD_TYPE_OPB:
		if (gpio_get(opb_e1s_power_control_gpio[index].present) == GPIO_LOW) {
			present = true;
		}
		break;
	default:
		LOG_ERR("UNKNOWN CARD TYPE");
		break;
	}
	return present;
}

bool get_e1s_power_good(uint8_t index)
{
	uint8_t card_type = get_card_type();
	bool power_good = false;

	switch (card_type) {
	case CARD_TYPE_OPA:
		power_good = (gpio_get(opa_e1s_power_control_gpio[index].p12v_efuse_power_good) &
			      gpio_get(opa_e1s_power_control_gpio[index].p3v3_efuse_power_good));
		break;
	case CARD_TYPE_OPB:
		power_good = (gpio_get(opb_e1s_power_control_gpio[index].p12v_efuse_power_good) &
			      gpio_get(opb_e1s_power_control_gpio[index].p3v3_efuse_power_good));
		break;
	default:
		LOG_ERR("UNKNOWN CARD TYPE");
		break;
	}

	return power_good;
}

bool get_edge_power_good()
{
	uint8_t card_type = get_card_type();
	bool power_good = false;

	switch (card_type) {
	case CARD_TYPE_OPA:
		power_good = gpio_get(OPA_PWRGD_P12V_MAIN);
		break;
	case CARD_TYPE_OPB:
		power_good = gpio_get(OPB_PWRGD_P12V_MAIN);
		break;
	default:
		LOG_ERR("UNKNOWN CARD TYPE");
		break;
	}

	return power_good;
}

uint8_t get_e1s_pcie_reset_status(uint8_t index)
{
	uint8_t card_type = get_card_type();
	uint8_t pcie_reset = 0;

	switch (card_type) {
	case CARD_TYPE_OPA:
		pcie_reset = gpio_get(opa_e1s_power_control_gpio[index].e1s_pcie_reset);
		break;
	case CARD_TYPE_OPB:
		pcie_reset = gpio_get(opb_e1s_power_control_gpio[index].e1s_pcie_reset);
		break;
	default:
		LOG_ERR("UNKNOWN CARD TYPE");
		break;
	}
	return pcie_reset;
}

void init_sequence_status()
{
	uint8_t card_type = get_card_type();
	uint8_t index = 0;

	switch (card_type) {
	case CARD_TYPE_OPA:
		if (gpio_get(OPA_PERST_BIC_RTM_N) == GPIO_HIGH) {
			is_retimer_sequence_done = true;
			check_pcie_retimer_type();
			cache_pcie_retimer_version();
		}

		for (index = 0; index < OPA_MAX_E1S_IDX; ++index) {
			if (get_e1s_present(index) == true) {
				//clear bit for low present
				cpld_e1s_prsnt_reg = CLEARBIT(cpld_e1s_prsnt_reg, index);
				if (get_e1s_pcie_reset_status(index) == GPIO_HIGH) {
					is_e1s_sequence_done[index] = true;
				}
			}
		}
		break;
	case CARD_TYPE_OPB:
		for (index = 0; index < MAX_E1S_IDX; ++index) {
			if (get_e1s_present(index) == true) {
				//clear bit for low present
				cpld_e1s_prsnt_reg = CLEARBIT(cpld_e1s_prsnt_reg, index);
				if (get_e1s_pcie_reset_status(index) == GPIO_HIGH) {
					is_e1s_sequence_done[index] = true;
				}
			}
		}
		break;
	default:
		LOG_ERR("UNKNOWN CARD TYPE");
		break;
	}

	//init the e1s present status to cpld
	notify_cpld_e1s_present(MAX_E1S_IDX, GPIO_LOW);
}

bool is_all_sequence_done(uint8_t status)
{
	bool all_sequence_done = true;
	uint8_t card_type = get_card_type();
	uint8_t index = 0;

	switch (status) {
	case POWER_ON:
		if (card_type == CARD_TYPE_OPA) {
			all_sequence_done &= is_retimer_sequence_done;
			for (index = 0; index < OPA_MAX_E1S_IDX; ++index) {
				// return false if one of e1s do not power on;
				all_sequence_done &= is_e1s_sequence_done[index];
			}
		} else {
			for (index = 0; index < MAX_E1S_IDX; ++index) {
				// return false if one of e1s do not power on;
				all_sequence_done &= is_e1s_sequence_done[index];
			}
		}
		break;
	case POWER_OFF:
		if (card_type == CARD_TYPE_OPA) {
			all_sequence_done &= (!is_retimer_sequence_done);
			for (index = 0; index < OPA_MAX_E1S_IDX; ++index) {
				// return false if one of e1s do not power off;
				all_sequence_done &= (!is_e1s_sequence_done[index]);
			}
		} else {
			for (index = 0; index < MAX_E1S_IDX; ++index) {
				// return false if one of e1s do not power off;
				all_sequence_done &= (!is_e1s_sequence_done[index]);
			}
		}
		break;
	default:
		LOG_ERR("Invalid power option!");
		break;
	}

	return all_sequence_done;
}

bool is_retimer_done(void)
{
	return is_retimer_sequence_done;
}

void control_power_stage(uint8_t control_mode, uint8_t control_seq)
{
	switch (control_mode) {
	case ENABLE_POWER_MODE: // Control power on stage
	case HIGH_DISABLE_POWER_MODE:
		if (gpio_get(control_seq) != POWER_ON) {
			gpio_set(control_seq, POWER_ON);
		}
		break;
	case LOW_ENABLE_POWER_MODE:
	case DISABLE_POWER_MODE: // Control power off stage
		if (gpio_get(control_seq) != POWER_OFF) {
			gpio_set(control_seq, POWER_OFF);
		}
		break;
	default:
		LOG_ERR("Not support control mode 0x%x", control_mode);
		break;
	}
}

int check_power_stage(uint8_t check_mode, uint8_t check_seq)
{
	int ret = 0;
	switch (check_mode) {
	case ENABLE_POWER_MODE: // Control power on stage
	case HIGH_DISABLE_POWER_MODE:
		if (gpio_get(check_seq) != POWER_ON) {
			ret = -1;
		}
		break;
	case LOW_ENABLE_POWER_MODE:
	case DISABLE_POWER_MODE: // Control power off stage
		if (gpio_get(check_seq) != POWER_OFF) {
			ret = -1;
		}
		break;
	default:
		LOG_ERR("Check mode 0x%x not supported!", check_mode);
		ret = 1;
		break;
	}

	if (ret == -1) {
		LOG_ERR("Check mode 0x%x check sequence 0x%x failed", check_seq, check_seq);
		//Todo: Addsel if check power stage fail
	}

	return ret;
}

bool notify_cpld_e1s_present(uint8_t index, uint8_t present)
{
	uint8_t card_type = get_card_type();
	uint8_t card_position = get_card_position();
	ipmb_error status;
	ipmi_msg *msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
	if (msg == NULL) {
		LOG_ERR("Memory allocation failed.");
		return false;
	}

	if (k_mutex_lock(&cpld_e1s_prsnt_reg_mutex, K_MSEC(100))) {
		LOG_ERR("cpld present mutex lock failed");
		SAFE_FREE(msg);
		return false;
	}

	memset(msg, 0, sizeof(ipmi_msg));

	//set single e1s
	if (index < MAX_E1S_IDX) {
		if (present == GPIO_LOW) {
			cpld_e1s_prsnt_reg = CLEARBIT(cpld_e1s_prsnt_reg, index);
		} else {
			cpld_e1s_prsnt_reg = SETBIT(cpld_e1s_prsnt_reg, index);
		}
	}

	if (card_type == CARD_TYPE_OPA) {
		// record Unified SEL
		msg->data_len = 5;
		msg->InF_source = SELF;
		msg->InF_target = HD_BIC_IPMB;
		msg->netfn = NETFN_APP_REQ;
		msg->cmd = CMD_APP_MASTER_WRITE_READ;
		msg->data[0] = 0x01; // (bus 0 << 1) + 1
		msg->data[1] = 0x42; // 8 bits cpld address
		msg->data[2] = 0x00; // read bytes
		if (card_position == CARD_POSITION_1OU) {
			msg->data[3] = 0x80; // cpld offset
		} else {
			msg->data[3] = 0x82; // cpld offset
		}
		msg->data[4] = cpld_e1s_prsnt_reg;
	} else {
		msg->data_len = 11;
		msg->InF_source = SELF;
		if (card_position == CARD_POSITION_2OU) {
			msg->InF_target = EXP1_IPMB;
			msg->data[9] = 0x81; // cpld offset
		} else {
			msg->InF_target = EXP3_IPMB;
			msg->data[9] = 0x83; // cpld offset
		}
		msg->netfn = NETFN_OEM_1S_REQ;
		msg->cmd = CMD_OEM_1S_MSG_OUT;
		msg->data[0] = IANA_ID & 0xFF;
		msg->data[1] = (IANA_ID >> 8) & 0xFF;
		msg->data[2] = (IANA_ID >> 16) & 0xFF;
		msg->data[3] = HD_BIC_IPMB;
		msg->data[4] = NETFN_APP_REQ << 2;
		msg->data[5] = CMD_APP_MASTER_WRITE_READ;
		msg->data[6] = 0x01; // (bus 0 << 1) + 1
		msg->data[7] = 0x42; // 8 bits cpld address
		msg->data[8] = 0x00; // read bytes
		msg->data[10] = cpld_e1s_prsnt_reg;
	}

	status = ipmb_read(msg, IPMB_inf_index_map[msg->InF_target]);
	if (status != IPMB_ERROR_SUCCESS) {
		LOG_ERR("Failed to write sb cpld, ret %d", status);
		SAFE_FREE(msg);
		return false;
	}

	SAFE_FREE(msg);

	if (k_mutex_unlock(&cpld_e1s_prsnt_reg_mutex)) {
		LOG_ERR("unlock cpld e1s prsnt reg mutex fail\n");
		return false;
	}
	return true;
}

bool e1s_power_on_handler(uint8_t initial_stage, e1s_power_control_gpio *e1s_gpio,
			  uint8_t device_index)
{
	CHECK_NULL_ARG_WITH_RETURN(e1s_gpio, false);

	int check_power_ret = -1;
	bool enable_power_on_handler = true;
	uint8_t control_stage = initial_stage;

	uint8_t card_type = get_card_type();
	if (card_type == CARD_TYPE_UNKNOWN) {
		LOG_ERR("UNKNOWN CARD TYPE");
		return false;
	}

	while (enable_power_on_handler == true) {
		switch (control_stage) {
		case E1S_POWER_ON_STAGE0:
			//skip this control stage to check the device present status below.
			break;
		case E1S_POWER_ON_STAGE1:
			control_power_stage(ENABLE_POWER_MODE, e1s_gpio->p12v_efuse_enable);
			control_power_stage(ENABLE_POWER_MODE, e1s_gpio->p3v3_efuse_enable);
			break;
		case E1S_POWER_ON_STAGE2:
			control_power_stage(LOW_ENABLE_POWER_MODE, e1s_gpio->clkbuf_oe_en);
			break;
		case E1S_POWER_ON_STAGE3:
			control_power_stage(ENABLE_POWER_MODE, e1s_gpio->e1s_pcie_reset);
			break;
		default:
			LOG_ERR("Stage 0x%x not supported", initial_stage);
			enable_power_on_handler = false;
			break;
		}
		k_msleep(CHKPWR_DELAY_MSEC);

		switch (control_stage) { // Check VR power machine
		case E1S_POWER_ON_STAGE0:
			if (check_power_stage(LOW_ENABLE_POWER_MODE, e1s_gpio->present) != 0) {
				LOG_ERR("e1s %d is not present!", device_index);
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = E1S_POWER_ON_STAGE1;
			break;
		case E1S_POWER_ON_STAGE1:
			if (check_power_stage(ENABLE_POWER_MODE, e1s_gpio->p12v_efuse_power_good) !=
			    0) {
				LOG_ERR("els %d p12v_efuse_power_good is not power good!",
					device_index);
				check_power_ret = -1;
				break;
			}
			if (check_power_stage(ENABLE_POWER_MODE, e1s_gpio->p3v3_efuse_power_good) !=
			    0) {
				LOG_ERR("els %d p3v3_efuse_power_good is not power good!",
					device_index);
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = E1S_POWER_ON_STAGE2;
			break;
		case E1S_POWER_ON_STAGE2:
			if (check_power_stage(LOW_ENABLE_POWER_MODE, e1s_gpio->clkbuf_oe_en) != 0) {
				LOG_ERR("els %d clkbuf_oe_en is not enabled!", device_index);
				check_power_ret = -1;
				break;
			}

			if (gpio_get(e1s_gpio->cpu_pcie_reset) != GPIO_HIGH) {
				LOG_INF("els %d power on stop because CPU PCIE RESET is not enable.",
					device_index);
				check_power_ret = 1;
				enable_power_on_handler = false;
			} else {
				check_power_ret = 0;
				control_stage = E1S_POWER_ON_STAGE3;
			}

			break;
		case E1S_POWER_ON_STAGE3:
			if (check_power_stage(ENABLE_POWER_MODE, e1s_gpio->e1s_pcie_reset) != 0) {
				LOG_ERR("els %d pcie_reset is not enabled!", device_index);
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			enable_power_on_handler = false;
			break;
		default:
			LOG_ERR("Not support stage 0x%x", initial_stage);
			enable_power_on_handler = false;
			break;
		}

		if (check_power_ret < 0) {
			enable_power_on_handler = false;
			e1s_power_off_handler(E1S_POWER_OFF_STAGE0, e1s_gpio, device_index);
		}
	}

	switch (check_power_ret) {
	case E1S_POWER_SUCCESS:
		is_e1s_sequence_done[device_index] = true;
		return true;
	case E1S_PERST_SUCCESS:
		is_e1s_sequence_done[device_index] = false;
		return true;
	default:
		is_e1s_sequence_done[device_index] = false;
		return false;
	}
}

bool e1s_power_off_handler(uint8_t initial_stage, e1s_power_control_gpio *e1s_gpio,
			   uint8_t device_index)
{
	CHECK_NULL_ARG_WITH_RETURN(e1s_gpio, false);

	bool enable_power_off_handler = true;
	int check_power_ret = -1;
	uint8_t control_stage = initial_stage;

	uint8_t card_type = get_card_type();
	if (card_type == CARD_TYPE_UNKNOWN) {
		LOG_ERR("UNKNOWN CARD TYPE");
		return false;
	}

	is_e1s_sequence_done[device_index] = false;

	while (enable_power_off_handler == true) {
		switch (control_stage) { // Disable VR power machine
		case E1S_POWER_OFF_STAGE0:
			control_power_stage(DISABLE_POWER_MODE, e1s_gpio->e1s_pcie_reset);
			break;
		case E1S_POWER_OFF_STAGE1:
			control_power_stage(HIGH_DISABLE_POWER_MODE, e1s_gpio->clkbuf_oe_en);
			break;
		case E1S_POWER_OFF_STAGE2:
			control_power_stage(DISABLE_POWER_MODE, e1s_gpio->p12v_efuse_enable);
			control_power_stage(DISABLE_POWER_MODE, e1s_gpio->p3v3_efuse_enable);
			break;
		default:
			LOG_ERR("Stage 0x%x not supported", initial_stage);
			enable_power_off_handler = false;
			break;
		}
		k_msleep(CHKPWR_DELAY_MSEC);

		switch (control_stage) { // Check VR power machine
		case E1S_POWER_OFF_STAGE0:
			if (check_power_stage(DISABLE_POWER_MODE, e1s_gpio->e1s_pcie_reset) != 0) {
				LOG_ERR("E1S %d pcie_reset is not disabled!", device_index);
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = E1S_POWER_OFF_STAGE1;
			break;
		case E1S_POWER_OFF_STAGE1:
			if (check_power_stage(HIGH_DISABLE_POWER_MODE, e1s_gpio->clkbuf_oe_en) !=
			    0) {
				LOG_ERR("E1S %d clkbuf_oe_en is not disabled!", device_index);
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = E1S_POWER_OFF_STAGE2;
			break;
		case E1S_POWER_OFF_STAGE2:
			if (check_power_stage(DISABLE_POWER_MODE,
					      e1s_gpio->p12v_efuse_power_good) != 0) {
				LOG_ERR("E1S %d p12v_efuse_enable is not disabled!", device_index);
				check_power_ret = -1;
				break;
			}
			if (check_power_stage(DISABLE_POWER_MODE,
					      e1s_gpio->p3v3_efuse_power_good) != 0) {
				LOG_ERR("E1S %d p3v3_efuse_enable is not disabled!", device_index);
				check_power_ret = -1;
				break;
			}
			enable_power_off_handler = false;
			break;
		default:
			LOG_ERR("Stage 0x%x not supported", initial_stage);
			enable_power_off_handler = false;
			break;
		}

		if (check_power_ret != 0) {
			enable_power_off_handler = false;
		}
	}
	if (check_power_ret == 0) {
		return true;
	} else {
		return false;
	}
}

void control_e1s_power_on_sequence(void *pvParameters, void *initial_stage, void *arvg1)
{
	CHECK_NULL_ARG(pvParameters);
	CHECK_NULL_ARG(initial_stage);

	int dev_index = ((int)pvParameters) - 1;
	int stage = (int)initial_stage - 1;
	bool is_power_on = false;
	uint8_t card_type = get_card_type();

	switch (card_type) {
	case CARD_TYPE_OPA:
		is_power_on = e1s_power_on_handler(
			(uint8_t)stage, &opa_e1s_power_control_gpio[dev_index], dev_index);
		break;
	case CARD_TYPE_OPB:
		is_power_on = e1s_power_on_handler(
			(uint8_t)stage, &opb_e1s_power_control_gpio[dev_index], dev_index);
		break;
	default:
		LOG_ERR("UNKNOWN card type power on e1s %d failed.", dev_index);
		break;
	}

	if (is_power_on == true) {
		LOG_INF("E1S %d Power on success", dev_index);
	} else {
		LOG_ERR("E1S %d Power on fail", dev_index);
	}
}

void control_e1s_power_off_sequence(void *pvParameters, void *arvg0, void *arvg1)
{
	CHECK_NULL_ARG(pvParameters);

	int dev_index = ((int)pvParameters) - 1;
	bool is_power_off = false;
	uint8_t card_type = get_card_type();

	switch (card_type) {
	case CARD_TYPE_OPA:
		is_power_off = e1s_power_off_handler(
			E1S_POWER_OFF_STAGE0, &opa_e1s_power_control_gpio[dev_index], dev_index);
		break;
	case CARD_TYPE_OPB:
		is_power_off = e1s_power_off_handler(
			E1S_POWER_OFF_STAGE0, &opb_e1s_power_control_gpio[dev_index], dev_index);
		break;
	default:
		LOG_ERR("UNKNOWN card type power off e1s %d failed.", dev_index);
		break;
	}

	if (is_power_off == true) {
		LOG_INF("E1S %d Power off success", dev_index);
	} else {
		LOG_ERR("E1S %d Power off fail", dev_index);
	}
}

void abort_e1s_power_thread(uint8_t index)
{
	if (e1s_power_tid[index] != NULL &&
	    strcmp(k_thread_state_str(e1s_power_tid[index]), "dead") != 0) {
		k_thread_abort(e1s_power_tid[index]);
	}
}

void e1s_power_on_thread(uint8_t index, uint8_t initial_stage)
{
	// Avoid re-create thread by checking thread status and thread id
	if (e1s_power_tid[index] != NULL &&
	    ((strcmp(k_thread_state_str(e1s_power_tid[index]), "dead") != 0) &&
	     (strcmp(k_thread_state_str(e1s_power_tid[index]), "unknown") != 0))) {
		LOG_ERR("e1s power on thread exists status %s",
			k_thread_state_str(e1s_power_tid[index]));
		return;
	}

	if (get_e1s_present(index) == true) {
		// index add 1 to prevent become null pointer
		int dev_index = index + 1;
		int stage = initial_stage + 1;
		e1s_power_tid[index] =
			k_thread_create(&e1s_power_thread_handler[index], e1s_power_threads[index],
					K_THREAD_STACK_SIZEOF(e1s_power_threads[index]),
					control_e1s_power_on_sequence, (void *)dev_index,
					(void *)stage, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0,
					K_NO_WAIT);
		char thread_name[MAX_WORK_NAME_LEN];
		sprintf(thread_name, "e1s%d_power_on_sequence_thread", index);
		k_thread_name_set(&e1s_power_thread_handler[index], thread_name);
	} else {
		LOG_INF("E1S %d not present can not power on", index);
	}
}

void e1s_power_off_thread(uint8_t index)
{
	// Avoid re-create thread by checking thread status and thread id
	if (e1s_power_tid[index] != NULL &&
	    ((strcmp(k_thread_state_str(e1s_power_tid[index]), "dead") != 0) &&
	     (strcmp(k_thread_state_str(e1s_power_tid[index]), "unknown") != 0))) {
		LOG_ERR("e1s power off thread exists status %s",
			k_thread_state_str(e1s_power_tid[index]));
		return;
	}

	// index add 1 to prevent become null pointer
	int dev_index = index + 1;
	e1s_power_tid[index] =
		k_thread_create(&e1s_power_thread_handler[index], e1s_power_threads[index],
				K_THREAD_STACK_SIZEOF(e1s_power_threads[index]),
				control_e1s_power_off_sequence, (void *)dev_index, NULL, NULL,
				CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	char thread_name[MAX_WORK_NAME_LEN];
	sprintf(thread_name, "e1s%d_power_off_sequence_thread", index);
	k_thread_name_set(&e1s_power_thread_handler[index], thread_name);
}

void control_cpu_perst_low(void *arvg0, void *arvg1, void *arvg2)
{
	ARG_UNUSED(arvg0);
	ARG_UNUSED(arvg1);
	ARG_UNUSED(arvg2);

	uint8_t card_type = get_card_type();
	uint8_t index;

	switch (card_type) {
	case CARD_TYPE_OPA:
		for (index = 0; index < OPA_MAX_E1S_IDX; ++index) {
			if (get_e1s_present(index) == true) {
				is_e1s_sequence_done[index] = false;
				control_power_stage(
					DISABLE_POWER_MODE,
					opa_e1s_power_control_gpio[index].e1s_pcie_reset);
			}
		}
		is_retimer_sequence_done = false;
		control_power_stage(DISABLE_POWER_MODE, OPA_PERST_BIC_RTM_N);
		control_power_stage(DISABLE_POWER_MODE, OPA_RESET_BIC_RTM_N);
		break;
	case CARD_TYPE_OPB:
		for (index = 0; index < MAX_E1S_IDX; ++index) {
			if (get_e1s_present(index) == true) {
				is_e1s_sequence_done[index] = false;
				control_power_stage(
					DISABLE_POWER_MODE,
					opb_e1s_power_control_gpio[index].e1s_pcie_reset);
			}
		}
		break;
	default:
		LOG_ERR("UNKNOWN card type control cpu reset low failed.");
		break;
	}
}

void abort_cpu_perst_low_thread()
{
	if (cpu_pcie_reset_tid != NULL &&
	    strcmp(k_thread_state_str(cpu_pcie_reset_tid), "dead") != 0) {
		k_thread_abort(cpu_pcie_reset_tid);
	}
}

void cpu_perst_low_thread()
{
	// Avoid re-create thread by checking thread status and thread id
	if (cpu_pcie_reset_tid != NULL &&
	    ((strcmp(k_thread_state_str(cpu_pcie_reset_tid), "dead") != 0) &&
	     (strcmp(k_thread_state_str(cpu_pcie_reset_tid), "unknown") != 0))) {
		LOG_ERR("cpu_pcie_reset_tid exists status %s",
			k_thread_state_str(cpu_pcie_reset_tid));
		return;
	}

	cpu_pcie_reset_tid =
		k_thread_create(&cpu_pcie_reset_thread_handler, cpu_pcie_reset_thread,
				K_THREAD_STACK_SIZEOF(cpu_pcie_reset_thread), control_cpu_perst_low,
				NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&cpu_pcie_reset_thread_handler, "cpu_pcie_reset_thread");
}

bool power_on_handler(uint8_t initial_stage)
{
	int check_power_ret = -1;
	bool enable_power_on_handler = true;
	uint8_t control_stage = initial_stage;
	uint8_t index = 0;
	uint8_t board_revision = get_board_revision();
	uint8_t card_type = get_card_type();
	if (card_type == CARD_TYPE_UNKNOWN) {
		LOG_ERR("UNKNOWN CARD TYPE");
		return false;
	}

	while (enable_power_on_handler == true) {
		switch (control_stage) { // Enable VR power machine
		case BOARD_POWER_ON_STAGE0:
			if (board_revision != EVT_STAGE && card_type == CARD_TYPE_OPB) {
				control_power_stage(ENABLE_POWER_MODE, OPB_BIC_MAIN_PWR_EN_R);
			}
			break;
		case BOARD_POWER_ON_STAGE1:
			control_power_stage(ENABLE_POWER_MODE, OPA_EN_P0V9_VR);
			break;
		case BOARD_POWER_ON_STAGE2:
			control_power_stage(ENABLE_POWER_MODE, OPA_PWRGD_EXP_PWR);
			break;
		case RETIMER_POWER_ON_STAGE0:
			control_power_stage(LOW_ENABLE_POWER_MODE, OPA_CLKBUF_RTM_OE_N);
			break;
		case RETIMER_POWER_ON_STAGE1:
			if (card_type == CARD_TYPE_OPA) {
				control_power_stage(ENABLE_POWER_MODE, OPA_RESET_BIC_RTM_N);
				control_power_stage(ENABLE_POWER_MODE, OPA_PERST_BIC_RTM_N);
			}
			LOG_INF("wait for retimer boot up");
			//Wait for retimer boot up
			k_msleep(RETIMER_DELAY_MSEC);
			set_DC_on_delayed_status();
			break;
		case E1S_POWER_ON_STAGE0:
			for (index = 0;
			     index < ((card_type == CARD_TYPE_OPA) ? OPA_MAX_E1S_IDX : MAX_E1S_IDX);
			     ++index) {
				if (get_e1s_present(index) == true) {
					abort_e1s_power_thread(index);
					if (initial_stage == RETIMER_POWER_ON_STAGE1) {
						e1s_power_on_thread(index, E1S_POWER_ON_STAGE3);
					} else {
						e1s_power_on_thread(index, E1S_POWER_ON_STAGE0);
					}
				}
			}
			break;
		default:
			LOG_ERR("Stage 0x%x not supported", initial_stage);
			enable_power_on_handler = false;
			break;
		}
		k_msleep(CHKPWR_DELAY_MSEC);

		switch (control_stage) { // Check VR power machine
		case BOARD_POWER_ON_STAGE0:
			if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_01) != 0) {
				LOG_ERR("FM_EXP_MAIN_PWR_EN is not enabled!");
				check_power_ret = -1;
				break;
			}
			if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_02) != 0) {
				if (board_revision != EVT_STAGE && card_type == CARD_TYPE_OPB) {
					control_power_stage(DISABLE_POWER_MODE,
							    OPB_BIC_MAIN_PWR_EN_R);
				}
				LOG_ERR("PWRGD_P12V_MAIN is not enabled!");
				check_power_ret = -1;
				break;
			}
			if (card_type == CARD_TYPE_OPA) {
				if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_03) != 0) {
					LOG_ERR("OPA_PWRGD_P1V8_VR is not enabled!");
					check_power_ret = -1;
					break;
				}
			}
			check_power_ret = 0;
			if (card_type == CARD_TYPE_OPA) {
				control_stage = BOARD_POWER_ON_STAGE1;
			} else {
				control_stage = E1S_POWER_ON_STAGE0;
			}
			break;
		case BOARD_POWER_ON_STAGE1:
			if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_04) != 0) {
				LOG_ERR("OPA_PWRGD_P0V9_VR is not enabled!");
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = BOARD_POWER_ON_STAGE2;
			break;
		case BOARD_POWER_ON_STAGE2:
			if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_05) != 0) {
				LOG_ERR("OPA_PWRGD_EXP_PWR is not enabled!");
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = RETIMER_POWER_ON_STAGE0;
			break;
		case RETIMER_POWER_ON_STAGE0:
			if (check_power_stage(LOW_ENABLE_POWER_MODE, CHECK_POWER_SEQ_06) != 0) {
				LOG_ERR("OPA_CLKBUF_RTM_OE_N is not enabled!");
				check_power_ret = -1;
				break;
			}

			if (gpio_get(OPA_RST_PCIE_EXP_PERST0_N) != GPIO_HIGH) {
				check_power_ret = 0;
				control_stage = E1S_POWER_ON_STAGE0;
				break;
			} else {
				check_power_ret = 0;
				control_stage = RETIMER_POWER_ON_STAGE1;
			}
			break;
		case RETIMER_POWER_ON_STAGE1:
			if (card_type == CARD_TYPE_OPA) {
				if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_07) != 0) {
					LOG_ERR("OPA_RESET_BIC_RTM_N is not enabled!");
					check_power_ret = -1;
					break;
				}

				if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_08) != 0) {
					LOG_ERR("OPA_PERST_BIC_RTM_N is not enabled!");
					check_power_ret = -1;
					break;
				}
				is_retimer_sequence_done = true;
				check_pcie_retimer_type();
				cache_pcie_retimer_version();
			}
			check_power_ret = 0;
			control_stage = E1S_POWER_ON_STAGE0;
			break;
		case E1S_POWER_ON_STAGE0:
			check_power_ret = 0;
			enable_power_on_handler = false;
			break;
		default:
			LOG_ERR("Not support stage 0x%x", initial_stage);
			enable_power_on_handler = false;
			break;
		}

		if (check_power_ret != 0) {
			power_off_handler(BOARD_POWER_OFF_STAGE0);
			enable_power_on_handler = false;
		}
	}
	if (check_power_ret == 0) {
		return true;
	} else {
		return false;
	}
}

bool power_off_handler(uint8_t initial_stage)
{
	bool enable_power_off_handler = true;
	bool e1s_is_power_off[MAX_E1S_IDX] = { false, false, false, false, false };
	int check_power_ret = -1;
	int all_e1s_power_check = 0;
	uint8_t control_stage = initial_stage;
	uint8_t index = 0;
	uint8_t board_revision = get_board_revision();
	uint8_t card_type = get_card_type();
	if (card_type == CARD_TYPE_UNKNOWN) {
		LOG_ERR("UNKNOWN CARD TYPE");
		return false;
	}

	while (enable_power_off_handler == true) {
		switch (control_stage) { // Disable VR power machine
		case E1S_POWER_OFF_STAGE0:
			if (card_type == CARD_TYPE_OPA) {
				for (index = 0; index < OPA_MAX_E1S_IDX; ++index) {
					e1s_is_power_off[index] = e1s_power_off_handler(
						E1S_POWER_OFF_STAGE0,
						&opa_e1s_power_control_gpio[index], index);
				}
			} else {
				for (index = 0; index < MAX_E1S_IDX; ++index) {
					e1s_is_power_off[index] = e1s_power_off_handler(
						E1S_POWER_OFF_STAGE0,
						&opb_e1s_power_control_gpio[index], index);
				}
			}
			break;
		case RETIMER_POWER_OFF_STAGE0:
			is_retimer_sequence_done = false;
			control_power_stage(DISABLE_POWER_MODE, OPA_PERST_BIC_RTM_N);
			break;
		case RETIMER_POWER_OFF_STAGE1:
			control_power_stage(DISABLE_POWER_MODE, OPA_RESET_BIC_RTM_N);
			break;
		case RETIMER_POWER_OFF_STAGE2:
			control_power_stage(HIGH_DISABLE_POWER_MODE, OPA_CLKBUF_RTM_OE_N);
			break;
		case BOARD_POWER_OFF_STAGE0:
			control_power_stage(DISABLE_POWER_MODE, OPA_PWRGD_EXP_PWR);
			break;
		case BOARD_POWER_OFF_STAGE1:
			control_power_stage(DISABLE_POWER_MODE, OPA_EN_P0V9_VR);
			break;
		case BOARD_POWER_OFF_STAGE2:
			if (board_revision != EVT_STAGE && card_type == CARD_TYPE_OPB) {
				control_power_stage(DISABLE_POWER_MODE, OPB_BIC_MAIN_PWR_EN_R);
			}
			break;
		default:
			LOG_ERR("Stage 0x%x not supported", initial_stage);
			enable_power_off_handler = false;
			break;
		}
		k_msleep(CHKPWR_DELAY_MSEC);

		switch (control_stage) {
		case E1S_POWER_OFF_STAGE0:
			for (index = 0;
			     index < ((card_type == CARD_TYPE_OPA) ? OPA_MAX_E1S_IDX : MAX_E1S_IDX);
			     ++index) {
				if (e1s_is_power_off[index] == false) {
					LOG_ERR("e1s %d power off fall!", index);
					all_e1s_power_check = -1;
					break;
				}
			}

			if (all_e1s_power_check != 0) {
				check_power_ret = -1;
			} else {
				check_power_ret = 0;
				if (card_type == CARD_TYPE_OPA) {
					control_stage = RETIMER_POWER_OFF_STAGE0;
				} else {
					enable_power_off_handler = false;
				}
			}
			break;
		case RETIMER_POWER_OFF_STAGE0:
			if (check_power_stage(DISABLE_POWER_MODE, CHECK_POWER_SEQ_08) != 0) {
				LOG_ERR("OPA_PERST_BIC_RTM_N is not disabled!");
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = RETIMER_POWER_OFF_STAGE1;
			break;
		case RETIMER_POWER_OFF_STAGE1:
			if (check_power_stage(DISABLE_POWER_MODE, CHECK_POWER_SEQ_07) != 0) {
				LOG_ERR("OPA_RESET_BIC_RTM_N is not disabled!");
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = RETIMER_POWER_OFF_STAGE2;
			break;
		case RETIMER_POWER_OFF_STAGE2:
			if (check_power_stage(HIGH_DISABLE_POWER_MODE, CHECK_POWER_SEQ_06) != 0) {
				LOG_ERR("OPA_CLKBUF_RTM_OE_N is not disabled!");
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = BOARD_POWER_OFF_STAGE0;
			break;
		case BOARD_POWER_OFF_STAGE0:
			if (check_power_stage(DISABLE_POWER_MODE, CHECK_POWER_SEQ_05) != 0) {
				LOG_ERR("OPA_PWRGD_EXP_PWR is not disabled!");
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = BOARD_POWER_OFF_STAGE1;
			break;
		case BOARD_POWER_OFF_STAGE1:
			if (check_power_stage(DISABLE_POWER_MODE, CHECK_POWER_SEQ_04) != 0) {
				LOG_ERR("OPA_PWRGD_P0V9_VR is not disabled!");
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = BOARD_POWER_OFF_STAGE2;
			break;
		case BOARD_POWER_OFF_STAGE2:
			if (board_revision != EVT_STAGE && card_type == CARD_TYPE_OPB) {
				if (check_power_stage(DISABLE_POWER_MODE, CHECK_POWER_SEQ_02) !=
				    0) {
					LOG_ERR("OPB_BIC_MAIN_PWR_EN_R is not disabled!");
					check_power_ret = -1;
					break;
				}
			}
			enable_power_off_handler = false;
			break;
		default:
			LOG_ERR("Stage 0x%x not supported", initial_stage);
			enable_power_off_handler = false;
			break;
		}

		if (check_power_ret != 0) {
			enable_power_off_handler = false;
		}
	}
	if (check_power_ret == 0) {
		return true;
	} else {
		return false;
	}
}

void control_power_on_sequence(void *initial_stage, void *arvg0, void *arvg1)
{
	CHECK_NULL_ARG(initial_stage);
	bool is_power_on = false;
	int stage = (int)initial_stage - 1;
	is_power_on = power_on_handler((uint8_t)stage);

	if (is_power_on == true) {
		LOG_INF("Power on success");
	} else {
		LOG_ERR("Power on fail");
	}
}

void control_power_off_sequence()
{
	bool is_power_off = false;

	set_DC_on_delayed_status_with_value(false);
	is_power_off = power_off_handler(E1S_POWER_OFF_STAGE0);

	if (is_power_off == true) {
		LOG_INF("Power off success");
	} else {
		LOG_ERR("Power off fail");
	}
}
