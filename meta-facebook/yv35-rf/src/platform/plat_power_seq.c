#include <zephyr.h>
#include <stdio.h>
#include "ipmi.h"
#include "ipmb.h"
#include "libipmi.h"
#include "plat_gpio.h"
#include "plat_isr.h"
#include "plat_class.h"
#include "plat_sensor_table.h"
#include "plat_power_seq.h"

static uint8_t power_on_seq = DEFAULT_POWER_ON_SEQ;
static uint8_t power_off_seq = DEFAULT_POWER_OFF_SEQ;

void set_MB_DC_status(uint8_t gpio_num)
{
	static bool is_MB_DC_ON = false;
	is_MB_DC_ON = (gpio_get(gpio_num) == GPIO_HIGH) ? true : false;
	printf("[%s] gpio number(%d) status(%d)\n", __func__, gpio_num, is_MB_DC_ON);
}

void set_power_on_seq(uint8_t seq_num)
{
	power_on_seq = seq_num;
}

void set_power_off_seq(uint8_t seq_num)
{
	power_off_seq = seq_num;
}

void control_power_on_sequence()
{
	bool is_power_on = false;
	is_power_on = power_on_handler(ASIC_POWER_ON_STAGE);

	if (is_power_on == true) {
		gpio_set(PWRGD_CARD_PWROK, POWER_ON);
		printf("[%s] power on success\n", __func__);
	} else {
		printf("[%s] power on fail\n", __func__);
	}
}

void control_power_off_sequence()
{
	bool is_power_off = false;
	// Inform CraterLake expansion board power off
	gpio_set(PWRGD_CARD_PWROK, POWER_OFF);
	gpio_set(ASIC_DEV_RST_N, POWER_OFF);

	// Disable i2c synchronized during error recovery/ASIC i2c pin
	gpio_set(I2CS_SRSTB_GPIO, HIGH_INACTIVE);
	gpio_set(LSFT_SMB_DIMM_EN, HIGH_INACTIVE);

	is_power_off = power_off_handler(DIMM_POWER_OFF_STAGE1);

	if (is_power_off == true) {
		printf("[%s] power off success\n", __func__);
	} else {
		printf("[%s] power off fail\n", __func__);
	}
}

void control_power_stage(uint8_t control_mode, uint8_t control_seq)
{
	switch (control_mode) {
	case ENABLE_POWER_MODE: // Control power on stage
		if (gpio_get(control_seq) != POWER_ON) {
			gpio_set(control_seq, POWER_ON);
		}
		break;
	case DISABLE_POWER_MODE: // Control power off stage
		if (gpio_get(control_seq) != POWER_OFF) {
			gpio_set(control_seq, POWER_OFF);
		}
		break;
	default:
		printf("[%s] not support control mode 0x%x\n", __func__, control_mode);
		break;
	}
}

int check_power_stage(uint8_t check_mode, uint8_t check_seq)
{
	int ret = 0;
	switch (check_mode) {
	case ENABLE_POWER_MODE: // Check power on stage
		power_on_seq += 1;
		if (gpio_get(check_seq) != POWER_ON) {
			ret = -1;
		}
		break;
	case DISABLE_POWER_MODE: // Check power off stage
		power_off_seq -= 1;
		if (gpio_get(check_seq) != POWER_OFF) {
			ret = -1;
		}
		break;
	default:
		printf("[%s] not support check mode 0x%x\n", __func__, check_mode);
		ret = 1;
		break;
	}

	if (ret == -1) { // Addsel if check power stage fail
		bool is_addsel_success = false;
		common_addsel_msg_t sel_msg;
		memset(&sel_msg, 0, sizeof(common_addsel_msg_t));

		sel_msg.InF_target = CL_BIC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_OEM_C3;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		sel_msg.sensor_number = SENSOR_NUM_POWER_ERROR;
		sel_msg.event_data1 =
			((check_mode == ENABLE_POWER_MODE) ? IPMI_OEM_EVENT_OFFSET_EXP_PWRON_FAIL :
								   IPMI_OEM_EVENT_OFFSET_EXP_PWROFF_FAIL);
		sel_msg.event_data2 =
			((check_mode == ENABLE_POWER_MODE) ? power_on_seq : power_off_seq);
		sel_msg.event_data3 = get_board_id();

		is_addsel_success = common_add_sel_evt_record(&sel_msg);
		if (is_addsel_success == false) {
			printf("[%s] control power event addsel fail  mode: 0x%x  seq: 0x%x\n",
			       __func__, check_mode, check_seq);
		}
	}

	return ret;
}

bool power_on_handler(uint8_t initial_stage)
{
	int check_power_ret = -1;
	bool enable_power_on_handler = true;
	uint8_t control_stage = initial_stage;
	while (enable_power_on_handler == true) {
		switch (control_stage) { // Enable VR power machine
		case ASIC_POWER_ON_STAGE:
			control_power_stage(ENABLE_POWER_MODE, CONTROL_POWER_SEQ_01);
			control_power_stage(ENABLE_POWER_MODE, CONTROL_POWER_SEQ_02);
			control_power_stage(ENABLE_POWER_MODE, CONTROL_POWER_SEQ_03);
			control_power_stage(ENABLE_POWER_MODE, CONTROL_POWER_SEQ_04);
			break;
		case DIMM_POWER_ON_STAGE1:
			control_power_stage(ENABLE_POWER_MODE, CONTROL_POWER_SEQ_05);
			control_power_stage(ENABLE_POWER_MODE, CONTROL_POWER_SEQ_06);
			break;
		case DIMM_POWER_ON_STAGE2:
			control_power_stage(ENABLE_POWER_MODE, CONTROL_POWER_SEQ_07);
			control_power_stage(ENABLE_POWER_MODE, CONTROL_POWER_SEQ_08);
			break;
		case DIMM_POWER_ON_STAGE3:
			control_power_stage(ENABLE_POWER_MODE, CONTROL_POWER_SEQ_09);
			control_power_stage(ENABLE_POWER_MODE, CONTROL_POWER_SEQ_10);
			break;
		case BOARD_POWER_ON_STAGE:
			control_power_stage(ENABLE_POWER_MODE, CLK_100M_OSC_EN);
			break;
		default:
			printf("[%s] not support stage 0x%x\n", __func__, initial_stage);
			enable_power_on_handler = false;
			break;
		}
		k_msleep(CHKPWR_DELAY_MSEC);

		switch (control_stage) { // Check VR power machine
		case ASIC_POWER_ON_STAGE:
			if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_01) != 0) {
				check_power_ret = -1;
				break;
			}
			if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_02) != 0) {
				check_power_ret = -1;
				break;
			}
			if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_03) != 0) {
				check_power_ret = -1;
				break;
			}
			if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_04) != 0) {
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = DIMM_POWER_ON_STAGE1;
			break;
		case DIMM_POWER_ON_STAGE1:
			if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_05) != 0) {
				check_power_ret = -1;
				break;
			}
			if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_06) != 0) {
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = DIMM_POWER_ON_STAGE2;
			break;
		case DIMM_POWER_ON_STAGE2:
			if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_07) != 0) {
				check_power_ret = -1;
				break;
			}
			if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_08) != 0) {
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = DIMM_POWER_ON_STAGE3;
			break;
		case DIMM_POWER_ON_STAGE3:
			if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_09) != 0) {
				check_power_ret = -1;
				break;
			}
			if (check_power_stage(ENABLE_POWER_MODE, CHECK_POWER_SEQ_10) != 0) {
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = BOARD_POWER_ON_STAGE;
			break;
		case BOARD_POWER_ON_STAGE:
			control_power_stage(ENABLE_POWER_MODE, ASIC_DEV_RST_N);

			// Enable i2c synchronized during error recovery/ASIC i2c pin
			gpio_set(I2CS_SRSTB_GPIO, HIGH_ACTIVE);
			gpio_set(LSFT_SMB_DIMM_EN, HIGH_ACTIVE);

			check_power_ret = 0;
			enable_power_on_handler = false;
			break;
		default:
			printf("[%s] not support stage 0x%x\n", __func__, initial_stage);
			enable_power_on_handler = false;
			break;
		}

		if (check_power_ret != 0) {
			if (control_stage == ASIC_POWER_ON_STAGE) {
				power_off_handler(ASIC_POWER_OFF_STAGE1);
			} else {
				power_off_handler(DIMM_POWER_OFF_STAGE1);
			}
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
	int check_power_ret = -1;
	uint8_t control_stage = initial_stage;
	while (enable_power_off_handler == true) {
		switch (control_stage) { // Disable VR power machine
		case DIMM_POWER_OFF_STAGE1:
			control_power_stage(DISABLE_POWER_MODE, CONTROL_POWER_SEQ_09);
			control_power_stage(DISABLE_POWER_MODE, CONTROL_POWER_SEQ_10);
			break;
		case DIMM_POWER_OFF_STAGE2:
			control_power_stage(DISABLE_POWER_MODE, CONTROL_POWER_SEQ_07);
			control_power_stage(DISABLE_POWER_MODE, CONTROL_POWER_SEQ_08);
			break;
		case DIMM_POWER_OFF_STAGE3:
			control_power_stage(DISABLE_POWER_MODE, CONTROL_POWER_SEQ_05);
			control_power_stage(DISABLE_POWER_MODE, CONTROL_POWER_SEQ_06);
			break;
		case ASIC_POWER_OFF_STAGE1:
			control_power_stage(DISABLE_POWER_MODE, CONTROL_POWER_SEQ_04);
			break;
		case ASIC_POWER_OFF_STAGE2:
			control_power_stage(DISABLE_POWER_MODE, CONTROL_POWER_SEQ_01);
			control_power_stage(DISABLE_POWER_MODE, CONTROL_POWER_SEQ_02);
			control_power_stage(DISABLE_POWER_MODE, CONTROL_POWER_SEQ_03);
			break;
		case BOARD_POWER_OFF_STAGE:
			control_power_stage(DISABLE_POWER_MODE, CLK_100M_OSC_EN);
			break;
		default:
			printf("[%s] not support stage 0x%x\n", __func__, initial_stage);
			enable_power_off_handler = false;
			break;
		}
		k_msleep(CHKPWR_DELAY_MSEC);

		switch (control_stage) { // Check VR power machine
		case DIMM_POWER_OFF_STAGE1:
			if (check_power_stage(DISABLE_POWER_MODE, CHECK_POWER_SEQ_10) != 0) {
				check_power_ret = -1;
				break;
			}
			if (check_power_stage(DISABLE_POWER_MODE, CHECK_POWER_SEQ_09) != 0) {
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = DIMM_POWER_OFF_STAGE2;
			break;
		case DIMM_POWER_OFF_STAGE2:
			if (check_power_stage(DISABLE_POWER_MODE, CHECK_POWER_SEQ_08) != 0) {
				check_power_ret = -1;
				break;
			}
			if (check_power_stage(DISABLE_POWER_MODE, CHECK_POWER_SEQ_07) != 0) {
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = DIMM_POWER_OFF_STAGE3;
			break;
		case DIMM_POWER_OFF_STAGE3:
			if (check_power_stage(DISABLE_POWER_MODE, CHECK_POWER_SEQ_06) != 0) {
				check_power_ret = -1;
				break;
			}
			if (check_power_stage(DISABLE_POWER_MODE, CHECK_POWER_SEQ_05) != 0) {
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = ASIC_POWER_OFF_STAGE1;
			break;
		case ASIC_POWER_OFF_STAGE1:
			if (check_power_stage(DISABLE_POWER_MODE, CHECK_POWER_SEQ_04) != 0) {
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = ASIC_POWER_OFF_STAGE2;
			break;
		case ASIC_POWER_OFF_STAGE2:
			if (check_power_stage(DISABLE_POWER_MODE, CHECK_POWER_SEQ_03) != 0) {
				check_power_ret = -1;
				break;
			}
			if (check_power_stage(DISABLE_POWER_MODE, CHECK_POWER_SEQ_02) != 0) {
				check_power_ret = -1;
				break;
			}
			if (check_power_stage(DISABLE_POWER_MODE, CHECK_POWER_SEQ_01) != 0) {
				check_power_ret = -1;
				break;
			}
			check_power_ret = 0;
			control_stage = BOARD_POWER_OFF_STAGE;
			break;
		case BOARD_POWER_OFF_STAGE:
			enable_power_off_handler = false;
			break;
		default:
			printf("[%s] not support stage 0x%x\n", __func__, initial_stage);
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
