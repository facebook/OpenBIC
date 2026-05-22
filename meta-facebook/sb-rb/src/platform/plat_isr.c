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
#include <logging/log.h>

#include "plat_gpio.h"
#include "plat_cpld.h"
#include "plat_log.h"
#include "plat_event.h"
#include "plat_ioexp.h"
#include "plat_hook.h"
#include "plat_iris_smbus.h"
#include "plat_util.h"
#include "plat_i2c.h"
#include "shell_iris_power.h"
#include "plat_class.h"
#include "plat_vr_test_mode.h"
#include "plat_power_capping.h"
#include "plat_hwmon.h"
#include "shell_plat_power_sequence.h"
#include "plat_adc.h"
#include "plat_user_setting.h"

LOG_MODULE_REGISTER(plat_isr);

#define CLK_APLL_CHECK_INTERVAL K_SECONDS(10)

static void clk_apll_check_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(clk_apll_check_work, clk_apll_check_work_handler);
static void start_clk_apll_check_work(void);

void pwr_sequence_event_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(pwr_sequence_event_work_timer, pwr_sequence_event_timer_handler, NULL);
void pwr_sequence_event(struct k_work *work);
K_WORK_DEFINE(pwr_sequence_event_work, pwr_sequence_event);

void check_read_100MHz_clock_status()
{
	uint8_t lock_status = 0;
	//write to two-byte mode
	uint8_t write_data = 0x05;
	if (!plat_i2c_write(I2C_BUS3, 0x9, 0x26, &write_data, 1)) {
		LOG_ERR("Failed to write 100MHz clock SSI 2-Byte address register");
		goto end;
	}

	lock_status = clk_100mhz_get_lock_status();
	if (lock_status == 0) {
		uint16_t error_code = CLOCK_APLL_UNLOCK_EVENT_CAUSE | CLK_100MHZ_ERR_IDX;
		error_log_event(error_code, LOG_ASSERT);
	}
end:
	//write back to 1-byte mode
	write_data = 0x01;
	if (!plat_i2c_write(I2C_BUS3, 0x9, 0x26, &write_data, 1)) {
		LOG_ERR("Failed to write 100MHz clock SSI 1-Byte address register");
	}
	return;
}

void check_read_312_5MHz_clock_status()
{
	uint8_t lock_status = 0;
	lock_status = clk_312_5mhz_get_lock_status();
	if (lock_status == 0xFF) {
		// fail to get lock status
		return;
	} else if (lock_status == 0) {
		uint16_t error_code = CLOCK_APLL_UNLOCK_EVENT_CAUSE | CLK_312_5MHZ_ERR_IDX;
		error_log_event(error_code, LOG_ASSERT);
	}
}

void check_clk_buf_loss_status()
{
	uint8_t clk_buf_loss_status = 0;
	if (!plat_read_cpld(CLK_100MHZ_BUF_LOSS_REG, &clk_buf_loss_status, 1)) {
		LOG_ERR("Failed to read 100MHz clock buffer loss status from CPLD");
		return;
	}
	/*
	Bit7: BUFF0_100M_LOSB_PLD
	Bit6: BUFF1_100M_LOSB_PLD
	Bit5: BUFF2_100M_LOSB_PLD
	if bit7 bit6 bit5 is 0 means fail
	*/
	if ((clk_buf_loss_status & 0xE0) != 0xE0) {
		if ((clk_buf_loss_status & BIT(7)) == 0) {
			uint16_t error_code =
				CLOCK_APLL_UNLOCK_EVENT_CAUSE | CLK_BUF0_100M_LOSB_PLD;
			error_log_event(error_code, LOG_ASSERT);
		}
		if ((clk_buf_loss_status & BIT(6)) == 0) {
			uint16_t error_code =
				CLOCK_APLL_UNLOCK_EVENT_CAUSE | CLK_BUF1_100M_LOSB_PLD;
			error_log_event(error_code, LOG_ASSERT);
		}
		if ((clk_buf_loss_status & BIT(5)) == 0) {
			uint16_t error_code =
				CLOCK_APLL_UNLOCK_EVENT_CAUSE | CLK_BUF2_100M_LOSB_PLD;
			error_log_event(error_code, LOG_ASSERT);
		}
	}
}
static void clk_apll_check_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!is_mb_dc_on()) {
		return;
	}

	check_read_100MHz_clock_status();
	uint8_t rev_id = get_board_rev_id();
	if (rev_id >= REV_ID_DVT_FAB4) {
		check_read_312_5MHz_clock_status();
	}
	check_clk_buf_loss_status();
	start_clk_apll_check_work();
}

static void start_clk_apll_check_work(void)
{
	k_work_reschedule(&clk_apll_check_work, CLK_APLL_CHECK_INTERVAL);
}

void pwr_sequence_event_timer_handler(struct k_timer *timer)
{
	k_work_submit(&pwr_sequence_event_work);
}

void pwr_sequence_event(struct k_work *work)
{
	bool is_dc_on_status = is_mb_dc_on();
	//if dc off
	if (!is_dc_on_status) {
		plat_find_power_seq_fail();
		uint8_t idx = plat_get_power_seq_fail_id();

		uint16_t error_code = (POWER_ON_SEQUENCE_TRIGGER_CAUSE << 13);
		error_log_event(error_code, LOG_ASSERT);
		LOG_ERR("power on sequence fail, error_code: 0x%x", error_code);

		//send event to bmc
		struct pldm_addsel_data sel_msg = { 0 };
		sel_msg.assert_type = LOG_ASSERT;
		sel_msg.event_type = IRIS_FAULT;
		sel_msg.event_data_1 = IRIS_POWER_ON_SEQUENCE_FAIL;
		sel_msg.event_data_2 = idx;

		if (PLDM_SUCCESS != send_event_log_to_bmc(sel_msg)) {
			LOG_ERR("Send SEL fail: 0x%x 0x%x 0x%x 0x%x", sel_msg.assert_type,
				sel_msg.event_data_1, sel_msg.event_data_2, sel_msg.event_data_3);
		} else {
			LOG_INF("Send SEL: 0x%x 0x%x 0x%x 0x%x", sel_msg.assert_type,
				sel_msg.event_data_1, sel_msg.event_data_2, sel_msg.event_data_3);
		}
	}
	//check clock APLL lock status
	start_clk_apll_check_work();
}

uint8_t pwr_steps_on_flag = 0;

void set_pwr_steps_on_flag(uint8_t flag_value)
{
	pwr_steps_on_flag = flag_value;
	//check value
	if (pwr_steps_on_flag != flag_value)
		LOG_ERR("set pwr_steps_on_flag failed, now pwr_steps_on_flag = %d",
			pwr_steps_on_flag);
}

uint8_t get_pwr_steps_on_flag(void)
{
	return pwr_steps_on_flag;
}

void ISR_GPIO_ALL_VR_PM_ALERT_R_N()
{
	if (gpio_get(ALL_VR_PM_ALERT_R_N) == GPIO_LOW) {
		give_all_vr_pm_alert_sem();
	}
}

bool ubc_en_changed_callback(cpld_info *info, uint8_t *data)
{
	bool ubc_en;
	bool last_ubc_en;

	if (get_pwr_steps_on_flag())
		return false;

	ubc_en = !!(*data & info->bit_check_mask);
	last_ubc_en = !!(info->last_polling_value & info->bit_check_mask);

	/* no state change */
	if (ubc_en == last_ubc_en) {
		LOG_INF("UBC_EN state not changed, still %d", ubc_en);
		return false;
	}

	LOG_INF("UBC_EN changed: %d -> %d", last_ubc_en, ubc_en);

	if (ubc_en) {
		plat_set_dc_on_log(LOG_ASSERT);
		k_timer_start(&pwr_sequence_event_work_timer, K_MSEC(1000), K_NO_WAIT);
	} else {
		plat_set_dc_on_log(LOG_DEASSERT);
	}

	k_timer_start(get_ubc_delaytimer(), K_MSEC(1000), K_NO_WAIT);

	return true;
}

void ISR_GPIO_FM_PLD_UBC_EN_R()
{
	LOG_DBG("gpio_%d_isr called, val=%d , dir= %d", FM_PLD_UBC_EN_R, gpio_get(FM_PLD_UBC_EN_R),
		gpio_get_direction(FM_PLD_UBC_EN_R));
}

/* Pin A12 (GPIO73 / SPIP1_CS) dynamic mux switching
 *
 * Hardware mux: SCFG DEVALTC register, SPIP1_SL bit (bit 1):
 *   DEVALTC[1] = 0 → GPIO66/67/70/73/76/77 on pads   (A12 = GPIO73)
 *   DEVALTC[1] = 1 → SPIP1 signals on pads           (A12 = SPIP1_CS)
*/
#define NPCM4XX_SCFG_BASE DT_REG_ADDR_BY_NAME(DT_NODELABEL(scfg), scfg)
#define NPCM4XX_DEVALTC 0x1C
#define NPCM4XX_SPIP1_SEL 1 /* DEVALTC bit1: 0=GPIO73, 1=SPIP1_CS    */

#define NPCM4XX_GPIO7_BASE (REG_GPIO_BASE + 0xE000) /* GPIO_7 register base  */
#define NPCM4XX_GPIO7_PDOUT 0x00 /* Port data output offset               */
#define NPCM4XX_GPIO7_PDIR 0x02 /* Port direction offset                 */
#define NPCM4XX_GPIO73_BIT 3 /* GPIO73 = port-7 bit-3                 */

void plat_switch_pin_a12(bool use_gpio73)
{
	uint8_t devaltc = sys_read8(NPCM4XX_SCFG_BASE + NPCM4XX_DEVALTC);

	if (use_gpio73) {
		sys_write8(sys_read8(NPCM4XX_GPIO7_BASE + NPCM4XX_GPIO7_PDOUT) &
				   ~BIT(NPCM4XX_GPIO73_BIT),
			   NPCM4XX_GPIO7_BASE + NPCM4XX_GPIO7_PDOUT);
		sys_write8(sys_read8(NPCM4XX_GPIO7_BASE + NPCM4XX_GPIO7_PDIR) |
				   BIT(NPCM4XX_GPIO73_BIT),
			   NPCM4XX_GPIO7_BASE + NPCM4XX_GPIO7_PDIR);
		sys_write8(devaltc & ~BIT(NPCM4XX_SPIP1_SEL), NPCM4XX_SCFG_BASE + NPCM4XX_DEVALTC);
		LOG_INF("[PIN_A12] A12 -> GPIO73 (output LOW)");
	} else {
		sys_write8(devaltc | BIT(NPCM4XX_SPIP1_SEL), NPCM4XX_SCFG_BASE + NPCM4XX_DEVALTC);
		sys_write8(sys_read8(NPCM4XX_GPIO7_BASE + NPCM4XX_GPIO7_PDIR) &
				   ~BIT(NPCM4XX_GPIO73_BIT),
			   NPCM4XX_GPIO7_BASE + NPCM4XX_GPIO7_PDIR);
		LOG_INF("[PIN_A12] A12 -> SPIP1_CS");
	}
}

void ISR_GPIO_RST_IRIS_PWR_ON_PLD_R1_N()
{
	// dc on
	if (gpio_get(RST_IRIS_PWR_ON_PLD_R1_N)) {
		set_clock_u87_u88_lphcsl_amp_ctrl_to_1v();
		plat_switch_pin_a12(false); /* HIGH -> A12 = SPIP1_CS */
		ioexp_init();
		if (get_asic_board_id() == ASIC_BOARD_ID_EVB) {
			init_U200052_IO();
			init_U200053_IO();
			init_U200070_IO();
			power_on_p3v3_osfp();
		}
		for (int i = 0; i < CLK_COMPONENT_MAX; i++) {
			clear_clock_status(NULL, i);
		}
		add_sync_oc_warn_to_work();
		// if board id == EVB , ctrl fan pwm
		if (get_asic_board_id() == ASIC_BOARD_ID_EVB) {
			LOG_INF("dc on, set fan pwm 65");
			init_pwm_dev();
			ast_pwm_set(65, PWM_PORT1);
			ast_pwm_set(65, PWM_PORT6);
		}
		// when dc on clear cpld polling alert status
		uint8_t err_type = CPLD_UNEXPECTED_VAL_TRIGGER_CAUSE;
		reset_error_log_states(err_type);
		// re-init adc
		set_is_adc_init(0);
	} else {
		plat_switch_pin_a12(true); /* LOW -> A12 = GPIO73 output low */
		gpio_conf(SPI_ADC_CS1_N, GPIO_OUTPUT);
		gpio_set(SPI_ADC_CS1_N, 0);
		LOG_INF("dc off, clear io expander init flag");
		set_ioe_init_flag(0);
		if (get_vr_test_mode_flag() == true) {
			LOG_INF("dc off, exit the vr test mode");
			vr_test_mode_enable(false);
		}

		// if board id == EVB , ctrl fan pwm
		if (get_asic_board_id() == ASIC_BOARD_ID_EVB) {
			LOG_INF("dc off, set fan pwm 0");
			init_pwm_dev();
			ast_pwm_set(0, PWM_PORT1);
			ast_pwm_set(0, PWM_PORT6);
		}
	}
}

void ISR_GPIO_SMB_HAMSA_MMC_LVC33_ALERT_N()
{
	uint8_t data[ERROR_CODE_LEN] = { 0 };
	LOG_INF("smb hamsa mmc lvc33 alert triggered");
	//i2c_burst_read(smb_hamsa_i2c_dev, HAMSA_BOOT1_ADDR, SMBUS_ERROR, data, ERROR_CODE_LEN);

	// bus, uint8_t addr, uint8_t offset, uint8_t *data, uint8_t len
	plat_i2c_read(I2C_BUS12, HAMSA_BOOT1_ADDR, SMBUS_ERROR, data, ERROR_CODE_LEN);
	plat_asic_error_event asic_event = { 0 };
	asic_event.event_id_0 = data[1];
	asic_event.event_id_1 = data[2];
	asic_event.chip_id = data[3];
	asic_event.module_id = data[4];
	plat_asic_error_error_log(LOG_ASSERT, asic_event);

	struct pldm_addsel_data smb_hamsa_sel_msg = { 0 };
	smb_hamsa_sel_msg.assert_type = LOG_ASSERT;
	smb_hamsa_sel_msg.event_type = ASIC_MODULE_ERROR; //ASIC_MODULE_ERROR;
	smb_hamsa_sel_msg.event_data_1 = HAMSA_SMB_ERR_EVENT_HEADER;
	smb_hamsa_sel_msg.event_data_2 = data[1]; // 123
	smb_hamsa_sel_msg.event_data_3 = data[2]; // 124
	if (send_event_log_to_bmc(smb_hamsa_sel_msg) != PLDM_SUCCESS) {
		LOG_ERR("Failed to send hamsa smb error code to bmc, event data: 0x%x 0x%x 0x%x\n",
			smb_hamsa_sel_msg.event_data_1, smb_hamsa_sel_msg.event_data_2,
			smb_hamsa_sel_msg.event_data_3);
	}
}

void ISR_ASIC_THERMTRIP_TRIGGER()
{
	asic_thermtrip_error_log(LOG_ASSERT);
}

bool plat_gpio_immediate_int_cb(uint8_t gpio_num)
{
	bool ret = false;

	switch (gpio_num) {
	case ALL_VR_PM_ALERT_R_N:
		ret = true;
		break;
	default:
		break;
	}

	return ret;
}
