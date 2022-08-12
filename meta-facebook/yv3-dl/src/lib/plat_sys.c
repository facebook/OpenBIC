#include <stdint.h>
#include <stdlib.h>
#include "plat_sys.h"

#include "ipmi.h"
#include "ipmb.h"
#include "pmbus.h"
#include "sensor.h"
#include "libipmi.h"
#include "libutil.h"
#include "util_sys.h"
#include "hal_gpio.h"
#include "plat_gpio.h"
#include "xdpe12284c.h"
#include "plat_sensor_table.h"

/* BMC reset */
void BMC_reset_handler()
{
	gpio_set(RST_BMC_R_N, GPIO_LOW);
	k_msleep(10);
	gpio_set(RST_BMC_R_N, GPIO_HIGH);
}

K_WORK_DELAYABLE_DEFINE(BMC_reset_work, BMC_reset_handler);
int pal_submit_bmc_cold_reset()
{
	k_work_schedule(&BMC_reset_work, K_MSEC(1000));
	return 0;
}
/* BMC reset */

void clear_cmos_handler()
{
	gpio_set(FM_BIC_RST_RTCRST, GPIO_HIGH);
	k_msleep(200);
	gpio_set(FM_BIC_RST_RTCRST, GPIO_LOW);
}

K_WORK_DEFINE(clear_cmos_work, clear_cmos_handler);
int pal_clear_cmos()
{
	k_work_submit(&clear_cmos_work);
	return 0;
}

void check_Infineon_VR_VCCIO_UV_fault(uint8_t sensor_num)
{
	static bool is_asserted = false;
	uint8_t retry = 5;
	I2C_MSG msg;
	memset(&msg, 0, sizeof(I2C_MSG));

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 2;
	msg.data[0] = PMBUS_PAGE;
	msg.data[1] = INFINEON_STATUS_PAGE;

	if (i2c_master_write(&msg, retry)) {
		printf("[%s] Inf status page setting i2c writing failed\n", __func__);
		return;
	}

	msg.tx_len = 1;
	msg.rx_len = 1;

	// Register: Defined in XDPE12xxC-V1.6 Customer Register Map
	msg.data[0] = 0x06;

	if (i2c_master_read(&msg, retry)) {
		printf("[%s] i2c read vout status failed\n", __func__);
		return;
	}

	// BIT 5 PCC warning which means Current high
	if (GETBIT(msg.data[0], 5)) {
		if (VCCIO_VR_UV_fault_add_sel()) {
			is_asserted = true;
		}
	}
	return;
}

void check_Renesas_VR_VCCIO_UV_fault(uint8_t sensor_num)
{
	static bool is_asserted = false;
	uint8_t retry = 5;
	I2C_MSG msg;
	memset(&msg, 0, sizeof(I2C_MSG));

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = PMBUS_STATUS_VOUT;

	if (i2c_master_read(&msg, retry)) {
		printf("[%s] i2c read vout status fail\n", __func__);
		return;
	}

	// BIT 4: VOUT UV FAULT
	if (GETBIT(msg.data[0], 4)) {
		if (VCCIO_VR_UV_fault_add_sel()) {
			is_asserted = true;
		}
	}
	return;
}

bool VCCIO_VR_UV_fault_add_sel()
{
	common_addsel_msg_t *sel_msg = (common_addsel_msg_t *)malloc(sizeof(common_addsel_msg_t));
	if (sel_msg == NULL) {
		printf("[%s] Memory allocation failed!\n", __func__);
		return false;
	}

	sel_msg->InF_target = BMC_IPMB;
	sel_msg->sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
	sel_msg->event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
	sel_msg->sensor_number = SENSOR_NUM_VOL_PVCCIO_VR;
	sel_msg->event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_VCCIOFAULT;
	sel_msg->event_data2 = 0xFF;
	sel_msg->event_data3 = 0xFF;

	if (!common_add_sel_evt_record(sel_msg)) {
		printf("[%s] VCCIO UV fault add sel failed", __func__);
		SAFE_FREE(sel_msg);
		return false;
	}

	SAFE_FREE(sel_msg);
	return true;
}
