#ifndef SHELL_PLATFORM_H
#define SHELL_PLATFORM_H

#include <stdint.h>

/* Include GPIO */
#include <drivers/gpio.h>

/* Declare Common */
#define sensor_name_to_num(x) #x,
#define GET_BIT_VAL(val, n) ((val & BIT(n)) >> (n))

/* Declare GPIO */
#define PINMASK_RESERVE_CHECK 1
#define GPIO_DEVICE_PREFIX "GPIO0_"
#define GPIO_RESERVE_PREFIX "Reserve"
#define NUM_OF_GROUP 6
#define NUM_OF_GPIO_IS_DEFINE 167
#define REG_GPIO_BASE 0x7e780000
#define REG_SCU 0x7E6E2000

int num_of_pin_in_one_group_lst[NUM_OF_GROUP] = { 32, 32, 32, 32, 32, 16 };
char GPIO_GROUP_NAME_LST[NUM_OF_GROUP][10] = { "GPIO0_A_D", "GPIO0_E_H", "GPIO0_I_L",
					       "GPIO0_M_P", "GPIO0_Q_T", "GPIO0_U_V" };

uint32_t GPIO_GROUP_REG_ACCESS[NUM_OF_GROUP] = {
	REG_GPIO_BASE + 0x00, /* GPIO_A/B/C/D Data Value Register */
	REG_GPIO_BASE + 0x20, /* GPIO_E/F/G/H Data Value Register */
	REG_GPIO_BASE + 0x70, /* GPIO_I/J/K/L Data Value Register */
	REG_GPIO_BASE + 0x78, /* GPIO_M/N/O/P Data Value Register */
	REG_GPIO_BASE + 0x80, /* GPIO_Q/R/S/T Data Value Register */
	REG_GPIO_BASE + 0x88 /* GPIO_U Data Value Register */
};

uint32_t GPIO_MULTI_FUNC_PIN_CTL_REG_ACCESS[] = {
	REG_SCU + 0x410, /* Multi-function pin ctl #1 */
	REG_SCU + 0x414, /* Multi-function pin ctl #2 */
	REG_SCU + 0x418, /* Multi-function pin ctl #3 */
	REG_SCU + 0x41C, /* Multi-function pin ctl #4 */
	REG_SCU + 0x430, /* Multi-function pin ctl #5 */
	REG_SCU + 0x434, /* Multi-function pin ctl #6 */
	REG_SCU + 0x438, /* Multi-function pin ctl #7 */
	REG_SCU + 0x450, /* Multi-function pin ctl #9 */
	REG_SCU + 0x454, /* Multi-function pin ctl #10 */
	REG_SCU + 0x458, /* Multi-function pin ctl #11 */
	REG_SCU + 0x4B0, /* Multi-function pin ctl #13 */
	REG_SCU + 0x4B4, /* Multi-function pin ctl #14 */
	REG_SCU + 0x4B8, /* Multi-function pin ctl #15 */
	REG_SCU + 0x4BC, /* Multi-function pin ctl #16 */
	REG_SCU + 0x4D4, /* Multi-function pin ctl #18 */
	REG_SCU + 0x4D8, /* Multi-function pin ctl #19 */
	REG_SCU + 0x510, /* Hardware Strap2 Register */
	REG_SCU + 0x51C, /* Hardware Strap2 Clear Register */
};

gpio_flags_t int_type_table[] = { GPIO_INT_DISABLE,   GPIO_INT_EDGE_RISING, GPIO_INT_EDGE_FALLING,
				  GPIO_INT_EDGE_BOTH, GPIO_INT_LEVEL_LOW,   GPIO_INT_LEVEL_HIGH };

enum GPIO_ACCESS { GPIO_READ, GPIO_WRITE };

/* Declare SENSOR */
const char *const sensor_type_name[] = {
	sensor_name_to_num(tmp75) sensor_name_to_num(adc) sensor_name_to_num(
		peci) sensor_name_to_num(isl69259) sensor_name_to_num(hsc) sensor_name_to_num(nvme)
		sensor_name_to_num(pch) sensor_name_to_num(mp5990) sensor_name_to_num(
			isl28022) sensor_name_to_num(pex89000) sensor_name_to_num(tps53689)
			sensor_name_to_num(xdpe15284) sensor_name_to_num(
				ltc4282) sensor_name_to_num(fan) sensor_name_to_num(tmp431)
				sensor_name_to_num(pmic) sensor_name_to_num(ina233)
					sensor_name_to_num(isl69254) sensor_name_to_num(max16550a)
						sensor_name_to_num(ina230)
							sensor_name_to_num(raa229621)
								sensor_name_to_num(nct7718w)
};

const char *const sensor_status_name[] = {
	sensor_name_to_num(read_success) sensor_name_to_num(read_acur_success) sensor_name_to_num(
		not_found) sensor_name_to_num(not_accesible) sensor_name_to_num(fail_to_access)
		sensor_name_to_num(init_status) sensor_name_to_num(unspecified_err)
			sensor_name_to_num(polling_disable) sensor_name_to_num(pre_read_error)
				sensor_name_to_num(post_read_error)
					sensor_name_to_num(api_unregister)
						sensor_name_to_num(4byte_acur_read_success)
};

enum SENSOR_ACCESS { SENSOR_READ, SENSOR_WRITE };

#endif /* SHELL_PLATFORM_H */
