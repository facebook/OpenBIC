/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *	 http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string.h>
#include <logging/log.h>
#include "util_sys.h"
#include "plat_isr.h"
#include "plat_class.h"
#include "plat_gpio.h"
#include "plat_i2c.h"

LOG_MODULE_REGISTER(plat_gpio);

#define gpio_name_to_num(x) #x,
char *gpio_name[] = {
	name_gpioA name_gpioB name_gpioC name_gpioD name_gpioE name_gpioF name_gpioG name_gpioH
		name_gpioI name_gpioJ name_gpioK name_gpioL name_gpioM name_gpioN name_gpioO
			name_gpioP name_gpioQ name_gpioR name_gpioS name_gpioT name_gpioU
};
#undef gpio_name_to_num

/* chip,
 * number,
 * is_init,is_latch,
 * direction,
 * status,
 * property,
 * int_type,
 * int_cb,
*/

// clang-format off

GPIO_CFG plat_gpio_cfg[] = {
	/** Group A: 00-07 **/
	{ CHIP_GPIO, 0, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 1, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 2, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 3, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 4, ENABLE, DISABLE, GPIO_INPUT, GPIO_HIGH, PUSH_PULL, GPIO_INT_EDGE_BOTH, ISR_MB_PCIE_RST },
	{ CHIP_GPIO, 5, ENABLE, DISABLE, GPIO_INPUT, GPIO_HIGH, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 6, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL  },
	{ CHIP_GPIO, 7, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_EDGE_BOTH, ISR_MB_DC_STAGUS_CHAGNE },

	/** Group B: 08-15 **/
	{ CHIP_GPIO, 8, ENABLE, DISABLE, GPIO_OUTPUT, GPIO_HIGH, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 9, ENABLE, DISABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 10, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 11, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 12, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 13, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 14, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 15, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },

	/** Group C: 16-23 **/
	{ CHIP_GPIO, 16, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 17, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 18, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 19, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 20, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 21, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 22, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 23, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },

	/** Group D: 24-31 **/
	{ CHIP_GPIO, 24, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 25, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 26, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 27, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 28, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 29, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 30, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 31, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_EDGE_BOTH, ISR_SET_CXL_LED },

	/** Group E: 32-39 **/
	{ CHIP_GPIO, 32, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 33, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 34, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 35, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 36, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 37, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 38, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 39, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },

	/** Group F: 40-47 **/
	{ CHIP_GPIO, 40, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 41, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 42, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 43, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_EDGE_BOTH, ISR_E1S_PWR_CHANGE },
	{ CHIP_GPIO, 44, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 45, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 46, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 47, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_EDGE_BOTH, ISR_SET_CXL_LED },

	/** Group G: 48-55 **/
	{ CHIP_GPIO, 48, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 49, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 50, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 51, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 52, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 53, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 54, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 55, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },

	/** Group H: 56-63 **/
	{ CHIP_GPIO, 56, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 57, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 58, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_HIGH, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 59, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 60, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 61, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 62, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 63, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },

	/** Group I: 64-71 **/
	{ CHIP_GPIO, 64, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 65, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 66, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 67, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 68, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 69, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 70, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 71, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },

	/** Group J: 72-79 **/
	{ CHIP_GPIO, 72, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 73, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 74, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 75, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 76, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 77, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 78, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 79, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },

	/** Group K: 80-87 **/
	{ CHIP_GPIO, 80, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 81, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 82, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 83, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 84, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 85, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 86, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 87, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },

	/** Group L: 88-95 **/
	{ CHIP_GPIO, 88, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 89, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 90, DISABLE, DISABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 91, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 92, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 93, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 94, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 95, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },

	/** Group M: 96-103 **/
	{ CHIP_GPIO, 96, ENABLE, DISABLE, GPIO_OUTPUT, GPIO_HIGH, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 97, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 98, ENABLE, DISABLE, GPIO_OUTPUT, GPIO_HIGH, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 99, ENABLE, DISABLE, GPIO_INPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 100, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 101, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 102, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 103, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },

	/** Group N: 104-111 **/
	{ CHIP_GPIO, 104, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 105, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 106, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 107, ENABLE, ENABLE, GPIO_OUTPUT, GPIO_LOW, PUSH_PULL, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 108, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 109, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 110, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 111, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },

	/** Group O: 112-119 **/
	{ CHIP_GPIO, 112, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 113, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 114, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 115, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 116, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 117, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 118, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 119, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },

	/** Group P: 120-127 **/
	{ CHIP_GPIO, 120, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 121, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 122, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 123, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 124, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 125, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 126, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 127, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },

	/** Group Q: 128-135 **/
	{ CHIP_GPIO, 128, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 129, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 130, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 131, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 132, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 133, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 134, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 135, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },

	/** Group R: 136-143 **/
	{ CHIP_GPIO, 136, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 137, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 138, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 139, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 140, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 141, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 142, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 143, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },

	/** Group S: 144-151 **/
	{ CHIP_GPIO, 144, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 145, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 146, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 147, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 148, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 149, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 150, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 151, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },

	/** Group T: 152-159 **/
	{ CHIP_GPIO, 152, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 153, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 154, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 155, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 156, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 157, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 158, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 159, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },

	/** Group U: 160-167 **/
	{ CHIP_GPIO, 160, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 161, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 162, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 163, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 164, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 165, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 166, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
	{ CHIP_GPIO, 167, DISABLE, DISABLE, GPIO_INPUT, GPIO_LOW, OPEN_DRAIN, GPIO_INT_DISABLE, NULL },
};

// clang-format on

bool pal_load_gpio_config(void)
{
	if (get_board_revision() == BOARD_POC) {
		plat_gpio_cfg[POC_Reserve_GPIOF2].is_init = DISABLE;
		gpio_name[POC_Reserve_GPIOF2] = "Reserve_GPIOF2";

		plat_gpio_cfg[POC_Reserve_GPIOF3].is_init = DISABLE;
		gpio_name[POC_Reserve_GPIOF3] = "Reserve_GPIOF3";

		plat_gpio_cfg[POC_EN_P3V3_E1S_0_R].is_latch = ENABLE;
		plat_gpio_cfg[POC_EN_P3V3_E1S_0_R].direction = GPIO_OUTPUT;
		gpio_name[POC_EN_P3V3_E1S_0_R] = "EN_P3V3_E1S_0_R";

		plat_gpio_cfg[POC_PWRGD_P3V3_E1S_0_R].int_type = GPIO_INT_EDGE_BOTH;
		plat_gpio_cfg[POC_PWRGD_P3V3_E1S_0_R].int_cb = ISR_E1S_PWR_CHANGE;
		gpio_name[POC_PWRGD_P3V3_E1S_0_R] = "PWRGD_P3V3_E1S_0_R";
	}

	memcpy(&gpio_cfg[0], &plat_gpio_cfg[0], sizeof(plat_gpio_cfg));
	return true;
}

// IO Expander
IOE_CFG ioe_cfg[] = {
	{ ADDR_IOE1, TCA9555_CONFIG_REG_0, 0x18, TCA9555_OUTPUT_PORT_REG_0, 0x00 },
	{ ADDR_IOE1, TCA9555_CONFIG_REG_1, 0xC0, TCA9555_OUTPUT_PORT_REG_1, 0xFE },
	{ ADDR_IOE2, TCA9555_CONFIG_REG_0, 0xF0, TCA9555_OUTPUT_PORT_REG_0, 0x00 },
	{ ADDR_IOE2, TCA9555_CONFIG_REG_1, 0xC0, TCA9555_OUTPUT_PORT_REG_1, 0x00 },
	{ ADDR_IOE3, TCA9555_CONFIG_REG_0, 0xE7, TCA9555_OUTPUT_PORT_REG_0, 0x18 },
	{ ADDR_IOE3, TCA9555_CONFIG_REG_1, 0xF0, TCA9555_OUTPUT_PORT_REG_1, 0x00 },
	{ ADDR_IOE4, TCA9555_CONFIG_REG_0, 0x10, TCA9555_OUTPUT_PORT_REG_0, 0x3F },
	{ ADDR_IOE4, TCA9555_CONFIG_REG_1, 0x04, TCA9555_OUTPUT_PORT_REG_1, 0x3E },
};

int get_ioe_value(uint8_t ioe_addr, uint8_t ioe_reg, uint8_t *value)
{
	int ret = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = I2C_BUS6;
	msg.target_addr = ioe_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = ioe_reg;

	ret = i2c_master_read(&msg, retry);

	if (ret != 0) {
		LOG_DBG("Failed to read IOE(0x%02X). The register is 0x%02X.", ioe_addr, ioe_reg);
		return -1;
	}

	*value = msg.data[0];

	return 0;
}

int set_ioe_value(uint8_t ioe_addr, uint8_t ioe_reg, uint8_t value)
{
	int ret = 0;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	msg.bus = I2C_BUS6;
	msg.target_addr = ioe_addr;
	msg.tx_len = 2;
	msg.rx_len = 1;
	msg.data[0] = ioe_reg;
	msg.data[1] = value;

	ret = i2c_master_write(&msg, retry);

	if (ret != 0) {
		LOG_DBG("Failed to write IOE(0x%02X). The register is 0x%02X and the value is 0x%02X",
			ioe_addr, ioe_reg, value);
		return -1;
	}

	return 0;
}

void init_ioe_config()
{
	int ret = 0;
	uint8_t ioe_reg_value = 0;

	for (int i = 0; i < ARRAY_SIZE(ioe_cfg); i++) {
		ret = set_ioe_value(ioe_cfg[i].addr, ioe_cfg[i].conf_reg, ioe_cfg[i].conf_dir);

		if (ret != 0) {
			LOG_ERR("Failed to initialize IOE(0x%02x)'s direction. The register is :0x%02X",
				ioe_cfg[i].addr, ioe_cfg[i].conf_reg);
		}

		uint8_t init_val_mask, init_dir_mask;
		// If BIC AC off, need init E1S_PERSET
		if (is_ac_lost()) {
			// Reserve the bit 4&5's last state
			init_val_mask = 0xCF;
			init_dir_mask = 0x30;
		} else {
			// Reserve the bit 4&5&6's last state
			init_val_mask = 0x8F;
			init_dir_mask = 0x70;
		}

		if ((ioe_cfg[i].addr == ADDR_IOE4) &&
		    (ioe_cfg[i].output_reg == TCA9555_OUTPUT_PORT_REG_1)) {
			// Get the last state before initializing.
			ret = get_ioe_value(ioe_cfg[i].addr, ioe_cfg[i].output_reg, &ioe_reg_value);

			if (ret != 0) {
				LOG_ERR("Failed to get E1S present from IOE4");
				continue;
			}
			ioe_reg_value = (ioe_cfg[i].output_val & init_val_mask) |
					(ioe_reg_value & init_dir_mask);

		} else if ((ioe_cfg[i].addr == ADDR_IOE2) &&
			   (ioe_cfg[i].output_reg == TCA9555_OUTPUT_PORT_REG_0) &&
			   (gpio_get(PG_CARD_OK) == HIGH_ACTIVE)) { // If all CXL have been already.
			ioe_reg_value =
				(ioe_cfg[i].output_val |
				 IOE_SWITCH_MUX_TO_BIC); // Enable P0~P3 to switch mux to BIC.
		} else {
			ioe_reg_value = ioe_cfg[i].output_val;
		}

		ret = set_ioe_value(ioe_cfg[i].addr, ioe_cfg[i].output_reg, ioe_reg_value);

		if (ret != 0) {
			LOG_ERR("Failed to initialize IOE(0x%02x)'s state. The register is :0x%02X",
				ioe_cfg[i].addr, ioe_cfg[i].conf_reg);
		}
	}
}

int check_ioe4_e1s_prsnt_pin()
{
	int ret = 0;
	uint8_t io_input_status = 0, e1s_present_status = 0;

	ret = get_ioe_value(ADDR_IOE4, TCA9555_INPUT_PORT_REG_1, &io_input_status);

	if (ret != 0) {
		return ret;
	}

	e1s_present_status = io_input_status & TCA9555_PORT_2;

	if (e1s_present_status != 0) { // e1s is not present when present pin is low.
		LOG_WRN("E1S is not present");
		return -1;
	} else {
		return 0;
	}
}
