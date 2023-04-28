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

#ifndef PLAT_GPIO_H
#define PLAT_GPIO_H

#include "hal_gpio.h"

// gpio_cfg(chip, number, is_init, direction, status, int_type, int_callback)
// dedicate gpio A0~A7, B0~B7, C0~C7, D0~D7, E0~E7, total 40 gpios
// Default name: Reserve_GPIOH0

// clang-format off

#define name_gpioA \
	gpio_name_to_num(Reserve_GPIOA0) \
	gpio_name_to_num(FM_BIOS_POST_CMPLT_BIC_N) \
	gpio_name_to_num(FM_CPU_BIC_SLP_S3_N) \
	gpio_name_to_num(APML_CPU_ALERT_BIC_N) \
	gpio_name_to_num(IRQ_UV_DETECT_N) \
	gpio_name_to_num(Reserve_GPIOA5) \
	gpio_name_to_num(PVDDCR_CPU0_BIC_OCP_N) \
	gpio_name_to_num(HSC_OCP_GPIO1_R)

#define name_gpioB \
	gpio_name_to_num(PVDDCR_CPU1_BIC_OCP_N) \
	gpio_name_to_num(RST_USB_HUB_R_N) \
	gpio_name_to_num(P3V_BAT_SCALED_EN_R) \
	gpio_name_to_num(HDT_BIC_TRST_R_N) \
	gpio_name_to_num(Reserve_GPIOB4) \
	gpio_name_to_num(FM_CPU_BIC_SLP_S5_N) \
	gpio_name_to_num(Reserve_GPIOB6) \
	gpio_name_to_num(PVDD11_S3_BIC_OCP_N)

#define name_gpioC \
	gpio_name_to_num(FM_HSC_TIMER) \
	gpio_name_to_num(IRQ_SMB_IO_LVC3_STBY_ALRT_N) \
	gpio_name_to_num(PVDDCR_CPU1_PMALERT_N) \
	gpio_name_to_num(Reserve_GPIOC3) \
	gpio_name_to_num(FM_CPU_BIC_THERMTRIP_N) \
	gpio_name_to_num(FM_PRSNT_CPU_BIC_N) \
	gpio_name_to_num(AUTH_PRSNT_BIC_N) \
	gpio_name_to_num(RST_CPU_RESET_BIC_N)

#define name_gpioD \
	gpio_name_to_num(PWRBTN_R1_N) \
	gpio_name_to_num(RST_BMC_R_N) \
	gpio_name_to_num(HDT_BIC_DBREQ_R_N) \
	gpio_name_to_num(BMC_READY) \
	gpio_name_to_num(BIC_READY) \
	gpio_name_to_num(FM_SOL_UART_CH_SEL_R) \
	gpio_name_to_num(Reserve_GPIOD6) \
	gpio_name_to_num(Reserve_GPIOD7)

#define name_gpioE \
	gpio_name_to_num(PWRGD_CPU_LVC3) \
	gpio_name_to_num(CPU_ERROR_BIC_LVC3_R_N) \
	gpio_name_to_num(PVDD11_S3_PMALERT_N) \
	gpio_name_to_num(IRQ_HSC_ALERT1_N) \
	gpio_name_to_num(SMB_SENSOR_LVC3_ALERT_N) \
	gpio_name_to_num(Reserve_GPIOE5) \
	gpio_name_to_num(SYS_PWRBTN_BIC_N) \
	gpio_name_to_num(RST_PLTRST_BIC_N)

#define name_gpioF \
	gpio_name_to_num(CPU_SMERR_BIC_N) \
	gpio_name_to_num(IRQ_HSC_ALERT2_N) \
	gpio_name_to_num(BIC_CPU_NMI_R_N) \
	gpio_name_to_num(FM_BMC_DEBUG_ENABLE_N) \
	gpio_name_to_num(FM_DBP_PRESENT_N) \
	gpio_name_to_num(FM_FAST_PROCHOT_EN_R_N) \
	gpio_name_to_num(FM_BIOS_ABL_DEBUG_MSG_DIS_N) \
	gpio_name_to_num(Reserve_GPIOF7)

#define name_gpioG \
	gpio_name_to_num(Reserve_GPIOG0) \
	gpio_name_to_num(FM_BIOS_MRC_DEBUG_MSG_DIS) \
	gpio_name_to_num(FAST_PROCHOT_N) \
	gpio_name_to_num(BIC_CPU_RSVD_N) \
	gpio_name_to_num(BIC_JTAG_SEL_R) \
	gpio_name_to_num(Reserve_GPIOG5) \
	gpio_name_to_num(HSC_OCP_GPIO2_R) \
	gpio_name_to_num(HSC_OCP_GPIO3_R)

#define name_gpioH \
	gpio_name_to_num(RST_RSMRST_BMC_N) \
	gpio_name_to_num(Reserve_GPIOH1) \
	gpio_name_to_num(Reserve_GPIOH2) \
	gpio_name_to_num(FM_CPU_BIC_PROCHOT_LVT3_N) \
	gpio_name_to_num(Reserve_GPIOH4) \
	gpio_name_to_num(Reserve_GPIOH5) \
	gpio_name_to_num(Reserve_GPIOH6) \
	gpio_name_to_num(Reserve_GPIOH7)

#define name_gpioI \
	gpio_name_to_num(Reserve_GPIOI0) \
	gpio_name_to_num(Reserve_GPIOI1) \
	gpio_name_to_num(Reserve_GPIOI2) \
	gpio_name_to_num(Reserve_GPIOI3) \
	gpio_name_to_num(Reserve_GPIOI4) \
	gpio_name_to_num(Reserve_GPIOI5) \
	gpio_name_to_num(Reserve_GPIOI6) \
	gpio_name_to_num(Reserve_GPIOI7)

#define name_gpioJ \
	gpio_name_to_num(Reserve_GPIOJ0) \
	gpio_name_to_num(Reserve_GPIOJ1) \
	gpio_name_to_num(Reserve_GPIOJ2) \
	gpio_name_to_num(Reserve_GPIOJ3) \
	gpio_name_to_num(Reserve_GPIOJ4) \
	gpio_name_to_num(Reserve_GPIOJ5) \
	gpio_name_to_num(Reserve_GPIOJ6) \
	gpio_name_to_num(Reserve_GPIOJ7)

#define name_gpioK \
	gpio_name_to_num(Reserve_GPIOK0) \
	gpio_name_to_num(Reserve_GPIOK1) \
	gpio_name_to_num(Reserve_GPIOK2) \
	gpio_name_to_num(Reserve_GPIOK3) \
	gpio_name_to_num(Reserve_GPIOK4) \
	gpio_name_to_num(Reserve_GPIOK5) \
	gpio_name_to_num(Reserve_GPIOK6) \
	gpio_name_to_num(Reserve_GPIOK7)

#define name_gpioL \
	gpio_name_to_num(Reserve_GPIOL0) \
	gpio_name_to_num(Reserve_GPIOL1) \
	gpio_name_to_num(BIC_JTAG_MUX_SEL) \
	gpio_name_to_num(BOARD_ID2) \
	gpio_name_to_num(Reserve_GPIOL4) \
	gpio_name_to_num(Reserve_GPIOL5) \
	gpio_name_to_num(BOARD_ID0) \
	gpio_name_to_num(BOARD_ID1)

#define name_gpioM \
	gpio_name_to_num(BIC_SECUREBOOT) \
	gpio_name_to_num(BOARD_ID3) \
	gpio_name_to_num(BIC_ESPI_SELECT) \
	gpio_name_to_num(Reserve_GPIOM3) \
	gpio_name_to_num(BOARD_ID5) \
	gpio_name_to_num(BOARD_ID4) \
	gpio_name_to_num(Reserve_GPIOM6) \
	gpio_name_to_num(Reserve_GPIOM7)

#define name_gpioN \
	gpio_name_to_num(Reserve_GPION0) \
	gpio_name_to_num(Reserve_GPION1) \
	gpio_name_to_num(Reserve_GPION2) \
	gpio_name_to_num(Reserve_GPION3) \
	gpio_name_to_num(Reserve_GPION4) \
	gpio_name_to_num(Reserve_GPION5) \
	gpio_name_to_num(Reserve_GPION6) \
	gpio_name_to_num(Reserve_GPION7)

#define name_gpioO \
	gpio_name_to_num(Reserve_GPIOO0) \
	gpio_name_to_num(Reserve_GPIOO1) \
	gpio_name_to_num(Reserve_GPIOO2) \
	gpio_name_to_num(Reserve_GPIOO3) \
	gpio_name_to_num(Reserve_GPIOO4) \
	gpio_name_to_num(Reserve_GPIOO5) \
	gpio_name_to_num(Reserve_GPIOO6) \
	gpio_name_to_num(Reserve_GPIOO7)

#define name_gpioP \
	gpio_name_to_num(Reserve_GPIOP0) \
	gpio_name_to_num(Reserve_GPIOP1) \
	gpio_name_to_num(Reserve_GPIOP2) \
	gpio_name_to_num(Reserve_GPIOP3) \
	gpio_name_to_num(Reserve_GPIOP4) \
	gpio_name_to_num(Reserve_GPIOP5) \
	gpio_name_to_num(Reserve_GPIOP6) \
	gpio_name_to_num(Reserve_GPIOP7)

// GPIOQ5 hardware not define
#define name_gpioQ \
	gpio_name_to_num(Reserve_GPIOQ0) \
	gpio_name_to_num(Reserve_GPIOQ1) \
	gpio_name_to_num(Reserve_GPIOQ2) \
	gpio_name_to_num(Reserve_GPIOQ3) \
	gpio_name_to_num(Reserve_GPIOQ4) \
	gpio_name_to_num(Reserve_GPIOQ5) \
	gpio_name_to_num(Reserve_GPIOQ6) \
	gpio_name_to_num(Reserve_GPIOQ7)

#define name_gpioR \
	gpio_name_to_num(Reserve_GPIOR0) \
	gpio_name_to_num(Reserve_GPIOR1) \
	gpio_name_to_num(Reserve_GPIOR2) \
	gpio_name_to_num(Reserve_GPIOR3) \
	gpio_name_to_num(Reserve_GPIOR4) \
	gpio_name_to_num(Reserve_GPIOR5) \
	gpio_name_to_num(Reserve_GPIOR6) \
	gpio_name_to_num(Reserve_GPIOR7)

// GPIOS3, S4, S5, S6, S7 hardware not define
#define name_gpioS \
	gpio_name_to_num(Reserve_GPIOS0) \
	gpio_name_to_num(Reserve_GPIOS1) \
	gpio_name_to_num(Reserve_GPIOS2) \
	gpio_name_to_num(Reserve_GPIOS3) \
	gpio_name_to_num(Reserve_GPIOS4) \
	gpio_name_to_num(Reserve_GPIOS5) \
	gpio_name_to_num(Reserve_GPIOS6) \
	gpio_name_to_num(Reserve_GPIOS7)

// GPIOT input only
#define name_gpioT \
	gpio_name_to_num(Reserve_GPIOT0) \
	gpio_name_to_num(Reserve_GPIOT1) \
	gpio_name_to_num(Reserve_GPIOT2) \
	gpio_name_to_num(Reserve_GPIOT3) \
	gpio_name_to_num(Reserve_GPIOT4) \
	gpio_name_to_num(Reserve_GPIOT5) \
	gpio_name_to_num(Reserve_GPIOT6) \
	gpio_name_to_num(HSC_TYPE_0)

// GPIOU input only
#define name_gpioU \
	gpio_name_to_num(Reserve_GPIOU0) \
	gpio_name_to_num(Reserve_GPIOU1) \
	gpio_name_to_num(Reserve_GPIOU2) \
	gpio_name_to_num(Reserve_GPIOU3) \
	gpio_name_to_num(Reserve_GPIOU4) \
	gpio_name_to_num(Reserve_GPIOU5) \
	gpio_name_to_num(Reserve_GPIOU6) \
	gpio_name_to_num(HSC_TYPE_1)

// clang-format on

#define gpio_name_to_num(x) x,
enum _GPIO_NUMS_ {
	name_gpioA name_gpioB name_gpioC name_gpioD name_gpioE name_gpioF name_gpioG name_gpioH
		name_gpioI name_gpioJ name_gpioK name_gpioL name_gpioM name_gpioN name_gpioO
			name_gpioP name_gpioQ name_gpioR name_gpioS name_gpioT name_gpioU
};

extern enum _GPIO_NUMS_ GPIO_NUMS;
#undef gpio_name_to_num

extern char *gpio_name[];

#endif
