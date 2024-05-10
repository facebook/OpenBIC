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

void gpio_int_default();
uint16_t plat_gpio_cfg_size(void);

// clang-format off

// gpio_cfg(chip, number, is_init, direction, status, int_type, int_callback)
// dedicate gpio A0~A7, B0~B7, C0~C7, D0~D7, E0~E7, total 40 gpios
// Default name: Reserve_GPIOH0
#define name_gpioA	\
	gpio_name_to_num(OCP_DEBUG_PRSNT_R2) \
	gpio_name_to_num(PWRGD_P24V_RS485_1_LF_R) \
	gpio_name_to_num(PWRGD_P24V_LF_R) \
	gpio_name_to_num(SMB_CDU_PMBUS_ALERT_LF_R1_N) \
	gpio_name_to_num(PWRGD_P12V_LF_R) \
	gpio_name_to_num(PWRGD_P5V_LF_R) \
	gpio_name_to_num(PWRGD_P3V3_LF_R) \
	gpio_name_to_num(PWRGD_P48V_HSC_LF_R)
#define name_gpioB	\
	gpio_name_to_num(PWRGD_P12V_PUMP) \
	gpio_name_to_num(PWRGD_P24V_VALVE_1) \
	gpio_name_to_num(SMB_PDB_PMBUS_ALERT_N) \
	gpio_name_to_num(PWRGD_P24V_RS485_2) \
	gpio_name_to_num(PWRGD_P24V_RS485_3) \
	gpio_name_to_num(PWRGD_P24V_VALVE_2) \
	gpio_name_to_num(PWRGD_P12V_AUX_R1) \
	gpio_name_to_num(FM_BP_RS485_DE)
#define name_gpioC	\
	gpio_name_to_num(PUMP1_PRSNT_BUF_N) \
	gpio_name_to_num(PUMP2_PRSNT_BUF_N) \
	gpio_name_to_num(PUMP3_PRSNT_BUF_N) \
	gpio_name_to_num(FW_BIC_P12V_PUMP_EN) \
	gpio_name_to_num(CDU_PWR_BTN) \
	gpio_name_to_num(FM_P5V_USB_SW_EN_N) \
	gpio_name_to_num(FM_OC_USB_N) \
	gpio_name_to_num(RPU_LEAK_ALERT_N)
#define name_gpioD	\
	gpio_name_to_num(PWRGD_P48V_BRI) \
	gpio_name_to_num(FM_LED_FP_1_EN) \
	gpio_name_to_num(FM_LED_FP_2_EN) \
	gpio_name_to_num(FM_LED_FP_3_EN) \
	gpio_name_to_num(FM_LED_FP_4_EN) \
	gpio_name_to_num(REV_ID0) \
	gpio_name_to_num(REV_ID1) \
	gpio_name_to_num(REV_ID2)
#define name_gpioE	\
	gpio_name_to_num(LED_BP_CPC_1) \
	gpio_name_to_num(LED_BP_CPC_2) \
	gpio_name_to_num(FP_WATER_BTN_BUF_N) \
	gpio_name_to_num(FM_BIC_P48V_BUS_EN) \
	gpio_name_to_num(FM_BIC_GPIOF3) \
	gpio_name_to_num(BRI_ADDR_A0) \
	gpio_name_to_num(BRI_ADDR_A1) \
	gpio_name_to_num(BRI_ADDR_A2)
#define name_gpioF	\
	gpio_name_to_num(Reserve_GPIOF0) \
	gpio_name_to_num(Reserve_GPIOF1) \
	gpio_name_to_num(Reserve_GPIOF2) \
	gpio_name_to_num(Reserve_GPIOF3) \
	gpio_name_to_num(Reserve_GPIOF4) \
	gpio_name_to_num(Reserve_GPIOF5) \
	gpio_name_to_num(Reserve_GPIOF6) \
	gpio_name_to_num(Reserve_GPIOF7)
#define name_gpioG	\
	gpio_name_to_num(Reserve_GPIOG0) \
	gpio_name_to_num(Reserve_GPIOG1) \
	gpio_name_to_num(BIC_RPU_READY0) \
	gpio_name_to_num(IT_LEAK_ALERT0_R) \
	gpio_name_to_num(RJ45_CONN0_GPIO0_R) \
	gpio_name_to_num(RJ45_CONN0_GPIO1_R) \
	gpio_name_to_num(RJ45_CONN0_GPIO2_R) \
	gpio_name_to_num(BIC_RPU_READY1)
#define name_gpioH	\
	gpio_name_to_num(IT_LEAK_ALERT1_R) \
	gpio_name_to_num(RJ45_CONN1_GPIO0_R) \
	gpio_name_to_num(RJ45_CONN1_GPIO1_R) \
	gpio_name_to_num(RJ45_CONN1_GPIO2_R) \
	gpio_name_to_num(Reserve_GPIOH4) \
	gpio_name_to_num(Reserve_GPIOH5) \
	gpio_name_to_num(Reserve_GPIOH6) \
	gpio_name_to_num(Reserve_GPIOH7)
#define name_gpioI	\
	gpio_name_to_num(Reserve_GPIOI0) \
	gpio_name_to_num(Reserve_GPIOI1) \
	gpio_name_to_num(Reserve_GPIOI2) \
	gpio_name_to_num(Reserve_GPIOI3) \
	gpio_name_to_num(Reserve_GPIOI4) \
	gpio_name_to_num(Reserve_GPIOI5) \
	gpio_name_to_num(Reserve_GPIOI6) \
	gpio_name_to_num(Reserve_GPIOI7)
#define name_gpioJ	\
	gpio_name_to_num(Reserve_GPIOJ0) \
	gpio_name_to_num(Reserve_GPIOJ1) \
	gpio_name_to_num(Reserve_GPIOJ2) \
	gpio_name_to_num(Reserve_GPIOJ3) \
	gpio_name_to_num(Reserve_GPIOJ4) \
	gpio_name_to_num(Reserve_GPIOJ5) \
	gpio_name_to_num(Reserve_GPIOJ6) \
	gpio_name_to_num(Reserve_GPIOJ7)
#define name_gpioK	\
	gpio_name_to_num(Reserve_GPIOK0) \
	gpio_name_to_num(Reserve_GPIOK1) \
	gpio_name_to_num(Reserve_GPIOK2) \
	gpio_name_to_num(Reserve_GPIOK3) \
	gpio_name_to_num(Reserve_GPIOK4) \
	gpio_name_to_num(Reserve_GPIOK5) \
	gpio_name_to_num(Reserve_GPIOK6) \
	gpio_name_to_num(Reserve_GPIOK7)
#define name_gpioL	\
	gpio_name_to_num(Reserve_GPIOL0) \
	gpio_name_to_num(Reserve_GPIOL1) \
	gpio_name_to_num(Reserve_GPIOL2) \
	gpio_name_to_num(Reserve_GPIOL3) \
	gpio_name_to_num(Reserve_GPIOL4) \
	gpio_name_to_num(Reserve_GPIOL5) \
	gpio_name_to_num(Reserve_GPIOL6) \
	gpio_name_to_num(Reserve_GPIOL7)
// GPIOM6, M7 hardware not define
#define name_gpioM	\
	gpio_name_to_num(Reserve_GPIOM0) \
	gpio_name_to_num(Reserve_GPIOM1) \
	gpio_name_to_num(Reserve_GPIOM2) \
	gpio_name_to_num(Reserve_GPIOM3) \
	gpio_name_to_num(Reserve_GPIOM4) \
	gpio_name_to_num(Reserve_GPIOM5) \
	gpio_name_to_num(Reserve_GPIOM6) \
	gpio_name_to_num(Reserve_GPIOM7)
#define name_gpioN	\
	gpio_name_to_num(Reserve_GPION0) \
	gpio_name_to_num(Reserve_GPION1) \
	gpio_name_to_num(Reserve_GPION2) \
	gpio_name_to_num(Reserve_GPION3) \
	gpio_name_to_num(Reserve_GPION4) \
	gpio_name_to_num(SMB_FRONT_IO_TMP_ALERT_N) \
	gpio_name_to_num(P12V_BRICK_ALERT_LF_R) \
	gpio_name_to_num(P48V_BRI_STBY_FAULT)
#define name_gpioO	\
	gpio_name_to_num(Reserve_GPIOO0) \
	gpio_name_to_num(Reserve_GPIOO1) \
	gpio_name_to_num(Reserve_GPIOO2) \
	gpio_name_to_num(Reserve_GPIOO3) \
	gpio_name_to_num(BIC_RPU_READY2) \
	gpio_name_to_num(IT_LEAK_ALERT2_R) \
	gpio_name_to_num(RJ45_CONN2_GPIO0_R) \
	gpio_name_to_num(RJ45_CONN2_GPIO1_R)
#define name_gpioP	\
	gpio_name_to_num(RJ45_CONN2_GPIO2_R) \
	gpio_name_to_num(Reserve_GPIOP1) \
	gpio_name_to_num(Reserve_GPIOP2) \
	gpio_name_to_num(Reserve_GPIOP3) \
	gpio_name_to_num(Reserve_GPIOP4) \
	gpio_name_to_num(Reserve_GPIOP5) \
	gpio_name_to_num(BIC_RCVR_BTN_N) \
	gpio_name_to_num(FM_BIC_READY_R_N)
// GPIOQ5 hardware not define
#define name_gpioQ	\
	gpio_name_to_num(BIC_RPU_READY3) \
	gpio_name_to_num(IT_LEAK_ALERT3_R) \
	gpio_name_to_num(RJ45_CONN3_GPIO0_R) \
	gpio_name_to_num(RJ45_CONN3_GPIO1_R) \
	gpio_name_to_num(RJ45_CONN3_GPIO2_R) \
	gpio_name_to_num(Reserve_GPIOQ5) \
	gpio_name_to_num(Reserve_GPIOQ6) \
	gpio_name_to_num(Reserve_GPIOQ7)
#define name_gpioR	\
	gpio_name_to_num(Reserve_GPIOR0) \
	gpio_name_to_num(Reserve_GPIOR1) \
	gpio_name_to_num(Reserve_GPIOR2) \
	gpio_name_to_num(Reserve_GPIOR3) \
	gpio_name_to_num(Reserve_GPIOR4) \
	gpio_name_to_num(Reserve_GPIOR5) \
	gpio_name_to_num(Reserve_GPIOR6) \
	gpio_name_to_num(Reserve_GPIOR7)
// GPIOS3, S4, S5, S6, S7 hardware not define
#define name_gpioS	\
	gpio_name_to_num(Reserve_GPIOS0) \
	gpio_name_to_num(Reserve_GPIOS1) \
	gpio_name_to_num(Reserve_GPIOS2) \
	gpio_name_to_num(Reserve_GPIOS3) \
	gpio_name_to_num(Reserve_GPIOS4) \
	gpio_name_to_num(Reserve_GPIOS5) \
	gpio_name_to_num(Reserve_GPIOS6) \
	gpio_name_to_num(Reserve_GPIOS7)
// GPIOT input only
#define name_gpioT	\
	gpio_name_to_num(Reserve_GPIOT0) \
	gpio_name_to_num(Reserve_GPIOT1) \
	gpio_name_to_num(Reserve_GPIOT2) \
	gpio_name_to_num(Reserve_GPIOT3) \
	gpio_name_to_num(Reserve_GPIOT4) \
	gpio_name_to_num(Reserve_GPIOT5) \
	gpio_name_to_num(Reserve_GPIOT6) \
	gpio_name_to_num(Reserve_GPIOT7)
// GPIOU input only
#define name_gpioU	\
	gpio_name_to_num(Reserve_GPIOU0) \
	gpio_name_to_num(Reserve_GPIOU1) \
	gpio_name_to_num(Reserve_GPIOU2) \
	gpio_name_to_num(Reserve_GPIOU3) \
	gpio_name_to_num(Reserve_GPIOU4) \
	gpio_name_to_num(Reserve_GPIOU5) \
	gpio_name_to_num(Reserve_GPIOU6) \
	gpio_name_to_num(Reserve_GPIOU7)

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
