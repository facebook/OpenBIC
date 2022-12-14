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

// clang-format off

// gpio_cfg(chip, number, is_init, direction, status, int_type, int_callback)
// dedicate gpio A0~A7, B0~B7, C0~C7, D0~D7, E0~E7, total 40 gpios
// Default name: Reserve_GPIOH0
#define name_gpioA	\
	gpio_name_to_num(IRQ_P12V_E1S_3_FLT_N) \
	gpio_name_to_num(IRQ_P12V_E1S_2_FLT_N) \
	gpio_name_to_num(IRQ_P12V_E1S_1_FLT_N) \
	gpio_name_to_num(IRQ_P12V_E1S_0_FLT_N) \
	gpio_name_to_num(IRQ_P3V3_E1S_3_FLT_N) \
	gpio_name_to_num(IRQ_P3V3_E1S_2_FLT_N) \
	gpio_name_to_num(IRQ_P3V3_E1S_1_FLT_N) \
	gpio_name_to_num(IRQ_P3V3_E1S_0_FLT_N)
#define name_gpioB	\
	gpio_name_to_num(PWRGD_P12V_AUX) \
	gpio_name_to_num(IRQ_P12V_HSC_ALERT1_N) \
	gpio_name_to_num(IRQ_P12V_HSC_ALERT2_N) \
	gpio_name_to_num(PU_DB800_HI_BW) \
	gpio_name_to_num(LED_PWRGD_P12V_E1S_ALL) \
	gpio_name_to_num(FM_AUX_PWR_EN) \
	gpio_name_to_num(RST_SMB_E1S_3_N) \
	gpio_name_to_num(RST_SMB_E1S_2_N)
#define name_gpioC	\
	gpio_name_to_num(BOARD_ID0) \
	gpio_name_to_num(BOARD_ID1) \
	gpio_name_to_num(BOARD_ID2) \
	gpio_name_to_num(BOARD_ID3) \
	gpio_name_to_num(RST_BIC_E1S_3_N) \
	gpio_name_to_num(RST_BIC_E1S_2_N) \
	gpio_name_to_num(RST_BIC_E1S_1_N) \
	gpio_name_to_num(RST_BIC_E1S_0_N)
#define name_gpioD	\
	gpio_name_to_num(FM_MB_SLOT_ID0) \
	gpio_name_to_num(FM_PWRDIS_E1S_3) \
	gpio_name_to_num(FM_PWRDIS_E1S_2) \
	gpio_name_to_num(FM_PWRDIS_E1S_1) \
	gpio_name_to_num(FM_PWRDIS_E1S_0) \
	gpio_name_to_num(FM_P12V_E1S_0_EN) \
	gpio_name_to_num(IRQ_TMP75_ALERT_N) \
	gpio_name_to_num(FM_P3V3_E1S_0_SW_EN)
#define name_gpioE	\
	gpio_name_to_num(FM_POWER_EN) \
	gpio_name_to_num(PWRGD_EXP_PWROK) \
	gpio_name_to_num(RST_MB_N) \
	gpio_name_to_num(Reserve_GPIOE3) \
	gpio_name_to_num(FM_FRU_WC_N) \
	gpio_name_to_num(P1V2_VDD_PG_R) \
	gpio_name_to_num(RST_SMB_E1S_1_N) \
	gpio_name_to_num(RST_SMB_E1S_0_N)
#define name_gpioF	\
	gpio_name_to_num(Reserve_GPIOF0) \
	gpio_name_to_num(IRQ_SMB_ALERT_N) \
	gpio_name_to_num(FM_P3V3_E1S_EN) \
	gpio_name_to_num(FM_P12V_EDGE_EN) \
	gpio_name_to_num(Reserve_GPIOF4) \
	gpio_name_to_num(IRQ_P3V3_EDGE_FLT_N) \
	gpio_name_to_num(IRQ_P12V_EDGE_FLT_N) \
	gpio_name_to_num(Reserve_GPIOF7)
#define name_gpioG	\
	gpio_name_to_num(LED_BIC_E1S_3) \
	gpio_name_to_num(LED_BIC_E1S_2) \
	gpio_name_to_num(LED_BIC_E1S_1) \
	gpio_name_to_num(LED_BIC_E1S_0) \
	gpio_name_to_num(FM_MB_SLOT_ID1) \
	gpio_name_to_num(FM_PRSNT_E1S_3_N) \
	gpio_name_to_num(FM_PRSNT_E1S_2_N) \
	gpio_name_to_num(FM_PRSNT_E1S_1_N)
#define name_gpioH	\
	gpio_name_to_num(FM_PRSNT_E1S_0_N) \
	gpio_name_to_num(Reserve_GPIOH1) \
	gpio_name_to_num(Reserve_GPIOH2) \
	gpio_name_to_num(Reserve_GPIOH3) \
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
	gpio_name_to_num(FM_BOARD_REV_ID2) \
	gpio_name_to_num(FM_BOARD_REV_ID1) \
	gpio_name_to_num(FM_BOARD_REV_ID0) \
	gpio_name_to_num(HSC_SEL_ID2) \
	gpio_name_to_num(HSC_SEL_ID1) \
	gpio_name_to_num(HSC_SEL_ID0)
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
	gpio_name_to_num(FM_P12V_E1S_2_EN) \
	gpio_name_to_num(FM_P12V_E1S_1_EN) \
	gpio_name_to_num(IRQ_INA230_E1S_0_ALERT_N) \
	gpio_name_to_num(FM_CLKBUF_EN)
#define name_gpioO	\
	gpio_name_to_num(FM_P12V_E1S_3_EN) \
	gpio_name_to_num(IRQ_INA230_E1S_3_ALERT_N) \
	gpio_name_to_num(IRQ_INA230_E1S_2_ALERT_N) \
	gpio_name_to_num(IRQ_INA230_E1S_1_ALERT_N) \
	gpio_name_to_num(Reserve_GPIOO4) \
	gpio_name_to_num(Reserve_GPIOO5) \
	gpio_name_to_num(Reserve_GPIOO6) \
	gpio_name_to_num(Reserve_GPIOO7)
#define name_gpioP	\
	gpio_name_to_num(Reserve_GPIOP0) \
	gpio_name_to_num(Reserve_GPIOP1) \
	gpio_name_to_num(Reserve_GPIOP2) \
	gpio_name_to_num(Reserve_GPIOP3) \
	gpio_name_to_num(Reserve_GPIOP4) \
	gpio_name_to_num(Reserve_GPIOP5) \
	gpio_name_to_num(CLKBUF_E1S_3_OE_N) \
	gpio_name_to_num(CLKBUF_E1S_2_OE_N)
// GPIOQ5 hardware not define
#define name_gpioQ	\
	gpio_name_to_num(CLKBUF_E1S_1_OE_N) \
	gpio_name_to_num(CLKBUF_E1S_0_OE_N) \
	gpio_name_to_num(FM_P3V3_E1S_3_SW_EN) \
	gpio_name_to_num(FM_P3V3_E1S_2_SW_EN) \
	gpio_name_to_num(FM_P3V3_E1S_1_SW_EN) \
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
