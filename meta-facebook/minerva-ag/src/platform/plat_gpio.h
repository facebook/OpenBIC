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
#define name_gpio0	\
	gpio_name_to_num(FM_ASIC_0_THERMTRIP_R_N) \
	gpio_name_to_num(RST_ATH_PWR_ON_PLD_R1_N) \
	gpio_name_to_num(ATH_CURRENT_SENSE_0_LC) \
	gpio_name_to_num(ATH_CURRENT_SENSE_1_HC) \
	gpio_name_to_num(FM_ATH_HBM3_CATTRIP_ALARM_LV33_R) \
	gpio_name_to_num(ALL_VR_PM_ALERT_R_N) \
	gpio_name_to_num(ATH_SMB_ALERT_NPCM_LVC33_R_N) \
	gpio_name_to_num(FM_PLD_UBC_EN_R)
#define name_gpio1	\
	gpio_name_to_num(RSVD_GPIO_2_R) \
	gpio_name_to_num(Reserve_GPIO11) \
	gpio_name_to_num(Reserve_GPIO12) \
	gpio_name_to_num(Reserve_GPIO13) \
	gpio_name_to_num(Reserve_GPIO14) \
	gpio_name_to_num(Reserve_GPIO15) \
	gpio_name_to_num(Reserve_GPIO16) \
	gpio_name_to_num(Reserve_GPIO17)
#define name_gpio2	\
	gpio_name_to_num(Reserve_GPIO20) \
	gpio_name_to_num(Reserve_GPIO21) \
	gpio_name_to_num(Reserve_GPIO22) \
	gpio_name_to_num(Reserve_GPIO23) \
	gpio_name_to_num(Reserve_GPIO24) \
	gpio_name_to_num(Reserve_GPIO25) \
	gpio_name_to_num(Reserve_GPIO26) \
	gpio_name_to_num(Reserve_GPIO27)
#define name_gpio3	\
	gpio_name_to_num(Reserve_GPIO30) \
	gpio_name_to_num(Reserve_GPIO31) \
	gpio_name_to_num(Reserve_GPIO32) \
	gpio_name_to_num(Reserve_GPIO33) \
	gpio_name_to_num(Reserve_GPIO34) \
	gpio_name_to_num(Reserve_GPIO35) \
	gpio_name_to_num(Reserve_GPIO36) \
	gpio_name_to_num(Reserve_GPIO37)
#define name_gpio4	\
	gpio_name_to_num(Reserve_GPIO40) \
	gpio_name_to_num(Reserve_GPIO41) \
	gpio_name_to_num(Reserve_GPIO42) \
	gpio_name_to_num(Reserve_GPIO43) \
	gpio_name_to_num(Reserve_GPIO44) \
	gpio_name_to_num(Reserve_GPIO45) \
	gpio_name_to_num(Reserve_GPIO46) \
	gpio_name_to_num(Reserve_GPIO47)
#define name_gpio5	\
	gpio_name_to_num(Reserve_GPIO50) \
	gpio_name_to_num(Reserve_GPIO51) \
	gpio_name_to_num(Reserve_GPIO52) \
	gpio_name_to_num(Reserve_GPIO53) \
	gpio_name_to_num(Reserve_GPIO54) \
	gpio_name_to_num(Reserve_GPIO55) \
	gpio_name_to_num(Reserve_GPIO56) \
	gpio_name_to_num(Reserve_GPIO57)
#define name_gpio6	\
	gpio_name_to_num(Reserve_GPIO60) \
	gpio_name_to_num(Reserve_GPIO61) \
	gpio_name_to_num(Reserve_GPIO62) \
	gpio_name_to_num(Reserve_GPIO63) \
	gpio_name_to_num(Reserve_GPIO64) \
	gpio_name_to_num(Reserve_GPIO65) \
	gpio_name_to_num(Reserve_GPIO66) \
	gpio_name_to_num(Reserve_GPIO67)
#define name_gpio7	\
	gpio_name_to_num(Reserve_GPIO70) \
	gpio_name_to_num(Reserve_GPIO71) \
	gpio_name_to_num(Reserve_GPIO72) \
	gpio_name_to_num(Reserve_GPIO73) \
	gpio_name_to_num(GPIO74_Strap_DSW_EN) \
	gpio_name_to_num(GPIO75_Strap_JEN_N) \
	gpio_name_to_num(Reserve_GPIO76) \
	gpio_name_to_num(Reserve_GPIO77)
#define name_gpio8	\
	gpio_name_to_num(Reserve_GPIO80) \
	gpio_name_to_num(Reserve_GPIO81) \
	gpio_name_to_num(LED_NPCM_HEARTBEAT_R) \
	gpio_name_to_num(Reserve_GPIO83) \
	gpio_name_to_num(Reserve_GPIO84) \
	gpio_name_to_num(Reserve_GPIO85) \
	gpio_name_to_num(Reserve_GPIO86) \
	gpio_name_to_num(Reserve_GPIO87)
#define name_gpio9	\
	gpio_name_to_num(Reserve_GPIO90) \
	gpio_name_to_num(Reserve_GPIO91) \
	gpio_name_to_num(Reserve_GPIO92) \
	gpio_name_to_num(Reserve_GPIO93) \
	gpio_name_to_num(Reserve_GPIO94) \
	gpio_name_to_num(Reserve_GPIO95) \
	gpio_name_to_num(Reserve_GPIO96) \
	gpio_name_to_num(Reserve_GPIO97)
#define name_gpioA	\
	gpio_name_to_num(Reserve_GPIOA0) \
	gpio_name_to_num(Reserve_GPIOA1) \
	gpio_name_to_num(Reserve_GPIOA2) \
	gpio_name_to_num(Reserve_GPIOA3) \
	gpio_name_to_num(Reserve_GPIOA4) \
	gpio_name_to_num(Reserve_GPIOA5) \
	gpio_name_to_num(Reserve_GPIOA6) \
	gpio_name_to_num(Reserve_GPIOA7)
#define name_gpioB	\
	gpio_name_to_num(RSVD_GPIO_1_R) \
	gpio_name_to_num(Reserve_GPIOB1) \
	gpio_name_to_num(Reserve_GPIOB2) \
	gpio_name_to_num(Reserve_GPIOB3) \
	gpio_name_to_num(Reserve_GPIOB4) \
	gpio_name_to_num(Reserve_GPIOB5) \
	gpio_name_to_num(Reserve_GPIOB6) \
	gpio_name_to_num(Reserve_GPIOB7)
#define name_gpioC	\
	gpio_name_to_num(Reserve_GPIOC0) \
	gpio_name_to_num(Reserve_GPIOC1) \
	gpio_name_to_num(Reserve_GPIOC2) \
	gpio_name_to_num(Reserve_GPIOC3) \
	gpio_name_to_num(Reserve_GPIOC4) \
	gpio_name_to_num(Reserve_GPIOC5) \
	gpio_name_to_num(Reserve_GPIOC6) \
	gpio_name_to_num(Reserve_GPIOC7)
#define name_gpioD	\
	gpio_name_to_num(Reserve_GPIOD0) \
	gpio_name_to_num(Reserve_GPIOD1) \
	gpio_name_to_num(Reserve_GPIOD2) \
	gpio_name_to_num(Reserve_GPIOD3) \
	gpio_name_to_num(Reserve_GPIOD4) \
	gpio_name_to_num(Reserve_GPIOD5) \
	gpio_name_to_num(Reserve_GPIOD6) \
	gpio_name_to_num(Reserve_GPIOD7)
#define name_gpioE	\
	gpio_name_to_num(Reserve_GPIOE0) \
	gpio_name_to_num(Reserve_GPIOE1) \
	gpio_name_to_num(Reserve_GPIOE2) \
	gpio_name_to_num(Reserve_GPIOE3) \
	gpio_name_to_num(Reserve_GPIOE4) \
	gpio_name_to_num(Reserve_GPIOE5) \
	gpio_name_to_num(Reserve_GPIOE6) \
	gpio_name_to_num(Reserve_GPIOE7)
#define name_gpioF	\
	gpio_name_to_num(Reserve_GPIOF0) \
	gpio_name_to_num(Reserve_GPIOF1) \
	gpio_name_to_num(Reserve_GPIOF2) \
	gpio_name_to_num(Reserve_GPIOF3)

// clang-format on

#define gpio_name_to_num(x) x,
enum _GPIO_NUMS_ {
	name_gpio0 name_gpio1 name_gpio2 name_gpio3 name_gpio4 name_gpio5 name_gpio6 name_gpio7
		name_gpio8 name_gpio9 name_gpioA name_gpioB name_gpioC name_gpioD name_gpioE
			name_gpioF
};

extern enum _GPIO_NUMS_ GPIO_NUMS;
#undef gpio_name_to_num

extern char *gpio_name[];

#endif
