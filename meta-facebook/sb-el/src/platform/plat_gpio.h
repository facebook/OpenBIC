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
	gpio_name_to_num(RST_ARKE_PWR_ON_PLD_R1_N) \
	gpio_name_to_num(HAMSA_SW_EN) \
	gpio_name_to_num(NUWA0_SW_EN) \
	gpio_name_to_num(FM_OWL_E_JTAG_MUX_SEL_00) \
	gpio_name_to_num(ALL_VR_PM_ALERT_R_N) \
	gpio_name_to_num(SMB_HAMSA_MMC_LVC33_ALERT_N) \
	gpio_name_to_num(FM_PLD_UBC_EN_R)
#define name_gpio1	\
	gpio_name_to_num(Reserve_GPIO10) \
	gpio_name_to_num(Reserve_GPIO11) \
	gpio_name_to_num(Reserve_GPIO12) \
	gpio_name_to_num(Reserve_GPIO13) \
	gpio_name_to_num(Reserve_GPIO14) \
	gpio_name_to_num(Reserve_GPIO15) \
	gpio_name_to_num(NUWA1_CHIP_STRAP1_MMC) \
	gpio_name_to_num(Reserve_GPIO17)
#define name_gpio2	\
	gpio_name_to_num(FM_OWL_E_JTAG_MUX_SEL_02) \
	gpio_name_to_num(Reserve_GPIO21) \
	gpio_name_to_num(Reserve_GPIO22) \
	gpio_name_to_num(Reserve_GPIO23) \
	gpio_name_to_num(Reserve_GPIO24) \
	gpio_name_to_num(Reserve_GPIO25) \
	gpio_name_to_num(NUWA1_CHIP_STRAP0_MMC) \
	gpio_name_to_num(NUWA0_HBM_CATTRIP_MMC_LVC33_R_ALARM)
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
	gpio_name_to_num(SPI_NUWA0_MUX_IN1) \
	gpio_name_to_num(SPI_NUWA1_MUX_IN1) \
	gpio_name_to_num(SPI_HAMSA_MUX_IN1) \
	gpio_name_to_num(QSPI_CPLD_SEL_0) \
	gpio_name_to_num(Reserve_GPIO54) \
	gpio_name_to_num(Reserve_GPIO55) \
	gpio_name_to_num(Reserve_GPIO56) \
	gpio_name_to_num(NUWA1_HBM_CATTRIP_MMC_LVC33_R_ALARM)
#define name_gpio6	\
	gpio_name_to_num(Reserve_GPIO60) \
	gpio_name_to_num(Reserve_GPIO61) \
	gpio_name_to_num(QSPI_CPLD_SEL_1) \
	gpio_name_to_num(NC_SPI_NUWA0_CRM_MUX_IN1) \
	gpio_name_to_num(NC_SPI_NUWA1_CRM_MUX_IN1) \
	gpio_name_to_num(Reserve_GPIO65) \
	gpio_name_to_num(Reserve_GPIO66) \
	gpio_name_to_num(Reserve_GPIO67)
#define name_gpio7	\
	gpio_name_to_num(Reserve_GPIO70) \
	gpio_name_to_num(NC_SPI_HAMSA_CRM_MUX_IN1) \
	gpio_name_to_num(NUWA1_SW_EN) \
	gpio_name_to_num(SPI_ADC_CS0_N) \
	gpio_name_to_num(GPIO74_STRAP_DSW_EN) \
	gpio_name_to_num(GPIO75_STRAP_JEN_N) \
	gpio_name_to_num(Reserve_GPIO76) \
	gpio_name_to_num(Reserve_GPIO77)
#define name_gpio8	\
	gpio_name_to_num(Reserve_GPIO80) \
	gpio_name_to_num(I3C_ELECTRA_ALERT_R_N) \
	gpio_name_to_num(LED_MMC_HEARTBEAT_R) \
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
	gpio_name_to_num(NUWA0_PWR_CAP_LV1_LVC33) \
	gpio_name_to_num(Reserve_GPIOA1) \
	gpio_name_to_num(Reserve_GPIOA2) \
	gpio_name_to_num(NUWA1_PWR_CAP_LV1_LVC33) \
	gpio_name_to_num(Reserve_GPIOA4) \
	gpio_name_to_num(Reserve_GPIOA5) \
	gpio_name_to_num(Reserve_GPIOA6) \
	gpio_name_to_num(Reserve_GPIOA7)
#define name_gpioB	\
	gpio_name_to_num(RSVD_GPIO_1) \
	gpio_name_to_num(Reserve_GPIOB1) \
	gpio_name_to_num(NUWA0_CHIP_STRAP1_MMC) \
	gpio_name_to_num(FM_OWL_E_JTAG_MUX_SEL_01) \
	gpio_name_to_num(Reserve_GPIOB4) \
	gpio_name_to_num(Reserve_GPIOB5) \
	gpio_name_to_num(NUWA0_CHIP_STRAP0_MMC) \
	gpio_name_to_num(Reserve_GPIOB7)
#define name_gpioC	\
	gpio_name_to_num(Reserve_GPIOC0) \
	gpio_name_to_num(SPI_ADC_CS1_N) \
	gpio_name_to_num(Reserve_GPIOC2) \
	gpio_name_to_num(Reserve_GPIOC3) \
	gpio_name_to_num(NUWA0_CNV) \
	gpio_name_to_num(NUWA1_CNV) \
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
	gpio_name_to_num(SMB_NUWA0_CRM_MMC_LVC33_ALERT_N) \
	gpio_name_to_num(SMB_NUWA1_CRM_MMC_LVC33_ALERT_N)

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
