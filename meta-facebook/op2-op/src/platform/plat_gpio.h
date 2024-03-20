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

#ifndef PLAT_GPIO_H
#define PLAT_GPIO_H

#include "hal_gpio.h"
#include "plat_class.h"

// gpio_cfg(chip, number, is_init, direction, status, int_type, int_callback)
// dedicate gpio A0~A7, B0~B7, C0~C7, D0~D7, E0~E7, total 40 gpios
// Default name: RESERVE_GPIOH0

// clang-format off

#define OPA_name_gpioA \
	gpio_name_to_num(OPA_CLKBUF_E1S_2_OE_N) \
	gpio_name_to_num(OPA_CLKBUF_E1S_1_OE_N) \
	gpio_name_to_num(OPA_CLKBUF_E1S_0_OE_N) \
	gpio_name_to_num(OPA_CLKBUF_RTM_OE_N) \
	gpio_name_to_num(OPA_CLKBUF_RISER_OE_N) \
	gpio_name_to_num(OPA_PWRGD_P12V_E1S_2_R) \
	gpio_name_to_num(OPA_PWRGD_P12V_E1S_1_R) \
	gpio_name_to_num(OPA_PWRGD_P12V_E1S_0_R)

#define OPA_name_gpioB \
	gpio_name_to_num(OPA_PWRGD_EXP_PWR) \
	gpio_name_to_num(OPA_RESERVE_GPIOB1) \
	gpio_name_to_num(OPA_E1S_2_PRSNT_N) \
	gpio_name_to_num(OPA_E1S_1_PRSNT_N) \
	gpio_name_to_num(OPA_E1S_0_PRSNT_N) \
	gpio_name_to_num(OPA_PWRGD_P3V3_STBY) \
	gpio_name_to_num(OPA_PWRGD_P1V2_STBY ) \
	gpio_name_to_num(OPA_SMB_TMP_SENSOR_ALT_N)

#define OPA_name_gpioC \
	gpio_name_to_num(OPA_PWRGD_P3V3_E1S_2_R) \
	gpio_name_to_num(OPA_PWRGD_P3V3_E1S_1_R) \
	gpio_name_to_num(OPA_PWRGD_P3V3_E1S_0_R) \
	gpio_name_to_num(OPA_PWRGD_P1V8_VR) \
	gpio_name_to_num(OPA_PWRGD_P0V9_VR) \
	gpio_name_to_num(OPA_BOARD_REV_0) \
	gpio_name_to_num(OPA_BOARD_REV_1) \
	gpio_name_to_num(OPA_BOARD_REV_2)

#define OPA_name_gpioD \
	gpio_name_to_num(OPA_PWRGD_P12V_MAIN) \
	gpio_name_to_num(OPA_RST_SMBRST_BIC_E1S_2_N) \
	gpio_name_to_num(OPA_RST_SMBRST_BIC_E1S_1_N) \
	gpio_name_to_num(OPA_RST_SMBRST_BIC_E1S_0_N) \
	gpio_name_to_num(OPA_RESET_BIC_RTM_N) \
	gpio_name_to_num(OPA_RST_USB_HUB_N) \
	gpio_name_to_num(OPA_PERST_BIC_RTM_N) \
	gpio_name_to_num(OPA_RESERVE_GPIOD7)

#define OPA_name_gpioE \
	gpio_name_to_num(OPA_PERST_E1S_2_N) \
	gpio_name_to_num(OPA_PERST_E1S_1_N) \
	gpio_name_to_num(OPA_PERST_E1S_0_N) \
	gpio_name_to_num(OPA_E1S_2_P12V_POWER_EN) \
	gpio_name_to_num(OPA_E1S_1_P12V_POWER_EN) \
	gpio_name_to_num(OPA_E1S_0_P12V_POWER_EN) \
	gpio_name_to_num(OPA_EN_P0V9_VR) \
	gpio_name_to_num(OPA_RESERVE_GPIOE7)

#define OPA_name_gpioF \
	gpio_name_to_num(OPA_SMB_PCIE_EXPB_ALERT_N) \
	gpio_name_to_num(OPA_RST_PCIE_EXP_PERST0_N) \
	gpio_name_to_num(OPA_CLKBUF_PWRDOWN_R_N) \
	gpio_name_to_num(OPA_FM_PWRDIS_E1S_2) \
	gpio_name_to_num(OPA_FM_PWRDIS_E1S_1) \
	gpio_name_to_num(OPA_FM_PWRDIS_E1S_0) \
	gpio_name_to_num(OPA_SMB_PCIE_EXP1_ALERT_N) \
	gpio_name_to_num(OPA_RESERVE_GPIOF7)

#define OPA_name_gpioG \
	gpio_name_to_num(OPA_SMB_E1S_2_INA233_ALT_N) \
	gpio_name_to_num(OPA_SMB_E1S_1_INA233_ALT_N) \
	gpio_name_to_num(OPA_SMB_E1S_0_INA233_ALT_N) \
	gpio_name_to_num(OPA_SMB_P12V_EDGE_INA233_ALT_N) \
	gpio_name_to_num(OPA_RTM_IOEXP_INT_N) \
	gpio_name_to_num(OPA_LED_E1S_2_ATTN_R) \
	gpio_name_to_num(OPA_LED_E1S_1_ATTN_R) \
	gpio_name_to_num(OPA_LED_E1S_0_ATTN_R)

#define OPA_name_gpioH \
	gpio_name_to_num(OPA_RESERVE_GPIOH0) \
	gpio_name_to_num(OPA_RESERVE_GPIOH1) \
	gpio_name_to_num(OPA_HUB1_MASTER_SELECT_R) \
	gpio_name_to_num(OPA_SMB_IOEXP_ALT_N)\
	gpio_name_to_num(OPA_RESERVE_GPIOH4) \
	gpio_name_to_num(OPA_RESERVE_GPIOH5) \
	gpio_name_to_num(OPA_RESERVE_GPIOH6) \
	gpio_name_to_num(OPA_RESERVE_GPIOH7)

#define OPA_name_gpioI \
	gpio_name_to_num(OPA_RESERVE_GPIOI0) \
	gpio_name_to_num(OPA_RESERVE_GPIOI1) \
	gpio_name_to_num(OPA_RESERVE_GPIOI2) \
	gpio_name_to_num(OPA_RESERVE_GPIOI3) \
	gpio_name_to_num(OPA_RESERVE_GPIOI4) \
	gpio_name_to_num(OPA_RESERVE_GPIOI5) \
	gpio_name_to_num(OPA_RESERVE_GPIOI6) \
	gpio_name_to_num(OPA_RESERVE_GPIOI7)

#define OPA_name_gpioJ \
	gpio_name_to_num(OPA_RESERVE_GPIOJ0) \
	gpio_name_to_num(OPA_RESERVE_GPIOJ1) \
	gpio_name_to_num(OPA_RESERVE_GPIOJ2) \
	gpio_name_to_num(OPA_RESERVE_GPIOJ3) \
	gpio_name_to_num(OPA_RESERVE_GPIOJ4) \
	gpio_name_to_num(OPA_RESERVE_GPIOJ5) \
	gpio_name_to_num(OPA_RESERVE_GPIOJ6) \
	gpio_name_to_num(OPA_RESERVE_GPIOJ7)

#define OPA_name_gpioK \
	gpio_name_to_num(OPA_RESERVE_GPIOK0) \
	gpio_name_to_num(OPA_RESERVE_GPIOK1) \
	gpio_name_to_num(OPA_RESERVE_GPIOK2) \
	gpio_name_to_num(OPA_RESERVE_GPIOK3) \
	gpio_name_to_num(OPA_RESERVE_GPIOK4) \
	gpio_name_to_num(OPA_RESERVE_GPIOK5) \
	gpio_name_to_num(OPA_RESERVE_GPIOK6) \
	gpio_name_to_num(OPA_RESERVE_GPIOK7)

#define OPA_name_gpioL \
	gpio_name_to_num(OPA_RESERVE_GPIOL0) \
	gpio_name_to_num(OPA_RESERVE_GPIOL1) \
	gpio_name_to_num(OPA_E1S_2_P3V3_POWER_EN) \
	gpio_name_to_num(OPA_E1S_1_P3V3_POWER_EN) \
	gpio_name_to_num(OPA_E1S_0_P3V3_POWER_EN) \
	gpio_name_to_num(OPA_RESERVE_GPIOL5) \
	gpio_name_to_num(OPA_RESERVE_GPIOL6) \
	gpio_name_to_num(OPA_RESERVE_GPIOL7)

#define OPA_name_gpioM \
	gpio_name_to_num(OPA_RESERVE_GPIOM0) \
	gpio_name_to_num(OPA_RESERVE_GPIOM1) \
	gpio_name_to_num(OPA_RESERVE_GPIOM2) \
	gpio_name_to_num(OPA_RESERVE_GPIOM3) \
	gpio_name_to_num(SMB_LS_MUX_EN) \
	gpio_name_to_num(SELECT_SMB_MUX_N) \
	gpio_name_to_num(OPA_RESERVE_GPIOM6) \
	gpio_name_to_num(OPA_RESERVE_GPIOM7)

#define OPA_name_gpioN \
	gpio_name_to_num(OPA_RESERVE_GPION0) \
	gpio_name_to_num(OPA_RESERVE_GPION1) \
	gpio_name_to_num(OPA_RESERVE_GPION2) \
	gpio_name_to_num(OPA_RESERVE_GPION3) \
	gpio_name_to_num(OPA_RESERVE_GPION4) \
	gpio_name_to_num(OPA_RESERVE_GPION5) \
	gpio_name_to_num(OPA_RESERVE_GPION6) \
	gpio_name_to_num(OPA_RESERVE_GPION7)

#define OPA_name_gpioO \
	gpio_name_to_num(OPA_RESERVE_GPIOO0) \
	gpio_name_to_num(OPA_RESERVE_GPIOO1) \
	gpio_name_to_num(OPA_RESERVE_GPIOO2) \
	gpio_name_to_num(OPA_RESERVE_GPIOO3) \
	gpio_name_to_num(OPA_RESERVE_GPIOO4) \
	gpio_name_to_num(OPA_RESERVE_GPIOO5) \
	gpio_name_to_num(OPA_RESERVE_GPIOO6) \
	gpio_name_to_num(OPA_RESERVE_GPIOO7)

#define OPA_name_gpioP \
	gpio_name_to_num(OPA_RESERVE_GPIOP0) \
	gpio_name_to_num(OPA_RESERVE_GPIOP1) \
	gpio_name_to_num(OPA_RESERVE_GPIOP2) \
	gpio_name_to_num(OPA_RESERVE_GPIOP3) \
	gpio_name_to_num(OPA_RESERVE_GPIOP4) \
	gpio_name_to_num(OPA_RESERVE_GPIOP5) \
	gpio_name_to_num(OPA_BIC_EXP_ID) \
	gpio_name_to_num(OPA_BIC_BOARD_ID)

#define OPA_name_gpioQ \
	gpio_name_to_num(OPA_FM_EXP_MAIN_PWR_EN) \
	gpio_name_to_num(OPA_PRSNT0_EXPB) \
	gpio_name_to_num(OPA_RESERVE_GPIOQ2) \
	gpio_name_to_num(OPA_RESERVE_GPIOQ3) \
	gpio_name_to_num(OPA_RESERVE_GPIOQ4) \
	gpio_name_to_num(OPA_RESERVE_GPIOQ5) \
	gpio_name_to_num(OPA_RESERVE_GPIOQ6) \
	gpio_name_to_num(OPA_RESERVE_GPIOQ7)

#define OPA_name_gpioR \
	gpio_name_to_num(OPA_RESERVE_GPIOR0) \
	gpio_name_to_num(OPA_RESERVE_GPIOR1) \
	gpio_name_to_num(OPA_RESERVE_GPIOR2) \
	gpio_name_to_num(OPA_RESERVE_GPIOR3) \
	gpio_name_to_num(OPA_RESERVE_GPIOR4) \
	gpio_name_to_num(OPA_RESERVE_GPIOR5) \
	gpio_name_to_num(OPA_RESERVE_GPIOR6) \
	gpio_name_to_num(OPA_RESERVE_GPIOR7)

#define OPA_name_gpioS \
	gpio_name_to_num(OPA_RESERVE_GPIOS0) \
	gpio_name_to_num(OPA_RESERVE_GPIOS1) \
	gpio_name_to_num(OPA_RESERVE_GPIOS2) \
	gpio_name_to_num(OPA_RESERVE_GPIOS3) \
	gpio_name_to_num(OPA_RESERVE_GPIOS4) \
	gpio_name_to_num(OPA_RESERVE_GPIOS5) \
	gpio_name_to_num(OPA_RESERVE_GPIOS6) \
	gpio_name_to_num(OPA_RESERVE_GPIOS7)

#define OPA_name_gpioT \
	gpio_name_to_num(OPA_RESERVE_GPIOT0) \
	gpio_name_to_num(OPA_RESERVE_GPIOT1) \
	gpio_name_to_num(OPA_RESERVE_GPIOT2) \
	gpio_name_to_num(OPA_RESERVE_GPIOT3) \
	gpio_name_to_num(OPA_RESERVE_GPIOT4) \
	gpio_name_to_num(OPA_RESERVE_GPIOT5) \
	gpio_name_to_num(OPA_RESERVE_GPIOT6) \
	gpio_name_to_num(OPA_RESERVE_GPIOT7)

#define OPA_name_gpioU \
	gpio_name_to_num(OPA_RESERVE_GPIOU0) \
	gpio_name_to_num(OPA_RESERVE_GPIOU1) \
	gpio_name_to_num(OPA_RESERVE_GPIOU2) \
	gpio_name_to_num(OPA_RESERVE_GPIOU3) \
	gpio_name_to_num(OPA_RESERVE_GPIOU4) \
	gpio_name_to_num(OPA_RESERVE_GPIOU5) \
	gpio_name_to_num(OPA_RESERVE_GPIOU6) \
	gpio_name_to_num(OPA_RESERVE_GPIOU7)

// For OPB BIC GPIO name
#define OPB_name_gpioA \
	gpio_name_to_num(OPB_CLKBUF_E1S_4_OE_N) \
	gpio_name_to_num(OPB_CLKBUF_E1S_3_OE_N) \
	gpio_name_to_num(OPB_CLKBUF_E1S_2_OE_N) \
	gpio_name_to_num(OPB_CLKBUF_E1S_1_OE_N) \
	gpio_name_to_num(OPB_CLKBUF_E1S_0_OE_N) \
	gpio_name_to_num(OPB_PWRGD_P12V_E1S_4_R) \
	gpio_name_to_num(OPB_PWRGD_P12V_E1S_3_R) \
	gpio_name_to_num(OPB_PWRGD_P12V_E1S_2_R)

#define OPB_name_gpioB \
	gpio_name_to_num(OPB_PWRGD_P12V_E1S_1_R) \
	gpio_name_to_num(OPB_PWRGD_P12V_E1S_0_R) \
	gpio_name_to_num(OPB_E1S_4_PRSNT_N) \
	gpio_name_to_num(OPB_E1S_3_PRSNT_N) \
	gpio_name_to_num(OPB_E1S_2_PRSNT_N) \
	gpio_name_to_num(OPB_E1S_1_PRSNT_N) \
	gpio_name_to_num(OPB_E1S_0_PRSNT_N ) \
	gpio_name_to_num(OPB_SMB_TEMP_SENSOR_ALT_N)

#define OPB_name_gpioC \
	gpio_name_to_num(OPB_PWRGD_P3V3_E1S_4_R) \
	gpio_name_to_num(OPB_PWRGD_P3V3_E1S_3_R) \
	gpio_name_to_num(OPB_PWRGD_P3V3_E1S_2_R) \
	gpio_name_to_num(OPB_PWRGD_P3V3_E1S_1_R) \
	gpio_name_to_num(OPB_PWRGD_P3V3_E1S_0_R) \
	gpio_name_to_num(OPB_BOARD_REV_0) \
	gpio_name_to_num(OPB_BOARD_REV_1) \
	gpio_name_to_num(OPB_BOARD_REV_2)

#define OPB_name_gpioD \
	gpio_name_to_num(OPB_PWRGD_P12V_MAIN) \
	gpio_name_to_num(OPB_RST_SMBRST_BIC_E1S_4) \
	gpio_name_to_num(OPB_RST_SMBRST_BIC_E1S_3) \
	gpio_name_to_num(OPB_RST_SMBRST_BIC_E1S_2) \
	gpio_name_to_num(OPB_RST_SMBRST_BIC_E1S_1) \
	gpio_name_to_num(OPB_RST_SMBRST_BIC_E1S_0) \
	gpio_name_to_num(OPB_RST_E1S_1_PERST) \
	gpio_name_to_num(OPB_RST_E1S_0_PERST)

#define OPB_name_gpioE \
	gpio_name_to_num(OPB_RST_E1S_4_PERST) \
	gpio_name_to_num(OPB_RST_E1S_3_PERST) \
	gpio_name_to_num(OPB_RST_E1S_2_PERST) \
	gpio_name_to_num(OPB_P12V_E1S_4_EN_R) \
	gpio_name_to_num(OPB_P12V_E1S_3_EN_R) \
	gpio_name_to_num(OPB_P12V_E1S_2_EN_R) \
	gpio_name_to_num(OPB_P12V_E1S_1_EN_R) \
	gpio_name_to_num(OPB_P12V_E1S_0_EN_R)

#define OPB_name_gpioF \
	gpio_name_to_num(OPB_SMB_P12V_MAIN_INA233_ALT_N) \
	gpio_name_to_num(OPB_PMB_P12V_MAIN_ALT_N) \
	gpio_name_to_num(OPB_CLKBUF_PWRDOWN_R_N) \
	gpio_name_to_num(OPB_FM_PWRDIS_E1S_4) \
	gpio_name_to_num(OPB_FM_PWRDIS_E1S_3) \
	gpio_name_to_num(OPB_FM_PWRDIS_E1S_2) \
	gpio_name_to_num(OPB_FM_PWRDIS_E1S_1) \
	gpio_name_to_num(OPB_FM_PWRDIS_E1S_0)

#define OPB_name_gpioG \
	gpio_name_to_num(OPB_SMB_E1S_4_INA233_ALT_N) \
	gpio_name_to_num(OPB_SMB_E1S_3_INA233_ALT_N) \
	gpio_name_to_num(OPB_SMB_E1S_2_INA233_ALT_N) \
	gpio_name_to_num(OPB_SMB_E1S_1_INA233_ALT_N) \
	gpio_name_to_num(OPB_SMB_E1S_0_INA233_ALT_N) \
	gpio_name_to_num(OPB_LED_E1S_4_ATTN_R) \
	gpio_name_to_num(OPB_LED_E1S_3_ATTN_R) \
	gpio_name_to_num(OPB_LED_E1S_2_ATTN_R)

#define OPB_name_gpioH \
	gpio_name_to_num(OPB_LED_E1S_1_ATTN_R) \
	gpio_name_to_num(OPB_LED_E1S_0_ATTN_R) \
	gpio_name_to_num(OPB_HUB1_MASTER_SELECT_R) \
	gpio_name_to_num(OPB_SMB_IOEXP_ALT_N) \
	gpio_name_to_num(OPB_RESERVE_GPIOH4) \
	gpio_name_to_num(OPB_RESERVE_GPIOH5) \
	gpio_name_to_num(OPB_RESERVE_GPIOH6) \
	gpio_name_to_num(OPB_RESERVE_GPIOH7)

#define OPB_name_gpioI \
	gpio_name_to_num(OPB_RESERVE_GPIOI0) \
	gpio_name_to_num(OPB_RESERVE_GPIOI1) \
	gpio_name_to_num(OPB_RESERVE_GPIOI2) \
	gpio_name_to_num(OPB_RESERVE_GPIOI3) \
	gpio_name_to_num(OPB_RESERVE_GPIOI4) \
	gpio_name_to_num(OPB_RESERVE_GPIOI5) \
	gpio_name_to_num(OPB_RESERVE_GPIOI6) \
	gpio_name_to_num(OPB_RESERVE_GPIOI7)

#define OPB_name_gpioJ \
	gpio_name_to_num(OPB_RESERVE_GPIOJ0) \
	gpio_name_to_num(OPB_RESERVE_GPIOJ1) \
	gpio_name_to_num(OPB_RESERVE_GPIOJ2) \
	gpio_name_to_num(OPB_RESERVE_GPIOJ3) \
	gpio_name_to_num(OPB_RESERVE_GPIOJ4) \
	gpio_name_to_num(OPB_RESERVE_GPIOJ5) \
	gpio_name_to_num(OPB_RESERVE_GPIOJ6) \
	gpio_name_to_num(OPB_RESERVE_GPIOJ7)

#define OPB_name_gpioK \
	gpio_name_to_num(OPB_RESERVE_GPIOK0) \
	gpio_name_to_num(OPB_RESERVE_GPIOK1) \
	gpio_name_to_num(OPB_RESERVE_GPIOK2) \
	gpio_name_to_num(OPB_RESERVE_GPIOK3) \
	gpio_name_to_num(OPB_RESERVE_GPIOK4) \
	gpio_name_to_num(OPB_RESERVE_GPIOK5) \
	gpio_name_to_num(OPB_RESERVE_GPIOK6) \
	gpio_name_to_num(OPB_RESERVE_GPIOK7)

#define OPB_name_gpioL \
	gpio_name_to_num(OPB_RESERVE_GPIOL0) \
	gpio_name_to_num(OPB_RESERVE_GPIOL1) \
	gpio_name_to_num(OPB_P3V3_E1S_4_EN_R) \
	gpio_name_to_num(OPB_P3V3_E1S_3_EN_R) \
	gpio_name_to_num(OPB_P3V3_E1S_2_EN_R) \
	gpio_name_to_num(OPB_P3V3_E1S_1_EN_R) \
	gpio_name_to_num(OPB_P3V3_E1S_0_EN_R) \
	gpio_name_to_num(OPB_RESERVE_GPIOL7)

#define OPB_name_gpioM \
	gpio_name_to_num(OPB_RESERVE_GPIOM0) \
	gpio_name_to_num(OPB_RESERVE_GPIOM1) \
	gpio_name_to_num(OPB_RESERVE_GPIOM2) \
	gpio_name_to_num(OPB_RESERVE_GPIOM3) \
	gpio_name_to_num(OPB_RESERVE_GPIOM4) \
	gpio_name_to_num(OPB_RESERVE_GPIOM5) \
	gpio_name_to_num(OPB_RESERVE_GPIOM6) \
	gpio_name_to_num(OPB_RESERVE_GPIOM7)

#define OPB_name_gpioN \
	gpio_name_to_num(OPB_RESERVE_GPION0) \
	gpio_name_to_num(OPB_RESERVE_GPION1) \
	gpio_name_to_num(OPB_RESERVE_GPION2) \
	gpio_name_to_num(OPB_RESERVE_GPION3) \
	gpio_name_to_num(OPB_RESERVE_GPION4) \
	gpio_name_to_num(OPB_RESERVE_GPION5) \
	gpio_name_to_num(OPB_RESERVE_GPION6) \
	gpio_name_to_num(OPB_RESERVE_GPION7)

#define OPB_name_gpioO \
	gpio_name_to_num(OPB_RESERVE_GPIOO0) \
	gpio_name_to_num(OPB_RESERVE_GPIOO1) \
	gpio_name_to_num(OPB_RESERVE_GPIOO2) \
	gpio_name_to_num(OPB_RESERVE_GPIOO3) \
	gpio_name_to_num(OPB_RESERVE_GPIOO4) \
	gpio_name_to_num(OPB_RESERVE_GPIOO5) \
	gpio_name_to_num(OPB_RESERVE_GPIOO6) \
	gpio_name_to_num(OPB_RESERVE_GPIOO7)

#define OPB_name_gpioP \
	gpio_name_to_num(OPB_RESERVE_GPIOP0) \
	gpio_name_to_num(OPB_RESERVE_GPIOP1) \
	gpio_name_to_num(OPB_RESERVE_GPIOP2) \
	gpio_name_to_num(OPB_RESERVE_GPIOP3) \
	gpio_name_to_num(OPB_RESERVE_GPIOP4) \
	gpio_name_to_num(OPB_RESERVE_GPIOP5) \
	gpio_name_to_num(OPB_BIC_EXP_ID) \
	gpio_name_to_num(OPB_BIC_BOARD_ID)

#define OPB_name_gpioQ \
	gpio_name_to_num(OPB_FM_EXP_MAIN_PWR_EN) \
	gpio_name_to_num(OPB_RESERVE_GPIOQ1) \
	gpio_name_to_num(OPB_BIC_MAIN_PWR_EN_R) \
	gpio_name_to_num(OPB_SMB_BIC_ALERT_N_R) \
	gpio_name_to_num(OPB_RST_CPLD_PERST1_N) \
	gpio_name_to_num(OPB_RESERVE_GPIOQ5) \
	gpio_name_to_num(OPB_RESERVE_GPIOQ6) \
	gpio_name_to_num(OPB_RESERVE_GPIOQ7)

#define OPB_name_gpioR \
	gpio_name_to_num(OPB_RESERVE_GPIOR0) \
	gpio_name_to_num(OPB_RESERVE_GPIOR1) \
	gpio_name_to_num(OPB_RESERVE_GPIOR2) \
	gpio_name_to_num(OPB_RESERVE_GPIOR3) \
	gpio_name_to_num(OPB_RESERVE_GPIOR4) \
	gpio_name_to_num(OPB_RESERVE_GPIOR5) \
	gpio_name_to_num(OPB_RESERVE_GPIOR6) \
	gpio_name_to_num(OPB_RESERVE_GPIOR7)

#define OPB_name_gpioS \
	gpio_name_to_num(OPB_RESERVE_GPIOS0) \
	gpio_name_to_num(OPB_RESERVE_GPIOS1) \
	gpio_name_to_num(OPB_RESERVE_GPIOS2) \
	gpio_name_to_num(OPB_RESERVE_GPIOS3) \
	gpio_name_to_num(OPB_RESERVE_GPIOS4) \
	gpio_name_to_num(OPB_RESERVE_GPIOS5) \
	gpio_name_to_num(OPB_RESERVE_GPIOS6) \
	gpio_name_to_num(OPB_RESERVE_GPIOS7)

#define OPB_name_gpioT \
	gpio_name_to_num(OPB_RESERVE_GPIOT0) \
	gpio_name_to_num(OPB_RESERVE_GPIOT1) \
	gpio_name_to_num(OPB_RESERVE_GPIOT2) \
	gpio_name_to_num(OPB_RESERVE_GPIOT3) \
	gpio_name_to_num(OPB_RESERVE_GPIOT4) \
	gpio_name_to_num(OPB_RESERVE_GPIOT5) \
	gpio_name_to_num(OPB_RESERVE_GPIOT6) \
	gpio_name_to_num(OPB_RESERVE_GPIOT7)

#define OPB_name_gpioU \
	gpio_name_to_num(OPB_RESERVE_GPIOU0) \
	gpio_name_to_num(OPB_RESERVE_GPIOU1) \
	gpio_name_to_num(OPB_RESERVE_GPIOU2) \
	gpio_name_to_num(OPB_RESERVE_GPIOU3) \
	gpio_name_to_num(OPB_RESERVE_GPIOU4) \
	gpio_name_to_num(OPB_RESERVE_GPIOU5) \
	gpio_name_to_num(OPB_RESERVE_GPIOU6) \
	gpio_name_to_num(OPB_RESERVE_GPIOU7)

// clang-format on

#define gpio_name_to_num(x) x,

enum _OPA_GPIO_NUMS_ {
	OPA_name_gpioA OPA_name_gpioB OPA_name_gpioC OPA_name_gpioD OPA_name_gpioE OPA_name_gpioF
		OPA_name_gpioG OPA_name_gpioH OPA_name_gpioI OPA_name_gpioJ OPA_name_gpioK
			OPA_name_gpioL OPA_name_gpioM OPA_name_gpioN OPA_name_gpioO OPA_name_gpioP
				OPA_name_gpioQ OPA_name_gpioR OPA_name_gpioS OPA_name_gpioT
					OPA_name_gpioU
};

enum _OPB_GPIO_NUMS_ {
	OPB_name_gpioA OPB_name_gpioB OPB_name_gpioC OPB_name_gpioD OPB_name_gpioE OPB_name_gpioF
		OPB_name_gpioG OPB_name_gpioH OPB_name_gpioI OPB_name_gpioJ OPB_name_gpioK
			OPB_name_gpioL OPB_name_gpioM OPB_name_gpioN OPB_name_gpioO OPB_name_gpioP
				OPB_name_gpioQ OPB_name_gpioR OPB_name_gpioS OPB_name_gpioT
					OPB_name_gpioU
};

// define  opa opb same pin
#define BIC_EXP_ID OPA_BIC_EXP_ID
#define BIC_BOARD_ID OPA_BIC_BOARD_ID
#define FM_EXP_MAIN_PWR_EN OPA_FM_EXP_MAIN_PWR_EN
#define PWRGD_P12V_MAIN OPA_PWRGD_P12V_MAIN

extern enum _OPA_GPIO_NUMS_ OPA_GPIO_NUMS;
extern enum _OPB_GPIO_NUMS_ OPB_GPIO_NUMS;
#undef gpio_name_to_num

void init_card_position_gpio();

#endif
