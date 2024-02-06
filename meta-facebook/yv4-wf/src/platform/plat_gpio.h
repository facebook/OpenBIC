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

#define POC_Reserve_GPIOF2 42
#define POC_Reserve_GPIOF3 43
#define POC_EN_P3V3_E1S_0_R 48
#define POC_PWRGD_P3V3_E1S_0_R 49

// clang-format off
#define name_gpioA \
	gpio_name_to_num(PWR_ON_RST_ASIC1_N) \
	gpio_name_to_num(SYS_RST_ASIC1_N_R) \
	gpio_name_to_num(PERST_ASIC1_N_R) \
	gpio_name_to_num(PERST_ASIC2_N_R) \
	gpio_name_to_num(RST_PCIE_MB_EXP_N) \
	gpio_name_to_num(BIC_PWR_EN_MCIO) \
	gpio_name_to_num(PG_CARD_OK) \
	gpio_name_to_num(FM_POWER_EN_R)

#define name_gpioB \
	gpio_name_to_num(PWRGD_AUX_R) \
	gpio_name_to_num(BIC_READY_R) \
	gpio_name_to_num(EN_P0V85_BIC_ASIC2_R) \
	gpio_name_to_num(PWRGD_P0V85_ASIC2) \
	gpio_name_to_num(EN_P1V2_BIC_ASIC2_R) \
	gpio_name_to_num(PWRGD_P1V2_ASIC2) \
	gpio_name_to_num(EN_P1V8_BIC_ASIC2_R) \
	gpio_name_to_num(PWRGD_P1V8_ASIC2)

#define name_gpioC \
	gpio_name_to_num(EN_P0V85_BIC_ASIC1_R) \
	gpio_name_to_num(PWRGD_P0V85_ASIC1) \
	gpio_name_to_num(EN_P1V2_BIC_ASIC1_R) \
	gpio_name_to_num(PWRGD_P1V2_ASIC1) \
	gpio_name_to_num(EN_P1V8_BIC_ASIC1_R) \
	gpio_name_to_num(PWRGD_P1V8_ASIC1) \
	gpio_name_to_num(EN_P0V8_BIC_ASIC1_R) \
	gpio_name_to_num(PWRGD_P0V8_ASIC1)

#define name_gpioD \
	gpio_name_to_num(EN_PVPP_AB_ASIC1_2V5_R) \
	gpio_name_to_num(PWRGD_PVPP_AB_ASIC1) \
	gpio_name_to_num(EN_PVPP_CD_ASIC1_2V5_R) \
	gpio_name_to_num(PWRGD_PVPP_CD_ASIC1) \
	gpio_name_to_num(EN_PVTT_AB_ASIC1_0V6_R) \
	gpio_name_to_num(PWRGD_PVTT_AB_ASIC1) \
	gpio_name_to_num(EN_PVTT_CD_ASIC1_0V6_R) \
	gpio_name_to_num(PWRGD_PVTT_CD_ASIC1)

#define name_gpioE \
	gpio_name_to_num(EN_P0V8_BIC_ASIC2_R) \
	gpio_name_to_num(PWRGD_P0V8_ASIC2) \
	gpio_name_to_num(PWR_ON_RST_ASIC2_N) \
	gpio_name_to_num(SYS_RST_ASIC2_N_R) \
	gpio_name_to_num(EN_PVPP_AB_ASIC2_2V5_R) \
	gpio_name_to_num(PWRGD_PVPP_AB_ASIC2) \
	gpio_name_to_num(EN_PVPP_CD_ASIC2_2V5_R) \
	gpio_name_to_num(PWRGD_PVPP_CD_ASIC2)

#define name_gpioF \
	gpio_name_to_num(PWRGD_PVDDQ_CD_ASIC2) \
	gpio_name_to_num(PWRGD_P0V75_ASIC2) \
	gpio_name_to_num(EN_P3V3_E1S_0_R) \
	gpio_name_to_num(PWRGD_P3V3_E1S_0_R) \
	gpio_name_to_num(EN_PVTT_AB_ASIC2_0V6_R) \
	gpio_name_to_num(PWRGD_PVTT_AB_ASIC2) \
	gpio_name_to_num(EN_PVTT_CD_ASIC2_0V6_R) \
	gpio_name_to_num(PWRGD_PVTT_CD_ASIC2)

#define name_gpioG \
	gpio_name_to_num(LED_ASIC1_LS_HB) \
	gpio_name_to_num(LED_ASIC2_LS_HB) \
	gpio_name_to_num(EN_P12V_E1S_0_R) \
	gpio_name_to_num(PWRGD_P12V_E1S_0_R) \
	gpio_name_to_num(EN_PVDDQ_AB_ASIC1_R) \
	gpio_name_to_num(PWRGD_PVDDQ_AB_ASIC1) \
	gpio_name_to_num(EN_PVDDQ_CD_ASIC1_R) \
	gpio_name_to_num(PWRGD_PVDDQ_CD_ASIC1)

#define name_gpioH \
	gpio_name_to_num(EN_P0V75_BIC_ASIC1_R) \
	gpio_name_to_num(PWRGD_P0V75_ASIC1) \
	gpio_name_to_num(EN_P5V_STBY_BIC_R) \
	gpio_name_to_num(PWRGD_P5V_STBY_BIC_R) \
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
	gpio_name_to_num(Reserve_GPIOL2) \
	gpio_name_to_num(Reserve_GPIOL3) \
	gpio_name_to_num(PWRGD_P1V2_STBY) \
	gpio_name_to_num(EN_PVDDQ_AB_ASIC2_R) \
	gpio_name_to_num(PWRGD_PVDDQ_AB_ASIC2) \
	gpio_name_to_num(EN_PVDDQ_CD_ASIC2_R)

#define name_gpioM \
	gpio_name_to_num(EN_SPI_BIC_ASIC2_SHIFT_R) \
	gpio_name_to_num(EN_P0V75_BIC_ASIC2_R) \
	gpio_name_to_num(EN_SPI_BIC_ASIC1_SHIFT_R) \
	gpio_name_to_num(FM_PWRBRK_PRIMARY_R_N) \
	gpio_name_to_num(Reserve_GPIOM4) \
	gpio_name_to_num(Reserve_GPIOM5) \
	gpio_name_to_num(Reserve_GPIOM6) \
	gpio_name_to_num(Reserve_GPIOM7)

#define name_gpioN \
	gpio_name_to_num(Reserve_GPION0) \
	gpio_name_to_num(EN_CLK_100M_ASIC1_OSC) \
	gpio_name_to_num(Reserve_GPION2) \
	gpio_name_to_num(EN_CLK_100M_ASIC2_OSC) \
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

#define name_gpioS \
	gpio_name_to_num(Reserve_GPIOS0) \
	gpio_name_to_num(Reserve_GPIOS1) \
	gpio_name_to_num(Reserve_GPIOS2) \
	gpio_name_to_num(Reserve_GPIOS3) \
	gpio_name_to_num(Reserve_GPIOS4) \
	gpio_name_to_num(Reserve_GPIOS5) \
	gpio_name_to_num(Reserve_GPIOS6) \
	gpio_name_to_num(Reserve_GPIOS7)

#define name_gpioT \
	gpio_name_to_num(Reserve_GPIOT0) \
	gpio_name_to_num(Reserve_GPIOT1) \
	gpio_name_to_num(Reserve_GPIOT2) \
	gpio_name_to_num(Reserve_GPIOT3) \
	gpio_name_to_num(Reserve_GPIOT4) \
	gpio_name_to_num(Reserve_GPIOT5) \
	gpio_name_to_num(Reserve_GPIOT6) \
	gpio_name_to_num(Reserve_GPIOT7)

#define name_gpioU \
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

/*IO expander cofiguration*/

enum IOE_PIN_NUM {
	IOE_P00 = 0,
	IOE_P01 = 1,
	IOE_P02 = 2,
	IOE_P03 = 3,
	IOE_P04 = 4,
	IOE_P05 = 5,
	IOE_P06 = 6,
	IOE_P07 = 7,
	IOE_P10 = 0,
	IOE_P11 = 1,
	IOE_P12 = 2,
	IOE_P13 = 3,
	IOE_P14 = 4,
	IOE_P15 = 5,
	IOE_P16 = 6,
	IOE_P17 = 7,
};

int get_ioe_value(uint8_t ioe_addr, uint8_t ioe_reg, uint8_t *value);
int set_ioe_value(uint8_t ioe_addr, uint8_t ioe_reg, uint8_t value);
void init_ioe_config();
int check_ioe4_e1s_prsnt_pin();

#endif
