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

// clang-format off

#define name_gpioA \
    gpio_name_to_num(PWR_ON_RST_N_R) \
    gpio_name_to_num(SYS_RST_N_R) \
    gpio_name_to_num(PERST_ASIC_N_R) \
    gpio_name_to_num(Reserve_GPIOA3) \
    gpio_name_to_num(RST_PCIE_MB_EXP_N) \
    gpio_name_to_num(BIC_PWR_EN_4C) \
    gpio_name_to_num(PG_CARD_OK) \
    gpio_name_to_num(POWER_EN_R)

#define name_gpioB \
    gpio_name_to_num(PAD_CORE_TAP_CTRL_L_R) \
    gpio_name_to_num(IOE_RST_N) \
    gpio_name_to_num(INT_IOE_R_N) \
    gpio_name_to_num(BIC_SECUREBOOT) \
    gpio_name_to_num(SMBUS_ALERT_R_N) \
    gpio_name_to_num(FM_PWRBRK_PRIMARY_R_N) \
    gpio_name_to_num(BIC_ESPI_SELECT) \
    gpio_name_to_num(SPI_RST_FLASH_N_R)

#define name_gpioC \
    gpio_name_to_num(P0V85_BIC_ASIC_EN) \
    gpio_name_to_num(PWRGD_P0V85_ASIC) \
    gpio_name_to_num(P1V2_BIC_ASIC_EN) \
    gpio_name_to_num(PWRGD_P1V2_ASIC) \
    gpio_name_to_num(P1V8_BIC_ASIC_EN) \
    gpio_name_to_num(PWRGD_P1V8_ASIC) \
    gpio_name_to_num(P0V8_BIC_ASIC_EN) \
    gpio_name_to_num(PWRGD_P0V8_ASIC)

#define name_gpioD \
    gpio_name_to_num(PVPP_AB_BIC_DIMM_EN) \
    gpio_name_to_num(PWRGD_PVPP_AB) \
    gpio_name_to_num(PVPP_CD_BIC_DIMM_EN) \
    gpio_name_to_num(PWRGD_PVPP_CD) \
    gpio_name_to_num(PVTT_AB_BIC_DIMM_EN) \
    gpio_name_to_num(PWRGD_PVTT_AB) \
    gpio_name_to_num(PVTT_CD_BIC_DIMM_EN) \
    gpio_name_to_num(PWRGD_PVTT_CD)

#define name_gpioE \
    gpio_name_to_num(ALT_SMB_BIC_PMIC_R_N) \
    gpio_name_to_num(ALT_TMPS_N) \
    gpio_name_to_num(ALT_INA_R_N) \
    gpio_name_to_num(USB_MUX1_SEL_R) \
    gpio_name_to_num(USB_MUX2_SEL_R) \
    gpio_name_to_num(SEL_SMB_MUX_PMIC_R) \
    gpio_name_to_num(SEL_SMB_MUX_DIMM_R) \
    gpio_name_to_num(SEL_SPI_MUX_R)

#define name_gpioF \
    gpio_name_to_num(SLOT_ID0_R) \
    gpio_name_to_num(SLOT_ID1_R) \
    gpio_name_to_num(BOARD_ID0) \
    gpio_name_to_num(BOARD_ID1) \
    gpio_name_to_num(BOARD_ID2) \
    gpio_name_to_num(BOARD_ID3) \
    gpio_name_to_num(LED_CXL_FAULT) \
    gpio_name_to_num(LED_CXL_POWER)

#define name_gpioG \
    gpio_name_to_num(PWRGD_AUX_R) \
    gpio_name_to_num(PVDDQ_AB_EN_BIC) \
    gpio_name_to_num(PWRGD_PVDDQ_AB_BIC) \
    gpio_name_to_num(PVDDQ_CD_EN_BIC) \
    gpio_name_to_num(PWRGD_PVDDQ_CD_BIC) \
    gpio_name_to_num(P0V75_ASIC_EN_BIC) \
    gpio_name_to_num(PWRGD_P0V75_ASIC_BIC) \
    gpio_name_to_num(P5V_STBY_EN_BIC)

#define name_gpioH \
    gpio_name_to_num(PWRGD_P5V_STBY_BIC) \
    gpio_name_to_num(LED_ASIC_R_HB) \
    gpio_name_to_num(BIC_READY_R) \
    gpio_name_to_num(ADAPTER_SEL_R) \
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
    gpio_name_to_num(Reserve_GPIOL4) \
    gpio_name_to_num(EN_LS_BIC_ASIC_R) \
    gpio_name_to_num(EN_JTAG2_ASIC_SEL_R) \
    gpio_name_to_num(OE_USB_MUX1_R_N)

#define name_gpioM \
    gpio_name_to_num(OE_USB_MUX2_R_N) \
    gpio_name_to_num(OE_FM_SPI_MUX_R_N) \
    gpio_name_to_num(EN_LS_JTAG2_BIC_R) \
    gpio_name_to_num(EN_LS_SPI_BIC_R) \
    gpio_name_to_num(Reserve_GPIOM4) \
    gpio_name_to_num(Reserve_GPIOM5) \
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
    gpio_name_to_num(LED_ASIC_R_1) \
    gpio_name_to_num(LED_ASIC_R_2)

#define name_gpioQ \
    gpio_name_to_num(Reserve_GPIOQ0) \
    gpio_name_to_num(LED_ASIC_R_3) \
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
    name_gpioA
    name_gpioB
    name_gpioC
    name_gpioD
    name_gpioE
    name_gpioF
    name_gpioG
    name_gpioH
    name_gpioI
    name_gpioJ
    name_gpioK
    name_gpioL
    name_gpioM
    name_gpioN
    name_gpioO
    name_gpioP
    name_gpioQ
    name_gpioR
    name_gpioS
    name_gpioT
    name_gpioU
};

extern enum _GPIO_NUMS_ GPIO_NUMS;
#undef gpio_name_to_num

extern char *gpio_name[];

#endif
