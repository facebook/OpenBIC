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
    gpio_name_to_num(CLKBUF_E1S_0_OE_N) \
    gpio_name_to_num(CLKBUF_E1S_1_OE_N) \
    gpio_name_to_num(CLKBUF_E1S_2_OE_N) \
    gpio_name_to_num(CLKBUF_E1S_3_OE_N) \
    gpio_name_to_num(CLKBUF_E1S_4_OE_N) \
    gpio_name_to_num(PWRGD_P12V_E1S_0_R) \
    gpio_name_to_num(PWRGD_P12V_E1S_1_R) \
    gpio_name_to_num(PWRGD_P12V_E1S_2_R)

#define name_gpioB \
    gpio_name_to_num(PWRGD_P12V_E1S_3_R) \
    gpio_name_to_num(PWRGD_P12V_E1S_4_R) \
    gpio_name_to_num(E1S_0_PRSNT_N) \
    gpio_name_to_num(E1S_1_PRSNT_N) \
    gpio_name_to_num(E1S_2_PRSNT_N) \
    gpio_name_to_num(E1S_3_PRSNT_N) \
    gpio_name_to_num(E1S_4_PRSNT_N ) \
    gpio_name_to_num(SMB_TEMP_SENSOR_ALT_N)

#define name_gpioC \
    gpio_name_to_num(PWRGD_P3V3_E1S_0_R) \
    gpio_name_to_num(PWRGD_P3V3_E1S_1_R) \
    gpio_name_to_num(PWRGD_P3V3_E1S_2_R) \
    gpio_name_to_num(PWRGD_P3V3_E1S_3_R) \
    gpio_name_to_num(PWRGD_P3V3_E1S_4_R) \
    gpio_name_to_num(BOARD_REV_0) \
    gpio_name_to_num(BOARD_REV_1) \
    gpio_name_to_num(BOARD_REV_2)

#define name_gpioD \
    gpio_name_to_num(PWRGD_P12V_MAIN) \
    gpio_name_to_num(RST_SMBRST_BIC_E1S_0) \
    gpio_name_to_num(RST_SMBRST_BIC_E1S_1) \
    gpio_name_to_num(RST_SMBRST_BIC_E1S_2) \
    gpio_name_to_num(RST_SMBRST_BIC_E1S_3) \
    gpio_name_to_num(RST_SMBRST_BIC_E1S_4) \
    gpio_name_to_num(RST_E1S_3_PERST) \
    gpio_name_to_num(RST_E1S_4_PERST)

#define name_gpioE \
    gpio_name_to_num(RST_E1S_0_PERST) \
    gpio_name_to_num(RST_E1S_1_PERST) \
    gpio_name_to_num(RST_E1S_2_PERST) \
    gpio_name_to_num(E1S_0_POWER_EN) \
    gpio_name_to_num(E1S_1_POWER_EN) \
    gpio_name_to_num(E1S_2_POWER_EN) \
    gpio_name_to_num(E1S_3_POWER_EN) \
    gpio_name_to_num(E1S_4_POWER_EN)

#define name_gpioF \
    gpio_name_to_num(Reserve_GPIOF0) \
    gpio_name_to_num(Reserve_GPIOF1) \
    gpio_name_to_num(Reserve_GPIOF2) \
    gpio_name_to_num(Reserve_GPIOF3) \
    gpio_name_to_num(Reserve_GPIOF4) \
    gpio_name_to_num(Reserve_GPIOF5) \
    gpio_name_to_num(Reserve_GPIOF6) \
    gpio_name_to_num(Reserve_GPIOF7)

#define name_gpioG \
    gpio_name_to_num(Reserve_GPIOG0) \
    gpio_name_to_num(Reserve_GPIOG1) \
    gpio_name_to_num(Reserve_GPIOG2) \
    gpio_name_to_num(Reserve_GPIOG3) \
    gpio_name_to_num(Reserve_GPIOG4) \
    gpio_name_to_num(Reserve_GPIOG5) \
    gpio_name_to_num(Reserve_GPIOG6) \
    gpio_name_to_num(Reserve_GPIOG7)

#define name_gpioH \
    gpio_name_to_num(Reserve_GPIOH0) \
    gpio_name_to_num(Reserve_GPIOH1) \
    gpio_name_to_num(Reserve_GPIOH2) \
    gpio_name_to_num(Reserve_GPIOH3) \
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
    gpio_name_to_num(Reserve_GPIOL5) \
    gpio_name_to_num(Reserve_GPIOL6) \
    gpio_name_to_num(Reserve_GPIOL7)


#define name_gpioM \
    gpio_name_to_num(Reserve_GPIOM0) \
    gpio_name_to_num(Reserve_GPIOM1) \
    gpio_name_to_num(Reserve_GPIOM2) \
    gpio_name_to_num(Reserve_GPIOM3) \
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

extern const char *const gpio_name[];

#endif
