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

#define EVT_EXAMAX_RESERVED_1 18
#define EVT_EXAMAX_RESERVED_2 25
#define EVT_EXAMAX_RESERVED_4 43
#define EVT_CPU_TYPE_1 52
#define EVT_RTM_IOEXP_INT_N 53
#define EVT_EXAMAX_RESERVED_3 55
#define EVT_AUTH_COMPLETE 72
#define EVT_BIC_READY_TOP_EXP 73
#define EVT_RTM2_IOEXP_INT_N 74
#define EVT_SIDECAR_CABLE_PRESENT 75
#define EVT_BIC_READY_FRONT_EXP 76
#define EVT_CPU_BIC_RTC_GET_N 77
#define EVT_SMB_RTM1_INA233_ALRT_N 78
#define EVT_SMB_RTM2_INA233_ALRT_N 79
#define EVT_EXAMMAX_TYPE 95
#define EVT_Reserve_GPIOM0 96
#define EVT_MEDUSA_HSC_R_PG 98
#define EVT_Reserve_GPIOM3 99

#define name_gpioA \
	gpio_name_to_num(FM_CPU_BIC_SLP_S5_N) \
	gpio_name_to_num(FM_CPU_BIC_SLP_S3_N) \
	gpio_name_to_num(RST_RSMRST_BMC_N) \
	gpio_name_to_num(PWRGD_CPU_LVC3) \
	gpio_name_to_num(CPU_SMERR_BIC_N) \
	gpio_name_to_num(BMC_READY) \
	gpio_name_to_num(RST_CPU_RESET_BIC_N) \
	gpio_name_to_num(RST_USB_HUB_R_N)

#define name_gpioB \
	gpio_name_to_num(AUTH_PRSNT_BIC_N) \
	gpio_name_to_num(BIC_CPU_NMI_N) \
	gpio_name_to_num(FM_SMI_ACTIVE_N) \
	gpio_name_to_num(IRQ_BIC_CPU_SMI_N) \
	gpio_name_to_num(FM_CPU_BIC_THERMTRIP_N) \
	gpio_name_to_num(APML_CPU_ALERT_BIC_N) \
	gpio_name_to_num(PRSNT_CPU_R_N) \
	gpio_name_to_num(SYS_PWRBTN_BIC_N)

#define name_gpioC \
	gpio_name_to_num(PVDDCR_CPU0_PMALERT_N) \
	gpio_name_to_num(FM_HSC_TIMER_ALT_N) \
	gpio_name_to_num(SMB_RTM1_INA233_ALRT_N) \
	gpio_name_to_num(PVDDCR_CPU1_PMALERT_N) \
	gpio_name_to_num(PWRBTN_N) \
	gpio_name_to_num(RST_BMC_R_N) \
	gpio_name_to_num(HDT_BIC_DBREQ_R_N) \
	gpio_name_to_num(BIC_READY_R)

#define name_gpioD \
	gpio_name_to_num(FM_SOL_UART_CH_SEL_R) \
	gpio_name_to_num(SMB_RTM2_INA233_ALRT_N) \
	gpio_name_to_num(FAST_PROCHOT_N) \
	gpio_name_to_num(BIC_JTAG_SEL_R) \
	gpio_name_to_num(FM_CPU_BIC_PROCHOT_LVT3_N) \
	gpio_name_to_num(SMB_E1S_0_RST_R_N) \
	gpio_name_to_num(SMB_E1S_1_RST_R_N) \
	gpio_name_to_num(SMB_E1S_0_INA233_ALRT_N)

#define name_gpioE \
	gpio_name_to_num(FM_BIOS_MRC_DEBUG_MSG_DIS) \
	gpio_name_to_num(SMB_SENSOR_LVC3_ALERT_N) \
	gpio_name_to_num(FM_BIOS_POST_CMPLT_BIC_N) \
	gpio_name_to_num(IRQ_UV_DETECT_N) \
	gpio_name_to_num(PVDDCR_CPU0_OCP_N) \
	gpio_name_to_num(PVDDCR_CPU1_OCP_N) \
	gpio_name_to_num(P3V_BAT_SCALED_R_EN) \
	gpio_name_to_num(HDT_BIC_TRST_R_N)

#define name_gpioF \
	gpio_name_to_num(CARD_TYPE_EXP) \
	gpio_name_to_num(CPU_ERROR_BIC_LVC3_R_N) \
	gpio_name_to_num(PVDD11_S3_PMALERT_N) \
	gpio_name_to_num(BIC_READY_FRONT_EXP) \
	gpio_name_to_num(CPU_TYPE0) \
	gpio_name_to_num(FM_BMC_DEBUG_ENABLE_R_N) \
	gpio_name_to_num(FM_DBP_PRESENT_N) \
	gpio_name_to_num(FM_FAST_PROCHOT_R_EN_N)

#define name_gpioG \
	gpio_name_to_num(IO_EXP_ALERT) \
	gpio_name_to_num(FM_CPLD_BMC_BIC_READY) \
	gpio_name_to_num(BIC_JTAG_MUX_SEL) \
	gpio_name_to_num(RST_PLTRST_BIC_N) \
	gpio_name_to_num(CPU_BIC_RTC_GET_N) \
	gpio_name_to_num(AUTH_COMPLETE) \
	gpio_name_to_num(P3V3_STBY_SIDECAR_FAULT_N) \
	gpio_name_to_num(BIC_READY_TOP_EXP)

#define name_gpioH \
	gpio_name_to_num(PRSNT_CEM_CONN) \
	gpio_name_to_num(TOP_CXL_SMBUS_ALERT_N) \
	gpio_name_to_num(FRONT_CXL_SMBUS_ALERT_N) \
	gpio_name_to_num(P1V2_STBY_SIDECAR_FAULT_N) \
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
	gpio_name_to_num(RTM2_INT_N) \
	gpio_name_to_num(RTM_IOEXP_INT_N) \
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
	gpio_name_to_num(PWRGD_HSC_SLOT_BIC) \
	gpio_name_to_num(SIDECAR_PRESENT_BIC_N) \
	gpio_name_to_num(SMB_E1S_1_INA233_ALRT_N) \
	gpio_name_to_num(VR_TYPE_0) \
	gpio_name_to_num(VR_TYPE_1) \
	gpio_name_to_num(RTM_TYPE_0)

#define name_gpioM \
	gpio_name_to_num(RTM_TYPE_1) \
	gpio_name_to_num(Reserve_GPIOM1) \
	gpio_name_to_num(Reserve_GPIOM2) \
	gpio_name_to_num(BIC_ESPI_SELECT) \
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
	name_gpioA name_gpioB name_gpioC name_gpioD name_gpioE name_gpioF name_gpioG name_gpioH
		name_gpioI name_gpioJ name_gpioK name_gpioL name_gpioM name_gpioN name_gpioO
			name_gpioP name_gpioQ name_gpioR name_gpioS name_gpioT name_gpioU
};

extern enum _GPIO_NUMS_ GPIO_NUMS;
#undef gpio_name_to_num

extern char *gpio_name[];

void sync_bmc_ready_pin();
void reset_usb_hub();
#endif
