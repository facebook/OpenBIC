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
	gpio_name_to_num(FM_BMC_PCH_SCI_LPC_N) \
	gpio_name_to_num(FM_BIOS_POST_CMPLT_BMC_N) \
	gpio_name_to_num(FM_SLPS3_R_N) \
	gpio_name_to_num(IRQ_BMC_PCH_SMI_LPC_R_N) \
	gpio_name_to_num(IRQ_UV_DETECT_N) \
	gpio_name_to_num(FM_UV_ADR_TRIGGER_EN) \
	gpio_name_to_num(IRQ_SMI_ACTIVE_BMC_N) \
	gpio_name_to_num(HSC_SET_EN)

#define name_gpioB \
	gpio_name_to_num(FM_BIC_RST_RTCRST) \
	gpio_name_to_num(RST_USB_HUB_N_R) \
	gpio_name_to_num(A_P3V_BAT_SCALED_EN) \
	gpio_name_to_num(FM_CPU_FIVR_FAULT_LVT3_N) \
	gpio_name_to_num(DBP_SYSPWROK_R) \
	gpio_name_to_num(FM_SLPS4_R_N) \
	gpio_name_to_num(RST_RSTBTN_OUT_N) \
	gpio_name_to_num(PWRGD_SYS_PWROK)

#define name_gpioC \
	gpio_name_to_num(FM_HSC_TIMER) \
	gpio_name_to_num(IRQ_SMB_IO_LVC3_STBY_ALRT_N) \
	gpio_name_to_num(IRQ_PVCCIN_CPU_VRHOT_LVC3_N) \
	gpio_name_to_num(FM_BMC_PREQ_N_NODE_R1) \
	gpio_name_to_num(FM_CPU_THERMTRIP_LATCH_LVT3_N) \
	gpio_name_to_num(FM_CPU_SKTOCC_LVT3_N) \
	gpio_name_to_num(FM_CPU_MEMHOT_OUT_N) \
	gpio_name_to_num(RST_PLTRST_FROM_PCH_N)

#define name_gpioD \
	gpio_name_to_num(PWRBTN_N) \
	gpio_name_to_num(RST_BMC_R_N) \
	gpio_name_to_num(IRQ_BMC_PRDY_NODE_OD_N) \
	gpio_name_to_num(BMC_READY) \
	gpio_name_to_num(BIC_READY) \
	gpio_name_to_num(FM_SOL_UART_CH_SEL_R) \
	gpio_name_to_num(HSC_MUX_SWITCH_R) \
	gpio_name_to_num(RST_PLTRST_BMC_N)

#define name_gpioE \
	gpio_name_to_num(PWRGD_CPU_LVC3_R) \
	gpio_name_to_num(FM_PCH_BMC_THERMTRIP_N) \
	gpio_name_to_num(HSC_DETECT0) \
	gpio_name_to_num(HSC_DETECT1) \
	gpio_name_to_num(HSC_DETECT2) \
	gpio_name_to_num(FM_CPU_MSMI_CATERR_LVT3_N) \
	gpio_name_to_num(FM_PWRBTN_OUT_N) \
	gpio_name_to_num(PWRGD_BMC_PS_PWROK_R)

#define name_gpioF \
	gpio_name_to_num(IRQ_BMC_PCH_NMI_R) \
	gpio_name_to_num(IRQ_SML1_PMBUS_ALERT_N) \
	gpio_name_to_num(IRQ_NMI_EVENT_R_N) \
	gpio_name_to_num(FM_BMC_DEBUG_ENABLE_N) \
	gpio_name_to_num(DBP_PRESENT_R2_N) \
	gpio_name_to_num(FM_FAST_PROCHOT_EN_N) \
	gpio_name_to_num(FM_BMC_ONCTL_R_N) \
	gpio_name_to_num(FM_BMC_CPU_PWR_DEBUG_N)

#define name_gpioG \
	gpio_name_to_num(FM_PEHPCPU_INT) \
	gpio_name_to_num(FM_MRC_DEBUG_EN) \
	gpio_name_to_num(FAST_PROCHOT_N) \
	gpio_name_to_num(FM_JTAG_TCK_MUX_SEL) \
	gpio_name_to_num(BMC_JTAG_SEL) \
	gpio_name_to_num(FM_CPU_ERR0_LVT3_N) \
	gpio_name_to_num(FM_CPU_ERR1_LVT3_N) \
	gpio_name_to_num(FM_CPU_ERR2_LVT3_N)

#define name_gpioH \
	gpio_name_to_num(RST_RSMRST_BMC_N) \
	gpio_name_to_num(FM_MP_PS_FAIL_N) \
	gpio_name_to_num(FM_MEM_THERM_EVENT_LVT3_N) \
	gpio_name_to_num(IRQ_SML0_ALERT_MUX_R_N) \
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
	gpio_name_to_num(BOARD_ID2) \
	gpio_name_to_num(IRQ_PVDDQ_ABC_VRHOT_LVT3_N) \
	gpio_name_to_num(FM_CPU_THERMTRIP_LVT3_N) \
	gpio_name_to_num(BOARD_ID0) \
	gpio_name_to_num(BOARD_ID1)

// GPIOM6, M7 hardware not define
#define name_gpioM \
	gpio_name_to_num(Reserve_GPIOM0) \
	gpio_name_to_num(BOARD_ID3) \
	gpio_name_to_num(Reserve_GPIOM2) \
	gpio_name_to_num(FM_THERMTRIP_DLY_TO_PCH) \
	gpio_name_to_num(IRQ_PVDDQ_DEF_VRHOT_LVT3_N) \
	gpio_name_to_num(IRQ_PVCCIO_CPU_VRHOT_LVC3_N) \
	gpio_name_to_num(Reserve_GPIOM6) \
	gpio_name_to_num(Reserve_GPIOM7)

#define name_gpioN \
	gpio_name_to_num(SGPIO_BMC_CLK_R) \
	gpio_name_to_num(SGPIO_BMC_LD_N) \
	gpio_name_to_num(SGPIO_BMC_DOUT_R) \
	gpio_name_to_num(SGPIO_BMC_DIN) \
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
	gpio_name_to_num(FM_SPD_DDRCPU_LVLSHFT_EN) \
	gpio_name_to_num(FM_BMC_PCHIE_N)

// GPIOQ5 hardware not define
#define name_gpioQ \
	gpio_name_to_num(IRQ_HSC_ALERT2_N) \
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
	gpio_name_to_num(Reserve_GPIOT7)

// GPIOU input only
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

//align ti bic for bmc get gpio inform
#define PVCCIO_CPU 0xFF
#define BMC_HEARTBEAT_LED_R 0xFF
#define FM_FORCE_ADR_N_R 0xFF
#define JTAG_BMC_NTRST_R_N 0xff

extern char *gpio_name[];
//  GPIO Table SET/GET GPIO Configuration align to Ti BIC
extern uint8_t gpio_align_t[];
extern int gpio_align_table_length;

void enable_PRDY_interrupt();
void disable_PRDY_interrupt();
void enable_UV_detect_interrupt();
void disable_UV_detect_interrupt();
void enable_SYS_Throttle_interrupt();
void disable_SYS_Throttle_interrupt();
uint8_t get_exported_gpio_num(uint8_t internal_gpio_num);
#endif
