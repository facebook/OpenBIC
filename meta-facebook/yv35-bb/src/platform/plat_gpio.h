#ifndef PLAT_GPIO_H
#define PLAT_GPIO_H

#include "hal_gpio.h"
#define BB_CABLE_MATCH_SLOT1 0x3
#define BB_CABLE_MATCH_SLOT3 0x1

// gpio_cfg(chip, number, is_init, direction, status, int_type, int_callback)
// dedicate gpio A0~A7, B0~B7, C0~C7, D0~D7, E0~E7, total 40 gpios
// Default name: Reserve_GPIOH0

// clang-format off

#define name_gpioA \
	gpio_name_to_num(SMB_TEMP_ALERT_BIC_N) \
	gpio_name_to_num(SMB_BIC_HOTSWAP_ALERT_N_R) \
	gpio_name_to_num(PWROK_STBY_BIC_SLOT1_R) \
	gpio_name_to_num(PRSNT_MB_BIC_SLOT3_BB_N_R) \
	gpio_name_to_num(AC_ON_OFF_BTN_BIC_SLOT1_N_R) \
	gpio_name_to_num(AC_ON_OFF_BTN_BIC_SLOT3_N_R) \
	gpio_name_to_num(HSC_FAULT_BIC_SLOT1_N_R) \
	gpio_name_to_num(HSC_FAULT_BIC_SLOT3_N_R)

#define name_gpioB \
	gpio_name_to_num(FM_RESBTN_SLOT1_BIC_N) \
	gpio_name_to_num(FM_RESBTN_SLOT3_BIC_N) \
	gpio_name_to_num(FM_BIC_SLOT1_ISOLATED_EN_R) \
	gpio_name_to_num(FM_BIC_SLOT3_ISOLATED_EN_R) \
	gpio_name_to_num(FAST_PROCHOT_BIC_N_R) \
	gpio_name_to_num(PWROK_STBY_BIC_SLOT3_R) \
	gpio_name_to_num(FM_DEBUG_UART_MUX_BIC_R) \
	gpio_name_to_num(PRSNT_MB_BIC_SLOT1_BB_N_R)

#define name_gpioC \
	gpio_name_to_num(DUAL_FAN0_DETECT_BIC_N_R) \
	gpio_name_to_num(DUAL_FAN1_DETECT_BIC_N_R) \
	gpio_name_to_num(FAN0_BIC_CPLD_EN_R) \
	gpio_name_to_num(FAN1_BIC_CPLD_EN_R) \
	gpio_name_to_num(FAN2_BIC_CPLD_EN_R) \
	gpio_name_to_num(FAN3_BIC_CPLD_EN_R) \
	gpio_name_to_num(BB_BUTTON_BMC_BIC_N_R) \
	gpio_name_to_num(USB_CPLD_BIC_EN_R)

#define name_gpioD \
	gpio_name_to_num(P5V_USB_PG_BIC) \
	gpio_name_to_num(OCP_DEBUG_BIC_PRSNT_N_R) \
	gpio_name_to_num(FM_HSC_BIC_FAULT_N_R) \
	gpio_name_to_num(BIC_READY_R) \
	gpio_name_to_num(BOARD_ID0) \
	gpio_name_to_num(BOARD_ID1) \
	gpio_name_to_num(BOARD_ID2) \
	gpio_name_to_num(BOARD_ID3)

#define name_gpioE \
	gpio_name_to_num(SLOT1_ID1_DETECT_BIC_N) \
	gpio_name_to_num(SLOT1_ID0_DETECT_BIC_N) \
	gpio_name_to_num(SLOT2_ID1_DETECT_BIC_N) \
	gpio_name_to_num(SLOT2_ID0_DETECT_BIC_N) \
	gpio_name_to_num(SLOT3_ID1_DETECT_BIC_N) \
	gpio_name_to_num(SLOT3_ID0_DETECT_BIC_N) \
	gpio_name_to_num(SLOT4_ID1_DETECT_BIC_N) \
	gpio_name_to_num(SLOT4_ID0_DETECT_BIC_N)

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
	gpio_name_to_num(TP_SLOT1_BIC_RSVD2) \
	gpio_name_to_num(TP_SLOT1_BIC_RSVD3) \
	gpio_name_to_num(TP_SLOT1_BIC_RSVD4) \
	gpio_name_to_num(TP_SLOT1_BIC_RSVD5) \
	gpio_name_to_num(TP_SLOT1_BIC_RSVD6) \
	gpio_name_to_num(TP_SLOT1_BIC_RSVD7) \
	gpio_name_to_num(TP_SLOT1_BIC_RSVD0)

#define name_gpioM \
	gpio_name_to_num(BIC_STRAP_TXD6) \
	gpio_name_to_num(TP_SLOT3_BIC_RSVD2) \
	gpio_name_to_num(BIC_STRAP_TXD7) \
	gpio_name_to_num(TP_SLOT3_BIC_RSVD4) \
	gpio_name_to_num(USB_SW_EN_BIC_N_R) \
	gpio_name_to_num(USB_SW_BIC_CB_R) \
	gpio_name_to_num(Reserve_GPIOM6) \
	gpio_name_to_num(Reserve_GPIOM7)

#define name_gpioN \
	gpio_name_to_num(BIC_SGPMCK) \
	gpio_name_to_num(Reserve_GPION1) \
	gpio_name_to_num(BIC_SGPMO) \
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

extern const char *const gpio_name[];

#endif
