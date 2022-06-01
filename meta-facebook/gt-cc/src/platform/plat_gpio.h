#ifndef PLAT_GPIO_H
#define PLAT_GPIO_H

#include "hal_gpio.h"

// gpio_cfg(chip, number, is_init, direction, status, int_type, int_callback)
// dedicate gpio A0~A7, B0~B7, C0~C7, D0~D7, E0~E7, total 40 gpios
// Default name: Reserve_GPIOH0
#define name_gpioA                                                                                 \
	gpio_name_to_num(PRSNT_NIC0_R_N) gpio_name_to_num(PRSNT_NIC1_R_N)                          \
		gpio_name_to_num(PRSNT_NIC2_R_N) gpio_name_to_num(PRSNT_NIC3_R_N)                  \
			gpio_name_to_num(PRSNT_NIC4_R_N) gpio_name_to_num(PRSNT_NIC5_R_N)          \
				gpio_name_to_num(PRSNT_NIC6_R_N) gpio_name_to_num(PRSNT_NIC7_R_N)
#define name_gpioB                                                                                 \
	gpio_name_to_num(PRSNT_SSD0_R_N) gpio_name_to_num(PRSNT_SSD1_R_N)                          \
		gpio_name_to_num(PRSNT_SSD2_R_N) gpio_name_to_num(PRSNT_SSD3_R_N)                  \
			gpio_name_to_num(PRSNT_SSD4_R_N) gpio_name_to_num(PRSNT_SSD5_R_N)          \
				gpio_name_to_num(PRSNT_SSD6_R_N) gpio_name_to_num(PRSNT_SSD7_R_N)
#define name_gpioC                                                                                 \
	gpio_name_to_num(SSD0_PWRDIS_BIC_R) gpio_name_to_num(SSD1_PWRDIS_BIC_R)                    \
		gpio_name_to_num(SSD2_PWRDIS_BIC_R) gpio_name_to_num(SSD3_PWRDIS_BIC_R)            \
			gpio_name_to_num(SSD4_PWRDIS_BIC_R) gpio_name_to_num(SSD5_PWRDIS_BIC_R)    \
				gpio_name_to_num(SSD6_PWRDIS_BIC_R)                                \
					gpio_name_to_num(SSD7_PWRDIS_BIC_R)
#define name_gpioD                                                                                 \
	gpio_name_to_num(SSD8_PWRDIS_BIC_R) gpio_name_to_num(SSD9_PWRDIS_BIC_R)                    \
		gpio_name_to_num(SSD10_PWRDIS_BIC_R) gpio_name_to_num(SSD11_PWRDIS_BIC_R)          \
			gpio_name_to_num(SSD12_PWRDIS_BIC_R) gpio_name_to_num(SSD13_PWRDIS_BIC_R)  \
				gpio_name_to_num(SSD14_PWRDIS_BIC_R)                               \
					gpio_name_to_num(SSD15_PWRDIS_BIC_R)
#define name_gpioE                                                                                 \
	gpio_name_to_num(PRSNT_SSD8_R_N) gpio_name_to_num(PRSNT_SSD9_R_N)                          \
		gpio_name_to_num(PRSNT_SSD10_R_N) gpio_name_to_num(PRSNT_SSD11_R_N)                \
			gpio_name_to_num(PRSNT_SSD12_R_N) gpio_name_to_num(PRSNT_SSD13_R_N)        \
				gpio_name_to_num(PRSNT_SSD14_R_N)                                  \
					gpio_name_to_num(PRSNT_SSD15_R_N)
#define name_gpioF                                                                                 \
	gpio_name_to_num(NIC0_MAIN_PWR_BIC_EN) gpio_name_to_num(NIC1_MAIN_PWR_BIC_EN)              \
		gpio_name_to_num(NIC2_MAIN_PWR_BIC_EN) gpio_name_to_num(NIC3_MAIN_PWR_BIC_EN)      \
			gpio_name_to_num(NIC4_MAIN_PWR_BIC_EN)                                     \
				gpio_name_to_num(NIC5_MAIN_PWR_BIC_EN)                             \
					gpio_name_to_num(NIC6_MAIN_PWR_BIC_EN)                     \
						gpio_name_to_num(NIC7_MAIN_PWR_BIC_EN)
#define name_gpioG                                                                                 \
	gpio_name_to_num(REV_ID0) gpio_name_to_num(REV_ID1) gpio_name_to_num(REV_ID2)              \
		gpio_name_to_num(BOARD_ID0) gpio_name_to_num(BOARD_ID1)                            \
			gpio_name_to_num(BOARD_ID2) gpio_name_to_num(HSC_TIMER_R_N)                \
				gpio_name_to_num(HSC_D_OC_BUF_R_N)
#define name_gpioH                                                                                 \
	gpio_name_to_num(HSC_UV_R_N) gpio_name_to_num(BIC_FORCE_THROTTLE_N)                        \
		gpio_name_to_num(PRSNT_DBV2_SLIMSAS_R_N) gpio_name_to_num(Reserve_GPIOH3)          \
			gpio_name_to_num(Reserve_GPIOH4) gpio_name_to_num(Reserve_GPIOH5)          \
				gpio_name_to_num(Reserve_GPIOH6) gpio_name_to_num(Reserve_GPIOH7)
#define name_gpioI                                                                                 \
	gpio_name_to_num(Reserve_GPIOI0) gpio_name_to_num(Reserve_GPIOI1)                          \
		gpio_name_to_num(Reserve_GPIOI2) gpio_name_to_num(Reserve_GPIOI3)                  \
			gpio_name_to_num(Reserve_GPIOI4) gpio_name_to_num(Reserve_GPIOI5)          \
				gpio_name_to_num(Reserve_GPIOI6) gpio_name_to_num(Reserve_GPIOI7)
#define name_gpioJ                                                                                 \
	gpio_name_to_num(Reserve_GPIOJ0) gpio_name_to_num(Reserve_GPIOJ1)                          \
		gpio_name_to_num(Reserve_GPIOJ2) gpio_name_to_num(Reserve_GPIOJ3)                  \
			gpio_name_to_num(Reserve_GPIOJ4) gpio_name_to_num(Reserve_GPIOJ5)          \
				gpio_name_to_num(Reserve_GPIOJ6) gpio_name_to_num(Reserve_GPIOJ7)
#define name_gpioK                                                                                 \
	gpio_name_to_num(Reserve_GPIOK0) gpio_name_to_num(Reserve_GPIOK1)                          \
		gpio_name_to_num(Reserve_GPIOK2) gpio_name_to_num(Reserve_GPIOK3)                  \
			gpio_name_to_num(Reserve_GPIOK4) gpio_name_to_num(Reserve_GPIOK5)          \
				gpio_name_to_num(Reserve_GPIOK6) gpio_name_to_num(Reserve_GPIOK7)
#define name_gpioL                                                                                 \
	gpio_name_to_num(BIC_UART_BIC_SEL) gpio_name_to_num(Reserve_GPIOL1)                        \
		gpio_name_to_num(NIC_ADC_ALERT_N) gpio_name_to_num(SSD_0_7_ADC_ALERT_N)            \
			gpio_name_to_num(SSD_8_15_ADC_ALERT_N)                                     \
				gpio_name_to_num(RST_BIC_I2C9_SSD_MUX0_N)                          \
					gpio_name_to_num(RST_BIC_I2C9_SSD_MUX1_N)                  \
						gpio_name_to_num(RST_BIC_I2C6_MUX_N)
// GPIOM6, M7 hardware not define
#define name_gpioM                                                                                 \
	gpio_name_to_num(RST_SSD_LED_IOEXP_N) gpio_name_to_num(RST_PEX_USB_HUB_N)                  \
		gpio_name_to_num(RST_BIC_USB_HUB_N) gpio_name_to_num(IRQ_SSD_LED_IOEXP_R_N)        \
			gpio_name_to_num(PEX_ADC_ALERT_N) gpio_name_to_num(MB_BMC_MON_ISO_R)       \
				gpio_name_to_num(Reserve_GPIOM6) gpio_name_to_num(Reserve_GPIOM7)
#define name_gpioN                                                                                 \
	gpio_name_to_num(SGPIO_BIC_CLK) gpio_name_to_num(FM_SYS_THROTTLE_R1)                       \
		gpio_name_to_num(SGPIO_BIC_DOUT) gpio_name_to_num(JTAG_BIC_EN)                     \
			gpio_name_to_num(SMB_FPGA_ALERT_R_N) gpio_name_to_num(SMB_ALERT_PMBUS_R_N) \
				gpio_name_to_num(USB_DEBUG_RST_BTN_R_N)                            \
					gpio_name_to_num(SMB_ALERT_HSC_R_N)
#define name_gpioO                                                                                 \
	gpio_name_to_num(RST_MB_BMC_N) gpio_name_to_num(Reserve_GPIOO1)                            \
		gpio_name_to_num(PRSNT_GPU_ISO_R_N) gpio_name_to_num(RST_BIC_SELF_HW_RST_N)        \
			gpio_name_to_num(RST_SMB_SSD_N) gpio_name_to_num(RST_SMB_FPGA_N)           \
				gpio_name_to_num(MB_BIC_MON) gpio_name_to_num(RST_SMB_FIO_N)
#define name_gpioP                                                                                 \
	gpio_name_to_num(RST_SMB_NIC_MUX_N) gpio_name_to_num(Reserve_GPIOP1)                       \
		gpio_name_to_num(Reserve_GPIOP2) gpio_name_to_num(Reserve_GPIOP3)                  \
			gpio_name_to_num(Reserve_GPIOP4) gpio_name_to_num(Reserve_GPIOP5)          \
				gpio_name_to_num(Reserve_GPIOP6) gpio_name_to_num(BIC_SYS_READY_N)
// GPIOQ5 hardware not define
#define name_gpioQ                                                                                 \
	gpio_name_to_num(SYS_PWR_READY_N) gpio_name_to_num(BIC_SEL_FLASH_SW0)                      \
		gpio_name_to_num(BIC_SEL_FLASH_SW1) gpio_name_to_num(BIC_SEL_FLASH_SW2)            \
			gpio_name_to_num(BIC_SEL_FLASH_SW3) gpio_name_to_num(Reserve_GPIOQ5)       \
				gpio_name_to_num(Reserve_GPIOQ6) gpio_name_to_num(Reserve_GPIOQ7)
#define name_gpioR                                                                                 \
	gpio_name_to_num(Reserve_GPIOR0) gpio_name_to_num(Reserve_GPIOR1)                          \
		gpio_name_to_num(Reserve_GPIOR2) gpio_name_to_num(Reserve_GPIOR3)                  \
			gpio_name_to_num(Reserve_GPIOR4) gpio_name_to_num(Reserve_GPIOR5)          \
				gpio_name_to_num(Reserve_GPIOR6) gpio_name_to_num(Reserve_GPIOR7)
// GPIOS3, S4, S5, S6, S7 hardware not define
#define name_gpioS                                                                                 \
	gpio_name_to_num(Reserve_GPIOS0) gpio_name_to_num(Reserve_GPIOS1)                          \
		gpio_name_to_num(Reserve_GPIOS2) gpio_name_to_num(Reserve_GPIOS3)                  \
			gpio_name_to_num(Reserve_GPIOS4) gpio_name_to_num(Reserve_GPIOS5)          \
				gpio_name_to_num(Reserve_GPIOS6) gpio_name_to_num(Reserve_GPIOS7)
// GPIOT input only
#define name_gpioT                                                                                 \
	gpio_name_to_num(Reserve_GPIOT0) gpio_name_to_num(Reserve_GPIOT1)                          \
		gpio_name_to_num(Reserve_GPIOT2) gpio_name_to_num(Reserve_GPIOT3)                  \
			gpio_name_to_num(Reserve_GPIOT4) gpio_name_to_num(Reserve_GPIOT5)          \
				gpio_name_to_num(Reserve_GPIOT6) gpio_name_to_num(Reserve_GPIOT7)
// GPIOU input only
#define name_gpioU                                                                                 \
	gpio_name_to_num(Reserve_GPIOU0) gpio_name_to_num(Reserve_GPIOU1)                          \
		gpio_name_to_num(Reserve_GPIOU2) gpio_name_to_num(Reserve_GPIOU3)                  \
			gpio_name_to_num(Reserve_GPIOU4) gpio_name_to_num(Reserve_GPIOU5)          \
				gpio_name_to_num(Reserve_GPIOU6) gpio_name_to_num(Reserve_GPIOU7)

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
