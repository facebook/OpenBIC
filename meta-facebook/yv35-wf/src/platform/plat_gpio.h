#ifndef PLAT_GPIO_H
#define PLAT_GPIO_H

#include "hal_gpio.h"

// gpio_cfg(chip, number, is_init, direction, status, int_type, int_callback)
// dedicate gpio A0~A7, B0~B7, C0~C7, D0~D7, E0~E7, total 40 gpios
// Default name: Reserve_GPIOH0
#define name_gpioA                                                                                 \
	gpio_name_to_num(ASIC_DEV_RST_N) gpio_name_to_num(ASIC_PERST0_N)                           \
		gpio_name_to_num(ASIC_EVENT_N) gpio_name_to_num(FM_P12V_HSC_EN)                    \
			gpio_name_to_num(IRQ_P12V_HSC_FLT_N) gpio_name_to_num(FM_P3V3_HSC_SW_EN)   \
				gpio_name_to_num(IRQ_P3V3_HSC_FLT_N)                               \
					gpio_name_to_num(Reserve_GPIOA7)
#define name_gpioB                                                                                 \
	gpio_name_to_num(Reserve_GPIOB0) gpio_name_to_num(P0V9_ASICA_1_FT_R)                       \
		gpio_name_to_num(P0V9_ASICA_1_PWRGD) gpio_name_to_num(ASIC_GPIO_R_0)               \
			gpio_name_to_num(JTAG2_NTRST2_R) gpio_name_to_num(FM_AUX_PWR_EN)           \
				gpio_name_to_num(I2CS_SRSTB_GPIO) gpio_name_to_num(ISO_EN_R_N)
#define name_gpioC                                                                                 \
	gpio_name_to_num(FM_P0V9_ASICD_EN) gpio_name_to_num(P0V9_ASICD_FT_R)                       \
		gpio_name_to_num(P0V9_ASICD_PWRGD) gpio_name_to_num(PVPP_AB_EN_R)                  \
			gpio_name_to_num(PVPP_AB_PG_R) gpio_name_to_num(RST_BIC_E1S_0_R_N)         \
				gpio_name_to_num(RST_BIC_E1S_1_R_N)                                \
					gpio_name_to_num(RST_BIC_E1S_2_R_N)
#define name_gpioD                                                                                 \
	gpio_name_to_num(PVTT_AB_EN_R) gpio_name_to_num(PVTT_AB_PG_R) gpio_name_to_num(SLOT_ID0)   \
		gpio_name_to_num(FM_PWRDIS_E1S_R_0) gpio_name_to_num(FM_PWRDIS_E1S_R_1)            \
			gpio_name_to_num(FM_PWRDIS_E1S_R_2)                                        \
				gpio_name_to_num(SMB_SENSOR_LVC3_ALERT_N)                          \
					gpio_name_to_num(CXL_CLOCK_R_OE)
#define name_gpioE                                                                                 \
	gpio_name_to_num(FM_POWER_EN) gpio_name_to_num(PWRGD_EXP_PWROK_R)                          \
		gpio_name_to_num(RST_MB_N) gpio_name_to_num(SPI_MASTER_SEL_R)                      \
			gpio_name_to_num(SMB_VR_PVDDQ_AB_ALERT_N) gpio_name_to_num(P1V2_VDD_PG_R)  \
				gpio_name_to_num(RST_SMB_E1S_0_R_N)                                \
					gpio_name_to_num(FM_MEM_THERM_EVENT_LVT3_N)
#define name_gpioF                                                                                 \
	gpio_name_to_num(SPI_RST_FLASH_N) gpio_name_to_num(IRQ_SMB_ALERT_R_N)                      \
		gpio_name_to_num(FM_P0V9_ASICA_2_EN) gpio_name_to_num(P0V9_ASICA_2_FT_R)           \
			gpio_name_to_num(P0V9_ASICA_2_PWRGD)                                       \
				gpio_name_to_num(JTAG2_ASIC_PORT_SEL_EN)                           \
					gpio_name_to_num(SMB_VR_P0V9_ASICD_ALERT_N)                \
						gpio_name_to_num(E1S_SMB_MUX_SEL_0_R)
#define name_gpioG                                                                                 \
	gpio_name_to_num(LED_BIC_E1S_R_0) gpio_name_to_num(LED_BIC_E1S_R_1)                        \
		gpio_name_to_num(LED_BIC_E1S_R_2) gpio_name_to_num(SLOT_ID1)                       \
			gpio_name_to_num(FM_PRSENT_E1S_0_R_N)                                      \
				gpio_name_to_num(FM_PRSENT_E1S_1_R_N)                              \
					gpio_name_to_num(FM_PRSENT_E1S_2_R_N)                      \
						gpio_name_to_num(FM_PVDDQ_AB_EN)
#define name_gpioH                                                                                 \
	gpio_name_to_num(PVDDQ_AB_FT_R) gpio_name_to_num(PWRGD_PVDDQ_AB)                           \
		gpio_name_to_num(SPI_BIC_SHIFT_EN) gpio_name_to_num(JTAG2_BIC_SHIFT_EN)            \
			gpio_name_to_num(Reserve_GPIOH4) gpio_name_to_num(Reserve_GPIOH5)          \
				gpio_name_to_num(Reserve_GPIOH6) gpio_name_to_num(Reserve_GPIOH7)
#define name_gpioI                                                                                 \
	gpio_name_to_num(Reserve_GPIOI0) gpio_name_to_num(Reserve_GPIOI1)                          \
		gpio_name_to_num(Reserve_GPIOI2) gpio_name_to_num(Reserve_GPIOI3)                  \
			gpio_name_to_num(Reserve_GPIOI4) gpio_name_to_num(Reserve_GPIOI5)          \
				gpio_name_to_num(Reserve_GPIOI6) gpio_name_to_num(Reserve_GPIOI7)
#define name_gpioJ                                                                                 \
	gpio_name_to_num(Reserve_GPIOJ0) gpio_name_to_num(Reserve_GPIOJ1)                          \
		gpio_name_to_num(P1V8_VDD_EN) gpio_name_to_num(P1V8_VDD_PG)                        \
			gpio_name_to_num(Reserve_GPIOJ4) gpio_name_to_num(Reserve_GPIOJ5)          \
				gpio_name_to_num(Reserve_GPIOJ6) gpio_name_to_num(Reserve_GPIOJ7)
#define name_gpioK                                                                                 \
	gpio_name_to_num(Reserve_GPIOK0) gpio_name_to_num(Reserve_GPIOK1)                          \
		gpio_name_to_num(Reserve_GPIOK2) gpio_name_to_num(Reserve_GPIOK3)                  \
			gpio_name_to_num(Reserve_GPIOK4) gpio_name_to_num(Reserve_GPIOK5)          \
				gpio_name_to_num(Reserve_GPIOK6) gpio_name_to_num(Reserve_GPIOK7)
#define name_gpioL                                                                                 \
	gpio_name_to_num(Reserve_GPIOL0) gpio_name_to_num(Reserve_GPIOL1)                          \
		gpio_name_to_num(INT_IO_EXP_N) gpio_name_to_num(FM_BOARD_REV_ID2)                  \
			gpio_name_to_num(FM_BOARD_REV_ID1) gpio_name_to_num(FM_BOARD_REV_ID0)      \
				gpio_name_to_num(BOARD_ID0) gpio_name_to_num(BOARD_ID1)
#define name_gpioM                                                                                 \
	gpio_name_to_num(FM_SPI_MUX_OE_CTL_R_N) gpio_name_to_num(BOARD_ID2)                        \
		gpio_name_to_num(BOARD_ID3) gpio_name_to_num(LED_CXL_POWER_R)                      \
			gpio_name_to_num(LED_CXL_FAULT) gpio_name_to_num(Reserve_GPIOM5)           \
				gpio_name_to_num(Reserve_GPIOM6) gpio_name_to_num(Reserve_GPIOM7)
#define name_gpioN                                                                                 \
	gpio_name_to_num(E1S_SMB_MUX_SEL_1_R) gpio_name_to_num(ASIC_PERST0_D_R_N)                  \
		gpio_name_to_num(SGPIO_BMC_DOUT_R) gpio_name_to_num(E1S_SMB_MUX_SEL_2_R)           \
			gpio_name_to_num(Reserve_GPION4) gpio_name_to_num(Reserve_GPION5)          \
				gpio_name_to_num(FM_P0V9_ASICA_1_EN)                               \
					gpio_name_to_num(FM_CLKBUF_R_EN)
#define name_gpioO                                                                                 \
	gpio_name_to_num(Reserve_GPIOO0) gpio_name_to_num(IRQ_INA230_E1S_0_ALERT_R_N)              \
		gpio_name_to_num(IRQ_INA230_PVPP_AB_ALERT_N) gpio_name_to_num(Reserve_GPIOO3)      \
			gpio_name_to_num(Reserve_GPIOO4) gpio_name_to_num(Reserve_GPIOO5)          \
				gpio_name_to_num(Reserve_GPIOO6) gpio_name_to_num(Reserve_GPIOO7)
#define name_gpioP                                                                                 \
	gpio_name_to_num(Reserve_GPIOP0) gpio_name_to_num(Reserve_GPIOP1)                          \
		gpio_name_to_num(Reserve_GPIOP2) gpio_name_to_num(Reserve_GPIOP3)                  \
			gpio_name_to_num(Reserve_GPIOP4) gpio_name_to_num(Reserve_GPIOP5)          \
				gpio_name_to_num(CLKBUF_E1S_0_OE_R_N)                              \
					gpio_name_to_num(CLKBUF_E1S_1_OE_R_N)
#define name_gpioQ                                                                                 \
	gpio_name_to_num(CLKBUF_E1S_2_OE_R_N) gpio_name_to_num(P5V_STBY_PG)                        \
		gpio_name_to_num(Reserve_GPIOQ2) gpio_name_to_num(Reserve_GPIOQ3)                  \
			gpio_name_to_num(Reserve_GPIOQ4) gpio_name_to_num(Reserve_GPIOQ5)          \
				gpio_name_to_num(Reserve_GPIOQ6) gpio_name_to_num(Reserve_GPIOQ7)
#define name_gpioR                                                                                 \
	gpio_name_to_num(Reserve_GPIOR0) gpio_name_to_num(Reserve_GPIOR1)                          \
		gpio_name_to_num(Reserve_GPIOR2) gpio_name_to_num(Reserve_GPIOR3)                  \
			gpio_name_to_num(Reserve_GPIOR4) gpio_name_to_num(Reserve_GPIOR5)          \
				gpio_name_to_num(Reserve_GPIOR6) gpio_name_to_num(Reserve_GPIOR7)
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
				gpio_name_to_num(RST_SMB_E1S_1_R_N)                                \
					gpio_name_to_num(RST_SMB_E1S_2_R_N)

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