#ifndef PLAT_GPIO_H
#define PLAT_GPIO_H

#include "hal_gpio.h"

// gpio_cfg(chip, number, is_init, direction, status, int_type, int_callback)
// dedicate gpio A0~A7, B0~B7, C0~C7, D0~D7, E0~E7, total 40 gpios
// Default name: Reserve_GPIOH0
#define name_gpioA                                                                                 \
	gpio_name_to_num(ASIC_DEV_RST_N) gpio_name_to_num(ASIC_PERST0_N)                           \
		gpio_name_to_num(ASIC_PERST1_N) gpio_name_to_num(ASIC_FAIL_N)                      \
			gpio_name_to_num(ASIC_EVENT_N) gpio_name_to_num(ASIC_DUALPORTEN_N)         \
				gpio_name_to_num(JTAG2_BIC_ASIC_NTRST2)                            \
					gpio_name_to_num(ASIC_TAP_SEL)
#define name_gpioB                                                                                 \
	gpio_name_to_num(ASIC_CPU_BOOT_0) gpio_name_to_num(ASIC_CPU_BOOT_1)                        \
		gpio_name_to_num(ASIC_M_SCAN_PCAP_SEL) gpio_name_to_num(ASIC_GPIO_R_0)             \
			gpio_name_to_num(ASIC_GPIO_R_1) gpio_name_to_num(AUX_PWR_EN_4C)            \
				gpio_name_to_num(I2CS_SRSTB_GPIO)                                  \
					gpio_name_to_num(FM_ISOLATED_EN_N)
#define name_gpioC                                                                                 \
	gpio_name_to_num(FM_P0V8_ASICD_EN) gpio_name_to_num(P1V8_ASIC_EN_R)                        \
		gpio_name_to_num(FM_P0V8_ASICA_EN) gpio_name_to_num(PVTT_AB_EN_R)                  \
			gpio_name_to_num(PVTT_CD_EN_R) gpio_name_to_num(FM_P0V9_ASICA_EN)          \
				gpio_name_to_num(PVPP_CD_EN_R) gpio_name_to_num(FM_PVDDQ_AB_EN)
#define name_gpioD                                                                                 \
	gpio_name_to_num(PVPP_AB_EN_R) gpio_name_to_num(FM_PVDDQ_CD_EN) gpio_name_to_num(SLOT_ID0) \
		gpio_name_to_num(PVPP_CD_PG_R) gpio_name_to_num(P0V8_ASICA_PWRGD)                  \
			gpio_name_to_num(PVTT_AB_PG_R) gpio_name_to_num(SMB_SENSOR_LVC3_ALERT_N)   \
				gpio_name_to_num(PVTT_CD_PG_R)
#define name_gpioE                                                                                 \
	gpio_name_to_num(FM_POWER_EN) gpio_name_to_num(PWRGD_CARD_PWROK)                           \
		gpio_name_to_num(RST_MB_N) gpio_name_to_num(SPI_MASTER_SEL)                        \
			gpio_name_to_num(FM_SPI_MUX_OE_CTL_N) gpio_name_to_num(SMB_12V_INA_ALRT_N) \
				gpio_name_to_num(SMB_3V3_INA_ALRT_N)                               \
					gpio_name_to_num(FM_MEM_THERM_EVENT_LVT3_N)
#define name_gpioF                                                                                 \
	gpio_name_to_num(SPI_RST_FLASH_N) gpio_name_to_num(SMBUS_ALERT_R_N)                        \
		gpio_name_to_num(LSFT_SMB_DIMM_EN) gpio_name_to_num(P0V9_ASICA_PWRGD)              \
			gpio_name_to_num(P1V8_ASIC_PG_R)                                           \
				gpio_name_to_num(JTAG2_ASIC_PORT_SEL_EN_R)                         \
					gpio_name_to_num(SAVE_N_BIC)                               \
						gpio_name_to_num(FM_ADR_COMPLETE_DLY)
#define name_gpioG                                                                                 \
	gpio_name_to_num(P5V_STBY_PG) gpio_name_to_num(PVPP_AB_PG_R)                               \
		gpio_name_to_num(P1V2_STBY_PG_R) gpio_name_to_num(SLOT_ID1)                        \
			gpio_name_to_num(SMB_VR_PVDDQ_CD_ALERT_N)                                  \
				gpio_name_to_num(P0V8_ASICD_PWRGD)                                 \
					gpio_name_to_num(PWRGD_PVDDQ_CD)                           \
						gpio_name_to_num(SMB_VR_PASICA_ALERT_N)
#define name_gpioH                                                                                 \
	gpio_name_to_num(JTAG2_BIC_SHIFT_EN) gpio_name_to_num(SMB_VR_PVDDQ_AB_ALERT_N)             \
		gpio_name_to_num(SPI_BIC_SHIFT_EN) gpio_name_to_num(PWRGD_PVDDQ_AB)                \
			gpio_name_to_num(Reserve_GPIOH4) gpio_name_to_num(Reserve_GPIOH5)          \
				gpio_name_to_num(Reserve_GPIOH6) gpio_name_to_num(Reserve_GPIOH7)
#define name_gpioI                                                                                 \
	gpio_name_to_num(Reserve_GPIOI0) gpio_name_to_num(Reserve_GPIOI1)                          \
		gpio_name_to_num(Reserve_GPIOI2) gpio_name_to_num(Reserve_GPIOI3)                  \
			gpio_name_to_num(P0V9_ASICA_FT_R) gpio_name_to_num(PVDDQ_AB_FT_R)          \
				gpio_name_to_num(PVDDQ_CD_FT_R)                                    \
					gpio_name_to_num(FM_PWRBRK_PRIMARY_R_N)
#define name_gpioJ                                                                                 \
	gpio_name_to_num(Reserve_GPIOJ0) gpio_name_to_num(Reserve_GPIOJ1)                          \
		gpio_name_to_num(P0V8_ASICD_FT_R) gpio_name_to_num(P0V8_ASICA_FT_R)                \
			gpio_name_to_num(Reserve_GPIOJ4) gpio_name_to_num(Reserve_GPIOJ5)          \
				gpio_name_to_num(Reserve_GPIOJ6) gpio_name_to_num(Reserve_GPIOJ7)
#define name_gpioK                                                                                 \
	gpio_name_to_num(Reserve_GPIOK0) gpio_name_to_num(Reserve_GPIOK1)                          \
		gpio_name_to_num(Reserve_GPIOK2) gpio_name_to_num(Reserve_GPIOK3)                  \
			gpio_name_to_num(Reserve_GPIOK4) gpio_name_to_num(Reserve_GPIOK5)          \
				gpio_name_to_num(Reserve_GPIOK6) gpio_name_to_num(Reserve_GPIOK7)
#define name_gpioL                                                                                 \
	gpio_name_to_num(Reserve_GPIOL0) gpio_name_to_num(Reserve_GPIOL1)                          \
		gpio_name_to_num(LED_CXL_POWER) gpio_name_to_num(FM_BOARD_REV_ID2)                 \
			gpio_name_to_num(FM_BOARD_REV_ID1) gpio_name_to_num(FM_BOARD_REV_ID0)      \
				gpio_name_to_num(BOARD_ID0) gpio_name_to_num(BOARD_ID1)
// GPIOM6, M7 hardware not define
#define name_gpioM                                                                                 \
	gpio_name_to_num(BIC_SECUREBOOT) gpio_name_to_num(BOARD_ID2) gpio_name_to_num(BOARD_ID3)   \
		gpio_name_to_num(BIC_ESPI_SELECT) gpio_name_to_num(LED_CXL_FAULT)                  \
			gpio_name_to_num(Reserve_GPIOM5) gpio_name_to_num(Reserve_GPIOM6)          \
				gpio_name_to_num(Reserve_GPIOM7)
#define name_gpioN                                                                                 \
	gpio_name_to_num(Reserve_GPION0) gpio_name_to_num(Reserve_GPION1)                          \
		gpio_name_to_num(Reserve_GPION2) gpio_name_to_num(CLK_100M_OSC_EN)                 \
			gpio_name_to_num(Reserve_GPION4) gpio_name_to_num(Reserve_GPION5)          \
				gpio_name_to_num(Reserve_GPION6) gpio_name_to_num(Reserve_GPION7)
#define name_gpioO                                                                                 \
	gpio_name_to_num(Reserve_GPIOO0) gpio_name_to_num(Reserve_GPIOO1)                          \
		gpio_name_to_num(Reserve_GPIOO2) gpio_name_to_num(Reserve_GPIOO3)                  \
			gpio_name_to_num(Reserve_GPIOO4) gpio_name_to_num(Reserve_GPIOO5)          \
				gpio_name_to_num(Reserve_GPIOO6) gpio_name_to_num(Reserve_GPIOO7)
#define name_gpioP                                                                                 \
	gpio_name_to_num(Reserve_GPIOP0) gpio_name_to_num(Reserve_GPIOP1)                          \
		gpio_name_to_num(Reserve_GPIOP2) gpio_name_to_num(Reserve_GPIOP3)                  \
			gpio_name_to_num(Reserve_GPIOP4) gpio_name_to_num(Reserve_GPIOP5)          \
				gpio_name_to_num(Reserve_GPIOP6) gpio_name_to_num(Reserve_GPIOP7)
// GPIOQ5 hardware not define
#define name_gpioQ                                                                                 \
	gpio_name_to_num(Reserve_GPIOQ0) gpio_name_to_num(Reserve_GPIOQ1)                          \
		gpio_name_to_num(Reserve_GPIOQ2) gpio_name_to_num(Reserve_GPIOQ3)                  \
			gpio_name_to_num(Reserve_GPIOQ4) gpio_name_to_num(Reserve_GPIOQ5)          \
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
