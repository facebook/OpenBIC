#ifndef PLAT_GPIO_H
#define PLAT_GPIO_H

#include "hal_gpio.h"

// gpio_cfg(chip, number, is_init, direction, status, int_type, int_callback)
// dedicate gpio A0~A7, B0~B7, C0~C7, D0~D7, E0~E7, total 40 gpios
// Default name: Reserve_GPIOH0
#define name_gpioA                                                                                 \
	gpio_name_to_num(FM_BMC_PCH_SCI_LPC_R_N) gpio_name_to_num(FM_BIOS_POST_CMPLT_BMC_N)        \
		gpio_name_to_num(FM_SLPS3_PLD_N) gpio_name_to_num(IRQ_BMC_PCH_SMI_LPC_R_N)         \
			gpio_name_to_num(IRQ_UV_DETECT_N) gpio_name_to_num(FM_UV_ADR_TRIGGER_EN_R) \
				gpio_name_to_num(IRQ_SMI_ACTIVE_BMC_N)                             \
					gpio_name_to_num(HSC_SET_EN_R)
#define name_gpioB                                                                                 \
	gpio_name_to_num(FM_BIC_RST_RTCRST_R) gpio_name_to_num(RST_USB_HUB_N_R)                    \
		gpio_name_to_num(A_P3V_BAT_SCALED_EN_R) gpio_name_to_num(FM_SPI_PCH_MASTER_SEL_R)  \
			gpio_name_to_num(FM_PCHHOT_N) gpio_name_to_num(FM_SLPS4_PLD_N)             \
				gpio_name_to_num(FM_S3M_CPU0_CD_INIT_ERROR)                        \
					gpio_name_to_num(PWRGD_SYS_PWROK)
#define name_gpioC                                                                                 \
	gpio_name_to_num(FM_HSC_TIMER) gpio_name_to_num(IRQ_SMB_IO_LVC3_STBY_ALRT_N)               \
		gpio_name_to_num(IRQ_CPU0_VRHOT_N) gpio_name_to_num(DBP_CPU_PREQ_BIC_N)            \
			gpio_name_to_num(FM_CPU_THERMTRIP_LATCH_LVT3_N)                            \
				gpio_name_to_num(FM_CPU_SKTOCC_LVT3_PLD_N)                         \
					gpio_name_to_num(H_CPU_MEMHOT_OUT_LVC3_N)                  \
						gpio_name_to_num(RST_PLTRST_PLD_N)
#define name_gpioD                                                                                 \
	gpio_name_to_num(PWRBTN_N) gpio_name_to_num(RST_BMC_R_N)                                   \
		gpio_name_to_num(H_BMC_PRDY_BUF_N) gpio_name_to_num(BMC_READY)                     \
			gpio_name_to_num(BIC_READY) gpio_name_to_num(FM_RMCA_LVT3_N)               \
				gpio_name_to_num(HSC_MUX_SWITCH_R)                                 \
					gpio_name_to_num(FM_FORCE_ADR_N_R)
#define name_gpioE                                                                                 \
	gpio_name_to_num(PWRGD_CPU_LVC3) gpio_name_to_num(FM_PCH_BMC_THERMTRIP_N)                  \
		gpio_name_to_num(FM_THROTTLE_R_N) gpio_name_to_num(IRQ_HSC_ALERT2_N)               \
			gpio_name_to_num(SMB_SENSOR_LVC3_ALERT_N)                                  \
				gpio_name_to_num(FM_CATERR_LVT3_N) gpio_name_to_num(SYS_PWRBTN_N)  \
					gpio_name_to_num(RST_PLTRST_BUF_N)
#define name_gpioF                                                                                 \
	gpio_name_to_num(IRQ_BMC_PCH_NMI_R) gpio_name_to_num(IRQ_SML1_PMBUS_ALERT_N)               \
		gpio_name_to_num(IRQ_PCH_CPU_NMI_EVENT_N) gpio_name_to_num(FM_BMC_DEBUG_ENABLE_N)  \
			gpio_name_to_num(FM_DBP_PRESENT_N)                                         \
				gpio_name_to_num(FM_FAST_PROCHOT_EN_N_R)                           \
					gpio_name_to_num(FM_SPI_MUX_OE_CTL_PLD_N)                  \
						gpio_name_to_num(FBRK_N_R)
#define name_gpioG                                                                                 \
	gpio_name_to_num(FM_PEHPCPU_INT) gpio_name_to_num(FM_BIOS_MRC_DEBUG_MSG_DIS_R)             \
		gpio_name_to_num(FAST_PROCHOT_N) gpio_name_to_num(FM_JTAG_TCK_MUX_SEL_R)           \
			gpio_name_to_num(BMC_JTAG_SEL_R) gpio_name_to_num(H_CPU_ERR0_LVC3_N)       \
				gpio_name_to_num(H_CPU_ERR1_LVC3_N)                                \
					gpio_name_to_num(H_CPU_ERR2_LVC3_N)
#define name_gpioH                                                                                 \
	gpio_name_to_num(RST_RSMRST_BMC_N) gpio_name_to_num(FM_MP_PS_FAIL_N)                       \
		gpio_name_to_num(H_CPU_MEMTRIP_LVC3_N) gpio_name_to_num(FM_CPU_BIC_PROCHOT_LVT3_N) \
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
	gpio_name_to_num(Reserve_GPIOL0) gpio_name_to_num(Reserve_GPIOL1)                          \
		gpio_name_to_num(Reserve_GPIOL2) gpio_name_to_num(BOARD_ID2)                       \
			gpio_name_to_num(IRQ_PVCCD_CPU0_VRHOT_LVC3_N)                              \
				gpio_name_to_num(FM_PVCCIN_CPU0_PWR_IN_ALERT_N)                    \
					gpio_name_to_num(BOARD_ID0) gpio_name_to_num(BOARD_ID1)
// GPIOM6, M7 hardware not define
#define name_gpioM                                                                                 \
	gpio_name_to_num(Reserve_GPIOM0) gpio_name_to_num(BOARD_ID3)                               \
		gpio_name_to_num(Reserve_GPIOM2) gpio_name_to_num(FM_THROTTLE_IN_N)                \
			gpio_name_to_num(AUTH_COMPLETE) gpio_name_to_num(AUTH_PRSNT_N)             \
				gpio_name_to_num(Reserve_GPIOM6) gpio_name_to_num(Reserve_GPIOM7)
#define name_gpioN                                                                                 \
	gpio_name_to_num(SGPIO_BMC_CLK_R) gpio_name_to_num(SGPIO_BMC_LD_R_N)                       \
		gpio_name_to_num(SGPIO_BMC_DOUT_R) gpio_name_to_num(SGPIO_BMC_DIN)                 \
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

void enable_PRDY_interrupt();
void disable_PRDY_interrupt();
#endif
