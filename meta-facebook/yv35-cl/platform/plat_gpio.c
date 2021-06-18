/*
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include "cmsis_os.h"
#include "board_device.h"
#include "objects.h"
#include "gpio_aspeed.h"
#include "hal_gpio.h"
#include "plat_func.h"

// gpio_cfg(chip, number, is_init, direction, status, int_type, int_callback)
// dedicate gpio A0~A7, B0~B7, C0~C7, D0~D7, E0~E7, total 40 gpios
// Default name: Reserve_GPIOH0
#define name_gpioA \
  gpio_name_to_num(FM_BMC_PCH_SCI_LPC_R_N) \
  gpio_name_to_num(FM_BIOS_POST_CMPLT_BMC_N) \
  gpio_name_to_num(FM_SLPS3_PLD_N) \
  gpio_name_to_num(IRQ_BMC_PCH_SMI_LPC_R_N) \
  gpio_name_to_num(IRQ_UV_DETECT_N) \
  gpio_name_to_num(FM_UV_ADR_TRIGGER_EN_R) \
  gpio_name_to_num(IRQ_SMI_ACTIVE_BMC_N) \
  gpio_name_to_num(HSC_SET_EN_R) 
#define name_gpioB \
  gpio_name_to_num(FM_BIC_RST_RTCRST_R) \
  gpio_name_to_num(RST_USB_HUB_N_R) \
  gpio_name_to_num(A_P3V_BAT_SCALED_EN_R) \
  gpio_name_to_num(FM_SPI_PCH_MASTER_SEL_R) \
  gpio_name_to_num(FM_PCHHOT_N) \
  gpio_name_to_num(FM_SLPS4_PLD_N) \
  gpio_name_to_num(FM_S3M_CPU0_CD_INIT_ERROR) \
  gpio_name_to_num(PWRGD_SYS_PWROK) 
#define name_gpioC \
  gpio_name_to_num(FM_HSC_TIMER) \
  gpio_name_to_num(IRQ_SMB_IO_LVC3_STBY_ALRT_N) \
  gpio_name_to_num(IRQ_CPU0_VRHOT_N) \
  gpio_name_to_num(DBP_CPU_PREQ_BIC_N) \
  gpio_name_to_num(FM_CPU_THERMTRIP_LATCH_LVT3_N) \
  gpio_name_to_num(FM_CPU_SKTOCC_LVT3_PLD_N) \
  gpio_name_to_num(H_CPU_MEMHOT_OUT_LVC3_N) \
  gpio_name_to_num(RST_PLTRST_PLD_N) 
#define name_gpioD \
  gpio_name_to_num(PWRBTN_N) \
  gpio_name_to_num(RST_BMC_R_N) \
  gpio_name_to_num(H_BMC_PRDY_BUF_N) \
  gpio_name_to_num(BMC_READY) \
  gpio_name_to_num(BIC_READY) \
  gpio_name_to_num(FM_SOL_UART_CH_SEL_R) \
  gpio_name_to_num(HSC_MUX_SWITCH_R) \
  gpio_name_to_num(FM_FORCE_ADR_N_R) 
#define name_gpioE \
  gpio_name_to_num(PWRGD_CPU_LVC3) \
  gpio_name_to_num(FM_PCH_BMC_THERMTRIP_N) \
  gpio_name_to_num(FM_THROTTLE_R_N) \
  gpio_name_to_num(IRQ_HSC_ALERT2_N) \
  gpio_name_to_num(SMB_SENSOR_LVC3_ALERT_N) \
  gpio_name_to_num(FM_CPU_RMCA_CATERR_LVT3_N) \
  gpio_name_to_num(SYS_PWRBTN_N) \
  gpio_name_to_num(RST_PLTRST_BUF_N) 
#define name_gpioF \
  gpio_name_to_num(IRQ_BMC_PCH_NMI_R) \
  gpio_name_to_num(IRQ_SML1_PMBUS_ALERT_N) \
  gpio_name_to_num(IRQ_PCH_CPU_NMI_EVENT_N) \
  gpio_name_to_num(FM_BMC_DEBUG_ENABLE_N) \
  gpio_name_to_num(FM_DBP_PRESENT_N) \
  gpio_name_to_num(FM_FAST_PROCHOT_EN_N_R) \
  gpio_name_to_num(FM_SPI_MUX_OE_CTL_PLD_N) \
  gpio_name_to_num(FBRK_N_R) 
#define name_gpioG \
  gpio_name_to_num(FM_PEHPCPU_INT) \
  gpio_name_to_num(FM_BIOS_MRC_DEBUG_MSG_DIS_R) \
  gpio_name_to_num(FAST_PROCHOT_N) \
  gpio_name_to_num(FM_JTAG_TCK_MUX_SEL_R) \
  gpio_name_to_num(BMC_JTAG_SEL_R) \
  gpio_name_to_num(H_CPU_ERR0_LVC3_N) \
  gpio_name_to_num(H_CPU_ERR1_LVC3_N) \
  gpio_name_to_num(H_CPU_ERR2_LVC3_N) 
#define name_gpioH \
  gpio_name_to_num(RST_RSMRST_BMC_N) \
  gpio_name_to_num(FM_MP_PS_FAIL_N) \
  gpio_name_to_num(H_CPU_MEMTRIP_LVC3_N) \
  gpio_name_to_num(FM_CPU_BIC_PROCHOT_LVT3_N) \
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
  gpio_name_to_num(IRQ_PVCCD_CPU0_VRHOT_LVC3_N) \
  gpio_name_to_num(FM_PVCCIN_CPU0_PWR_IN_ALERT_N) \
  gpio_name_to_num(Reserve_GPIOL6) \
  gpio_name_to_num(Reserve_GPIOL7) 
// GPIOM6, M7 hardware not define
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
  gpio_name_to_num(SGPIO_BMC_CLK_R) \
  gpio_name_to_num(SGPIO_BMC_LD_R_N) \
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
  gpio_name_to_num(Reserve_GPIOP6) \
  gpio_name_to_num(Reserve_GPIOP7) 
// GPIOQ5 hardware not define  
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

#define gpio_name_to_num(x) x,
enum { name_gpioA name_gpioB };
#undef gpio_name_to_num

#define gpio_name_to_num(x) #x,
const char * const gpio_name[] = { name_gpioA name_gpioB name_gpioC name_gpioD name_gpioE name_gpioF name_gpioG name_gpioH name_gpioI name_gpioJ name_gpioK name_gpioL name_gpioM name_gpioN name_gpioO name_gpioP name_gpioQ name_gpioR name_gpioS name_gpioT name_gpioU };

GPIO_CFG gpio_cfg[] = {
//  chip,      number,   is_init, direction,    status,     property,    int_type,              int_cb
//  Defalut              DISABLE  GPIO_INPUT    LOW         PUSH_PULL    GPIO_INT_DISABLE       NULL
  { chip_gpio,  0,       ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            }, // GPIO A group
  { chip_gpio,  1,       ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  2,       ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_RISING_EDGE,  ISR_slp3        },
  { chip_gpio,  3,       ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  4,       ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  5,       ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  6,       ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  7,       ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  8,       ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO B group
  { chip_gpio,  9,       ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_RISING_EDGE,  ISR_usbhub      }, 
  { chip_gpio,  10,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  11,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  12,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  13,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  14,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  15,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  16,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            }, // GPIO C group
  { chip_gpio,  17,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  18,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  19,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  20,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  21,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  22,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  23,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  24,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            }, // GPIO D group
  { chip_gpio,  25,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  26,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  27,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  28,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  29,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  30,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  31,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  32,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO E group
  { chip_gpio,  33,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  34,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  35,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  36,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  37,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  38,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  39,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_RISING_EDGE,  ISR_pltrst      },
  { chip_gpio,  40,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO F group
  { chip_gpio,  41,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  42,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  43,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  44,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  45,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  46,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  47,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  48,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO G group
  { chip_gpio,  49,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  50,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  51,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  52,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  53,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  54,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  55,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  56,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO H group
  { chip_gpio,  57,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  58,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  59,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  60,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  61,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  62,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  63,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  64,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO I group
  { chip_gpio,  65,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  66,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  67,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  68,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  69,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  70,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  71,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  72,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO J group
  { chip_gpio,  73,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  74,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  75,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  76,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  77,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  78,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  79,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  80,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO K group
  { chip_gpio,  81,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  82,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  83,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  84,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  85,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  86,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  87,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  88,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO L group
  { chip_gpio,  89,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  90,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  91,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  92,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  93,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  94,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  95,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  96,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO M group
  { chip_gpio,  97,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  98,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  99,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  100,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  101,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  102,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  103,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  104,     ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO N group
  { chip_gpio,  105,     ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  106,     ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  107,     ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  108,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  109,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  110,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  111,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  112,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO O group
  { chip_gpio,  113,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  114,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  115,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  116,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  117,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  118,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  119,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  120,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO P group
  { chip_gpio,  121,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  122,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  123,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  124,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  125,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  126,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  127,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  128,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO Q group
  { chip_gpio,  129,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  130,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  131,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  132,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  133,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  134,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  135,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  136,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO R group
  { chip_gpio,  137,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  138,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  139,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  140,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  141,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  142,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  143,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  144,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO S group
  { chip_gpio,  145,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  146,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  147,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  148,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  149,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  150,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  151,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  152,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO T group
  { chip_gpio,  153,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  154,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  155,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  156,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  157,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  158,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  159,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  160,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO U group
  { chip_gpio,  161,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  162,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  163,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  164,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  165,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  166,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  167,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
};
