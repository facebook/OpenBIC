/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ARMCM3_H
#define ARMCM3_H

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum IRQn
{
/* -------------------  Processor Exceptions Numbers  ----------------------------- */
  NonMaskableInt_IRQn           = -14,     /*  2 Non Maskable Interrupt */
  HardFault_IRQn                = -13,     /*  3 HardFault Interrupt */
  MemoryManagement_IRQn         = -12,     /*  4 Memory Management Interrupt */
  BusFault_IRQn                 = -11,     /*  5 Bus Fault Interrupt */
  UsageFault_IRQn               = -10,     /*  6 Usage Fault Interrupt */
  SVCall_IRQn                   =  -5,     /* 11 SV Call Interrupt */
  DebugMonitor_IRQn             =  -4,     /* 12 Debug Monitor Interrupt */
  PendSV_IRQn                   =  -2,     /* 14 Pend SV Interrupt */
  SysTick_IRQn                  =  -1,     /* 15 System Tick Interrupt */

/* -------------------  Processor Interrupt Numbers  ------------------------------ */
  Interrupt0_IRQn               =   0,
  Interrupt1_IRQn               =   1,
  Interrupt2_IRQn               =   2,
  Interrupt3_IRQn               =   3,
  Interrupt4_IRQn               =   4,
  Interrupt5_IRQn               =   5,
  Interrupt6_IRQn               =   6,
  Interrupt7_IRQn               =   7,
  Interrupt8_IRQn               =   8,
  USB_IRQn               		=   9,
  /* Interrupts 10 .. 224 are left out */
  Gpio1p8_IRQn                  =   11,
  Jtag0_IRQn                    =   27,
  Peci_IRQn                     =   38,
  GpioPsp_IRQn                  =   40,
  Espi_IRQn                     =   42,  
  Tach_IRQn                     =   44,
  Sgpiom0_IRQn                  =   51,
  Jtag1_IRQn                    =   53,
  Sgpiom1_IRQn                  =   70, 
  GpioSsp_IRQn                  =   72,  
  /* Interrupts 10 .. 224 are left out */
  I2csec0_IRQn							=   75,
  I2csec1_IRQn							=   76,
  I2csec2_IRQn							=   77,
  I2csec3_IRQn							=   78,

  I3c0_IRQn			                =   102,
  I3c1_IRQn			                =   103,
  I3c2_IRQn			                =   104,
  I3c3_IRQn			                =   105,
  I3c4_IRQn			                =   106,
  I3c5_IRQn			                =   107,
  EspiMmbi_IRQn			            =   108,

  I2c0_IRQn							=   110,
  I2c1_IRQn							=   111,
  I2c2_IRQn							=   112,
  I2c3_IRQn							=   113,
  I2c4_IRQn							=   114,
  I2c5_IRQn							=   115,
  I2c6_IRQn							=   116,
  I2c7_IRQn							=   117,
  I2c8_IRQn							=   118,
  I2c9_IRQn							=   119,
  I2c10_IRQn						=   120,
  I2c11_IRQn						=   121,
  I2c12_IRQn						=   122,
  I2c13_IRQn						=   123,
  I2c14_IRQn						=   124,
  I2c15_IRQn						=   125,

  Kcs1_IRQn                         =   138,
  Kcs2_IRQn                         =   139,
  Kcs3_IRQn                         =   140,
  Kcs4_IRQn                         =   141,
  Bt_IRQn                           =   143,
  Snoop_IRQn                        =   144,
  Pcc_IRQn                          =   145,

  Ipi0_IRQn			                =   182,
  Ipi1_IRQn			                =   183,
  Ipi2_IRQn			                =   184,
  Ipi3_IRQn			                =   185,
  Ipi4_IRQn			                =   186,
  Ipi5_IRQn			                =   187,
  Ipi6_IRQn			                =   188,
  Ipi7_IRQn			                =   189,
  Ipi8_IRQn			                =   190,
  Ipi9_IRQn			                =   191,
  Ipi10_IRQn			              =   192,
  Ipi11_IRQn			              =   193,
  Ipi12_IRQn			              =   194,
  Ipi13_IRQn			              =   195,
  Ipi14_IRQn			              =   196,
  Ipi15_IRQn			              =   197,
} IRQn_Type;


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* -------  Start of section using anonymous unions and disabling warnings  ------- */
#if   defined (__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined (__ICCARM__)
  #pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wc11-extensions"
  #pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning 586
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif


/* --------  Configuration of Core Peripherals  ----------------------------------- */
#define __CM3_REV                 0x0201U   /* Core revision r2p1 */
#define __MPU_PRESENT             1U        /* MPU present */
#define __VTOR_PRESENT            1U        /* VTOR present */
#define __NVIC_PRIO_BITS          3U        /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0U        /* Set to 1 if different SysTick Config is used */

#include "core_cm3.h"                       /* Processor and core peripherals */
#include "system_ast2600.h"                  /* System Header */


/* --------  End of section using anonymous unions and disabling warnings  -------- */
#if   defined (__CC_ARM)
  #pragma pop
#elif defined (__ICCARM__)
  /* leave anonymous unions enabled */
#elif (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
  #pragma clang diagnostic pop
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning restore
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif


#ifdef __cplusplus
}
#endif

#endif  /* ARMCM3_H */
