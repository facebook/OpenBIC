/*
 * Copyright (c) 2020-2021 ASPEED Technology Inc.
 * Copyright (c) 2009-2020 Arm Limited. All rights reserved.
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
#include "config.h"
#include CONFIG_SOC_INCLUDE_FILE

/*----------------------------------------------------------------------------
  User Initial Stack & Heap
 *----------------------------------------------------------------------------*/
#ifndef __STACK_SIZE
  #define	__STACK_SIZE  0x00001000
#endif
static uint8_t stack[__STACK_SIZE] __attribute__ ((aligned(8), used, section(".stack")));

#if CONFIG_APPLICATION_ALLOCATED_HEAP
uint8_t ucHeap[CONFIG_TOTAL_HEAP_SIZE]   __attribute__ ((aligned(8), used, section(".heap")));
uint8_t ucHeapNc[CONFIG_TOTAL_HEAP_NC_SIZE]   __attribute__ ((aligned(8), used, section(".heap.nocache")));
#endif

extern unsigned int _end_stack;
extern unsigned int _end_text;
extern unsigned int _start_data;
extern unsigned int _end_data;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __bss_nc_start__;
extern uint32_t __bss_nc_end__;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __etext;
extern uint32_t __data_nc_start__;
extern uint32_t __data_nc_end__;
extern uint32_t __etext_nc;


/*----------------------------------------------------------------------------
  Exception / Interrupt Handler Function Prototype
 *----------------------------------------------------------------------------*/
typedef void( *pFunc )( void );

/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
extern uint32_t __INITIAL_SP;

extern __NO_RETURN void __PROGRAM_START(void) ;

/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
__NO_RETURN void Default_Handler(void);
__NO_RETURN void Reset_Handler  (void);

extern void aspeed_ipi_handler(void);
extern int main();

extern void xPortSysTickHandler(void);
extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Exceptions */
void NMI_Handler            (void) __attribute__ ((weak, __noreturn__, alias("Default_Handler")));
void HardFault_Handler      (void) __attribute__ ((weak, __noreturn__, alias("Default_Handler")));
void MemManage_Handler      (void) __attribute__ ((weak, __noreturn__, alias("Default_Handler")));
void BusFault_Handler       (void) __attribute__ ((weak, __noreturn__, alias("Default_Handler")));
void UsageFault_Handler     (void) __attribute__ ((weak, __noreturn__, alias("Default_Handler")));
//void SVC_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler       (void) __attribute__ ((weak, __noreturn__, alias("Default_Handler")));
void PendSV_Handler         (void) __attribute__ ((weak, __noreturn__, alias("Default_Handler")));

//void Interrupt0_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));

extern void systick_handler ( void );

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
extern const pFunc __VECTOR_TABLE[256];
const pFunc __VECTOR_TABLE[256] __VECTOR_TABLE_ATTRIBUTE = {
	(pFunc)(&__INITIAL_SP), /*     Initial Stack Pointer */
	Reset_Handler,          /*     Reset Handler */
	NMI_Handler,            /* -14 NMI Handler */
	HardFault_Handler,      /* -13 Hard Fault Handler */
	MemManage_Handler,      /* -12 MPU Fault Handler */
	BusFault_Handler,       /* -11 Bus Fault Handler */
	UsageFault_Handler,     /* -10 Usage Fault Handler */
	0,                      /*     Reserved */
	0,                      /*     Reserved */
	0,                      /*     Reserved */
	0,                      /*     Reserved */
	vPortSVCHandler,        /*  -5 SVCall Handler */
	DebugMon_Handler,       /*  -4 Debug Monitor Handler */
	0,                      /*     Reserved */
	xPortPendSVHandler,     /*  -2 PendSV Handler */
	xPortSysTickHandler,    /*  -1 SysTick Handler */

	/* Interrupts */
	[16 ... 255] = Default_Handler, /*   0 - 239 reserved */
};

struct sb_header {
	uint32_t key_location;
	uint32_t enc_img_addr;
	uint32_t img_size;
	uint32_t sign_location;
	uint32_t header_rev[2];
	uint32_t patch_location; /* address of the rom patch */
	uint32_t checksum;
};

struct sb_header sbh __attribute((used, section(".sboot"))) = {
	.img_size = (uint32_t)&__data_end__,
};

/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
void Reset_Handler(void)
{
	uint32_t *pDest;
	uint32_t *pSrc;

	/* clear BSS in cached region */
	pDest = &__bss_start__;
	for (; pDest < &__bss_end__;) {
		*pDest++ = 0UL;
	}

	/* clear BSS in non-cached region */
	pDest = &__bss_nc_start__;
	for (; pDest < &__bss_nc_end__;) {
		*pDest++ = 0UL;
	}

	/* copy DATA in non-cached region */
	pSrc = &__etext_nc;
	pDest = &__data_nc_start__;
	for (; pDest < &__data_nc_end__;) {
		*pDest++ = *pSrc++;
	}

	SystemInit ();
#if 0
	/* turn on this code if C lib _start is included */
	__PROGRAM_START();	/* Enter PreMain (C library entry point) */	
#else
	main();
	while(1);		/* Never go here, just to avoid build warning */
#endif	
}

/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void) 
{
	while (1);
}