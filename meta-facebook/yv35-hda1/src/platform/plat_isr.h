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

#ifndef PLAT_FUNC_H
#define PLAT_FUNC_H

void ISR_POST_COMPLETE();
void ISR_CPU_PRSNT();
void ISR_CPU_FAULT_ALERT();
void ISR_CPU_JTAG_CMPL2();
void ISR_MPRO_BOOT_OK();
void ISR_MPRO_HB();
void ISR_CPU_SHD_ACK();
void ISR_CPU_REBOOT_ACK();
void ISR_CPU_OVERTEMP();
void ISR_CPU_HIGHTEMP();
void ISR_CPU_SYS_AUTH_FAIL();
void ISR_CPU_SPI_AUTH_FAIL();
void ISR_AC_STATUS();
void ISR_VRHOT();
void ISR_VRFAULT();
void ISR_SYS_RST_BMC();
void ISR_CPU_SPI_ACCESS();
void ISR_SALT4();
void ISR_SALT7();

void ISR_DC_ON();
void ISR_SLP3();
void ISR_DBP_PRSNT();
void ISR_HSC_THROTTLE();
void ISR_MB_THROTTLE();
void ISR_SOC_THMALTRIP();
void ISR_SYS_THROTTLE();
void ISR_HSC_OC();
void ISR_PVDDCR_CPU1_OCP();
void ISR_PVDDCR_CPU0_OCP();
void ISR_PVDD11_S3_OCP();
void ISR_UV_DETECT();
void ISR_PLTRST();
void ISR_APML_ALERT();

#endif
