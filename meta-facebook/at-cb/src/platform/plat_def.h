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

#ifndef PLAT_DEF_H
#define PLAT_DEF_H

#define BMC_USB_PORT "CDC_ACM_0"
#define KEYWORD_CPLD_LATTICE "LCMXO3-4300C"
#define BIC_FW_VERSION_ADD_FRU_NAME
#define FW_UPDATE_RETRY_MAX_COUNT 4
#define MORE_THAN_ONE_ADM1272

#define DISABLE_ISL69259
#define DISABLE_MP5990
#define DISABLE_ISL28022
#define DISABLE_PCH
#define DISABLE_ADM1278
#define ISABLE_TPS53689
#define DISABLE_LTC4282
#define DISABLE_TMP431
#define DISABLE_PMIC
#define DISABLE_ISL69254IRAZ_T
#define DISABLE_MAX16550A
#define DISABLE_XDP12284C
#define DISABLE_RAA229621
#define DISABLE_NCT7718W
#define DISABLE_XDPE19283B
#define DISABLE_G788P81U
#define DISABLE_DDR5_POWER
#define DISABLE_DDR5_TEMP
#define DISABLE_MP2971
#define DISABLE_LTC2991
#define DISABLE_EMC1412
#define DISABLE_LM75BD118
#define DISABLE_TMP461
#define DISABLE_M88RT51632
#define DISABLE_CX7
#define DISABLE_MAX11617
#define DISABLE_NCT7363
#define DISABLE_ADS112C
#define DISABLE_HDC1080
#define DISABLE_INA238
#define DISABLE_NCT214
#define DISABLE_AST_TACH
#define DISABLE_ADC128D818
#define DISABLE_I3C_DIMM
#define DISABLE_PT5161L

#define PLDM_SEND_FAIL_DELAY_MS                                                                    \
	20 // Workaround to avoid responding too frequently and being unable to reply

#endif
