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

#define ADC_CALIBRATION 1

#define WORKER_STACK_SIZE 4096
#define ENABLE_MCTP_I3C
#define ENABLE_SSIF
#define ENABLE_SSIF_RSP_PEC
#define ENABLE_SBMR
#define ENABLE_NVIDIA
#define ENABLE_DS160PT801
#define ENABLE_RS31380R

#define DISABLE_ISL28022
#define DISABLE_PEX89000
#define DISABLE_PCH
#define DISABLE_ADM1278
#define DISABLE_TPS53689
#define DISABLE_XDPE15284
#define DISABLE_LTC4282
#define DISABLE_TMP431
#define DISABLE_PMIC
#define DISABLE_INA233
#define DISABLE_ISL69254IRAZ_T
#define DISABLE_MAX16550A
#define DISABLE_XDP12284C
#define DISABLE_RAA229621
#define DISABLE_NCT7718W
#define DISABLE_LTC4286
#define DISABLE_XDPE19283B
#define DISABLE_G788P81U
#define DISABLE_MP2856GUT
#define DISABLE_DDR5_POWER
#define DISABLE_DDR5_TEMP
#define DISABLE_ADM1272
#define DISABLE_Q50SN120A1

#define MAX_FWUPDATE_RSP_BUF_SIZE 200 // for pldm fw update max transfer size
#define BIC_UPDATE_MAX_OFFSET 0x60000

#define SSIF_TIMEOUT_MS 1000

#endif
