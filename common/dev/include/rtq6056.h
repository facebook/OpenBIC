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

#ifndef __RTQ6056__
#define __RTQ6056__

#define RTQ6056_BUS_VOL_OFFSET 0x02
#define RTQ6056_PWR_OFFSET 0x03
#define RTQ6056_CUR_OFFSET 0x04
#define RTQ6056_CALIBRATION_OFFSET 0x05
#define RTQ6056_MFR_ID_REG 0xFE

extern uint8_t RTQ6056_DEVICE_ID[2];

#endif
