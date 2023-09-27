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

#ifndef SQ52205_H
#define SQ52205_H

#define SQ52205_READ_VOL_OFFSET 0x02
#define SQ52205_READ_PWR_OFFSET 0x03
#define SQ52205_READ_CUR_OFFSET 0x04
#define SQ52205_READ_EIN_OFFSET 0x0A

#define SQ52205_ENABLE_OP BIT(0)
#define SQ52205_ENABLE_BUS_VOL_UV BIT(1)
#define SQ52205_ENABLE_BUS_VOL_OV BIT(2)
#define SQ52205_ENABLE_SHUNT_VOL_UV BIT(3)
#define SQ52205_ENABLE_SHUNT_VOL_OV BIT(4)

#endif
