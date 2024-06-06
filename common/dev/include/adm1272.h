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

#ifndef ADM1272_H
#define ADM1272_H

enum ADM1272_IRANGE {
	IRANGE_0MV_TO_15MV = 0x0,
	IRANGE_0MV_TO_30MV = 0x1,
};

enum ADM1272_VRANGE {
	VRANGE_0V_TO_60V = 0x0,
	VRANGE_0V_TO_100V = 0x1,
};

enum ADM1272_HSC {
	HSC_DISABLE = 0x0,
	HSC_ENABLE = 0x1,
};

bool enable_adm1272_hsc(uint8_t bus,uint8_t addr, uint8_t enable_flag);

#endif
