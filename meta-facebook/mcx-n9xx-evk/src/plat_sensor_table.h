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
 *
 * This EVK has no sensors wired to it - plat_sensor_config[] is
 * genuinely empty (SENSOR_CONFIG_SIZE == 0). sensor_init() reports
 * this as an error at boot ("Init sensor size is zero") - that's the
 * real common/service/sensor/sensor.c behavior, kept as-is rather than
 * silenced, since it's an honest reflection of this board's hardware.
 */

#ifndef PLAT_SENSOR_TABLE_H
#define PLAT_SENSOR_TABLE_H

#endif
