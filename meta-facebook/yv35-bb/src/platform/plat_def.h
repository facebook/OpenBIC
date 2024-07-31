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

/* TODO: Enabling this causes build failures in
 * common/service/sensor/sensor.c:389:29: error: 'ast_fan_init' undeclared here
 * We might need to create initialization to support this. Till we have
 * it, disable the feature.
 * #define ENABLE_FAN
 */

#define BMC_USB_PORT "CDC_ACM_0"
#define HSC_DEVICE_READY_DELAY_MS 2000

#endif
