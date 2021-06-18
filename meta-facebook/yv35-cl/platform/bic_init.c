/*
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "hal_i2c.h"
#include "timer.h"
#include "ipmi.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "worker.h"
#include "kcs.h"
#include "usb.h"

void pal_BIC_init(void) {

  gpio_init();
  pal_I2C_init();
  util_init_I2C();
  util_init_timer();
  util_init_worker();
  sensor_init();
  ipmi_init();  
  host_kcs_init();
  usb_slavedev_init();

  return;
}

