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

#include <stdio.h>
#include "cmsis_os.h"
#include "board_device.h"
#include "objects.h"
#include "hal_gpio.h"
#include "plat_func.h"
#include "usb_api.h"
#include "usb_aspeed.h"

extern usb_t usb[];


void usb_hubrst_hanlder(uint32_t arvg0, uint32_t arvg1) {
  int i;

  
  printf("USB init\n");
  for (i = 0; i < sizeof(&usb) / sizeof(usb[0]); i++) {
    aspeed_usb_disconnect(&usb[i]);
    usb_init(&usb[i]);
  }
}
void ISR_usbhub() {
  func_work_add(usb_hubrst_hanlder, osPriorityBelowNormal, NULL, NULL);
}

void slp3_hanlder(uint32_t arvg0, uint32_t arvg1) {
  printf("power on S3...\n");
}
void ISR_slp3() {
  func_work_add(slp3_hanlder, osPriorityBelowNormal, NULL, NULL);
}

void ISR_pltrst() {
  //func_work_add(func, osPriorityBelowNormal, NULL, NULL);
  return;
}

