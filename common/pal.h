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

#ifndef _pal_H
#define _pal_H

void pal_usb_handler(uint8_t *rx_buff,int rx_len);
bool pal_is_not_return_cmd(uint8_t netfn, uint8_t cmd);

// init
void pal_I2C_init(void);
void pal_BIC_init(void);

// sensor accessible
bool pal_stby_access(uint8_t snr_num);
bool pal_DC_access(uint8_t snr_num);

#endif
