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
#include "gpio_aspeed.h"
#include "cmsis_compiler.h"
#include "hal_gpio.h"


uint8_t gpio_ind_to_num_table[200];
uint8_t gpio_ind_to_num_table_cnt;

__WEAK const char * const gpio_name[] = { };
__WEAK GPIO_CFG gpio_cfg[] = {
//  chip,      number,   is_init, direction,    status,     property,    int_type,           int_cb
//  Defalut              DISABLE  GPIO_INPUT    LOW         PUSH_PULL    GPIO_INT_DISABLE    NULL
};

bool gpio_set(uint8_t gpio_num, uint8_t status) {
  if (gpio_num >= total_gpio_num) {
    printf("setting invalid gpio num %d", gpio_num);
    return false;
  }

  if (gpio_cfg[gpio_num].property == OPEN_DRAIN) {
    if(status == GPIO_HIGH) {
      gpio[0].set_direction(&gpio[0], gpio_num, GPIO_INPUT);
      gpio[0].set(&gpio[0], gpio_num, status);
    } else { // set low
      gpio[0].set(&gpio[0], gpio_num, status);
      gpio[0].set_direction(&gpio[0], gpio_num, GPIO_OUTPUT);
    }
  } else { // push pull
       gpio[0].set(&gpio[0], gpio_num, status); 
  }

  return true;
}

void gpio_show(void) {
  uint8_t i;
  uint8_t j = 0;

  printf("Number: Group Status Direction Name\n");
  for(i = 0; i < total_gpio_num; i++) {
    if (gpio_cfg[i].is_init == ENABLE) {
      printf("%02d:     %c%d    %d      %s    %s\n", j++, ((i/8)+65), (i%8), gpio[0].get(&gpio[0], i), gpio[0].get_direction(&gpio[0], i) == 1 ? "Output" : "Input ", gpio_name[i]);
    }
  }
  printf("\n");

  return;
}

void gpio_index_to_num(void) {
  uint8_t i = 0;
  for(uint8_t j = 0; j < total_gpio_num; j++) {
    if(gpio_cfg[j].is_init == ENABLE) {
      gpio_ind_to_num_table[i++] = gpio_cfg[j].number;
    }
  }
  gpio_ind_to_num_table_cnt = i;
}

bool gpio_init(void) {
  CMB_ASSERT(gpio_name[dedicate_gpio_num-1] != NULL);
  CMB_ASSERT(gpio_cfg[dedicate_gpio_num-1].is_init != NULL);

  gpio_index_to_num();

  uint8_t i;

  for(i = 0; i < total_gpio_num; i++) {
    if (gpio_cfg[i].is_init == ENABLE) {
      if (gpio_cfg[i].chip == chip_gpio) {
        gpio_set(gpio_cfg[i].number, gpio_cfg[i].status);
        gpio[chip_gpio].set_direction(&gpio[chip_gpio], i, gpio_cfg[i].direction);
        gpio_set(gpio_cfg[i].number, gpio_cfg[i].status);
//printf("gpio[%d], chip %d, num %d, dir %d, status %d, name %s\n", i, gpio_cfg[i].chip, gpio_cfg[i].number, gpio_cfg[i].direction, gpio_cfg[i].status, gpio_name[i]);
        if (gpio_cfg[i].int_type < 5) {
          gpio[gpio_cfg[i].chip].int_cb_hook(&gpio[gpio_cfg[i].chip], gpio_cfg[i].number, gpio_cfg[i].int_type, gpio_cfg[i].int_cb, &gpio[gpio_cfg[i].chip]);
        } else {
          gpio[gpio_cfg[i].chip].int_cb_unhook(&gpio[gpio_cfg[i].chip], gpio_cfg[i].number);
        }
      } else {
        printf("TODO: add sgpio handler\n");
      }
    }
  }

  return 1;
}


