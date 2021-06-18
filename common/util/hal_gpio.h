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

#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#define dedicate_gpio_num 9
#define total_gpio_num    168
#define ENABLE      1
#define DISABLE     0
#define chip_gpio   0
#define chip_sgpio  1
#define GPIO_INPUT  0
#define GPIO_OUTPUT 1
#define GPIO_LOW    0
#define GPIO_HIGH   1

#define GPIO_INT_FALLING_EDGE 0
#define GPIO_INT_RISING_EDGE  1
#define GPIO_INT_LEVEL_LOW    2
#define GPIO_INT_LEVEL_HIGH   3
#define GPIO_INT_BOTH_EDGE    4
#define GPIO_INT_DISABLE      5

#define OPEN_DRAIN 0
#define PUSH_PULL  1

typedef struct _GPIO_CFG_ {
  uint8_t chip;
  uint8_t number;
  uint8_t is_init;
  uint8_t direction;
  uint8_t status;
  uint8_t property;
  uint8_t int_type;
  uint8_t (*int_cb)(uint8_t);
} GPIO_CFG;

extern gpio_t gpio[];
extern const char * const gpio_name[];

extern uint8_t gpio_ind_to_num_table[];
extern uint8_t gpio_ind_to_num_table_cnt;

//void gpio_int_cb_test(void);
void gpio_show(void);
bool gpio_init(void);

#endif
