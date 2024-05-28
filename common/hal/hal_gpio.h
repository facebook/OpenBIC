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

#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <drivers/gpio.h>
#include <devicetree.h>

#define GEN_ENUM(ENUM) ENUM,
#define GEN_STR(STR) #STR,

#if defined(CONFIG_GPIO_ASPEED)
#define FOREACH_GPIO(APPLY)                                                                        \
	APPLY(GPIO0_A_D)                                                                            \
	APPLY(GPIO0_E_H)                                                                            \
	APPLY(GPIO0_I_L)                                                                            \
	APPLY(GPIO0_M_P)                                                                            \
	APPLY(GPIO0_Q_T)                                                                            \
	APPLY(GPIO0_U_V)

#define TOTAL_GPIO_NUM 168
#define GPIO_GROUP_NUM 6
#define GPIO_GROUP_SIZE 32
#define REG_GPIO_BASE 0x7e780000
#elif (CONFIG_GPIO_NPCM4XX)
#define FOREACH_GPIO(APPLY)                                                                        \
	APPLY(GPIO_0)                                                                              \
	APPLY(GPIO_1)                                                                              \
	APPLY(GPIO_2)                                                                              \
	APPLY(GPIO_3)                                                                              \
	APPLY(GPIO_4)                                                                              \
	APPLY(GPIO_5)                                                                              \
	APPLY(GPIO_6)                                                                              \
	APPLY(GPIO_7)                                                                              \
	APPLY(GPIO_8)                                                                              \
	APPLY(GPIO_9)                                                                              \
	APPLY(GPIO_A)                                                                              \
	APPLY(GPIO_B)                                                                              \
	APPLY(GPIO_C)                                                                              \
	APPLY(GPIO_D)                                                                              \
	APPLY(GPIO_E)                                                                              \
	APPLY(GPIO_F)

#define TOTAL_GPIO_NUM 124
#define GPIO_GROUP_NUM 16
#define GPIO_GROUP_SIZE 8
#define REG_GPIO_BASE 0x40081000
#else
#error "Unsupported GPIO driver"
#endif /* CONFIG_GPIO_ASPEED */

enum GPIO_GROUP { FOREACH_GPIO(GEN_ENUM) };

#define ENABLE 1
#define DISABLE 0
#define CHIP_GPIO 0
#define CHIP_SGPIO 1
#define GPIO_LOW 0
#define GPIO_HIGH 1
#define GPIO_STACK_SIZE 3072

#define OPEN_DRAIN 0
#define PUSH_PULL 1

#if defined(CONFIG_GPIO_ASPEED)
#define REG_ADC_BASE 0x7e6e9000
#define REG_SCU 0x7E6E2000
#define REG_DIRECTION_OFFSET 4
#define REG_INTERRUPT_ENABLE_OFFSET 8
#define REG_INTERRUPT_TYPE0_OFFSET 0x0C
#define REG_INTERRUPT_TYPE1_OFFSET 0x10
#define REG_INTERRUPT_TYPE2_OFFSET 0x14
#elif (CONFIG_GPIO_NPCM4XX)
#define REG_DEVALTX 0x400C3000
#define REG_DIRECTION_OFFSET 2
#else /* defined(CONFIG_GPIO_ASPEED) */
#endif /* defined(CONFIG_GPIO_ASPEED) */

extern uint32_t GPIO_GROUP_REG_ACCESS[];
extern uint32_t GPIO_MULTI_FUNC_PIN_CTL_REG_ACCESS[];
extern const int GPIO_MULTI_FUNC_CFG_SIZE;

#define GPIO_CFG_SIZE 168
typedef struct _GPIO_CFG_ {
	uint8_t chip;
	uint8_t number;
	uint8_t is_init;
	uint8_t is_latch;
	uint16_t direction;
	uint8_t status;
	uint8_t property;
	int int_type;
	void (*int_cb)();
} GPIO_CFG;

typedef struct _SET_GPIO_VALUE_CFG_ {
	uint8_t gpio_num;
	uint8_t gpio_value;
} SET_GPIO_VALUE_CFG;

extern GPIO_CFG gpio_cfg[];

enum POWER_STATUS {
	POWER_ON = GPIO_HIGH,
	POWER_OFF = GPIO_LOW,
};

enum CONTROL_STATUS {
	CONTROL_ON = GPIO_HIGH,
	CONTROL_OFF = GPIO_LOW,
};

enum GPIO_STATUS {
	LOW_ACTIVE = GPIO_LOW,
	LOW_INACTIVE = GPIO_HIGH,
	HIGH_ACTIVE = GPIO_HIGH,
	HIGH_INACTIVE = GPIO_LOW,
};

extern char *gpio_name[];

extern uint8_t gpio_ind_to_num_table[];
extern uint8_t gpio_ind_to_num_table_cnt;

typedef struct _SCU_CFG_ {
	int reg;
	int value;
} SCU_CFG;

//void gpio_int_cb_test(void);
void gpio_show(void);
int gpio_get(uint8_t);
int gpio_set(uint8_t, uint8_t);
bool pal_load_gpio_config(void);
int gpio_init(const struct device *args);
int gpio_interrupt_conf(uint8_t, gpio_flags_t);
uint8_t gpio_get_reg_value(uint8_t gpio_num, uint8_t reg_offset);
int gpio_conf(uint8_t gpio_num, int dir);
int gpio_get_direction(uint8_t gpio_num);
void scu_init(SCU_CFG cfg[], size_t size);

#endif
