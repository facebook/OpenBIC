#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <drivers/gpio.h>
#include <devicetree.h>

#if DT_NODE_EXISTS(DT_NODELABEL(gpio0_a_d))
#define DEV_GPIO_A_D
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(gpio0_e_h))
#define DEV_GPIO_E_H
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(gpio0_i_l))
#define DEV_GPIO_I_L
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(gpio0_m_p))
#define DEV_GPIO_M_P
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(gpio0_q_t))
#define DEV_GPIO_Q_T
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(gpio0_u_v))
#define DEV_GPIO_U_V
#endif

#define dedicate_gpio_num 9
#define total_gpio_num    168
#define ENABLE      1
#define DISABLE     0
#define chip_gpio   0
#define chip_sgpio  1
#define GPIO_LOW    0
#define GPIO_HIGH   1
#define GPIO_GROUP_NUM 6
#define GPIO_GROUP_SIZE 32

#define OPEN_DRAIN 0
#define PUSH_PULL  1

#define GPIO_CFG_SIZE 100
typedef struct _GPIO_CFG_ {
  uint8_t chip;
  uint8_t number;
  uint8_t is_init;
  uint16_t direction;
  uint8_t status;
  uint8_t property;
  int int_type;
  void* (*int_cb)(int);
} GPIO_CFG;

extern GPIO_CFG gpio_cfg[];

enum {
  gpio_a_d,
  gpio_e_h,
  gpio_i_l,
  gpio_m_p,
  gpio_q_t,
  gpio_u_v,
};

extern const char * const gpio_name[];

extern uint8_t gpio_ind_to_num_table[];
extern uint8_t gpio_ind_to_num_table_cnt;

//void gpio_int_cb_test(void);
void gpio_show(void);
int gpio_get(uint8_t);
int gpio_set(uint8_t, uint8_t);
bool gpio_init(void);

#endif
