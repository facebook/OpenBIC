#include <zephyr.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "hal_gpio.h"
#include "pal.h"

#define STACK_SIZE  1024

static const struct device *dev_gpio[GPIO_GROUP_NUM];
static struct gpio_callback callbacks[total_gpio_num];
static struct k_work_q gpio_work_queue;
static K_THREAD_STACK_DEFINE(gpio_work_stack, STACK_SIZE);

struct k_work gpio_work[total_gpio_num];

struct gpio_isr_info_t {
  struct k_work gpio_isr_work;
  uint32_t group;
  uint32_t bit_pin;
} gpio_isr_info;

uint8_t gpio_ind_to_num_table[200];
uint8_t gpio_ind_to_num_table_cnt;

__weak const char * const gpio_name[] = { };
GPIO_CFG gpio_cfg[GPIO_CFG_SIZE] = {
//  chip,      number,   is_init, direction,    status,     property,    int_type,           int_cb
//  Defalut              DISABLE  GPIO_INPUT    LOW         PUSH_PULL    GPIO_INT_DISABLE    NULL
};




//void gpio_isr_handler(uint32_t group, uint32_t pin) {
void gpio_isr_handler(struct k_work *work) {
  int i;
  uint8_t gpio_num;
  struct gpio_isr_info_t *drv_data = CONTAINER_OF(work, struct gpio_isr_info_t, gpio_isr_work);

  for(i = 0; i < GPIO_GROUP_SIZE; i++) {
    if ( (drv_data->bit_pin >> i) & 0x1 ) {
      drv_data->bit_pin = i;
      break;
    }
  }
  if ( i == GPIO_GROUP_SIZE ) {
    printk("gpio_isr_handler: pin %x not found\n", drv_data->bit_pin);
    return;
  }

  gpio_num = (drv_data->group * GPIO_GROUP_SIZE) + drv_data->bit_pin;
  if ( gpio_cfg[gpio_num].int_cb == NULL) {
    printk("CB func pointer NULL for gpio num %d\n", gpio_num);
    return;
  }

  k_work_submit_to_queue(&gpio_work_queue, &gpio_work[gpio_num]);
}

void irq_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
  uint8_t group;

  if( dev == dev_gpio[gpio_a_d] ) {
    group = gpio_a_d;
  } else if ( dev == dev_gpio[gpio_e_h] ) {
    group = gpio_e_h;
  } else if ( dev == dev_gpio[gpio_i_l] ) {
    group = gpio_i_l;
  } else if ( dev == dev_gpio[gpio_m_p] ) {
    group = gpio_m_p;
  } else if ( dev == dev_gpio[gpio_q_t] ) {
    group = gpio_q_t;
  } else if ( dev == dev_gpio[gpio_u_v] ) {
    group = gpio_u_v;
  } else {
    printk("invalid dev group for isr cb\n");
    return;
  }

  gpio_isr_info.group = group;
  gpio_isr_info.bit_pin = pins;
  
  k_work_submit_to_queue(&gpio_work_queue, &gpio_isr_info.gpio_isr_work);
}

static void gpio_init_cb(uint8_t gpio_num) {
  gpio_init_callback(&callbacks[gpio_num], irq_callback, BIT(gpio_num % GPIO_GROUP_SIZE));
  return;
}

static int gpio_add_cb(uint8_t gpio_num) {
  return gpio_add_callback(dev_gpio[gpio_num / GPIO_GROUP_SIZE], &callbacks[gpio_num]);
}
/*
interrupt type:
  GPIO_INT_DISABLE
  GPIO_INT_EDGE_RISING
  GPIO_INT_EDGE_FALLING
  GPIO_INT_EDGE_BOTH
  GPIO_INT_LEVEL_LOW
  GPIO_INT_LEVEL_HIGH
*/
static int gpio_interrupt_conf(uint8_t gpio_num, gpio_flags_t flags) {
  return gpio_pin_interrupt_configure(dev_gpio[gpio_num / GPIO_GROUP_SIZE], (gpio_num % GPIO_GROUP_SIZE), flags);
}

void gpio_cb_irq_init(uint8_t gpio_num, gpio_flags_t flags) {
  gpio_init_cb(gpio_num);
  gpio_add_cb(gpio_num);
  gpio_interrupt_conf(gpio_num, flags);
  
  k_work_init(&gpio_work[gpio_num], gpio_cfg[gpio_num].int_cb);
}


uint8_t gpio_conf(uint8_t gpio_num, int dir) {
  return gpio_pin_configure(dev_gpio[gpio_num/GPIO_GROUP_SIZE], (gpio_num%GPIO_GROUP_SIZE), dir);
}

int gpio_get(uint8_t gpio_num) {
  if (gpio_num >= total_gpio_num) {
    printf("getting invalid gpio num %d", gpio_num);
    return false;
  }

  return gpio_pin_get(dev_gpio[gpio_num/GPIO_GROUP_SIZE], (gpio_num%GPIO_GROUP_SIZE) );
//  return gpio[0].get(&gpio[0], gpio_num);  
}

int gpio_set(uint8_t gpio_num, uint8_t status) {
  if (gpio_num >= total_gpio_num) {
    printf("setting invalid gpio num %d", gpio_num);
    return false;
  }

  if (gpio_cfg[gpio_num].property == OPEN_DRAIN) { // should release gpio ctrl for OD high
    if(status) {
      return gpio_conf(gpio_num, GPIO_INPUT);
    } else {
      gpio_conf(gpio_num, GPIO_OUTPUT);
      return gpio_pin_set(dev_gpio[gpio_num/GPIO_GROUP_SIZE], (gpio_num%GPIO_GROUP_SIZE), status);
    }
  } else {
    return gpio_pin_set(dev_gpio[gpio_num/GPIO_GROUP_SIZE], (gpio_num%GPIO_GROUP_SIZE), status);
  }
}

/*void gpio_show(void) {
  uint8_t i, j, status;
  char direction[6];
  printf("Number: Group Status Direction Name\n");
  for(i = 0; i < total_gpio_num; i++) {
    if (gpio_cfg[i].is_init == ENABLE) {
      printf("%02d:     %c%d    %d      %s    %s\n", j++, ((i/8)+65), (i%8), gpio[0].get(&gpio[0], i), gpio[0].get_direction(&gpio[0], i) == 1 ? "Output" : "Input ", gpio_name[i]);
    }
  }
  printf("\n");

  return;
}*/

void gpio_index_to_num(void) {
  uint8_t i = 0;
  for(uint8_t j = 0; j < total_gpio_num; j++) {
    if(gpio_cfg[j].is_init == ENABLE) {
      gpio_ind_to_num_table[i++] = gpio_cfg[j].number;
    }
  }
  gpio_ind_to_num_table_cnt = i;
}

void init_gpio_dev(void) {
#ifdef DEV_GPIO_A_D
  dev_gpio[gpio_a_d] = device_get_binding("GPIO0_A_D");
#endif
#ifdef DEV_GPIO_E_H
  dev_gpio[gpio_e_h] = device_get_binding("GPIO0_E_H");
#endif
#ifdef DEV_GPIO_I_L
  dev_gpio[gpio_i_l] = device_get_binding("GPIO0_I_L");
#endif
#ifdef DEV_GPIO_M_P
  dev_gpio[gpio_m_p] = device_get_binding("GPIO0_M_P");
#endif
#ifdef DEV_GPIO_Q_T
  dev_gpio[gpio_q_t] = device_get_binding("GPIO0_Q_T");
#endif
#ifdef DEV_GPIO_U_V
  dev_gpio[gpio_u_v] = device_get_binding("GPIO0_U_V");
#endif
}

bool gpio_init(void) {
  uint8_t i;

  pal_load_gpio_config();
  init_gpio_dev();
  gpio_index_to_num();

  // gpio init is much earily than main init seq
  // set boot source here to skip gpio init for latched gpios
  set_boot_source();

  k_work_queue_start(&gpio_work_queue, gpio_work_stack, STACK_SIZE, K_PRIO_PREEMPT(osPriorityBelowNormal), NULL);
  k_thread_name_set(&gpio_work_queue.thread, "gpio_workq");

  k_work_init(&gpio_isr_info.gpio_isr_work, gpio_isr_handler);

  for(i = 0; i < total_gpio_num; i++) {
    if (gpio_cfg[i].is_init == ENABLE) {
      if (gpio_cfg[i].is_latch == ENABLE && !get_boot_source_ACon()) {
        continue;
      }
      if (gpio_cfg[i].chip == chip_gpio) {
        gpio_set(gpio_cfg[i].number, gpio_cfg[i].status);
        if (gpio_cfg[i].property == PUSH_PULL) { // OD config is set during status set
          gpio_conf(gpio_cfg[i].number, gpio_cfg[i].direction);
        }
        gpio_set(gpio_cfg[i].number, gpio_cfg[i].status);
        if ( (gpio_cfg[i].int_type == GPIO_INT_EDGE_RISING) ||
          (gpio_cfg[i].int_type == GPIO_INT_EDGE_FALLING) || 
          (gpio_cfg[i].int_type == GPIO_INT_EDGE_BOTH) || 
          (gpio_cfg[i].int_type == GPIO_INT_LEVEL_LOW) || 
          (gpio_cfg[i].int_type == GPIO_INT_LEVEL_HIGH) ) {
          gpio_cb_irq_init(gpio_cfg[i].number, gpio_cfg[i].int_type);
        }
      } else {
        printf("TODO: add sgpio handler\n");
      }
    }
  }

  return 1;
}
