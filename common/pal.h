#ifndef _pal_H
#define _pal_H

#include "ipmb.h"

// init
void pal_I2C_init(void);
void pal_BIC_init(void);
bool pal_load_ipmb_config(void);

// fru
void pal_load_fru_config(void);

// gpio
bool pal_load_gpio_config(void);
void gpio_AD_callback_handler(uint32_t pins);
void gpio_EH_callback_handler(uint32_t pins);
void gpio_IL_callback_handler(uint32_t pins);
void gpio_MP_callback_handler(uint32_t pins);
void gpio_QT_callback_handler(uint32_t pins);
void gpio_UV_callback_handler(uint32_t pins);

#endif
