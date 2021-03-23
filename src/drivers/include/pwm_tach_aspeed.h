/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _PWM_TACH_ASPEED_H_
#define _PWM_TACH_ASPEED_H_
#include "hal_def.h"
#include "objects.h"

/**********************************************************
 * PWM register fields
 *********************************************************/
typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t pwm_clock_division_l:8; /*[0-7]*/ 
		volatile uint32_t pwm_clock_division_h:4; /*[8-11]*/ 
		volatile uint32_t enable_pwm_pin:1; /*[12-12]*/ 
		volatile uint32_t enable_open_drain:1; /*[13-13]*/ 
		volatile uint32_t inverse_pwm_pin:1; /*[14-14]*/ 
		volatile uint32_t output_pwm_level:1; /*[15-15]*/ 
		volatile uint32_t enable_pwm_clock:1; /*[16-16]*/ 
		volatile uint32_t disable_pwm_duty_instant_change:1; /*[17-17]*/ 
		volatile uint32_t enable_pwm_duty_load_as_wdt:1; /*[18-18]*/ 
		volatile uint32_t load_selection_of_duty_as_wdt:1; /*[19-19]*/ 
		volatile uint32_t :12; /*[20-31]*/ 

	} fields;
} pwm_general_register_t; /* 00000000 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t pwm_rising_point:8; /*[0-7]*/ 
		volatile uint32_t pwm_falling_point:8; /*[8-15]*/ 
		volatile uint32_t pwm_rising_falling_point_as_wdt:8; /*[16-23]*/ 
		volatile uint32_t pwm_period:8; /*[24-31]*/ 

	} fields;
} pwm_duty_cycle_register_t; /* 00000004 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t tach_threshold:20; /*[0-19]*/ 
		volatile uint32_t tach_clock_division:4; /*[20-23]*/ 
		volatile uint32_t tach_edge:2; /*[24-25]*/ 
		volatile uint32_t tach_debounce:2; /*[26-27]*/ 
		volatile uint32_t enable_tach:1; /*[28-28]*/ 
		volatile uint32_t enable_tach_loopback_mode:1; /*[29-29]*/ 
		volatile uint32_t inverse_tach_limit_comparison:1; /*[30-30]*/ 
		volatile uint32_t enable_tach_interrupt:1; /*[31-31]*/ 

	} fields;
} tach_general_register_t; /* 00000008 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t tach_value:20; /*[0-19]*/ 
		volatile uint32_t tach_full_measurement:1; /*[20-20]*/ 
		volatile uint32_t tach_value_updated_since_last_read:1; /*[21-21]*/ 
		volatile uint32_t tach_raw_input:1; /*[22-22]*/ 
		volatile uint32_t tach_debounce_input:1; /*[23-23]*/ 
		volatile uint32_t pwm_out_status:1; /*[24-24]*/ 
		volatile uint32_t pwm_oen_status:1; /*[25-25]*/ 
		volatile uint32_t :5; /*[26-30]*/ 
		volatile uint32_t interrupt_status_and_clear:1; /*[31-31]*/ 

	} fields;
} tach_status_register_t; /* 0000000c */

typedef struct
{
    pwm_general_register_t pwm_general;
	pwm_duty_cycle_register_t pwm_duty_cycle;
	tach_general_register_t tach_general;
	tach_status_register_t tach_status;
} pwm_tach_gather_register_t;

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t pwm_clock_gating:16; /*[0-15]*/ 
		volatile uint32_t :16; /*[16-31]*/ 

	} fields;
} pwm_g100_register_t; /* 00000100 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t pwm_duty_gating:16; /*[0-15]*/ 
		volatile uint32_t :16; /*[16-31]*/ 

	} fields;
} pwm_g104_register_t; /* 00000104 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t :32; /*[0-31]*/ 

	} fields;
} pwm_g108_register_t; /* 00000108 */

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t interrupt_status:16; /*[0-15]*/ 
		volatile uint32_t :16; /*[16-31]*/ 

	} fields;
} pwm_g10c_register_t; /* 0000010c */

typedef struct{
    pwm_tach_gather_register_t pwm_tach_gather[16];
    pwm_g100_register_t pwm_g100; /* 00000100 */
    pwm_g104_register_t pwm_g104; /* 00000104 */
    pwm_g108_register_t pwm_g108; /* 00000108 */
    pwm_g10c_register_t pwm_g10c; /* 0000010c */
} pwm_tach_register_t;

/**********************************************************
 * PWM_TACH macro
 *********************************************************/
#define PWM_CHANNEL_NUM 16
#define TACH_CHANNEL_NUM 16
#define DEFAULT_PWM_PERIOD 0xff
/* load_selection_of_duty_as_wdt */
#define LOAD_SEL_FALLING 0
#define LOAD_SEL_RIGING  1

/* inverse_tach_limit_comparison */
#define COMPARE_MORE 1
#define COMPARE_LESS 0

/**********************************************************
 * PWM_TACH private data
 *********************************************************/
typedef struct aspeed_g_pwm_tach_priv_s {
	void *pwm[PWM_CHANNEL_NUM];
	void *tach[TACH_CHANNEL_NUM];
	uint16_t pwm_used_bit_map;
	uint16_t tach_used_bit_map;
} aspeed_g_pwm_tach_priv_t;

typedef void (*callback)(void *);

typedef struct aspeed_tach_adapter_s{
    callback cb;
    void *para;
} aspeed_tach_adapter_t;

typedef struct aspeed_pwm_priv_s {
    void *parent;
	uint32_t pwm_freq;
	uint32_t clk_source;
	uint8_t pwm_channel;
	uint8_t default_duty;
	uint8_t wdt_reload_enable;
	uint8_t wdt_reload_duty;
} aspeed_pwm_priv_t;

typedef struct aspeed_tach_priv_s {
	void *parent;
	uint32_t tach_freq;
	uint32_t tach_div;
	uint32_t sample_period;
	uint32_t clk_source;
	aspeed_tach_adapter_t adapter;
	osEventFlagsId_t evt_id;
	uint32_t fan_min_rpm;
	uint8_t cmp_more_less;
	uint8_t tach_channel;
	uint8_t fan_pulse_pr;
} aspeed_tach_priv_t;

/**********************************************************
 * PWM_TACH export api
 *********************************************************/
uint16_t aspeed_g_pwm_tach_get_pwm_status(aspeed_device_t *g_pwm_tach);
uint16_t aspeed_g_pwm_tach_get_tach_status(aspeed_device_t *g_pwm_tach);
hal_status_t aspeed_pwm_init(pwm_t *obj);
hal_status_t aspeed_pwm_set_freq(pwm_t *obj, uint32_t freq);
hal_status_t aspeed_pwm_set_duty(pwm_t *obj, uint8_t percent);
uint8_t aspeed_pwm_get_duty(pwm_t *obj);

hal_status_t aspeed_tach_init(tach_t *obj);
uint32_t aspeed_tach_get_rpm(tach_t *obj);
void aspeed_tach_int_cb_hook(tach_t *obj, uint32_t rpm_threshold, uint8_t cmp_more_less, callback cb, void *para);
void aspeed_tach_int_cb_unhook(tach_t *obj);
#endif /* end of "#ifndef _PWM_TACH_ASPEED_H_" */