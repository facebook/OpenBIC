/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "common.h"
#include "util.h"
#include "hal_def.h"
#include "reset_aspeed.h"
#include "clk_aspeed.h"
#include "cmsis_os.h"
#include <string.h>
#include <limits.h>
#include "pwm_tach_aspeed.h"
#include "log.h"
#include "irq_aspeed.h"

static void aspeed_tach_isr(void);
/**********************************************************
 * PWM_TACH global init
 *********************************************************/
static hal_status_t aspeed_g_pwm_tach_init(aspeed_device_t *g_pwm_tach)
{
	aspeed_g_pwm_tach_priv_t *priv =
			(aspeed_g_pwm_tach_priv_t *)g_pwm_tach->private;
	pwm_tach_register_t *pwm_tach_register =
			(volatile pwm_tach_register_t *)g_pwm_tach->base;
	pwm_tach_gather_register_t *pwm_tach_gather;
	uint32_t index;
	if (g_pwm_tach->init) {
		log_error("PWM TACH already initiated\n");
		return HAL_BUSY;
	}
	/* toggle pwm_tach reset */
	aspeed_reset_assert(g_pwm_tach);
	aspeed_reset_deassert(g_pwm_tach);
	/* Disable all of the tach interrupt */
	for (index = 0; index < 16; index++) {
		pwm_tach_gather = &pwm_tach_register->pwm_tach_gather[index];
		pwm_tach_gather->tach_general.fields.enable_tach_interrupt = 0x0;
	}
	priv->pwm_used_bit_map = 0x0;
	priv->tach_used_bit_map = 0x0;

	aspeed_irq_register(Tach_IRQn, (uint32_t)aspeed_tach_isr,
						(void *)g_pwm_tach);

	g_pwm_tach->init = 1;
	return HAL_OK;
}

uint16_t aspeed_g_pwm_tach_get_pwm_status(aspeed_device_t *g_pwm_tach)
{
	aspeed_g_pwm_tach_priv_t *priv =
			(aspeed_g_pwm_tach_priv_t *)g_pwm_tach->private;
	return priv->pwm_used_bit_map;
}

uint16_t aspeed_g_pwm_tach_get_tach_status(aspeed_device_t *g_pwm_tach)
{
	aspeed_g_pwm_tach_priv_t *priv =
			(aspeed_g_pwm_tach_priv_t *)g_pwm_tach->private;
	return priv->tach_used_bit_map;
}

/**********************************************************
 * PWM driver
 *********************************************************/
static void aspeed_pwm_ctrl(pwm_t *obj, uint8_t ctrl)
{
	aspeed_device_t *pwm = obj->device;
	aspeed_pwm_priv_t *priv = (aspeed_pwm_priv_t *)pwm->private;
	aspeed_device_t *parent = (aspeed_device_t *)priv->parent;
	aspeed_g_pwm_tach_priv_t *g_priv =
			(aspeed_g_pwm_tach_priv_t *)parent->private;
	pwm_tach_register_t *pwm_tach_register =
			(volatile pwm_tach_register_t *)pwm->base;
	uint32_t pwm_channel = priv->pwm_channel;
	pwm_tach_gather_register_t *pwm_tach_gather =
			&pwm_tach_register->pwm_tach_gather[pwm_channel];

	if (ctrl) {
		pwm_tach_gather->pwm_general.fields.enable_pwm_pin = 0x1;
		pwm_tach_gather->pwm_general.fields.enable_pwm_clock = 0x1;
		g_priv->pwm_used_bit_map |= (1 << pwm_channel);
	} else {
		pwm_tach_gather->pwm_general.fields.enable_pwm_pin = 0x0;
		pwm_tach_gather->pwm_general.fields.enable_pwm_clock = 0x0;
		g_priv->pwm_used_bit_map &= ~(1 << pwm_channel);
	}
}

hal_status_t aspeed_pwm_init(pwm_t *obj)
{
	aspeed_device_t *pwm = obj->device;
	aspeed_pwm_priv_t *priv = (aspeed_pwm_priv_t *)pwm->private;
	aspeed_device_t *parent = (aspeed_device_t *)priv->parent;
	aspeed_g_pwm_tach_priv_t *g_priv =
			(aspeed_g_pwm_tach_priv_t *)parent->private;
	pwm_tach_register_t *pwm_tach_register =
			(volatile pwm_tach_register_t *)pwm->base;
	uint32_t pwm_channel = priv->pwm_channel;
	pwm_tach_gather_register_t *pwm_tach_gather =
			&pwm_tach_register->pwm_tach_gather[pwm_channel];
	hal_status_t ret;

	if (parent->init != 1) {
		ret = aspeed_g_pwm_tach_init(parent);
		if (ret != HAL_OK)
			return ret;
	}

	if (pwm->init) {
		log_error("PWM%d already occupied\n", pwm_channel);
		return HAL_BUSY;
	}

	priv->clk_source = aspeed_clk_get_hclk();
	log_debug("clk_source = %d\n", priv->clk_source);
	ret = aspeed_pwm_set_freq(obj, priv->pwm_freq);
	if (ret != HAL_OK)
		return ret;
	pwm_tach_gather->pwm_general.fields.enable_pwm_duty_load_as_wdt =
			priv->wdt_reload_enable;
	pwm_tach_gather->pwm_general.fields.load_selection_of_duty_as_wdt =
			LOAD_SEL_FALLING;

	pwm_tach_gather->pwm_duty_cycle.fields.pwm_rising_point = 0;
	pwm_tach_gather->pwm_duty_cycle.fields.pwm_rising_falling_point_as_wdt =
			(DEFAULT_PWM_PERIOD * priv->wdt_reload_duty / 100);

	pwm->init = 1;
	g_priv->pwm[pwm_channel] = pwm;
	g_priv->pwm_used_bit_map |= (1 << pwm_channel);

	return HAL_OK;
}

hal_status_t aspeed_pwm_set_freq(pwm_t *obj, uint32_t freq)
{
	aspeed_device_t *pwm = obj->device;
	aspeed_pwm_priv_t *priv = (aspeed_pwm_priv_t *)pwm->private;
	pwm_tach_register_t *pwm_tach_register =
			(volatile pwm_tach_register_t *)pwm->base;
	uint32_t pwm_channel = priv->pwm_channel;
	pwm_tach_gather_register_t *pwm_tach_gather =
			&pwm_tach_register->pwm_tach_gather[pwm_channel];
	uint32_t target_div, div_h, div_l;
	uint32_t tmp_div_h, tmp_div_l;
	uint32_t freq_a_fix_div;
	uint8_t div_found;
	int diff, error_value = INT_MAX;

	/* Fixed Divide */
	freq_a_fix_div = priv->clk_source / (DEFAULT_PWM_PERIOD + 1);
	target_div = DIV_ROUND_UP(freq_a_fix_div, freq);
	log_debug("target div = %d\n", target_div);
	div_found = 0;
	for (tmp_div_h = 0; tmp_div_h < 0x10; tmp_div_h++) {
		tmp_div_l = (target_div >> tmp_div_h) - 1;
		if (tmp_div_l >= BIT(8))
			continue;
		diff = freq - (freq_a_fix_div >> tmp_div_h) / (tmp_div_l + 1);
		if (abs(diff) < error_value) {
			error_value = abs(diff);
			div_h = tmp_div_h;
			div_l = tmp_div_l;
			div_found = 1;
			if (error_value == 0)
				break;
		}
	}
	if (div_found == 0) {
		log_warn("target freq: %d too slow set minimal frequency\n", freq);
		div_h = BIT(5) - 1;
		div_l = BIT(8) - 1;
		error_value = 0;
	}
	log_debug("PWM%d Freq = %d / (256 * %d * %d)\n", priv->pwm_channel,
			  priv->clk_source, BIT(div_h), div_l + 1);
	log_debug("error value = %d\n", error_value);
	pwm_tach_gather->pwm_duty_cycle.fields.pwm_period = DEFAULT_PWM_PERIOD;
	pwm_tach_gather->pwm_general.fields.pwm_clock_division_h = div_h;
	pwm_tach_gather->pwm_general.fields.pwm_clock_division_l = div_l;
	return HAL_OK;
}

hal_status_t aspeed_pwm_set_duty(pwm_t *obj, uint8_t percent)
{
	aspeed_device_t *pwm = obj->device;
	aspeed_pwm_priv_t *priv = (aspeed_pwm_priv_t *)pwm->private;
	pwm_tach_register_t *pwm_tach_register =
			(volatile pwm_tach_register_t *)pwm->base;
	uint32_t pwm_channel = priv->pwm_channel;
	pwm_tach_gather_register_t *pwm_tach_gather =
			&pwm_tach_register->pwm_tach_gather[pwm_channel];
	uint8_t falling_point;
	if (percent > 100) {
		log_error("percent %d error: range from 0 to 100\n", percent);
		return HAL_ERROR;
	}
	if (percent == 0) {
		aspeed_pwm_ctrl(obj, 0);
		falling_point = 1;
	} else {
		aspeed_pwm_ctrl(obj, 1);
		if (percent == 100)
			falling_point = 0;
		else
			falling_point = (DEFAULT_PWM_PERIOD * percent / 100);
	}
	log_debug("Percent = %d, falling_point = %d\n", percent, falling_point);
	pwm_tach_gather->pwm_duty_cycle.fields.pwm_falling_point = falling_point;
	return HAL_OK;
}

uint8_t aspeed_pwm_get_duty(pwm_t *obj)
{
	aspeed_device_t *pwm = obj->device;
	aspeed_pwm_priv_t *priv = (aspeed_pwm_priv_t *)pwm->private;
	pwm_tach_register_t *pwm_tach_register =
			(volatile pwm_tach_register_t *)pwm->base;
	uint32_t pwm_channel = priv->pwm_channel;
	pwm_tach_gather_register_t *pwm_tach_gather =
			&pwm_tach_register->pwm_tach_gather[pwm_channel];
	uint8_t falling_point, percent;
	falling_point = pwm_tach_gather->pwm_duty_cycle.fields.pwm_falling_point;
	if (falling_point == 0)
		percent = 100;
	else
		percent = (falling_point * 100) / DEFAULT_PWM_PERIOD;
	log_debug("Percent = %d, falling_point = %d\n", percent, falling_point);
	return percent;
}

/**********************************************************
 * TACH driver
 *********************************************************/
static void aspeed_tach_isr(void)
{
	aspeed_device_t *g_pwm_tach = aspeed_irq_get_isr_context(Tach_IRQn);
	aspeed_g_pwm_tach_priv_t *g_priv =
			(aspeed_g_pwm_tach_priv_t *)g_pwm_tach->private;
	pwm_tach_register_t *pwm_tach_register =
			(volatile pwm_tach_register_t *)g_pwm_tach->base;
	uint32_t int_index = 0, int_pendding;
	aspeed_device_t *tach;
	aspeed_tach_priv_t *priv;
	uint32_t tach_channel;
	pwm_tach_gather_register_t *pwm_tach_gather;
	aspeed_tach_adapter_t *tach_adapter;
	int_pendding = pwm_tach_register->pwm_g10c.fields.interrupt_status;
	log_debug("int_pendding = 0x%08x\n", int_pendding);
	do {
		if (int_pendding & 0x1) {
			tach = g_priv->tach[int_index];
			priv = (aspeed_tach_priv_t *)tach->private;
			tach_channel = priv->tach_channel;
			pwm_tach_gather = &pwm_tach_register->pwm_tach_gather[tach_channel];
			tach_adapter = &priv->adapter;
			if (tach_adapter->cb != NULL) {
				tach_adapter->cb(tach_adapter->para);
			}
			pwm_tach_gather->tach_status.fields.interrupt_status_and_clear =
					0x1;
		}
		int_index++;
		int_pendding >>= 1;
	} while (int_pendding);
}

static uint32_t aspeed_tach_get_sample_period(uint8_t pulse_pr,
											  uint32_t min_rpm)
{
	uint32_t sample_period_ms;
	/* 
	 * min(Tach input clock) = (PulsePR * minRPM) / 60
	 * max(Tach input period) = 60 / (PulsePR * minRPM)
	 * Tach sample period > 2 * max(Tach input period) = (2*60) / (PulsePR * minRPM)
	 */
	sample_period_ms = DIV_ROUND_UP(1000 * 2 * 60, (pulse_pr * min_rpm));
	/* Add the margin (about 1.2) of tach sample period to avoid sample miss */
	sample_period_ms = (sample_period_ms * 1200) >> 10;
	return sample_period_ms;
}

void aspeed_tach_int_cb_hook(tach_t *obj, uint32_t rpm_threshold,
							 uint8_t cmp_more_less, callback cb, void *para)
{
	aspeed_device_t *tach = obj->device;
	aspeed_tach_priv_t *priv = (aspeed_tach_priv_t *)tach->private;
	pwm_tach_register_t *pwm_tach_register =
			(volatile pwm_tach_register_t *)tach->base;
	uint32_t tach_channel = priv->tach_channel;
	pwm_tach_gather_register_t *pwm_tach_gather =
			&pwm_tach_register->pwm_tach_gather[tach_channel];
	aspeed_tach_adapter_t *tach_adapter = &priv->adapter;
	uint32_t threshold;

	log_debug("cb(para) = 0x%08x(0x%08x)\n", cb, para);
	if (priv->tach_freq > (0xffffffff / 60))
		threshold =
				((priv->tach_freq) / (priv->fan_pulse_pr * rpm_threshold)) * 60;
	else
		threshold =
				((priv->tach_freq * 60) / (priv->fan_pulse_pr * rpm_threshold));
	log_debug("rpm_threshold = %d => tach threshold = %d\n", rpm_threshold,
			  threshold);
	log_debug("compare condition = [%s] than threshold\n",
			  cmp_more_less ? "more" : "less");
	pwm_tach_gather->tach_general.fields.enable_tach_interrupt = 0x0;
	tach_adapter->cb = cb;
	tach_adapter->para = para;
	priv->cmp_more_less = cmp_more_less;
	pwm_tach_gather->tach_general.fields.inverse_tach_limit_comparison =
			cmp_more_less;
	pwm_tach_gather->tach_general.fields.tach_threshold = threshold;
	/* Avoid the unexpected interrupt from tach glitch before set the new threshold */
	pwm_tach_gather->tach_status.fields.interrupt_status_and_clear = 0x1;
	pwm_tach_gather->tach_general.fields.enable_tach_interrupt = 0x1;
}

void aspeed_tach_int_cb_unhook(tach_t *obj)
{
	aspeed_device_t *tach = obj->device;
	aspeed_tach_priv_t *priv = (aspeed_tach_priv_t *)tach->private;
	pwm_tach_register_t *pwm_tach_register =
			(volatile pwm_tach_register_t *)tach->base;
	uint32_t tach_channel = priv->tach_channel;
	pwm_tach_gather_register_t *pwm_tach_gather =
			&pwm_tach_register->pwm_tach_gather[tach_channel];
	aspeed_tach_adapter_t *tach_adapter = &priv->adapter;
	pwm_tach_gather->tach_general.fields.enable_tach_interrupt = 0x0;
	tach_adapter->cb = NULL;
	tach_adapter->para = NULL;
}

hal_status_t aspeed_tach_init(tach_t *obj)
{
	aspeed_device_t *tach = obj->device;
	aspeed_tach_priv_t *priv = (aspeed_tach_priv_t *)tach->private;
	aspeed_device_t *parent = (aspeed_device_t *)priv->parent;
	aspeed_g_pwm_tach_priv_t *g_priv =
			(aspeed_g_pwm_tach_priv_t *)parent->private;
	pwm_tach_register_t *pwm_tach_register =
			(volatile pwm_tach_register_t *)tach->base;
	uint32_t tach_channel = priv->tach_channel;
	pwm_tach_gather_register_t *pwm_tach_gather =
			&pwm_tach_register->pwm_tach_gather[tach_channel];
	uint32_t tach_div = priv->tach_div;
	hal_status_t ret;

	if (parent->init != 1) {
		ret = aspeed_g_pwm_tach_init(parent);
		if (ret != HAL_OK)
			return ret;
	}

	if (tach->init) {
		log_error("TACH%d already occupied\n", tach_channel);
		return HAL_BUSY;
	}

	priv->clk_source = aspeed_clk_get_hclk();
	log_debug("clk_source = %d\n", priv->clk_source);
	/* divide = 2^(tacho_div*2) */
	priv->tach_freq = priv->clk_source / (1 << (tach_div << 1));
	log_debug("Tach Freq = %d, div = %d\n", priv->tach_freq,
			  (1 << (tach_div << 1)));
	pwm_tach_gather->tach_general.fields.tach_clock_division = tach_div;
	pwm_tach_gather->tach_general.fields.enable_tach = 0x1;

	priv->sample_period = aspeed_tach_get_sample_period(priv->fan_pulse_pr,
														priv->fan_min_rpm);

	tach->init = 1;
	g_priv->tach[tach_channel] = tach;
	g_priv->tach_used_bit_map |= (1 << tach_channel);

	aspeed_tach_int_cb_unhook(obj);
	/* init event ID for ISR */
	priv->evt_id = osEventFlagsNew(NULL);
	if (priv->evt_id == NULL)
		log_error("fail to create evt_id\n");

	return HAL_OK;
}

uint32_t aspeed_tach_get_rpm(tach_t *obj)
{
	aspeed_device_t *tach = obj->device;
	aspeed_tach_priv_t *priv = (aspeed_tach_priv_t *)tach->private;
	pwm_tach_register_t *pwm_tach_register =
			(volatile pwm_tach_register_t *)tach->base;
	uint32_t tach_channel = priv->tach_channel;
	pwm_tach_gather_register_t *pwm_tach_gather =
			&pwm_tach_register->pwm_tach_gather[tach_channel];
	uint32_t rpm, fan_pulse_pr, tach_value;
	tach_status_register_t tach_status;
	int ret;
	ret = reg_read_poll_timeout(
			pwm_tach_gather, tach_status, tach_status,
			(tach_status.fields.tach_full_measurement &&
			 tach_status.fields.tach_value_updated_since_last_read),
			0, priv->sample_period);

	if (ret) {
		log_warn("tach read timeout\n");
		return ret;
	}
	fan_pulse_pr = priv->fan_pulse_pr;
	tach_value = (tach_status.fields.tach_value + 1) & 0xfffff;
	log_debug("rpm = (%d / %d * %d) * 60\n", priv->tach_freq, fan_pulse_pr,
			  tach_value);
	if (tach_value) {
		if (priv->tach_freq > (0xffffffff / 60))
			rpm = ((priv->tach_freq) / (fan_pulse_pr * tach_value)) * 60;
		else
			rpm = ((priv->tach_freq * 60) / (fan_pulse_pr * tach_value));
	} else
		rpm = 0;
	return rpm;
}