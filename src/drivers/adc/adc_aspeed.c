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
#include "wait.h"
#include "adc_aspeed.h"
#include "log.h"
#define USE_OS_FLAG_FOR_WAIT
#ifdef USE_OS_FLAG_FOR_WAIT
#include "irq_aspeed.h"
#endif

/**********************************************************
 * ADC global
 *********************************************************/
static hal_status_t aspeed_g_adc_init(aspeed_device_t *g_adc)
{
	if (g_adc->init) {
	    log_warn("ADC parent already inited\n");
	    return HAL_ERROR;
	}
	/* toggle adc reset */
	aspeed_reset_assert(g_adc);
	aspeed_reset_deassert(g_adc);

	g_adc->init = 1;

	return HAL_OK;
}
/**********************************************************
 * ADC driver
 *********************************************************/
static hal_status_t aspeed_adc_engine_init(adc_t *obj, uint32_t timeout_us)
{
	aspeed_device_t *adc = obj->device;
	adc_register_t *adc_register = ( volatile adc_register_t * ) adc->base;
	adc_register->engine_ctrl.fields.adc_operation_mode = ADC_NORMAL;
	adc_register->engine_ctrl.fields.engine_enable = 1;
	while(timeout_us){
		if (adc_register->engine_ctrl.fields.initial_sequence_complete)
			return HAL_OK;
		aspeed_wait_us(1);
		timeout_us--;
	}
	return HAL_TIMEOUT;
}

static void aspeed_adc_calibration(adc_t *obj)
{
	aspeed_device_t *adc = obj->device;
	aspeed_adc_priv_t *priv = (aspeed_adc_priv_t *)adc->private;
	adc_register_t *adc_register = ( volatile adc_register_t * ) adc->base;
	uint32_t raw_data, index;
	/*TODO: trimming data setting */

	/* Enable Compensating Sensing mode */
	adc_register->engine_ctrl.fields.compensating_sensing_mode = 1;
	adc_register->engine_ctrl.fields.channel_enable = BIT(0);
	aspeed_wait_us(50);
	raw_data = adc_register->adc_data[0].fields.data_odd;
	for (index = 0; index < ASPEED_CV_SAMPLE_TIMES; index++) {
		aspeed_wait_us(50);
		raw_data += adc_register->adc_data[0].fields.data_odd;
		raw_data >>= 1;
	}
	/*
	 * At compensating sensing mode hardware will generate the voltage is half of the
	 * ref. voltage.
	 */
	priv->cv = (int)(BIT(ASPEED_RESOLUTION_BITS - 1) - raw_data);

	log_debug("Compensating value = %d\n", priv->cv);
	/* Disable Compensating Sensing mode */
	adc_register->engine_ctrl.fields.compensating_sensing_mode = 0;
	adc_register->engine_ctrl.fields.channel_enable = 0;
}

static hal_status_t aspeed_adc_set_ref_voltage(adc_t *obj)
{
	aspeed_device_t *adc = obj->device;
	aspeed_adc_priv_t *priv = (aspeed_adc_priv_t *)adc->private;
	adc_register_t *adc_register = ( volatile adc_register_t * ) adc->base;
	uint32_t ref_voltage_cfg;
	/*
	 * Set ref_voltage:
	 * If reference voltage is between 1550~1650mv, we can set
	 * fields either REF_VOLTAGE_EXT_HIGH or REF_VOLTAGE_EXT_LOW.
	 * In this place, we select REF_VOLTAGE_EXT_HIGH as higher priority.
	 */
	if (priv->ref_voltage_mv) {
		if (priv->ref_voltage_mv == 2500)
			ref_voltage_cfg = REF_VOLTAGE_2500mV;
		else if (priv->ref_voltage_mv == 1200)
			ref_voltage_cfg = REF_VOLTAGE_1200mV;
		else if ((priv->ref_voltage_mv >= 1550) &&
				(priv->ref_voltage_mv <= 2700))
			ref_voltage_cfg = REF_VOLTAGE_EXT_HIGH;
		else if ((priv->ref_voltage_mv >= 900) &&
				(priv->ref_voltage_mv <= 1650))
			ref_voltage_cfg = REF_VOLTAGE_EXT_LOW;
		else {
			log_error("ref_voltage_mv property is out of range: %d\n",
				priv->ref_voltage_mv);
			return HAL_ERROR;
		}
	} else {
		log_error("ref_voltage_mv property couldn't be 0\n");
		return HAL_ERROR;
	}
	adc_register->engine_ctrl.fields.reference_voltage_selection =
			ref_voltage_cfg;
	return HAL_OK;
}

static uint32_t aspeed_adc_read_raw(adc_t *obj, uint32_t channel)
{
	aspeed_device_t *adc = obj->device;
	aspeed_adc_priv_t *priv = (aspeed_adc_priv_t *)adc->private;
	adc_register_t *adc_register = ( volatile adc_register_t * ) adc->base;
	uint32_t raw_data;
	raw_data = (channel & 0x1) ?
					   adc_register->adc_data[channel >> 1].fields.data_even :
					   adc_register->adc_data[channel >> 1].fields.data_odd;
	log_debug("Raw data before cv: %u\n", raw_data);
	raw_data += priv->cv;
	return raw_data;
}

static uint32_t aspeed_adc_battery_read(adc_t *obj, uint32_t channel)
{
	aspeed_device_t *adc = obj->device;
	aspeed_adc_priv_t *priv = (aspeed_adc_priv_t *)adc->private;
	adc_register_t *adc_register = ( volatile adc_register_t * ) adc->base;
	uint32_t raw_data;
	adc_register->engine_ctrl.fields.channel_7_selection = Ch7_BATTERY_MODE;
	adc_register->engine_ctrl.fields.enable_battery_sensing = 1;
	/* After enable battery sensing need to wait 3*12T for adc stable */
	osDelay(1);
	raw_data = aspeed_adc_read_raw(obj, channel);
	log_debug("battery div: %s\n", priv->battery_div ? "1/3" : "2/3");
	if (priv->battery_en && channel == 0x7) {
		if (priv->battery_div == BATTERY_DIVIDE_1_3)
			raw_data = raw_data * 3;
		else
			raw_data = (raw_data * 3) >> 1;
	}
	adc_register->engine_ctrl.fields.channel_7_selection = Ch7_NORMAL_MODE;
	adc_register->engine_ctrl.fields.enable_battery_sensing = 0;
	return raw_data;
}

uint32_t aspeed_adc_read(adc_t *obj, uint32_t channel)
{
	aspeed_device_t *adc = obj->device;
	aspeed_adc_priv_t *priv = (aspeed_adc_priv_t *)adc->private;
	if (channel > ASPEED_ADC_CH_NUMBER) {
		log_error("Channel %u not support\n", channel);
		return 0;
	}
	if (priv->battery_en && channel == 7)
		return aspeed_adc_battery_read(obj, channel);
	else
		return aspeed_adc_read_raw(obj, channel);
}

uint32_t aspeed_adc_read_mv(adc_t *obj, uint32_t channel)
{
	uint32_t raw, mv;
	aspeed_device_t *adc = obj->device;
	aspeed_adc_priv_t *priv = (aspeed_adc_priv_t *)adc->private;
	raw = aspeed_adc_read(obj, channel);
	mv = (raw * priv->ref_voltage_mv) >> ASPEED_RESOLUTION_BITS;
	return mv;
}

uint32_t aspeed_adc_get_clk_rate(adc_t *obj){
	aspeed_device_t *adc = obj->device;
	aspeed_adc_priv_t *priv = (aspeed_adc_priv_t *)adc->private;
	return priv->clk_freq;
}

uint32_t aspeed_adc_get_sampling_rate(adc_t *obj){
	aspeed_device_t *adc = obj->device;
	aspeed_adc_priv_t *priv = (aspeed_adc_priv_t *)adc->private;
	return priv->sampling_rate;
}

hal_status_t aspeed_adc_set_clk_rate(adc_t *obj, uint32_t rate){
	aspeed_device_t *adc = obj->device;
	aspeed_adc_priv_t *priv = (aspeed_adc_priv_t *)adc->private;
	adc_register_t *adc_register = ( volatile adc_register_t * ) adc->base;
	uint32_t clk_src, divisor;
	/*
	 * Formula of adc clock:
	 * ADC clock = APB2 / (2 * (divisor_of_adc_clock + 1))
	 * ADC sampling rate = ADC clock / 12
	 */
#ifdef CONFIG_AST2600_SERIES
	clk_src = aspeed_clk_get_apb2() / 2;
#else
	clk_src = aspeed_clk_get_pclk() / 2;
#endif
	divisor = (clk_src / rate) - 1;
	if (divisor >= BIT(16)) {
		log_error("clock freq %d out of range\n", rate);
		return HAL_ERROR;
	}
	priv->clk_freq = clk_src / (divisor + 1);
	priv->sampling_rate = priv->clk_freq / ASPEED_CLOCKS_PER_SAMPLE;
	log_debug("clk freq = %d, sampling rate = %d\n", priv->clk_freq,
			 priv->sampling_rate);
	adc_register->adc_clk_ctrl.fields.divisor_of_adc_clock = divisor;
	return HAL_OK;
}

hal_status_t aspeed_adc_init(adc_t *obj)
{
	aspeed_device_t *adc = obj->device;
	aspeed_adc_priv_t *priv = (aspeed_adc_priv_t *)adc->private;
	aspeed_device_t *parent = (aspeed_device_t *)priv->parent;
	adc_register_t *adc_register = (volatile adc_register_t *)adc->base;
	hal_status_t ret;

	if (parent->init != 1) {
		ret = aspeed_g_adc_init(parent);
		if (ret != HAL_OK)
			return ret;
	}

	if (adc->init) {
	    log_error("ADC is occupied\n");
	    return HAL_BUSY;
	}

	ret = aspeed_adc_set_clk_rate(obj, ASPEED_CLOCK_FREQ_DEFAULT);
	if (ret != HAL_OK)
		return ret;

	ret = aspeed_adc_set_ref_voltage(obj);
	if (ret != HAL_OK)
		return ret;

	if (priv->battery_en) {
		if (adc_register->engine_ctrl.fields.reference_voltage_selection & 0x1) {
			priv->battery_div = BATTERY_DIVIDE_1_3;
		} else {
			priv->battery_div = BATTERY_DIVIDE_2_3;
		}
	}

	ret = aspeed_adc_engine_init(obj, ASPEED_ADC_INIT_TIMEOUT);
	if (ret != HAL_OK)
		return ret;

	if (priv->do_calibration)
		aspeed_adc_calibration(obj);

	adc_register->engine_ctrl.fields.channel_enable = 0xff;

	return HAL_OK;
}