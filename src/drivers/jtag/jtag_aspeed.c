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
#include "jtag_aspeed.h"
#include "log.h"
#include "irq_aspeed.h"

static scan_command_t _scan_;
static scan_field_t _out_fields_;

#define DEFAULT_JTAG_FREQ 8000000

struct name_mapping {
	enum tap_state symbol;
	const char *name;
};
/* NOTE:  do not change these state names.  They're documented,
 * and we rely on them to match SVF input (except for "RUN/IDLE").
 */
static const struct name_mapping tap_name_mapping[] = {
	{ TAP_RESET, "RESET", },
	{ TAP_IDLE, "RUN/IDLE", },
	{ TAP_DRSELECT, "DRSELECT", },
	{ TAP_DRCAPTURE, "DRCAPTURE", },
	{ TAP_DRSHIFT, "DRSHIFT", },
	{ TAP_DREXIT1, "DREXIT1", },
	{ TAP_DRPAUSE, "DRPAUSE", },
	{ TAP_DREXIT2, "DREXIT2", },
	{ TAP_DRUPDATE, "DRUPDATE", },
	{ TAP_IRSELECT, "IRSELECT", },
	{ TAP_IRCAPTURE, "IRCAPTURE", },
	{ TAP_IRSHIFT, "IRSHIFT", },
	{ TAP_IREXIT1, "IREXIT1", },
	{ TAP_IRPAUSE, "IRPAUSE", },
	{ TAP_IREXIT2, "IREXIT2", },
	{ TAP_IRUPDATE, "IRUPDATE", },

	/* only for input:  accept standard SVF name */
	{ TAP_IDLE, "IDLE", },
};

const char *tap_state_name(tap_state_t state)
{
	unsigned i;

	for (i = 0; i < ARRAY_SIZE(tap_name_mapping); i++) {
		if (tap_name_mapping[i].symbol == state)
			return tap_name_mapping[i].name;
	}
	return "???";
}

tap_state_t tap_state_by_name(const char *name)
{
	unsigned i;

	for (i = 0; i < ARRAY_SIZE(tap_name_mapping); i++) {
		/* be nice to the human */
		if (strcasecmp(name, tap_name_mapping[i].name) == 0)
			return tap_name_mapping[i].symbol;
	}
	/* not found */
	return TAP_INVALID;
}

static hal_status_t aspeed_jtag_xfer(jtag_t *obj, scan_command_t *scan)
{
	aspeed_device_t *jtag = obj->device;
    aspeed_jtag_priv_t *priv = (aspeed_jtag_priv_t *)jtag->private;
	jtag_register_t *jtag_register = (volatile jtag_register_t *) jtag->base;
	uint32_t remain_xfer = scan->fields->num_bits;
	uint32_t xfer_size;
	uint32_t shift_index = 0, curr_out_index = 0, curr_in_index = 0;
	uint8_t end_xfer;
	const uint32_t *out_value = (const uint32_t *)scan->fields->out_value;
	uint32_t *in_value = (uint32_t *)scan->fields->in_value;

	/* Disable SW mode */
	jtag_register->software_mode_and_status.value = 0;
	/* Enable HW mode 1 */
	jtag_register->mode_1_control.fields.engine_enable = 1;
	jtag_register->mode_1_control.fields.engine_output_enable = 1;
	jtag_register->mode_1_control.fields.msb_first = 0;
	jtag_register->mode_1_control.fields.last_xfer = 0;
	/* Clear internal fifo */
	jtag_register->mode_1_control.fields.reset_internal_fifo = 1;
	while(jtag_register->mode_1_control.fields.reset_internal_fifo)
		;
	log_debug("scan info: %sScan size:%d end_state:%s\n",
			  scan->ir_scan ? "IR" : "DR", scan->fields->num_bits,
			  tap_state_name(scan->end_state));

	while (remain_xfer) {
		if (remain_xfer > priv->fifo_length) {
			end_xfer = 0;
			xfer_size = priv->fifo_length;
			shift_index += priv->fifo_length >> 5;
		}
		else {
			end_xfer = 1;
			xfer_size = remain_xfer;
			shift_index += remain_xfer >> 5;
			if (remain_xfer & 0x1f)
				shift_index++;
		}
		/* Write out data to FIFO */
		for (; curr_out_index < shift_index; curr_out_index++) {
			if (xfer_size < 32) {
				log_debug("out_value[%d] = %08x\n", curr_out_index,
						out_value[curr_out_index] & GENMASK(xfer_size - 1,0));
				jtag_register->data_for_hw_mode_1[0].value =
						out_value[curr_out_index] & GENMASK(xfer_size - 1,0);
			} else {
				log_debug("out_value[%d] = %08x\n", curr_out_index,
						out_value[curr_out_index]);
				jtag_register->data_for_hw_mode_1[0].value =
						out_value[curr_out_index];
			}
		}
		if (end_xfer && scan->end_state == TAP_IDLE)
			jtag_register->mode_1_control.fields.last_xfer = 1;
		jtag_register->mode_1_control.fields.xfer_len = xfer_size;
		log_debug("Transfer ctrl: 0x%08x\n", jtag_register->mode_1_control.value);
		/* Enable transfer */
		/* TODO: Use interrupt mode to wait xfer complete or set wait timeout */
		if (scan->ir_scan) {
			jtag_register->mode_1_control.fields.ir_xfer_en = 1;
			if (jtag_register->mode_1_control.fields.last_xfer) {
				while(jtag_register->mode_1_int_ctrl.fields.instr_xfer_completed == 0)
					;
				jtag_register->mode_1_int_ctrl.fields.instr_xfer_completed = 1;
			} else {
				while(jtag_register->mode_1_int_ctrl.fields.instr_xfer_pause == 0)
					;
				jtag_register->mode_1_int_ctrl.fields.instr_xfer_pause = 1;
			}
			jtag_register->mode_1_control.fields.ir_xfer_en = 0;
		} else {
			jtag_register->mode_1_control.fields.dr_xfer_en = 1;
			if (jtag_register->mode_1_control.fields.last_xfer) {
				while(jtag_register->mode_1_int_ctrl.fields.data_xfer_completed == 0)
					;
				jtag_register->mode_1_int_ctrl.fields.data_xfer_completed = 1;
			} else {
				while(jtag_register->mode_1_int_ctrl.fields.data_xfer_pause == 0)
					;
				jtag_register->mode_1_int_ctrl.fields.data_xfer_pause = 1;
			}
			jtag_register->mode_1_control.fields.dr_xfer_en = 0;
		}
		remain_xfer -= xfer_size;
		/* Get in data to fifo */
		if(in_value != NULL) {
			for (; curr_in_index < shift_index; curr_in_index++) {
				if (xfer_size < 32)
					in_value[curr_in_index] =
							jtag_register->data_for_hw_mode_1[0].value >>
							(32 - xfer_size);
				else
					in_value[curr_in_index] =
							jtag_register->data_for_hw_mode_1[0].value;
				log_debug("in_value[%d] = %08x\n", curr_in_index,
						  in_value[curr_in_index]);
				xfer_size -= 32;
			}
		}
		else {
			for (; curr_in_index < shift_index; curr_in_index++) {
				if (xfer_size < 32)
					log_debug("in_value[%d] = %08x\n", curr_in_index,
							  jtag_register->data_for_hw_mode_1[0].value >>
									  (32 - xfer_size));
				else
					log_debug("in_value[%d] = %08x\n", curr_in_index,
							  jtag_register->data_for_hw_mode_1[0].value);
			}
		}
	}
	priv->state = scan->end_state;
	return HAL_OK;
}

hal_status_t aspeed_jtag_ir_scan(jtag_t *obj, int num_bits,
								 const uint8_t *out_value, uint8_t *in_value,
								 tap_state_t state)
{
	scan_command_t *scan = (scan_command_t *)&_scan_;
	scan_field_t *out_fields = (scan_field_t *)&_out_fields_;
	scan->ir_scan = 1;
	scan->num_fields = 1;
	scan->fields = out_fields;
	scan->end_state = state;
	out_fields->num_bits = num_bits;
	out_fields->out_value = out_value;
	out_fields->in_value = in_value;
	return aspeed_jtag_xfer(obj, scan);
}

hal_status_t aspeed_jtag_dr_scan(jtag_t *obj, int num_bits,
								 const uint8_t *out_value, uint8_t *in_value,
								 tap_state_t state)
{
	scan_command_t *scan = (scan_command_t *)&_scan_;
	scan_field_t *out_fields = (scan_field_t *)&_out_fields_;
	scan->ir_scan = 0;
	scan->num_fields = 1;
	scan->fields = out_fields;
	scan->end_state = state;
	out_fields->num_bits = num_bits;
	out_fields->out_value = out_value;
	out_fields->in_value = in_value;
	return aspeed_jtag_xfer(obj, scan);
}

void aspeed_jtag_set_clk(jtag_t *obj, uint32_t freq)
{
	aspeed_device_t *jtag = obj->device;
	aspeed_jtag_priv_t *priv = (aspeed_jtag_priv_t *)jtag->private;
	jtag_register_t *jtag_register = (volatile jtag_register_t *) jtag->base;
	uint32_t src_clk = aspeed_clk_get_hclk();
	uint32_t div, diff;
	div = DIV_ROUND_UP(src_clk, freq);
	diff = abs(src_clk - div * freq);
	if (diff > abs(src_clk - (div - 1) * freq))
		div = div - 1;
	/* TCK freq = HCLK / (tck_divisor + 1) */
	if (div >= 1)
		div = div - 1;
	/* HW constraint: minimal TCK divisor = 7*/
	if (div < 7)
		div = 7;
	priv->freq = src_clk / (div + 1);
	log_debug("tck divisor = %d, tck freq = %d\n", div, priv->freq);
	jtag_register->tck_control.fields.tck_divisor = div;
}

static void dummy(jtag_t *obj, uint32_t cnt)
{
	aspeed_device_t *jtag = obj->device;
	jtag_register_t *jtag_register = (volatile jtag_register_t *) jtag->base;
	int i = 0;

	for (i = 0; i < cnt; i++)
		(void)jtag_register->software_mode_and_status.value;
}

static char aspeed_jtag_tck_cycle(jtag_t *obj, uint8_t tms, uint8_t tdi)
{
	aspeed_device_t *jtag = obj->device;
	jtag_register_t *jtag_register = (volatile jtag_register_t *) jtag->base;
	software_mode_and_status_t sw_ctrl;
	uint8_t tdo;
	/* Disable HW mode */
	jtag_register->mode_1_control.fields.engine_enable = 0;
	jtag_register->mode_1_control.fields.engine_output_enable = 0;
	jtag_register->mode_2_control.fields.engine_enable = 0;
	jtag_register->mode_2_control.fields.engine_output_enable = 0;
	sw_ctrl.fields.software_mode_enable = 1;
	/* TCK = 0 */
	sw_ctrl.fields.software_tck = 0;
	sw_ctrl.fields.software_tdi_and_tdo = tdi;
	sw_ctrl.fields.software_tms = tms;
	jtag_register->software_mode_and_status.value = sw_ctrl.value;
	dummy(obj, 10);
	/* TCK = 1 */
	sw_ctrl.fields.software_tck = 1;
	jtag_register->software_mode_and_status.value = sw_ctrl.value;
	dummy(obj, 10);
	tdo = jtag_register->software_mode_and_status.fields.software_tdi_and_tdo;

	return tdo;
}

uint8_t aspeed_jtag_tap_is_stable(tap_state_t state)
{
	return (TAP_RESET == state) || (TAP_IDLE == state)
			|| (TAP_DRPAUSE == state) || (TAP_IRPAUSE == state);
}

tap_state_t aspeed_jtag_get_tap_state(jtag_t *obj)
{
	aspeed_device_t *jtag = obj->device;
    aspeed_jtag_priv_t *priv = (aspeed_jtag_priv_t *)jtag->private;
	return priv->state;
}

static hal_status_t aspeed_jtag_tap_reset(jtag_t *obj)
{
	aspeed_device_t *jtag = obj->device;
    aspeed_jtag_priv_t *priv = (aspeed_jtag_priv_t *)jtag->private;
	jtag_register_t *jtag_register = (volatile jtag_register_t *) jtag->base;
	/* Disable SW mode */
	jtag_register->software_mode_and_status.value = 0;
	/* Enable HW mode 1 */
	jtag_register->mode_1_control.fields.engine_enable = 1;
	jtag_register->mode_1_control.fields.engine_output_enable = 1;
	/* Reset target tap */
	jtag_register->mode_1_control.fields.reset_to_tlr = 1;
	/* FIXME: Need TIMEOUT? */
	while(jtag_register->software_mode_and_status.fields.engine_idle == 0)
		;
	priv->state = TAP_IDLE;
	return HAL_OK;
}

static hal_status_t aspeed_jtag_tap_idle(jtag_t *obj)
{
	aspeed_device_t *jtag = obj->device;
    aspeed_jtag_priv_t *priv = (aspeed_jtag_priv_t *)jtag->private;
	jtag_register_t *jtag_register = (volatile jtag_register_t *) jtag->base;
	if (priv->state == TAP_IDLE)
		return HAL_OK;
	if (aspeed_jtag_tap_is_stable(priv->state)) {
		jtag_register->mode_1_control.fields.xfer_len = 0;
		jtag_register->mode_1_control.fields.terminating_xfer = 1;
		jtag_register->mode_1_control.fields.last_xfer = 1;
		log_debug("mode_1_ctrl = 0x%08x, status = 0x%08x\n",
				  jtag_register->mode_1_control.value,
				  jtag_register->software_mode_and_status.value);
		if (jtag_register->software_mode_and_status.fields.instr_xfer_pause) {
			jtag_register->mode_1_control.fields.ir_xfer_en = 1;
			while(jtag_register->mode_1_int_ctrl.fields.instr_xfer_completed == 0)
					;
			jtag_register->mode_1_int_ctrl.fields.instr_xfer_completed = 1;
			jtag_register->mode_1_control.fields.ir_xfer_en = 0;
		} else if (jtag_register->software_mode_and_status.fields
						   .data_xfer_pause) {
			jtag_register->mode_1_control.fields.dr_xfer_en = 1;
			while(jtag_register->mode_1_int_ctrl.fields.data_xfer_completed == 0)
					;
			jtag_register->mode_1_int_ctrl.fields.data_xfer_completed = 1;
			jtag_register->mode_1_control.fields.dr_xfer_en = 0;
		}
		jtag_register->mode_1_control.fields.terminating_xfer = 0;
	} else {
		log_error("jtag tap state %d is not stable", priv->state);
		return HAL_ERROR;
	}
	priv->state = TAP_IDLE;
	return HAL_OK;
}

void aspeed_jtag_run_test(jtag_t *obj, uint32_t num_cycles)
{
	uint32_t i;
	for (i = 0; i < num_cycles; i++)
		aspeed_jtag_tck_cycle(obj, 0, 0);
}

hal_status_t aspeed_jtag_set_tap_state(jtag_t *obj, tap_state_t state)
{
	hal_status_t ret;
	if (state == TAP_IDLE)
		ret = aspeed_jtag_tap_idle(obj);
	else if (state == TAP_RESET)
		ret = aspeed_jtag_tap_reset(obj);
	else
		ret = HAL_ERROR;
	if (ret == HAL_OK)
		log_debug("Move tap state to %s\n", tap_state_name(state));
	else
		log_error("Move tap state to %s fail\n", tap_state_name(state));
	return ret;
}

hal_status_t aspeed_jtag_target_init(jtag_t *obj)
{
	aspeed_jtag_tap_reset(obj);
	aspeed_jtag_set_tap_state(obj, TAP_IDLE);
	return HAL_OK;
}

hal_status_t aspeed_jtag_init(jtag_t *obj)
{
	aspeed_device_t *jtag = obj->device;
    aspeed_jtag_priv_t *priv = (aspeed_jtag_priv_t *)jtag->private;

	if(priv->init) {
		log_error("JTAG is occupied\n");
	    return HAL_BUSY;
	}
	/* toggle jtag reset */
	aspeed_reset_assert(jtag);
	aspeed_reset_deassert(jtag);

	aspeed_jtag_set_clk(obj, DEFAULT_JTAG_FREQ);

	priv->init = 1;
	return HAL_OK;
}