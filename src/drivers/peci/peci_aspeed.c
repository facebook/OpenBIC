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
#include "peci_aspeed.h"
#include "log.h"
#define USE_OS_FLAG_FOR_WAIT
#ifdef USE_OS_FLAG_FOR_WAIT
#include "irq_aspeed.h"
#endif

#ifdef USE_OS_FLAG_FOR_WAIT
static osEventFlagsId_t evt_id;
#endif


static void aspeed_peci_isr(void)
{
	peci_t *obj = aspeed_irq_get_isr_context(Peci_IRQn);
	aspeed_device_t *peci = obj->device;
	aspeed_peci_priv_t *priv = (aspeed_peci_priv_t *)peci->private;
	peci_register_t *peci_register = ( volatile peci_register_t * ) peci->base;
	aspeed_peci_adapter_t *peci_adapter = &priv->adapter;
	uint32_t int_index = 0, int_pendding;
	int_pendding = peci_register->interrupt_status.value & PECI_INT_MASK;
	log_debug("int_pedding = 0x%08x\n", int_pendding);
	do
	{
		if((int_pendding & 0x1) && (peci_adapter->cb[int_index] != NULL))
		{
		    peci_adapter->cb[int_index](peci_adapter->para[int_index]);
		}
		peci_register->interrupt_status.value = BIT(int_index);
		peci_register->interrupt_status.value;
		int_index++;
		int_pendding >>= 1;
	} while (int_pendding);
}
#ifdef USE_OS_FLAG_FOR_WAIT
static void aspeed_peci_cmd_done_cb(void *para)
{
    osEventFlagsSet(evt_id, 0x00000001U);
}
#endif
void aspeed_peci_int_cb_hook(peci_t *obj, peci_int_id_t int_id, callback cb, void *para)
{
	aspeed_device_t *peci = obj->device;
    aspeed_peci_priv_t *priv = (aspeed_peci_priv_t *)peci->private;
	peci_register_t *peci_register = ( volatile peci_register_t * ) peci->base;
	aspeed_peci_adapter_t *peci_adapter = &priv->adapter;
	peci_register->interrupt.value &= (~BIT(int_id));
	peci_adapter->cb[int_id] = cb;
	peci_adapter->para[int_id] = para;
	peci_register->interrupt.value |= BIT(int_id);
}

void aspeed_peci_int_cb_unhook(peci_t *obj, peci_int_id_t int_id)
{
	aspeed_device_t *peci = obj->device;
    aspeed_peci_priv_t *priv = (aspeed_peci_priv_t *)peci->private;
	peci_register_t *peci_register = ( volatile peci_register_t * ) peci->base;
	aspeed_peci_adapter_t *peci_adapter = &priv->adapter;
	peci_register->interrupt.value &= (~BIT(int_id));
	peci_adapter->cb[int_id] = NULL;
	peci_adapter->para[int_id] = NULL;
}

uint8_t aspeed_get_expected_fcs(peci_t *obj, peci_fcs_t type)
{
	uint8_t fcs;
	aspeed_device_t *peci = obj->device;
	peci_register_t *peci_register = ( volatile peci_register_t * ) peci->base;
	switch(type)
	{
		case PECI_WFCS:
			fcs = peci_register->expected_fcs_data.fields.expected_write_fcs;
			break;
		case PECI_RFCS:
			fcs = peci_register->expected_fcs_data.fields.expected_read_fcs;
			break;
		default:
			fcs = 0;
			break;
	}
	log_debug("expected fcs = %08x\n", fcs);
	return fcs;
}

uint8_t aspeed_get_captured_fcs(peci_t *obj, peci_fcs_t type)
{
	uint8_t fcs;
	aspeed_device_t *peci = obj->device;
	peci_register_t *peci_register = ( volatile peci_register_t * ) peci->base;
	switch(type)
	{
		case PECI_WFCS:
			fcs = peci_register->captured_fcs_data.fields.captured_write_fcs;
			break;
		case PECI_RFCS:
			fcs = peci_register->captured_fcs_data.fields.captured_read_fcs;
			break;
		default:
			fcs = 0;
			break;
	}
	log_debug("captured fcs = %08x\n", fcs);
	return fcs;
}

hal_status_t aspeed_peci_xfer(peci_t *obj, peci_xfer_msg_t * msg)
{
	aspeed_device_t *peci = obj->device;
    aspeed_peci_priv_t *priv = (aspeed_peci_priv_t *)peci->private;
	peci_register_t *peci_register = ( volatile peci_register_t * ) peci->base;
	uint32_t tx_len, rx_len;
	uint32_t reg_index, byte_index;
	write_data_register_t write_val;
	uint32_t status;
	status = peci_register->command.fields.peci_controller_state;
	if (status != 0){
	    log_info("status = %x\n", status);
	    return HAL_BUSY;
	}
	log_debug("msg header addr= 0x%08x, tx_len = 0x%08x, rx_len = 0x%08x\n", msg->addr, msg->tx_len, msg->rx_len);
	/* set peci header */
	peci_register->peci_header.fields.target_address = msg->addr;
	peci_register->peci_header.fields.write_data_length = msg->tx_len;
	peci_register->peci_header.fields.read_data_length = msg->rx_len;
	/* set write data */
	log_debug("tx data:\n");
	for (tx_len = 0; tx_len < msg->tx_len; tx_len++)
	{
        byte_index = tx_len & 0x3;
		write_val.data[byte_index] = msg->tx_buf[tx_len];
		log_debug("%x\n", write_val.value);
		if (priv->byte_mode_64)
		{
            reg_index = tx_len>>2;
			peci_register->write_data_64_mode[reg_index].value = write_val.value;
		}
		else
		{
            reg_index = (tx_len & 0xf)>>2;
			if(tx_len <= 15)
				peci_register->write_data_32_mode_low[reg_index].value = write_val.value;
			else
				peci_register->write_data_32_mode_up[reg_index].value = write_val.value;
		}
	}
    /* Toggle command fire */
	peci_register->command.fields.fire_a_peci_command = 0;
	peci_register->command.fields.fire_a_peci_command = 1;
#ifdef USE_OS_FLAG_FOR_WAIT
	uint32_t ret;
	ret = osEventFlagsWait(evt_id, 0x00000001U, osFlagsWaitAny, CONFIG_DEVICE_PECI_CMD_TIMEOUT);
	if (ret != 0x1)
	{
		if(ret == -2)
		{
		    peci->init = 0;
		    aspeed_peci_init(obj);
		}
		log_error("osError: %d\n", ret);
		return HAL_ERROR;
	}
#else
	while(!peci_register->interrupt_status.fields.peci_done_interrupt_status)
		;
	peci_register->interrupt_status.fields.peci_done_interrupt_status = 1;
#endif
	log_debug("rx data:\n");
	/* get read data */
	for (rx_len = 0; rx_len < msg->rx_len; rx_len++)
	{
        byte_index = rx_len & 0x3;
		if (priv->byte_mode_64)
		{
            reg_index = rx_len>>2;
			msg->rx_buf[rx_len] = peci_register->read_data_64_mode[reg_index].data[byte_index];
		}
		else
		{
            reg_index = (rx_len & 0xf)>>2;
			if(rx_len <= 15)
				msg->rx_buf[rx_len] = peci_register->read_data_32_mode_low[reg_index].data[byte_index];
			else
				msg->rx_buf[rx_len] = peci_register->read_data_32_mode_up[reg_index].data[byte_index];
		}
		log_debug("%x\n", msg->rx_buf[rx_len]);
	}
	return HAL_OK;
}

void aspeed_peci_clk_set(peci_t *obj, uint32_t freq)
{
	aspeed_device_t *peci = obj->device;
    aspeed_peci_priv_t *priv = (aspeed_peci_priv_t *)peci->private;
	peci_register_t *peci_register = (volatile peci_register_t *)peci->base;
	uint32_t msg_timing, addr_timing;
	uint32_t clk_div_val = 0;
	uint32_t msg_timing_idx, clk_div_val_idx;
	uint32_t clk_divisor, clk_divisor_tmp;
	uint32_t bus_clk_rate;
	int delta_value, delta_tmp;
	if (freq > ASPEED_PECI_BUS_FREQ_MAX || freq < ASPEED_PECI_BUS_FREQ_MIN) {
		log_warn("Invalid clock-frequency : %u, Use default : %u\n", freq,
					ASPEED_PECI_BUS_FREQ_DEFAULT);
		freq = ASPEED_PECI_BUS_FREQ_DEFAULT;
	}
	/*
	 * PECI bus clock = (Bus clk rate) / (1 << PECI00[10:8])
	 * PECI operation clock = (PECI bus clock)/ 4*(PECI04[15:8]*4+1)
	 * (1 << PECI00[10:8]) * (PECI04[15:8]*4+1) =
	 * (Bus clk rate) / (4 * PECI operation clock)
	 */
	bus_clk_rate = aspeed_clk_get_hclk();
	clk_divisor = bus_clk_rate / (4 * freq);
	delta_value = clk_divisor;
	/* Find the closest divisor for clock-frequency */
	for (clk_div_val_idx = 0; clk_div_val_idx < 7; clk_div_val_idx++) {
		msg_timing_idx = ((clk_divisor >> clk_div_val_idx) - 1) >> 2;
		if (msg_timing_idx >= 1 && msg_timing_idx <= 255) {
			clk_divisor_tmp = (1 << clk_div_val_idx) * (msg_timing_idx * 4 + 1);
			delta_tmp = abs(clk_divisor - clk_divisor_tmp);
			if (delta_tmp < delta_value) {
				delta_value = delta_tmp;
				msg_timing = msg_timing_idx;
				clk_div_val = clk_div_val_idx;
			}
		}
	}
	addr_timing = msg_timing;
	priv->clk_freq =
			(bus_clk_rate >> (2+clk_div_val)) / (msg_timing * 4 + 1);
	log_debug("clk_div = %d, msg_timing = %d, addr_timing = %d\n", clk_div_val,
			  msg_timing, addr_timing);
	log_debug("Expect frequency: %d Real frequency is about: %d\n", freq,
			 priv->clk_freq);
	/**
	 * Timing negotiation period setting.
	 * The unit of the programmed value is 4 times of PECI clock period.
	 */
	peci_register->timing_negotiation.fields.message_timing_negotiation = msg_timing;
	peci_register->timing_negotiation.fields.address_timing_negotiation = addr_timing;
	peci_register->control.fields.clock_source_selection = PECI_CLK_SOURCE_HCLK;
	peci_register->control.fields.peci_clock_divider = clk_div_val;
}

hal_status_t aspeed_peci_init(peci_t *obj)
{
    aspeed_device_t *peci = obj->device;
    aspeed_peci_priv_t *priv = (aspeed_peci_priv_t *)peci->private;
	peci_register_t *peci_register = ( volatile peci_register_t * ) peci->base;
	uint32_t int_index;
	if (peci->init) {
	    log_error("PECI is occupied\n");
	    return HAL_BUSY;
	}
	/* toggle peci reset */
	aspeed_reset_assert(peci);
	aspeed_reset_deassert(peci);
	if (!priv->clk_freq)
		priv->clk_freq = ASPEED_PECI_BUS_FREQ_DEFAULT;
	aspeed_peci_clk_set(obj, priv->clk_freq);
	/* Read sampling point and byte mode setting */
	peci_register->control.fields.read_sampling_point_selection =
		priv->rd_sampling_point;
	peci_register->control.fields.enable_64_byte_mode = priv->byte_mode_64;
	/* PECI enable */
	peci_register->control.fields.enable_peci_clock = 1;
	peci_register->control.fields.enable_peci = 1;
	/* Unhook cb function and mask all interrupt */
	for (int_index = 0; int_index < PECI_INT_TYPE_NUM; int_index++)
	{
	    aspeed_peci_int_cb_unhook(obj, int_index);
	}
	/* Clear interrupts */
	peci_register->interrupt_status.value = PECI_INT_MASK;
	peci->init = 1;
#ifdef USE_OS_FLAG_FOR_WAIT
	/* init event ID for ISR */
	evt_id = osEventFlagsNew(NULL);
	if (evt_id == NULL)
		log_error("fail to create evt_id\n");
	aspeed_irq_register(Peci_IRQn, (uint32_t)aspeed_peci_isr, (void *)obj);
	aspeed_peci_int_cb_hook(obj, PECI_INT_CMD_DONE, aspeed_peci_cmd_done_cb, NULL);
#endif
	return HAL_OK;
}