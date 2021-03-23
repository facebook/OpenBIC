/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include "common.h"
#include "memory_map.h"
#include "log.h"
#include "pinctrl_aspeed.h"

extern int aspeed_pin_desc_table[];

#define SIG_DEFINE(sig, ...) \
    [sig] = SIG_SYM_PTR(sig),
const struct aspeed_sig_desc *aspeed_sig_desc_table[] __attribute__((section(".pinctrl_sig_tbl"))) = 
{
    #include "sig_def_list.h"
};
#undef SIG_DEFINE

#define FUN_DEFINE(fun, ...) \
    [GET_FUN_ID(fun)] = FUN_SYM_PTR(fun),
const struct aspeed_fun_desc *aspeed_fun_desc_table[] __attribute__((section(".pinctrl_fun_tbl")))  = 
{
    #include "fun_def_list.h"
};
#undef FUN_DEFINE


/**
 * aspeed_pinctrl_pin_status() - get the signal id of the occupied pin
 * @pin_id: the pin id from pin_id.h
 * @return  if > 0 :signal id, -1: the pin is free, -2: pin id Invalid
 */
static int aspeed_pinctrl_pin_status(int pin_id)
{
	if (pin_id >= MAX_PIN_ID || pin_id < 0) {
		log_error("Invalid argument %d\n", pin_id);
		return -2;
	}
	return aspeed_pin_desc_table[pin_id];
}
/**
 * aspeed_pinctrl_sig_request() - request the signal. If the pin of the signal
 * is free occupied it.
 * @sig_id: the signal id from sig_id.h
 * @return  hal status
 */
hal_status_t aspeed_pinctrl_sig_request(uint32_t sig_id)
{
	struct aspeed_sig_desc *sig_desc = aspeed_sig_desc_table[sig_id];
	struct aspeed_sig_en *sig_en;
	int ret_sig_id;
	int pin_id;
	int sig_en_number;
	int sig_en_idx;
	if (sig_desc == NULL) {
		log_error("Invalid argument %d\n", sig_desc);
		return HAL_ERROR;
	}
	pin_id = sig_desc->pin;
	ret_sig_id = aspeed_pinctrl_pin_status(pin_id);
	if (ret_sig_id == -1) {
		aspeed_pin_desc_table[pin_id] = sig_id;
#ifdef CONFIG_PINCTRL_NAME_TABLE
		log_debug("The pin %s is being occupied by signal %s\n",
				 aspeed_pin_name[pin_id], aspeed_sig_name[sig_id]);
#else
		log_debug("The pin %d is being occupied by signal %d\n", pin_id, sig_id);
#endif
		sig_en_number = sig_desc->nsig_en;
		for (sig_en_idx = 0; sig_en_idx < sig_en_number; sig_en_idx++) {
			sig_en = &sig_desc->sig_en[sig_en_idx];
			if (sig_en->op) {
				SCU_WR(sig_en->offset, SCU_RD(sig_en->offset) & ~BIT(sig_en->bits));
			} else {
				SCU_WR(sig_en->offset, SCU_RD(sig_en->offset) | BIT(sig_en->bits));
			}
		}
		return HAL_OK;
	} else if (ret_sig_id > 0) {
#ifdef CONFIG_PINCTRL_NAME_TABLE
		log_error("Request %s to pin %s error: already occupied by signal %s\n",
				  aspeed_sig_name[sig_id], aspeed_pin_name[pin_id],
				  aspeed_sig_name[ret_sig_id]);
#else
		log_error("Request %d to pin %d error: already occupied by signal %d\n",
				  sig_id, pin_id, ret_sig_id);
#endif
		return HAL_BUSY;
	} else {
		return HAL_ERROR;
	}
}
/**
 * aspeed_pinctrl_fn_group_request() - request all of the signal in the function group.
 * @fun_id: the function group id from fun_id.h
 * @return  hal status
 */
hal_status_t aspeed_pinctrl_fn_group_request(uint32_t fun_id)
{
	struct aspeed_fun_desc *fun_desc = aspeed_fun_desc_table[fun_id];
	int sig_number;
	int sig_idx;
	uint16_t sig_id;
	hal_status_t ret = HAL_OK;

	if (fun_desc == NULL) {
		log_error("Invalid argument %d\n", fun_desc);
		return HAL_ERROR;
	}
	sig_number = fun_desc->nsignal;
	for (sig_idx = 0; sig_idx < sig_number; sig_idx++) {
		sig_id = fun_desc->sig_id_list[sig_idx];
		ret |= aspeed_pinctrl_sig_request(sig_id);
	}
	if (ret)
		return HAL_ERROR;
	return HAL_OK;
}

/**
 * aspeed_pinctrl_init() - Init all of the function which descripts at fun_def_list.h.
 */
void aspeed_pinctrl_init(void)
{
	uint32_t fun_id;
	for (fun_id = 0; fun_id < MAX_FUN_ID; fun_id++) {
		aspeed_pinctrl_fn_group_request(fun_id);
	}
}