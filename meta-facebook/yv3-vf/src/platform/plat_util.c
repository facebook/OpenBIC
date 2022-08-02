#include <stdint.h>
#include <stdlib.h>
#include <kernel.h>

#include "libutil.h"
#include "util_worker.h"
#include "ipmb.h"
#include "ipmi.h"
#include "libipmi.h"

#include "plat_gpio.h"
#include "plat_m2.h"

#include "ipmi.h"
#include "plat_util.h"

typedef struct {
	uint32_t arg1;
	uint32_t arg2;
	uint8_t (*fn)(uint32_t, uint32_t);
} delay_item_t;

typedef struct {
	struct k_timer timer;
	struct k_work free_work; // for free()
	k_timeout_t period;
	k_timeout_t duration;
	uint32_t arg1;
	uint32_t arg2;
	uint8_t (*ex_fn)(uint32_t, uint32_t); // exec func & stop timer
	void (*stop_fn)(uint32_t, uint32_t);
} clock_t;

typedef struct {
	uint8_t gpio_num;
	int int_type;
	DEASSERT_CHK_TYPE_E idx;
	uint8_t (*fn)(DEASSERT_CHK_TYPE_E);
} assert_func_t;

assert_func_t deassert_list[] = {
	{ IRQ_INA230_E1S_0_ALERT_N, GPIO_INT_EDGE_BOTH, DEASSERT_CHK_TYPE_E_INA231_ALERT_0,
	  assert_func },
	{ IRQ_INA230_E1S_1_ALERT_N, GPIO_INT_EDGE_BOTH, DEASSERT_CHK_TYPE_E_INA231_ALERT_1,
	  assert_func },
	{ IRQ_INA230_E1S_2_ALERT_N, GPIO_INT_EDGE_BOTH, DEASSERT_CHK_TYPE_E_INA231_ALERT_2,
	  assert_func },
	{ IRQ_INA230_E1S_3_ALERT_N, GPIO_INT_EDGE_BOTH, DEASSERT_CHK_TYPE_E_INA231_ALERT_3,
	  assert_func },
};

void tmp_func(void *args, uint32_t x)
{
	if (!args)
		return;

	delay_item_t *p = (delay_item_t *)args;
	if (p->fn)
		p->fn(p->arg1, p->arg2);
	SAFE_FREE(p); // free
}

void delay_function(uint32_t delay_time, void *func, uint32_t arg1, uint32_t arg2)
{
	if (!func)
		return;

	worker_job job = { 0 };

	job.fn = func;
	job.ptr_arg = (void *)arg1;
	job.ui32_arg = arg2;
	job.delay_ms = delay_time;

	if (add_work(&job) != 1)
		printf("%s() add_work fail!\n", __func__);
}

void free_timer(struct k_work *work)
{
	if (!work)
		return;
	clock_t *p = CONTAINER_OF(work, clock_t, free_work);
	SAFE_FREE(p);
}

void clock_ex_fn_tmp(struct k_timer *my_timer)
{
	if (!my_timer)
		return;
	clock_t *p = CONTAINER_OF(my_timer, clock_t, timer);

	if (!p->ex_fn) {
		printf("%s() need a function", __func__);
		return;
	}

	if (p->ex_fn(p->arg1, p->arg2))
		k_timer_stop(my_timer);
}

void clock_stop_fn_tmp(struct k_timer *my_timer)
{
	if (!my_timer)
		return;
	clock_t *p = CONTAINER_OF(my_timer, clock_t, timer);

	if (p->stop_fn)
		p->stop_fn(p->arg1, p->arg2);

	// free clock
	k_work_init(&p->free_work, free_timer);
	k_work_submit(&p->free_work);
}

void add_clock(uint32_t arg1, uint32_t arg2, void *ex_fn, void *stop_fn, uint32_t duration,
	       uint32_t period)
{
	clock_t *clock_tmp = malloc(sizeof(clock_t));
	if (!clock_tmp) {
		printf("%s() malloc fail!\n", __func__);
		return;
	}
	clock_tmp->arg1 = arg1;
	clock_tmp->arg2 = arg2;
	clock_tmp->ex_fn = ex_fn; // check by clock_ex_fn_tmp
	clock_tmp->stop_fn = stop_fn; // check by clock_stop_fn_tmp
	clock_tmp->duration = K_MSEC(duration);
	clock_tmp->period = K_MSEC(period);

	k_timer_init(&clock_tmp->timer, clock_ex_fn_tmp, clock_stop_fn_tmp);
	k_timer_start(&clock_tmp->timer, clock_tmp->duration, clock_tmp->period);
}

/* noise */
K_TIMER_DEFINE(ignore_noise_timer_A, NULL, NULL);
K_TIMER_DEFINE(ignore_noise_timer_B, NULL, NULL);
K_TIMER_DEFINE(ignore_noise_timer_C, NULL, NULL);
K_TIMER_DEFINE(ignore_noise_timer_D, NULL, NULL);
static uint8_t noise_flag[NOSIE_E_M2PRSNT_MAX];
struct k_timer *idx_to_noise_timer(NOSIE_E idx)
{
	return (idx == NOSIE_E_M2PRSNT_A) ? &ignore_noise_timer_A :
	       (idx == NOSIE_E_M2PRSNT_B) ? &ignore_noise_timer_B :
	       (idx == NOSIE_E_M2PRSNT_C) ? &ignore_noise_timer_C :
	       (idx == NOSIE_E_M2PRSNT_D) ? &ignore_noise_timer_D :
						  NULL;
}
uint8_t ignore_noise(uint8_t idx, uint32_t m_sec) // 1: exec, 0: noise
{
	struct k_timer *ignore_noise_timer = idx_to_noise_timer(idx);

	if (k_timer_status_get(ignore_noise_timer) > 0) {
		k_timer_stop(ignore_noise_timer);
		noise_flag[idx] = 0;
		return 1;
	} else {
		if (!noise_flag[idx]) {
			k_timer_start(ignore_noise_timer, K_MSEC(m_sec), K_NO_WAIT);
			noise_flag[idx] = 1;
		}
		return 0;
	}
}

void add_sel(uint8_t sensor_type, uint8_t event_type, uint8_t sensor_number, uint8_t event_data1,
	     uint8_t event_data2, uint8_t event_data3)
{
	common_addsel_msg_t sel_msg;
	sel_msg.InF_target = CL_BIC_IPMB;

	sel_msg.sensor_type = sensor_type;
	sel_msg.event_type = event_type;
	sel_msg.sensor_number = sensor_number;
	sel_msg.event_data1 = event_data1;
	sel_msg.event_data2 = event_data2;
	sel_msg.event_data3 = event_data3;

	common_add_sel_evt_record(&sel_msg);

	return;
}

assert_func_t *assert_type_to_deassert_list(DEASSERT_CHK_TYPE_E assert_type)
{
	uint8_t i;

	for (i = 0; i < ARRAY_SIZE(deassert_list); i++) {
		assert_func_t *p = deassert_list + i;
		if (p->idx == assert_type)
			return p;
	}
	return NULL;
}

static void deassert_chk(void *unused, uint32_t assert_type)
{
	ARG_UNUSED(unused);

	if (assert_type >= DEASSERT_CHK_TYPE_E_MAX)
		return;

	assert_func_t *p = assert_type_to_deassert_list(assert_type);
	if (!p) {
		printf("%s() can't find deassert list!\n", __func__);
		return;
	}

	if (!gpio_get(p->gpio_num)) {
		/* the deassert event should be checked after 5 seconds */
		worker_job job = { 0 };
		job.delay_ms = 5000;
		job.fn = deassert_chk;
		job.ui32_arg = (uint32_t)assert_type;
		add_work(&job);
		return;
	}

	add_sel(IPMI_OEM_SENSOR_TYPE_OEM, IPMI_OEM_EVENT_TYPE_DEASSART, SENSOR_NUM_SYS_STA,
		IPMI_EVENT_OFFSET_SYS_INA231_PWR_ALERT, E1S_BOARD_TYPE, assert_type);

	gpio_interrupt_conf(p->gpio_num, p->int_type);
}

uint8_t assert_func(DEASSERT_CHK_TYPE_E assert_type) // 0:success
{
	if (assert_type >= DEASSERT_CHK_TYPE_E_MAX)
		return 1;

	if (!gpio_get(FM_P12V_EDGE_EN))
		return 1;

	assert_func_t *p = assert_type_to_deassert_list(assert_type);
	if (!p) {
		printf("%s() can't find deassert list!\n", __func__);
		return 1;
	}

	gpio_interrupt_conf(p->gpio_num, GPIO_INT_DISABLE);

	add_sel(IPMI_OEM_SENSOR_TYPE_OEM, IPMI_EVENT_TYPE_SENSOR_SPEC, SENSOR_NUM_SYS_STA,
		IPMI_EVENT_OFFSET_SYS_INA231_PWR_ALERT, E1S_BOARD_TYPE, assert_type);

	worker_job job = { 0 };
	job.delay_ms = 5000;
	job.fn = deassert_chk;
	job.ui32_arg = (uint32_t)assert_type;
	add_work(&job);

	return 0;
}