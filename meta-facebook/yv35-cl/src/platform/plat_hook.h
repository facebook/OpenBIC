#ifndef PLAT_HOOK_H
#define PLAT_HOOK_H

typedef struct _isl69259_pre_proc_arg {
	/* vr page to set */
	uint8_t vr_page;
} isl69259_pre_proc_arg;

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
extern adc_asd_init_arg adc_asd_init_args[];
extern adm1278_init_arg adm1278_init_args[];
extern mp5990_init_arg mp5990_init_args[];
extern pmic_init_arg pmic_init_args[];
extern max16550a_init_arg max16550a_init_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
extern struct tca9548 mux_conf_addr_0xe2[];
extern isl69259_pre_proc_arg isl69259_pre_read_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_isl69259_read(uint8_t sensor_num, void *args);
bool pre_nvme_read(uint8_t sensor_num, void *args);
bool pre_pmic_read(uint8_t sensor_num, void *args);
bool pre_vol_bat3v_read(uint8_t sensor_num, void *args);
bool post_vol_bat3v_read(uint8_t sensor_num, void *args, int *reading);
bool post_cpu_margin_read(uint8_t sensor_num, void *args, int *reading);
bool post_adm1278_power_read(uint8_t sensor_num, void *args, int *reading);
bool post_adm1278_current_read(uint8_t sensor_num, void *args, int *reading);

#endif
