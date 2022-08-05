#ifndef PLAT_HOOK_H
#define PLAT_HOOK_H

typedef struct _raa229621_pre_proc_arg {
	/* vr page to set */
	uint8_t vr_page;
} vr_pre_proc_arg;

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
extern adc_asd_init_arg ast_adc_init_args[];
extern adm1278_init_arg adm1278_init_args[];
extern apml_mailbox_init_arg apml_mailbox_init_args[];
extern ltc4282_init_arg ltc4282_init_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
extern struct tca9548 mux_conf_addr_0xe2[];
extern vr_pre_proc_arg vr_pre_read_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_nvme_read(uint8_t sensor_num, void *args);
bool pre_vr_read(uint8_t sensor_num, void *args);
bool pre_vol_bat3v_read(uint8_t sensor_num, void *args);
bool post_vol_bat3v_read(uint8_t sensor_num, void *args, int *reading);
bool post_adm1278_cur_read(uint8_t sensor_num, void *args, int *reading);
bool post_adm1278_pwr_read(uint8_t sensor_num, void *args, int *reading);

#endif
