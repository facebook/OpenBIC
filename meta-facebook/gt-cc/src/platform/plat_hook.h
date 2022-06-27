#ifndef PLAT_HOOK_H
#define PLAT_HOOK_H

typedef struct _isl69259_pre_proc_arg {
	struct tca9548 *mux_info_p;
	/* vr page to set */
	uint8_t vr_page;
} isl69259_pre_proc_arg;

typedef struct _pex89000_pre_proc_arg {
	struct tca9548 *mux_info_p;
} pex89000_pre_proc_arg;

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
extern isl28022_init_arg nic_sensor_init_args[];
extern mp5990_init_arg mp5990_init_args[];
extern adc_asd_init_arg adc_asd_init_args[];
extern isl28022_init_arg pex_p1v25_sensor_init_args[];
extern isl28022_init_arg pex_p1v8_sensor_init_args[];
extern isl28022_init_arg ssd_sensor_init_args[];
extern pex89000_init_arg pex_sensor_init_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
extern struct tca9548 mux_conf_addr_0xe0[];
extern struct tca9548 mux_conf_addr_0xe2[];
extern isl69259_pre_proc_arg isl69259_pre_read_args[];
extern pex89000_pre_proc_arg pex89000_pre_read_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_isl69259_read(uint8_t sensor_num, void *args);
bool pre_pex89000_read(uint8_t sensor_num, void *args);
bool pre_i2c_bus_read(uint8_t sensor_num, void *args);
bool post_i2c_bus_read(uint8_t sensor_num, void *args, int *reading);

struct k_mutex *find_bus_mutex(uint8_t sensor_num);
#endif
