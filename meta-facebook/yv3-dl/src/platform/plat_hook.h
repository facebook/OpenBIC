#ifndef PLAT_HOOK_H
#define PLAT_HOOK_H

typedef struct _vr_pre_proc_arg {
    /* vr page to set */
    uint8_t vr_page;
} vr_pre_proc_arg;


/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/

extern adc_asd_init_arg adc_asd_init_args[];
extern ltc4282_init_arg ltc4282_init_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/

extern struct tca9548 mux_conf_addr_0xe2[];
extern vr_pre_proc_arg vr_page_select[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/

bool pre_nvme_read(uint8_t sensor_num, void *args);
bool pre_vol_bat3v_read(uint8_t sensor_num, void *args);
bool post_vol_bat3v_read(uint8_t sensor_num, void *args, int *reading);
bool post_cpu_margin_read(uint8_t sensor_num, void *args, int *reading);
bool pre_vr_read(uint8_t sensor_num, void *args);
bool post_xdpe12284c_read(uint8_t sensor_num, void *args, int *reading);
bool post_isl69254_read(uint8_t sensor_num, void *args, int *reading);


#endif
