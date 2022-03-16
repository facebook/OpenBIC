#include <stdio.h>
#include <string.h>
#include "sdr.h"
#include "sensor.h"
#include "sensor_def.h"
#include "hal_i2c.h"
#include "plat_i2c.h"
#include "plat_func.h"
#include "pal.h"
#include "plat_gpio.h"
#include "plat_hook.h"
#include "intel_peci.h"

static uint8_t SensorCfg_num;

#define NONE 0

sensor_cfg plat_sensor_config[] = {
	/* number, type, port, address, offset, access check, arg0, arg1, cache, cache_status, 
  pre_hook_fn, pre_hook_args, post_hook_fn, post_hook_args, init_arg */

	// temperature
	{ SENSOR_NUM_TEMP_TMP75_IN, sensor_dev_tmp75, i2c_bus2, tmp75_in_addr, tmp75_tmp_offset,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP75_OUT, sensor_dev_tmp75, i2c_bus2, tmp75_out_addr, tmp75_tmp_offset,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP75_FIO, sensor_dev_tmp75, i2c_bus2, tmp75_fio_addr, tmp75_tmp_offset,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// NVME
	{ SENSOR_NUM_TEMP_SSD0, sensor_dev_nvme, i2c_bus2, SSD0_addr, SSD0_offset, post_access, 0,
	  0, 0, SENSOR_INIT_STATUS, pre_nvme_read, &mux_conf_addr_0xe2[1], NULL, NULL, NULL },

	// PECI
	{ SENSOR_NUM_TEMP_CPU, sensor_dev_intel_peci, NONE, CPU_PECI_addr, PECI_TEMP_CPU,
	  post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_CPU_MARGIN, sensor_dev_intel_peci, NONE, CPU_PECI_addr,
	  PECI_TEMP_CPU_MARGIN, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL,
	  post_cpu_margin_read, NULL, NULL },
	{ SENSOR_NUM_TEMP_CPU_TJMAX, sensor_dev_intel_peci, NONE, CPU_PECI_addr,
	  PECI_TEMP_CPU_TJMAX, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_DIMM_A, sensor_dev_intel_peci, NONE, CPU_PECI_addr,
	  PECI_TEMP_CHANNEL0_DIMM0, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_C, sensor_dev_intel_peci, NONE, CPU_PECI_addr,
	  PECI_TEMP_CHANNEL2_DIMM0, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_D, sensor_dev_intel_peci, NONE, CPU_PECI_addr,
	  PECI_TEMP_CHANNEL3_DIMM0, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_E, sensor_dev_intel_peci, NONE, CPU_PECI_addr,
	  PECI_TEMP_CHANNEL4_DIMM0, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_G, sensor_dev_intel_peci, NONE, CPU_PECI_addr,
	  PECI_TEMP_CHANNEL6_DIMM0, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_H, sensor_dev_intel_peci, NONE, CPU_PECI_addr,
	  PECI_TEMP_CHANNEL7_DIMM0, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_CPU, sensor_dev_intel_peci, NONE, CPU_PECI_addr, PECI_PWR_CPU, post_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// adc voltage
	{ SENSOR_NUM_VOL_STBY12V, sensor_dev_ast_adc, adc_port0, NONE, NONE, stby_access, 667, 100,
	  0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY3V, sensor_dev_ast_adc, adc_port2, NONE, NONE, stby_access, 2, 1, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V05, sensor_dev_ast_adc, adc_port3, NONE, NONE, stby_access, 1, 1, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_BAT3V, sensor_dev_ast_adc, adc_port4, NONE, NONE, stby_access, 3, 1, 0,
	  SENSOR_INIT_STATUS, pre_vol_bat3v_read, NULL, post_vol_bat3v_read, NULL,
	  &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY5V, sensor_dev_ast_adc, adc_port9, NONE, NONE, stby_access, 711, 200,
	  0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_DIMM12V, sensor_dev_ast_adc, adc_port11, NONE, NONE, DC_access, 667, 100,
	  0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V2, sensor_dev_ast_adc, adc_port13, NONE, NONE, stby_access, 1, 1, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_M2_3V3, sensor_dev_ast_adc, adc_port14, NONE, NONE, DC_access, 2, 1, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V8, sensor_dev_ast_adc, adc_port15, NONE, NONE, stby_access, 1, 1, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	// VR voltage
	{ SENSOR_NUM_VOL_PVCCD_HV, sensor_dev_isl69259, i2c_bus5, PVCCD_HV_addr, VR_VOL_CMD,
	  VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCINFAON, sensor_dev_isl69259, i2c_bus5, PVCCINFAON_addr, VR_VOL_CMD,
	  VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCFA_EHV, sensor_dev_isl69259, i2c_bus5, PVCCFA_EHV_addr, VR_VOL_CMD,
	  VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[1],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCIN, sensor_dev_isl69259, i2c_bus5, PVCCIN_addr, VR_VOL_CMD, VR_access,
	  0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0], NULL, NULL,
	  NULL },
	{ SENSOR_NUM_VOL_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, i2c_bus5, PVCCFA_EHV_FIVRA_addr,
	  VR_VOL_CMD, VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read,
	  &isl69259_pre_read_args[1], NULL, NULL, NULL },

	// VR current
	{ SENSOR_NUM_CUR_PVCCD_HV, sensor_dev_isl69259, i2c_bus5, PVCCD_HV_addr, VR_CUR_CMD,
	  VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCINFAON, sensor_dev_isl69259, i2c_bus5, PVCCINFAON_addr, VR_CUR_CMD,
	  VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCFA_EHV, sensor_dev_isl69259, i2c_bus5, PVCCFA_EHV_addr, VR_CUR_CMD,
	  VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[1],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCIN, sensor_dev_isl69259, i2c_bus5, PVCCIN_addr, VR_CUR_CMD, VR_access,
	  0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0], NULL, NULL,
	  NULL },
	{ SENSOR_NUM_CUR_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, i2c_bus5, PVCCFA_EHV_FIVRA_addr,
	  VR_CUR_CMD, VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read,
	  &isl69259_pre_read_args[1], NULL, NULL, NULL },

	// VR temperature
	{ SENSOR_NUM_TEMP_PVCCD_HV, sensor_dev_isl69259, i2c_bus5, PVCCD_HV_addr, VR_TEMP_CMD,
	  VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCINFAON, sensor_dev_isl69259, i2c_bus5, PVCCINFAON_addr, VR_TEMP_CMD,
	  VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCFA_EHV, sensor_dev_isl69259, i2c_bus5, PVCCFA_EHV_addr, VR_TEMP_CMD,
	  VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[1],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCIN, sensor_dev_isl69259, i2c_bus5, PVCCIN_addr, VR_TEMP_CMD,
	  VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, i2c_bus5, PVCCFA_EHV_FIVRA_addr,
	  VR_TEMP_CMD, VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read,
	  &isl69259_pre_read_args[1], NULL, NULL, NULL },

	// VR power
	{ SENSOR_NUM_PWR_PVCCD_HV, sensor_dev_isl69259, i2c_bus5, PVCCD_HV_addr, VR_PWR_CMD,
	  VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCINFAON, sensor_dev_isl69259, i2c_bus5, PVCCINFAON_addr, VR_PWR_CMD,
	  VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCFA_EHV, sensor_dev_isl69259, i2c_bus5, PVCCFA_EHV_addr, VR_PWR_CMD,
	  VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[1],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCIN, sensor_dev_isl69259, i2c_bus5, PVCCIN_addr, VR_PWR_CMD, VR_access,
	  0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0], NULL, NULL,
	  NULL },
	{ SENSOR_NUM_PWR_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, i2c_bus5, PVCCFA_EHV_FIVRA_addr,
	  VR_PWR_CMD, VR_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read,
	  &isl69259_pre_read_args[1], NULL, NULL, NULL },

	// ME
	{ SENSOR_NUM_TEMP_PCH, sensor_dev_pch, i2c_bus3, PCH_addr, PCH_TEMP_SENSOR_NUM, post_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// HSC
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_adm1278, i2c_bus2, HSC_addr, HSC_TEMP_CMD, stby_access, 0,
	  0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_adm1278, i2c_bus2, HSC_addr, HSC_VOL_CMD, stby_access, 0,
	  0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_adm1278, i2c_bus2, HSC_addr, HSC_CUR_CMD, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_adm1278, i2c_bus2, HSC_addr, HSC_PWR_CMD, stby_access, 0,
	  0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
};

sensor_cfg fix_C2Sensorconfig_table[] = {
	// number , type , port , address , offset , access check , arg0 , arg1 , cache , cache_status
};
sensor_cfg fix_1ouSensorconfig_table[] = {
	// number , type , port , address , offset , access check , arg0 , arg1 , cache , cache_status
};
sensor_cfg fix_DVPSensorconfig_table[] = {
	// number , type , port , address , offset , access check , arg0 , arg1 , cache , cache_status
};

bool pal_load_sensor_config(void)
{
	memcpy(&sensor_config[0], &plat_sensor_config[0], sizeof(plat_sensor_config));
	return 1;
};

uint8_t map_SensorNum_Sensorconfig(uint8_t sensor_num)
{
	uint8_t i, j;
	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		for (j = 0; j < SensorCfg_num; ++j) {
			if (sensor_num == sensor_config[j].num) {
				return j;
			} else if (i == SensorCfg_num) {
				return 0xFF;
			}
		}
	}
	return 0xFF;
};

void add_Sensorconfig(sensor_cfg add_Sensorconfig)
{
	if (map_SensorNum_Sensorconfig(add_Sensorconfig.num) != 0xFF) {
		printk("add sensor num is already exists\n");
		return;
	}
	sensor_config[SensorCfg_num++] = add_Sensorconfig;
};

void check_vr_type(uint8_t index)
{
	uint8_t retry = 5;
	I2C_MSG msg;

	msg.bus = sensor_config[index].port;
	msg.slave_addr = sensor_config[index].slave_addr;
	msg.tx_len = 1;
	msg.rx_len = 7;
	msg.data[0] = PMBUS_IC_DEVICE_ID;

	if (i2c_master_read(&msg, retry))
		return;

	if (msg.data[0] == 0x06 && msg.data[1] == 0x54 && msg.data[2] == 0x49 &&
	    msg.data[3] == 0x53 && msg.data[4] == 0x68 && msg.data[5] == 0x90 &&
	    msg.data[6] == 0x00) {
		/* TI tps53689 */
		sensor_config[index].type = sensor_dev_tps53689;
	} else if (msg.data[0] == 0x02 && msg.data[2] == 0x8A) {
		/* Infineon xdpe15284 */
		sensor_config[index].type = sensor_dev_xdpe15284;
	}
}

void pal_fix_Sensorconfig()
{
	uint8_t sensor_num = sizeof(plat_sensor_config) / sizeof(plat_sensor_config[0]);

	/* check sensor type of VR */
	for (uint8_t index = 0; index < sensor_num; index++) {
		if (sensor_config[index].type == sensor_dev_isl69259) {
			check_vr_type(index);
		}
	}
};
