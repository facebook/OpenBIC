#include "plat_fsc.h"
#include "plat_sensor_table.h"
#include "libutil.h"
#include "plat_pwm.h"
#include "plat_hwmon.h"

stepwise_cfg stepwise_table[] = {};

int stepwise_table_size = ARRAY_SIZE(stepwise_table);

pid_cfg pid_table[] = {};

int pid_table_size = ARRAY_SIZE(pid_table);

zone_cfg zone_table[] = {
	{ .table = (fsc_type_mapping[]){ { SENSOR_NUM_MB_RPU_AIR_INLET_TEMP_C, FSC_TYPE_DEFAULT } },
	  .table_size = 1,
	  .FF_gain = 1,
	  .interval = 1,
	  .set_duty = pwm_control,
	  .set_duty_arg = PWM_GROUP_E_HEX_FAN },
	{
		.table = (fsc_type_mapping[]){ { SENSOR_NUM_MB_RPU_AIR_INLET_TEMP_C,
						 FSC_TYPE_DEFAULT } },
		.table_size = 1,
		.FF_gain = 1,
		.interval = 2,
		.set_duty = pwm_control,
		.set_duty_arg = PWM_GROUP_E_PUMP,
	},
	{
		.table = (fsc_type_mapping[]){ { SENSOR_NUM_MB_RPU_AIR_INLET_TEMP_C,
						 FSC_TYPE_DEFAULT } },
		.table_size = 1,
		.FF_gain = 1,
		.interval = 1,
		.set_duty = pwm_control,
		.set_duty_arg = PWM_GROUP_E_RPU_FAN,
	},
};

int zone_table_size = ARRAY_SIZE(zone_table);