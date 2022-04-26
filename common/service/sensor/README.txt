###########################################################################
        README For Sensor Relative Common Code
###########################################################################
Description:
    User guides of new sensor common code

Version: 
    * 1.0 - First commit - 0000.00.00

Required:
    * Relative file
        - common/sensor/sensor.c
        - common/sensor/sensor.h
        - common/sensor/dev/<sensor_name>.c
        - meta-facebook/yv35-cl/src/sensor/plat_hook.c
        - meta-facebook/yv35-cl/src/sensor/plat_hook.h
        - meta-facebook/yv35-cl/src/sensor/plat_sensor.c

Usage:
    * How to add new sensor?
        [STEP1] Create a new sensor-type if type of the sensor you are about to add is not declare before.
            Take TMP75 sensor as an example.
            1. common/sensor/sensor.c
                '''
                SENSOR_DRIVE_INIT_DECLARE(tmp75);

                sensor_drive_tbl[] = {
                    SENSOR_DRIVE_TYPE_INIT_MAP(tmp75),
                    .....
                }
                '''
            2. common/sensor/sensor.h
                '''
                enum sensor_dev {
                    sensor_dev_tmp75 = 0,
                    .....
                }
                '''
            Note1: (IMPORTANT)Following rules need to be bellowed. 
                Ex: (1) SENSOR_DRIVE_INIT_DECLARE( <type-name> )
                    (2) SENSOR_DRIVE_TYPE_INIT_MAP( <type-name> )
                    (3) sensor_dev_<type-name>

        [STEP2] Add a new sensor file including reading & init function if it's new from existed sensor-type.
            Take TMP75 sensor as an example.
            1. common/sensor/dev/tmp75.c
                '''
                uint8_t tmp75_read(uint8_t sensor_num, int *reading) {
                    if (!reading)
                        return SENSOR_UNSPECIFIED_ERROR;
                    .....
                }

                uint8_t tmp75_init(uint8_t sensor_num) {
                    sensor_config[SensorNum_SensorCfg_map[sensor_num]].read = tmp75_read;
                    .....
                }
                '''

            Note1: <type-name>_init and <type-name>_read are requered!
            Note2: Please follow input/output args' type in <type-name>_init and <type-name>_read of this example.
            Note3: Output of <type-name>_read must to be reading status defined in sensor.h
            Note4: <type-name>_read must to be put to ".read" in <type-name>_init

        [STEP3] (optional)Add <pre-hook>/<post-hook> function if there's some actions need to take before/after sensor reading.
                (optional)Add "*pre_sensor_read_args"/"*post_sensor_read_args"/"*init_args" if needed in function of <pre-hook>/<post-hook>/<type-name>_init.
            Take AST ADC sensor as an example.
            1. meta-facebook/yv35-cl/src/sensor/plat_hook.c
                '''
                /* *init_args */
                adc_asd_init_arg adc_asd_init_args[] = {
                    [0] = {.is_init = false}
                };
                .....
                /* <pre-hook> function */
                bool pre_vol_bat3v_read(uint8_t sensor_num, void *args) {
                    .....
                }
                /* <post-hook> function */
                bool post_vol_bat3v_read(uint8_t sensor_num, void *args, int *reading) {
                    .....
                }
                '''
            2. meta-facebook/yv35-cl/src/sensor/plat_hook.h
                '''
                extern adc_asd_init_arg adc_asd_init_args[];
                bool pre_vol_bat3v_read(uint8_t sensor_num, void *args);
                bool post_vol_bat3v_read(uint8_t sensor_num, void *args, int *reading);
                '''

            Note1: Please follow input/output args' type in <pre-hook>/<post-hook> function of this example.
            Note2: <pre-hook>/<post-hook> function name is not restricted.
            Note3: One pre-hook/post-hook function per sensor-type is recommended.
            Note4: *pre_sensor_read_args/*post_sensor_read_args/*init_args could be any type of variable.

        [STEP4] Put your new sensor config into "plat_sensor_config" table.
            Take AST ADC sensor as an example.
            1. meta-facebook/yv35-cl/src/sensor/plat_sensor.c
                '''
                sensor_cfg plat_sensor_config[] = {
                .....
                { SENSOR_NUM_VOL_STBY5V  , sensor_dev_ast_adc  , adc_port9  , 0  , 0   , stby_access  , 711   , 200   , 0  , SENSOR_INIT_STATUS   , 
                  NULL  , NULL  , NULL  , NULL  , &adc_asd_init_args[0] },
                .....
                };
                '''
Note:
    1. "access_checker" functions are moved to meta-facebook/yv35-cl/src/sensor/plat_hook.c
