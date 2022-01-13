#include <stdio.h>
#include <string.h>
#include "sdr.h"
#include "sensor.h"
#include "sensor_def.h"
#include "hal_i2c.h"
#include "plat_i2c.h"
#include "plat_func.h"
#include "pal.h"

bool stby_access(uint8_t snr_num);

snr_cfg plat_sensor_config[] = {
  /* number,                           type,            port,           address,                  offset,                     access check       arg0,   arg1,   cache,   cache_status */

  // temperature
  {SENSOR_NUM_TEMP_TMP75_IN          , type_tmp75     , i2c_bus1      , tmp75_in_addr           , TMP75_TEMP_OFFSET         , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_TEMP_TMP75_OUT         , type_tmp75     , i2c_bus1      , tmp75_out_addr          , TMP75_TEMP_OFFSET         , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},

  // adc
  {SENSOR_NUM_VOL_P5V_STBY           , type_adc       , adc_port1     , NULL                    , NULL                      , stby_access      , 736   , 200   , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_VOL_P12V_STBY          , type_adc       , adc_port0     , NULL                    , NULL                      , stby_access      , 178   , 20    , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_VOL_P3V3_STBY          , type_adc       , adc_port2     , NULL                    , NULL                      , stby_access      , 487   , 200   , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_VOL_P5V_USB            , type_adc       , adc_port7     , NULL                    , NULL                      , stby_access      , 736   , 200   , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_VOL_P1V2_BIC_STBY      , type_adc       , adc_port5     , NULL                    , NULL                      , stby_access      , 1     , 1     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_VOL_P1V0_STBY          , type_adc       , adc_port4     , NULL                    , NULL                      , stby_access      , 1     , 1     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_CUR_P12V_FAN           , type_adc       , adc_port6     , NULL                    , NULL                      , stby_access      , 1     , 1     , 0      , SNR_INIT_STATUS},

  // medusa board
  {SENSOR_NUM_VOL_MEDUSA_12V_IN      , type_medusa    , i2c_bus2      , medusa_addr             , MEDUSA_VOL_OUT_OFFSET     , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_VOL_MEDUSA_12V_OUT     , type_medusa    , i2c_bus2      , medusa_addr             , MEDUSA_VOL_IN_OFFSET      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_CUR_MEDUSA_IOUT        , type_medusa    , i2c_bus2      , medusa_addr             , MEDUSA_CUR_OFFSET         , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_PWR_MEDUSA_12V         , type_medusa    , i2c_bus2      , medusa_addr             , MEDUSA_PWR_OFFSET         , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},

  // HSC
  {SENSOR_NUM_TEMP_HSC               , type_hsc       , i2c_bus2      , HSC_addr                , HSC_TEMP_OFFSET           , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_VOL_HSCIN              , type_hsc       , i2c_bus2      , HSC_addr                , HSC_VOL_OFFSET            , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_CUR_HSCOUT             , type_hsc       , i2c_bus2      , HSC_addr                , HSC_CUR_OFFSET            , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_PWR_HSCIN              , type_hsc       , i2c_bus2      , HSC_addr                , HSC_PWR_OFFSET            , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},

  // Fan
  {SENSOR_NUM_DUAL_FAN_BMC_TACH_0    , type_fan       , i2c_bus2      , fan_addr                , NULL                      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_DUAL_FAN_BMC_TACH_1    , type_fan       , i2c_bus2      , fan_addr                , NULL                      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_DUAL_FAN_BMC_TACH_2    , type_fan       , i2c_bus2      , fan_addr                , NULL                      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_DUAL_FAN_BMC_TACH_3    , type_fan       , i2c_bus2      , fan_addr                , NULL                      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_DUAL_FAN_BMC_TACH_4    , type_fan       , i2c_bus2      , fan_addr                , NULL                      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_DUAL_FAN_BMC_TACH_5    , type_fan       , i2c_bus2      , fan_addr                , NULL                      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_DUAL_FAN_BMC_TACH_6    , type_fan       , i2c_bus2      , fan_addr                , NULL                      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_DUAL_FAN_BMC_TACH_7    , type_fan       , i2c_bus2      , fan_addr                , NULL                      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_SINGLE_FAN_BMC_TACH_0  , type_fan       , i2c_bus2      , fan_addr                , NULL                      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_SINGLE_FAN_BMC_TACH_1  , type_fan       , i2c_bus2      , fan_addr                , NULL                      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_SINGLE_FAN_BMC_TACH_2  , type_fan       , i2c_bus2      , fan_addr                , NULL                      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
  {SENSOR_NUM_SINGLE_FAN_BMC_TACH_3  , type_fan       , i2c_bus2      , fan_addr                , NULL                      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS},
};

bool stby_access(uint8_t snr_num) {
  return 1;
}

bool pal_load_snr_config(void) {
  memcpy(&sensor_config[0], &plat_sensor_config[0], sizeof(plat_sensor_config));
  return 1;
};

