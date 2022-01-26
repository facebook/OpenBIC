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

static uint8_t SnrCfg_num;

snr_cfg plat_sensor_config[] = {
  /* number,                           type,                  port,           address,                  offset,                access check       arg0,   arg1,   cache,   cache_status,        pre_hook_fn,          pre_hook_args,                post_hook_fn,           post_hook_args,             init_arg */

  // temperature
  {SENSOR_NUM_TEMP_TMP75_IN          , sen_dev_tmp75        , i2c_bus2      , tmp75_in_addr           , tmp75_tmp_offset      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , NULL                  },
  {SENSOR_NUM_TEMP_TMP75_OUT         , sen_dev_tmp75        , i2c_bus2      , tmp75_out_addr          , tmp75_tmp_offset      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , NULL                  },
  {SENSOR_NUM_TEMP_TMP75_FIO         , sen_dev_tmp75        , i2c_bus2      , tmp75_fio_addr          , tmp75_tmp_offset      , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , NULL                  },

  // NVME
  {SENSOR_NUM_TEMP_SSD0              , sen_dev_nvme         , i2c_bus2      , SSD0_addr               , SSD0_offset           , post_access      , 0     , 0     , 0      , SNR_INIT_STATUS   , pre_nvme_read       , &mux_conf_addr_0xe2[1]      , NULL                  , NULL                      , NULL                  },

  // PECI                                                                                                                                                        
  {SENSOR_NUM_TEMP_CPU               , sen_dev_intel_peci   , 0             , CPU_PECI_addr           , PECI_TEMP_CPU         , post_access      , 0     , 0     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , NULL                  },
  {SENSOR_NUM_TEMP_CPU_MARGIN        , sen_dev_intel_peci   , 0             , CPU_PECI_addr           , PECI_TEMP_CPU_MARGIN  , post_access      , 0     , 0     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , post_cpu_margin_read  , NULL                      , NULL                  },
  {SENSOR_NUM_TEMP_CPU_TJMAX         , sen_dev_intel_peci   , 0             , CPU_PECI_addr           , PECI_TEMP_CPU_TJMAX   , post_access      , 0     , 0     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , NULL                  },
  {SENSOR_NUM_TEMP_DIMM_A            , sen_dev_intel_peci   , 0             , CPU_PECI_addr           , PECI_TEMP_DIMM_A      , post_access      , 0     , 0     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , NULL                  },
  {SENSOR_NUM_TEMP_DIMM_C            , sen_dev_intel_peci   , 0             , CPU_PECI_addr           , PECI_TEMP_DIMM_C      , post_access      , 0     , 0     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , NULL                  },
  {SENSOR_NUM_TEMP_DIMM_D            , sen_dev_intel_peci   , 0             , CPU_PECI_addr           , PECI_TEMP_DIMM_D      , post_access      , 0     , 0     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , NULL                  },
  {SENSOR_NUM_TEMP_DIMM_E            , sen_dev_intel_peci   , 0             , CPU_PECI_addr           , PECI_TEMP_DIMM_E      , post_access      , 0     , 0     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , NULL                  },
  {SENSOR_NUM_TEMP_DIMM_G            , sen_dev_intel_peci   , 0             , CPU_PECI_addr           , PECI_TEMP_DIMM_G      , post_access      , 0     , 0     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , NULL                  },
  {SENSOR_NUM_TEMP_DIMM_H            , sen_dev_intel_peci   , 0             , CPU_PECI_addr           , PECI_TEMP_DIMM_H      , post_access      , 0     , 0     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , NULL                  },
  {SENSOR_NUM_PWR_CPU                , sen_dev_intel_peci   , 0             , CPU_PECI_addr           , PECI_PWR_CPU          , post_access      , 0     , 0     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , NULL                  },

  // adc voltage                                                                                                                                                 
  {SENSOR_NUM_VOL_STBY12V            , sen_dev_ast_adc      , adc_port0     , 0                       , 0                     , stby_access      , 667   , 100   , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , &adc_asd_init_args[0] },
  {SENSOR_NUM_VOL_STBY3V             , sen_dev_ast_adc      , adc_port2     , 0                       , 0                     , stby_access      , 2     , 1     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , &adc_asd_init_args[0] },
  {SENSOR_NUM_VOL_STBY1V05           , sen_dev_ast_adc      , adc_port3     , 0                       , 0                     , stby_access      , 1     , 1     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , &adc_asd_init_args[0] },
  {SENSOR_NUM_VOL_BAT3V              , sen_dev_ast_adc      , adc_port4     , 0                       , 0                     , stby_access      , 3     , 1     , 0      , SNR_INIT_STATUS   , pre_vol_bat3v_read  , NULL                        , post_vol_bat3v_read   , NULL                      , &adc_asd_init_args[0] },
  {SENSOR_NUM_VOL_STBY5V             , sen_dev_ast_adc      , adc_port9     , 0                       , 0                     , stby_access      , 711   , 200   , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , &adc_asd_init_args[0] },
  {SENSOR_NUM_VOL_DIMM12V            , sen_dev_ast_adc      , adc_port11    , 0                       , 0                     , DC_access        , 667   , 100   , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , &adc_asd_init_args[0] },
  {SENSOR_NUM_VOL_STBY1V2            , sen_dev_ast_adc      , adc_port13    , 0                       , 0                     , stby_access      , 1     , 1     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , &adc_asd_init_args[0] },
  {SENSOR_NUM_VOL_M2_3V3             , sen_dev_ast_adc      , adc_port14    , 0                       , 0                     , DC_access        , 2     , 1     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , &adc_asd_init_args[0] },
  {SENSOR_NUM_VOL_STBY1V8            , sen_dev_ast_adc      , adc_port15    , 0                       , 0                     , stby_access      , 1     , 1     , 0      , SNR_INIT_STATUS   , NULL                , NULL                        , NULL                  , NULL                      , &adc_asd_init_args[0] },

  // VR voltage
  {SENSOR_NUM_VOL_PVCCD_HV           , sen_dev_isl69259     , i2c_bus5      , PVCCD_HV_addr           , VR_VOL_CMD            , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[0]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_VOL_PVCCINFAON         , sen_dev_isl69259     , i2c_bus5      , PVCCINFAON_addr         , VR_VOL_CMD            , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[0]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_VOL_PVCCFA_EHV         , sen_dev_isl69259     , i2c_bus5      , PVCCFA_EHV_addr         , VR_VOL_CMD            , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[1]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_VOL_PVCCIN             , sen_dev_isl69259     , i2c_bus5      , PVCCIN_addr             , VR_VOL_CMD            , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[0]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_VOL_PVCCFA_EHV_FIVRA   , sen_dev_isl69259     , i2c_bus5      , PVCCFA_EHV_FIVRA_addr   , VR_VOL_CMD            , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[1]  , NULL                  , NULL                      , NULL                 },

  // VR current
  {SENSOR_NUM_CUR_PVCCD_HV           , sen_dev_isl69259     , i2c_bus5      , PVCCD_HV_addr           , VR_CUR_CMD            , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[0]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_CUR_PVCCINFAON         , sen_dev_isl69259     , i2c_bus5      , PVCCINFAON_addr         , VR_CUR_CMD            , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[0]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_CUR_PVCCFA_EHV         , sen_dev_isl69259     , i2c_bus5      , PVCCFA_EHV_addr         , VR_CUR_CMD            , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[1]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_CUR_PVCCIN             , sen_dev_isl69259     , i2c_bus5      , PVCCIN_addr             , VR_CUR_CMD            , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[0]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_CUR_PVCCFA_EHV_FIVRA   , sen_dev_isl69259     , i2c_bus5      , PVCCFA_EHV_FIVRA_addr   , VR_CUR_CMD            , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[1]  , NULL                  , NULL                      , NULL                 },

  // VR temperature
  {SENSOR_NUM_TEMP_PVCCD_HV          , sen_dev_isl69259     , i2c_bus5      , PVCCD_HV_addr           , VR_TEMP_CMD           , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[0]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_TEMP_PVCCINFAON        , sen_dev_isl69259     , i2c_bus5      , PVCCINFAON_addr         , VR_TEMP_CMD           , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[0]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_TEMP_PVCCFA_EHV        , sen_dev_isl69259     , i2c_bus5      , PVCCFA_EHV_addr         , VR_TEMP_CMD           , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[1]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_TEMP_PVCCIN            , sen_dev_isl69259     , i2c_bus5      , PVCCIN_addr             , VR_TEMP_CMD           , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[0]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_TEMP_PVCCFA_EHV_FIVRA  , sen_dev_isl69259     , i2c_bus5      , PVCCFA_EHV_FIVRA_addr   , VR_TEMP_CMD           , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[1]  , NULL                  , NULL                      , NULL                 },

  // VR power 
  {SENSOR_NUM_PWR_PVCCD_HV           , sen_dev_isl69259     , i2c_bus5      , PVCCD_HV_addr           , VR_PWR_CMD            , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[0]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_PWR_PVCCINFAON         , sen_dev_isl69259     , i2c_bus5      , PVCCINFAON_addr         , VR_PWR_CMD            , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[0]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_PWR_PVCCFA_EHV         , sen_dev_isl69259     , i2c_bus5      , PVCCFA_EHV_addr         , VR_PWR_CMD            , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[1]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_PWR_PVCCIN             , sen_dev_isl69259     , i2c_bus5      , PVCCIN_addr             , VR_PWR_CMD            , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[0]  , NULL                  , NULL                      , NULL                 },
  {SENSOR_NUM_PWR_PVCCFA_EHV_FIVRA   , sen_dev_isl69259     , i2c_bus5      , PVCCFA_EHV_FIVRA_addr   , VR_PWR_CMD            , DC_access        , 0     , 0     , 0      , SNR_INIT_STATUS    , pre_isl69259_read   , &isl69259_pre_read_args[1]  , NULL                  , NULL                      , NULL                 },

  // ME
  {SENSOR_NUM_TEMP_PCH               , sen_dev_pch          , i2c_bus3      , PCH_addr                , PCH_TEMP_SNR_NUM      , post_access      , 0     , 0     , 0      , SNR_INIT_STATUS    , NULL                , NULL                         , NULL                  , NULL                     , NULL                 },

  // HSC
  {SENSOR_NUM_TEMP_HSC               , sen_dev_adm1278      , i2c_bus2      , HSC_addr                , HSC_TEMP_CMD          , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS    , NULL                , NULL                         , NULL                  , NULL                     , &adm1278_init_args[0]},
  {SENSOR_NUM_VOL_HSCIN              , sen_dev_adm1278      , i2c_bus2      , HSC_addr                , HSC_VOL_CMD           , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS    , NULL                , NULL                         , NULL                  , NULL                     , &adm1278_init_args[0]},
  {SENSOR_NUM_CUR_HSCOUT             , sen_dev_adm1278      , i2c_bus2      , HSC_addr                , HSC_CUR_CMD           , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS    , NULL                , NULL                         , NULL                  , NULL                     , &adm1278_init_args[0]},
  {SENSOR_NUM_PWR_HSCIN              , sen_dev_adm1278      , i2c_bus2      , HSC_addr                , HSC_PWR_CMD           , stby_access      , 0     , 0     , 0      , SNR_INIT_STATUS    , NULL                , NULL                         , NULL                  , NULL                     , &adm1278_init_args[0]},
};

snr_cfg fix_C2Snrconfig_table[] = {
// number , type , port , address , offset , access check , arg0 , arg1 , cache , cache_status
};
snr_cfg fix_1ouSnrconfig_table[] = {
// number , type , port , address , offset , access check , arg0 , arg1 , cache , cache_status
};
snr_cfg fix_DVPSnrconfig_table[] = {
// number , type , port , address , offset , access check , arg0 , arg1 , cache , cache_status
};

bool pal_load_snr_config(void) {
  memcpy(&sensor_config[0], &plat_sensor_config[0], sizeof(plat_sensor_config));
  return 1;
};

uint8_t map_SnrNum_Snrconfig( uint8_t sensor_num ) {
  uint8_t i , j;
  for ( i = 0 ; i < SENSOR_NUM_MAX ; i++ ) {
    for ( j = 0 ; j < SnrCfg_num ; ++j ) {
      if ( sensor_num == sensor_config[j].num ) {
        return j;
      } else if ( i == SnrCfg_num ) {
        return 0xFF;
      }
    }
  }
  return 0xFF;
};

void add_Snrconfig( snr_cfg add_Snrconfig ) {
  if ( map_SnrNum_Snrconfig( add_Snrconfig.num ) != 0xFF ) {
    printk( "add sensor num is already exists\n" );
    return;
  }
  sensor_config[ SnrCfg_num++ ] = add_Snrconfig;
};

void pal_fix_Snrconfig() {
/*
  SnrCfg_num = sizeof(plat_sensor_config) / sizeof(plat_sensor_config[0]);
  uint8_t fix_SnrCfg_num;
  if ( get_bic_class() ) {
    // fix usage when fix_C2Snrconfig_table is defined
    fix_SnrCfg_num = sizeof( fix_C2Snrconfig_table ) / sizeof( fix_C2Snrconfig_table[0] );
    while ( fix_SnrCfg_num ) {
      add_Snrconfig ( fix_C2Snrconfig_table[ fix_SnrCfg_num - 1 ] );
      fix_SnrCfg_num--;
    }
  }
  if ( get_1ou_status() ) {
    // fix usage when fix_1ouSnrconfig_table is defined
    fix_SnrCfg_num = sizeof( fix_1ouSnrconfig_table ) / sizeof( fix_1ouSnrconfig_table[0] );
    while ( fix_SnrCfg_num ) {
      add_Snrconfig ( fix_1ouSnrconfig_table[ fix_SnrCfg_num - 1 ] );
      fix_SnrCfg_num--;
    }
  }
  if ( get_2ou_status() ) {
    // fix usage when fix_DVPSnrconfig_table is defined
    fix_SnrCfg_num = sizeof( fix_DVPSnrconfig_table ) / sizeof( fix_DVPSnrconfig_table[0] );
    while ( fix_SnrCfg_num ) {
      add_Snrconfig ( fix_DVPSnrconfig_table[ fix_SnrCfg_num - 1 ] );
      fix_SnrCfg_num--;
    }
  }
  if ( SnrCfg_num != SDR_NUM ) {
    printk("fix sensor SDR and config table not match\n");
  }
*/
};
