#ifndef SENSOR_DEF_H
#define SENSOR_DEF_H

enum {
  adc_port0 = 0,
  adc_port1,
  adc_port2,
  adc_port3,
  adc_port4,
  adc_port5,
  adc_port6,
  adc_port7,
  adc_port8,
  adc_port9,
  adc_port10,
  adc_port11,
  adc_port12,
  adc_port13,
  adc_port14,
  adc_port15,
};

/*  define config for sensors  */
#define tmp75_in_addr          (0x92 >> 1)
#define tmp75_out_addr         (0x94 >> 1)
#define tmp75_fio_addr         (0x90 >> 1)
#define tmp75_tmp_offset       0x00
#define SSD0_addr              (0xD4 >> 1)
#define SSD0_offset            0x00
#define HSC_addr               (0x80 >> 1)
#define PCH_addr               (0x2C >> 1)
#define CPU_PECI_addr          0x30
#define PVCCD_HV_addr          (0xC4 >> 1)
#define PVCCINFAON_addr        (0xEC >> 1)
#define PVCCFA_EHV_addr        (0xEC >> 1)
#define PVCCIN_addr            (0xC0>> 1)
#define PVCCFA_EHV_FIVRA_addr  (0xC0 >> 1)
#define VR_VOL_CMD             0x8B
#define VR_CUR_CMD             0x8C
#define VR_TEMP_CMD            0x8D
#define VR_PWR_CMD             0x96
#define HSC_VOL_CMD            0x88
#define HSC_CUR_CMD            0x8C
#define HSC_TEMP_CMD           0x8D
#define HSC_PWR_CMD            0x97

/*  threshold sensor number, 1 based  */
#define SENSOR_NUM_TEMP_TMP75_IN          0x01
#define SENSOR_NUM_TEMP_TMP75_OUT         0x02
#define SENSOR_NUM_TEMP_TMP75_FIO         0x03
#define SENSOR_NUM_TEMP_PCH               0x04
#define SENSOR_NUM_TEMP_CPU               0x15
#define SENSOR_NUM_TEMP_DIMM_A            0x06
#define SENSOR_NUM_TEMP_DIMM_C            0x07
#define SENSOR_NUM_TEMP_DIMM_D            0x09
#define SENSOR_NUM_TEMP_DIMM_E            0x0A
#define SENSOR_NUM_TEMP_DIMM_G            0x0B
#define SENSOR_NUM_TEMP_DIMM_H            0x0C
#define SENSOR_NUM_TEMP_SSD0              0x0D
#define SENSOR_NUM_TEMP_HSC               0x0E
#define SENSOR_NUM_TEMP_CPU_MARGIN        0x14
#define SENSOR_NUM_TEMP_CPU_TJMAX         0x05
#define SENSOR_NUM_TEMP_PVCCIN            0x0F
#define SENSOR_NUM_TEMP_PVCCFA_EHV_FIVRA  0x10
#define SENSOR_NUM_TEMP_PVCCFA_EHV        0x11
#define SENSOR_NUM_TEMP_PVCCD_HV          0x12
#define SENSOR_NUM_TEMP_PVCCINFAON        0x13

#define SENSOR_NUM_VOL_STBY12V           0x20
#define SENSOR_NUM_VOL_BAT3V             0x21
#define SENSOR_NUM_VOL_STBY3V            0x22
#define SENSOR_NUM_VOL_STBY1V05          0x23
#define SENSOR_NUM_VOL_STBY1V8           0x24
#define SENSOR_NUM_VOL_HSCIN             0x26
#define SENSOR_NUM_VOL_VCCIN             0x0F
#define SENSOR_NUM_VOL_PVCCIN            0x27
#define SENSOR_NUM_VOL_PVCCFA_EHV_FIVRA  0x28
#define SENSOR_NUM_VOL_PVCCFA_EHV        0x29
#define SENSOR_NUM_VOL_PVCCD_HV          0x2A
#define SENSOR_NUM_VOL_PVCCINFAON        0x2C

#define SENSOR_NUM_CUR_HSCOUT            0x30
#define SENSOR_NUM_CUR_PVCCIN            0x31
#define SENSOR_NUM_CUR_PVCCFA_EHV_FIVRA  0x32
#define SENSOR_NUM_CUR_PVCCFA_EHV        0x33
#define SENSOR_NUM_CUR_PVCCD_HV          0x34
#define SENSOR_NUM_CUR_PVCCINFAON        0x35

#define SENSOR_NUM_PWR_HSCIN             0x39
#define SENSOR_NUM_PWR_PVCCIN            0x3A
#define SENSOR_NUM_PWR_PVCCFA_EHV_FIVRA  0x3C
#define SENSOR_NUM_PWR_PVCCFA_EHV        0x3D
#define SENSOR_NUM_PWR_PVCCD_HV          0x3E
#define SENSOR_NUM_PWR_PVCCINFAON        0x3F

#endif

