#include <stdio.h>
#include <stdlib.h>
#include "sensor.h"
#include "sensor_def.h"
#include "pal.h"
#include <drivers/peci.h>

const struct device *dev ;

bool peci_init(){
  dev = device_get_binding("PECI");
  int ret;
  uint32_t bitrate = 1000;
  if (!dev) {
    printk("peci device not found");
    return false;
  }
  ret = peci_config(dev, bitrate);
  if (ret) {
    printk("set bitrate %dKbps failed %d\n", bitrate, ret);
    return false;
  }
  ret = peci_enable(dev);
  if (ret) {
    printk("peci enable failed %d\n", ret);
    return false;
  }
  return true;
}

bool pal_peci_read(uint8_t sensor_num, int *reading) {
  struct peci_msg rdpkgcfg;
  uint8_t u8Index;
  uint16_t u16Param;
  int val , ret , complete_code;
  static uint8_t cpu_temp_tjmax = 0;
  static bool get_tjmax = 0;
  if ( sensor_num == SENSOR_NUM_TEMP_CPU_MARGIN ) {
    u8Index = 0x02;
    u16Param = 0x00ff;
  }else if ( sensor_num == SENSOR_NUM_TEMP_CPU ){
    if( !get_tjmax ) {
      return false;
    }else {
      return true;
    }
  }else if ( sensor_num == SENSOR_NUM_TEMP_CPU_TJMAX ){
    if( get_tjmax ) {
      return true;
    }else {
      u8Index = 0x10;
      u16Param = 0x0000;
    }
  }else if ( sensor_num == SENSOR_NUM_TEMP_DIMM_A ){
    u8Index = 0x0E;
    u16Param = 0x0000;
  }else if ( sensor_num == SENSOR_NUM_TEMP_DIMM_C ){
    u8Index = 0x0E;
    u16Param = 0x0002;
  }else if ( sensor_num == SENSOR_NUM_TEMP_DIMM_D ){
    u8Index = 0x0E;
    u16Param = 0x0003;
  }else if ( sensor_num == SENSOR_NUM_TEMP_DIMM_E ){
    u8Index = 0x0E;
    u16Param = 0x0004;
  }else if ( sensor_num == SENSOR_NUM_TEMP_DIMM_G ){
    u8Index = 0x0E;
    u16Param = 0x0006;
  }else if ( sensor_num == SENSOR_NUM_TEMP_DIMM_H ){
    u8Index = 0x0E;
    u16Param = 0x0007;
  }else {
    printf("Unrecognized sensor reading\n");
    return false;
  }

  rdpkgcfg.addr = sensor_config[SnrNum_SnrCfg_map[sensor_num]].slave_addr;
  rdpkgcfg.tx_buffer.len = PECI_RD_PKG_WR_LEN;
  rdpkgcfg.rx_buffer.len = PECI_RD_PKG_LEN_DWORD;
  rdpkgcfg.tx_buffer.buf = malloc(rdpkgcfg.tx_buffer.len);
  rdpkgcfg.rx_buffer.buf = malloc(rdpkgcfg.rx_buffer.len);
  rdpkgcfg.cmd_code = 0xa1 ;
  rdpkgcfg.tx_buffer.buf[0] =0x0;
  rdpkgcfg.tx_buffer.buf[1] =u8Index;
  rdpkgcfg.tx_buffer.buf[2] =u16Param & 0xff;
  rdpkgcfg.tx_buffer.buf[3] =u16Param >> 8;
  ret = peci_transfer(dev, &rdpkgcfg);
  complete_code = rdpkgcfg.rx_buffer.buf[0];
  if ( ret ) {
    if ( sensor_num == SENSOR_NUM_TEMP_CPU_MARGIN ) {
      sensor_config[SnrNum_SnrCfg_map[SENSOR_NUM_TEMP_CPU]].cache_status = SNR_FAIL_TO_ACCESS;
    }
    free(rdpkgcfg.tx_buffer.buf);
    free(rdpkgcfg.rx_buffer.buf);
    return false;
  }
  if ( complete_code != 0x40 ) {
    if ( complete_code == 0x80 || complete_code == 0x81 ) {
      printk("Response timeout. Retry is appropriate.\n");
    }else if ( complete_code == 0x90 ) {
      printk("Unknown request\n");
    }else{
      printk("PECI control hardware, firmware or associated logic error\n");
    }
    free(rdpkgcfg.tx_buffer.buf);
    free(rdpkgcfg.rx_buffer.buf);
    return false;
  }
  if ( sensor_num == SENSOR_NUM_TEMP_CPU_MARGIN ) {
    val = ( ( 0xFFFF - ( (rdpkgcfg.rx_buffer.buf[2] << 8) | rdpkgcfg.rx_buffer.buf[1] ) ) >> 6 ) +1;
    sensor_config[SnrNum_SnrCfg_map[SENSOR_NUM_TEMP_CPU]].cache = cpu_temp_tjmax - (cal_MBR(sensor_num,val) & 0xff);
    sensor_config[SnrNum_SnrCfg_map[SENSOR_NUM_TEMP_CPU]].cache_status = SNR_READ_SUCCESS;
  }else if ( sensor_num == SENSOR_NUM_TEMP_CPU_TJMAX ) {
    val = rdpkgcfg.rx_buffer.buf[3];
    cpu_temp_tjmax = val;
    get_tjmax = 1;
  }else if ( ( sensor_num == SENSOR_NUM_TEMP_DIMM_A ) || ( sensor_num == SENSOR_NUM_TEMP_DIMM_C ) ||
              ( sensor_num == SENSOR_NUM_TEMP_DIMM_D ) ||( sensor_num == SENSOR_NUM_TEMP_DIMM_E ) ||
              ( sensor_num == SENSOR_NUM_TEMP_DIMM_G ) ||( sensor_num == SENSOR_NUM_TEMP_DIMM_H ) ) {
    val = rdpkgcfg.rx_buffer.buf[1];
  }else {
    printf("Unrecognized sensor reading result\n");
    free(rdpkgcfg.tx_buffer.buf);
    free(rdpkgcfg.rx_buffer.buf);
    return false;
  }
  free(rdpkgcfg.tx_buffer.buf);
  free(rdpkgcfg.rx_buffer.buf);
  *reading = cal_MBR(sensor_num,val) & 0xff;
  sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache = *reading;
  sensor_config[SnrNum_SnrCfg_map[sensor_num]].cache_status = SNR_READ_SUCCESS;
  return true;
}
