#include <stdio.h>
#include <drivers/peci.h>
#include "hal_peci.h"


const struct device *dev;

int peci_init() {
  dev = device_get_binding("PECI");
  int ret;
  uint32_t bitrate = 1000;
  if (!dev) {
    printf("peci device not found");
    return false;
  }
  ret = peci_config(dev, bitrate);
  if (ret) {
    printf("set bitrate %dKbps failed %d\n", bitrate, ret);
    return ret;
  }
  ret = peci_enable(dev);
  if (ret) {
    printf("peci enable failed %d\n", ret);
    return ret;
  }

  return true;
}

int peci_ping(uint8_t address) {
  struct peci_msg pkgcfg;
  int ret;

  pkgcfg.addr = address;
  pkgcfg.cmd_code = PECI_CMD_PING;
  pkgcfg.tx_buffer.buf = NULL;
  pkgcfg.tx_buffer.len = 0x0;
  pkgcfg.rx_buffer.len = 0x0;
  
  ret = peci_transfer(dev, &pkgcfg);
  if (ret) {
    printf("ping failed %d\n", ret);
    free(pkgcfg.tx_buffer.buf);
    return ret;
  }
  
  free(pkgcfg.tx_buffer.buf);
  return ret;
}

int peci_read(uint8_t cmd, uint8_t address, uint8_t u8Index, uint16_t u16Param, uint8_t u8ReadLen, uint8_t *readBuf) {
  struct peci_msg rdpkgcfg;
  int ret;
  rdpkgcfg.cmd_code = cmd;
  rdpkgcfg.addr = address;
  rdpkgcfg.tx_buffer.len = 0x05;
  rdpkgcfg.rx_buffer.len = u8ReadLen;
  rdpkgcfg.tx_buffer.buf = malloc(rdpkgcfg.tx_buffer.len * sizeof(uint8_t));
  rdpkgcfg.rx_buffer.buf = readBuf;
  rdpkgcfg.tx_buffer.buf[0] = 0x00;
  rdpkgcfg.tx_buffer.buf[1] = u8Index;
  rdpkgcfg.tx_buffer.buf[2] = u16Param & 0xff;
  rdpkgcfg.tx_buffer.buf[3] = u16Param >> 8;  
  ret = peci_transfer(dev, &rdpkgcfg);
  
  if (DEBUG_PECI) {
    uint8_t index;
    for (index = 0; index < 5; index++)
      printf("%02x ", readBuf[index]);
    printf("\n");
  }

  if (ret) {
    printf("peci read failed %d\n", ret);
    free(rdpkgcfg.tx_buffer.buf);
    return ret;
  }

  free(rdpkgcfg.tx_buffer.buf);
  return ret;  
}

int peci_write(uint8_t cmd, uint8_t address, uint8_t u8ReadLen, uint8_t *readBuf, uint8_t u8WriteLen, uint8_t *writeBuf) {
  struct peci_msg wrpkgcfg;
  int ret;

  wrpkgcfg.addr = address;
  wrpkgcfg.cmd_code = cmd;
  wrpkgcfg.tx_buffer.len = u8WriteLen;
  wrpkgcfg.rx_buffer.len = u8ReadLen;
  wrpkgcfg.rx_buffer.buf = readBuf;
  wrpkgcfg.tx_buffer.buf = writeBuf;
  
  ret = peci_transfer(dev, &wrpkgcfg);
  if (ret) {
    printk("peci write failed %d\n", ret);
    return ret;
  }
  
  return ret;
}

bool peci_retry_read(uint8_t cmd, uint8_t address, uint8_t u8Index, uint16_t u16Param, uint8_t u8ReadLen, uint8_t *readBuf) {
  uint8_t i, ret, retry = 5;
  
  if(readBuf == NULL)
    return false;

  for(i = 0; i < retry; ++i) {
    k_msleep(10);
    memcpy(&readBuf[0], 0, u8ReadLen * sizeof(uint8_t));
    ret = peci_read(cmd, address, u8Index, u16Param, u8ReadLen, readBuf);
    if (!ret) {
      if (readBuf[0] == PECI_CC_RSP_SUCCESS) {
        return true;
      }
    }
  }
  return false;
}

bool peci_getPwr(uint8_t sensor_num, int *reading) {
  uint8_t rdpkgcfgCmd = PECI_RD_PKG_CFG0_CMD;
  uint8_t PECIaddr = 0x30;
  uint8_t readlen = 0x05;
  uint8_t *readbuf = malloc( 2 * readlen * sizeof(uint8_t) );
  uint8_t u8index[2] = { 0x03 , 0x1F };
  uint16_t u16Param[2] = { 0x00FF , 0x0000 };
  uint8_t ret, complete_code;
  bool is_retry_success = false;
  uint32_t pkg_energy, run_time, diff_energy, diff_time;
  static uint32_t last_pkg_energy = 0, last_run_time = 0;

  peci_read( rdpkgcfgCmd , PECIaddr , u8index[0] , u16Param[0] , readlen , readbuf );
  complete_code = readbuf[0];
  if ( complete_code == PECI_CC_RSP_TIMEOUT || complete_code == PECI_CC_OUT_OF_RESOURCES_TIMEOUT ) {
    is_retry_success = peci_retry_read( rdpkgcfgCmd , PECIaddr , u8index[0] , u16Param[0] , readlen , readbuf );
    if ( !is_retry_success ) {
      printf("PECI sensor [%x] response timeout. Reach Max retry.\n", sensor_num );
      free(readbuf);
      return false;
    }
  }
  ret = peci_read( rdpkgcfgCmd , PECIaddr , u8index[1] , u16Param[1] , readlen , &readbuf[5] );
  complete_code = readbuf[5];
  if ( complete_code == PECI_CC_RSP_TIMEOUT || complete_code == PECI_CC_OUT_OF_RESOURCES_TIMEOUT ) {
    is_retry_success = peci_retry_read( rdpkgcfgCmd , PECIaddr , u8index[1] , u16Param[1] , readlen , &readbuf[5] );
    if ( !is_retry_success ) {
      printf("PECI sensor [%x] response timeout. Reach Max retry.\n", sensor_num );
      free(readbuf);
      return false;
    }
  }
  if ( ret ) {
    free( readbuf );
    return false;
  }
  if ( readbuf[0] != PECI_CC_RSP_SUCCESS || readbuf[5] != PECI_CC_RSP_SUCCESS ) {
    if ( readbuf[0] == PECI_CC_ILLEGAL_REQUEST || readbuf[5] == PECI_CC_ILLEGAL_REQUEST ) {
      printf("Unknown request\n");
    } else {
      printf("PECI control hardware, firmware or associated logic error\n");
    }
    free( readbuf );
    return false;
  }

  pkg_energy = readbuf[4];
  pkg_energy = ( pkg_energy << 8 ) | readbuf[3];
  pkg_energy = ( pkg_energy << 8 ) | readbuf[2];
  pkg_energy = ( pkg_energy << 8 ) | readbuf[1];

  run_time = readbuf[9];
  run_time = ( run_time << 8 ) | readbuf[8];
  run_time = ( run_time << 8 ) | readbuf[7];
  run_time = ( run_time << 8 ) | readbuf[6];

  if ( last_pkg_energy == 0 && last_run_time == 0 ) { // first read, need second data to calculate
    last_pkg_energy = pkg_energy;
    last_run_time = run_time;
    free( readbuf );
    return false;
  }

  if( pkg_energy >= last_pkg_energy ) {
    diff_energy = pkg_energy - last_pkg_energy;
  } else {
    diff_energy = pkg_energy + ( 0xffffffff - last_pkg_energy + 1 );
  }
  last_pkg_energy = pkg_energy;

  if( run_time >= last_run_time ) {
    diff_time = run_time - last_run_time;
  } else {
    diff_time = run_time + ( 0xffffffff - last_run_time + 1 );
  }
  last_run_time = run_time;

  if( diff_time == 0 ) {
    free(readbuf);
    return false;
  } else {
    free(readbuf);
    *reading = ((float)diff_energy / (float)diff_time * 0.06103515625); // energy / unit time
    return true;
  }
}
