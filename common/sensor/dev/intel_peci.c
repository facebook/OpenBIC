#include <stdio.h>
#include <stdlib.h>
#include "sensor.h"
#include "pal.h"
#include <drivers/peci.h>
#include "hal_peci.h"
#include <string.h>
#include "intel_peci.h"

#define READ_RTY 0x03

#define RDPKG_IDX_PKG_TEMP 0x02
#define RDPKG_IDX_DIMM_TEMP 0x0E
#define RDPKG_IDX_TJMAX_TEMP 0x10

static bool peci_rty_read(uint8_t cmd, uint8_t addr, uint8_t idx, uint16_t param, uint8_t rlen, uint8_t *rbuf)
{
  if (!rbuf)
    return false;

  int i;
  for (i = 0; i < READ_RTY; i++) {
    if (!peci_read(cmd, addr, idx, param, rlen, rbuf)) {
      const uint8_t cc = rbuf[0];

      if (cc == PECI_CC_RSP_SUCCESS)
        break;
    }
    
    /* TODO: read failed, maybe add delay before next read */
  }

  return (i == READ_RTY) ? false : true;
}

static bool get_cpu_tjmax(uint8_t addr, int *reading)
{
  if (!reading)
    return false;

  const uint16_t param = 0x00;
  const uint8_t rlen = 0x05;
  uint8_t rbuf[rlen];
  memset(rbuf, 0, sizeof(rbuf));

  if (peci_rty_read(PECI_CMD_RD_PKG_CFG0, addr, RDPKG_IDX_TJMAX_TEMP, param, rlen, rbuf) == false)
    return false;

  sen_val *sval = (sen_val *)reading;
  sval->integer = rbuf[3];
  return true;
}

static bool get_cpu_margin(uint8_t addr, int *reading)
{
  if (!reading)
    return false;

  const uint16_t param = 0xFF;
  const uint8_t rlen = 0x05;
  uint8_t rbuf[rlen];
  memset(rbuf, 0, sizeof(rbuf));

  if (peci_rty_read(PECI_CMD_RD_PKG_CFG0, addr, RDPKG_IDX_PKG_TEMP, param, rlen, rbuf) == false)
    return false;

  sen_val *sval = (sen_val *)reading;
  sval->integer = ((int16_t)((rbuf[2] << 8) | rbuf[1]) >> 6) + 1;
  return true;
}

static bool get_cpu_pwr(uint8_t sen_num, int *reading)
{
  if (!reading)
    return false;

  int pwr = 0;
  if (peci_getPwr(sen_num, &pwr) == false)
    return false;

  sen_val *sval = (sen_val *)reading;
  sval->integer = (int16_t)pwr;
  return true;
}

static bool get_cpu_temp(uint8_t addr, int *reading)
{
  if (!reading)
    return false;

  sen_val tjmax = {0};
  if (get_cpu_tjmax(addr, (int *)&tjmax) == false)
    return false;

  sen_val margin = {0};
  if (get_cpu_margin(addr, (int *)&margin) == false)
    return false;

  sen_val *sval = (sen_val *)reading;
  sval->integer = tjmax.integer + margin.integer;
  return true;
}

static bool get_dimm_temp(uint8_t addr, uint8_t type, int *reading)
{
  if (!reading)
    return false;

  const uint16_t param = (type == PECI_TEMP_DIMM_A) ? 0x00 :
                          (type == PECI_TEMP_DIMM_C) ? 0x02 :
                          (type == PECI_TEMP_DIMM_D) ? 0x03 :
                          (type == PECI_TEMP_DIMM_E) ? 0x04 :
                          (type == PECI_TEMP_DIMM_G) ? 0x06 :
                          (type == PECI_TEMP_DIMM_H) ? 0x07 :
                          0xFF;
  
  if (param == 0xFF)
    return false;

  const uint8_t rlen = 0x05;
  uint8_t rbuf[rlen];
  memset(rbuf, 0, sizeof(rbuf));

  if (peci_rty_read(PECI_CMD_RD_PKG_CFG0, addr, RDPKG_IDX_DIMM_TEMP, param, rlen, rbuf) == false)
    return false;

  sen_val *sval = (sen_val *)reading;
  sval->integer = rbuf[1];
  return true;
}

uint8_t intel_peci_read(uint8_t sensor_num, int *reading)
{
  if (!reading)
    return SNR_UNSPECIFIED_ERROR;

  bool ret_val = false;
  snr_cfg *cfg = &sensor_config[SnrNum_SnrCfg_map[sensor_num]];
  const uint8_t read_type = cfg->offset;
  if (read_type <= PECI_UNKNOWN || read_type >= PECI_MAX)
    return SNR_NOT_FOUND;

  switch (read_type) {
  case PECI_TEMP_DIMM_A:
  case PECI_TEMP_DIMM_C:
  case PECI_TEMP_DIMM_D:
  case PECI_TEMP_DIMM_E:
  case PECI_TEMP_DIMM_G:
  case PECI_TEMP_DIMM_H:
    ret_val = get_dimm_temp(cfg->slave_addr, read_type, reading);
    break;
  case PECI_TEMP_CPU_MARGIN:
    ret_val = get_cpu_margin(cfg->slave_addr, reading);
    break;
  case PECI_TEMP_CPU_TJMAX:
    ret_val = get_cpu_tjmax(cfg->slave_addr, reading);
    break;
  case PECI_TEMP_CPU:
    ret_val = get_cpu_temp(cfg->slave_addr, reading);
    break;
  case PECI_PWR_CPU:
    ret_val = get_cpu_pwr(sensor_num, reading);
    break;
  default:
    break;
  }

  return ret_val ? SNR_READ_SUCCESS : SNR_FAIL_TO_ACCESS;
}

uint8_t intel_peci_init(uint8_t sensor_num)
{
  static bool is_init = false;
  sensor_config[SnrNum_SnrCfg_map[sensor_num]].read = intel_peci_read;
  if (!is_init) {
    peci_init();
    is_init = true;
  }
  return true;
}
