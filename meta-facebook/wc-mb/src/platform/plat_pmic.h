#ifndef PLAT_PMIC_H
#define PLAT_PMIC_H

#include <stdint.h>

#define MONITOR_PMIC_ERROR_STACK_SIZE 4096
#define MONITOR_PMIC_ERROR_TIME_MS (30 * 1000) // 30s

#define MAX_LEN_GET_PMIC_ERROR_INFO 6 //include R05,  R06,  R08,  R09,  R0A,  R0B

#define MAX_COUNT_DIMM 4
#define MAX_COUNT_PMIC_ERROR_TYPE 17

void start_monitor_pmic_error_thread();
void monitor_pmic_error_handler();
int compare_pmic_error(uint8_t dimm_id, uint8_t *pmic_err_data);
int get_dimm_info(uint8_t dimm_id, uint8_t *bus, uint8_t *addr);
void add_pmic_error_sel(uint8_t dimm_id, uint8_t error_type);

#endif
