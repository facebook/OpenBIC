#ifndef PLAT_SYS_H
#define PLAT_SYS_H
#include <stdbool.h>

void check_Infineon_VR_VCCIO_UV_fault(uint8_t sensor_num);
void check_Renesas_VR_VCCIO_UV_fault(uint8_t sensor_num);
bool VCCIO_VR_UV_fault_add_sel();

#endif
