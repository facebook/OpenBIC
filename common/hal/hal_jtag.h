#ifndef HAL_JTAG_H
#define HAL_JTAG_H

#include <drivers/jtag.h>

void jtag_set_tap(uint8_t data, uint8_t bitlength);
void jtag_shift_data(uint16_t Wbit, uint8_t *Wdate, uint16_t Rbit, uint8_t *Rdate, uint8_t lastidx);

#endif
