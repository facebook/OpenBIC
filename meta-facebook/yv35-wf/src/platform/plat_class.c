#include <stdio.h>
#include "hal_gpio.h"
#include "plat_class.h"

static uint8_t system_board_id = 0;


uint8_t get_board_id()
{
	return system_board_id;
}
