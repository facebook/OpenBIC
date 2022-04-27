#include "hal_gpio.h"
#include "plat_gpio.h"
#include "util_spi.h"

int pal_get_bios_flash_position()
{
	return DEVSPI_SPI1_CS0;
}