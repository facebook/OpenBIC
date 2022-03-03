#include "plat_spi.h"

#include "util_spi.h"

int pal_get_bios_flash_pos()
{
	return devspi_spi1_cs0;
}
