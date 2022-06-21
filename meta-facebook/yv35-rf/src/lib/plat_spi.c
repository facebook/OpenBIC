#include "hal_gpio.h"
#include "util_spi.h"
#include "plat_gpio.h"

bool pal_switch_cxl_spi_mux()
{
	// Switch CXL MUX selection pin to BIC
	int ret = gpio_set(SPI_MASTER_SEL, GPIO_HIGH);
	if (ret != 0) {
		return false;
	}

	// Enable CXL MUX
	ret = gpio_set(FM_SPI_MUX_OE_CTL_N, GPIO_LOW);
	if (ret != 0) {
		return false;
	}
	return true;
}

int pal_get_cxl_flash_position()
{
	return DEVSPI_SPI1_CS0;
}
