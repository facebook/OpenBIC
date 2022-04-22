#include "plat_altera.h"
#include "altera.h"

altera_max10_attr plat_altera_max10_config = { CPLD_UPDATE_I2C_BUS, CPLD_UPDATE_ADDR,
					       M04_CFM1_START_ADDR, M04_CFM1_END_ADDR };

int pal_load_altera_max10_attr(altera_max10_attr *altera_max10_config)
{
	if (altera_max10_config == NULL) {
		return -1;
	}
	*altera_max10_config = plat_altera_max10_config;
	return 0;
};
