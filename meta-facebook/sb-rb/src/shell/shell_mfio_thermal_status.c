#include <stdlib.h>
#include <shell/shell.h>

#include "plat_log.h"
#include "plat_cpld.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_mfio_thermal_status_shell);

typedef struct {
	const char *name;
	uint8_t reg;
	uint8_t bit;
} asic_mifo_for_rainbow_map_t;

static const asic_mifo_for_rainbow_map_t mfio_for_rainbow_list[] = {
	{ "MEDHA1_MFIO31", MFIO_FOR_RAINBOW, MEDHA1_MFIO31 },
	{ "MEDHA1_MFIO24", MFIO_FOR_RAINBOW, MEDHA1_MFIO24 },
	{ "MEDHA0_MFIO31", MFIO_FOR_RAINBOW, MEDHA0_MFIO31 },
	{ "MEDHA0_MFIO24", MFIO_FOR_RAINBOW, MEDHA0_MFIO24 },
	{ "HAMSA_MFIO23", MFIO_FOR_RAINBOW, HAMSA_MFIO23 },
	{ "HAMSA_MFIO22", MFIO_FOR_RAINBOW, HAMSA_MFIO22 },
};

void mfio_thermal_status_get_cmds(const struct shell *shell, size_t argc, char **argv)
{
	if (get_asic_board_id() != ASIC_BOARD_ID_RAINBOW) {
		shell_warn(shell, "Rainbow support only!");
		return;
	}

	shell_print(shell, "%-20s|%-15s|%-25s", "MFIO for rainbow", "status");

	uint8_t output_value = 0;

	for (int i = 0; i < ARRAY_SIZE(mfio_for_rainbow_list); i++) {
		if (!plat_read_cpld(mfio_for_rainbow_list[i].reg, &output_value, 1)) {
			shell_error(shell, "read MFIO for rainbow from CPLD failed");
			return;
		}
		shell_print(shell, "%-20s|%-10d", mfio_for_rainbow_list[i].name,
			    (output_value >> mfio_for_rainbow_list[i].bit) & 1);
	}

	return;
}

SHELL_STATIC_SUBCMD_SET_CREATE(mfio_thermal_status_sub_cmds,
			       SHELL_CMD(get, NULL, "mfio_thermal_status get",
					 mfio_thermal_status_get_cmds),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(mfio_thermal_status, &mfio_thermal_status_sub_cmds,
		   "mfio_thermal_status commands", NULL);