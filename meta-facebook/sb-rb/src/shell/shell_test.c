#include <stdlib.h>
#include <shell/shell.h>

#include "plat_pldm_sensor.h"
#include "plat_cpld.h"

// test command
void cmd_test(const struct shell *shell, size_t argc, char **argv)
{
	// test code
	shell_warn(shell, "Hello!");
}

void cmd_read_raw(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t sensor_id = strtoul(argv[1], NULL, 16);
	uint8_t offset = strtoul(argv[2], NULL, 16);
	uint8_t len = strtoul(argv[3], NULL, 10);

	if (!len)
		len = 1;
	uint8_t data[len];
	memset(data, 0, len);

	if ((sensor_id == 0) || (sensor_id >= SENSOR_NUM_NUMBERS)) {
		if (!plat_read_cpld(offset, data, 1)) {
			shell_warn(shell, "cpld read 0x%02x fail", offset);
			return;
		}
	} else {
		if (!get_raw_data_from_sensor_id(sensor_id, offset, data, len)) {
			shell_warn(shell, "sensor_id 0x%02x read 0x%02x fail", sensor_id, offset);
			return;
		}
	}

	shell_hexdump(shell, data, len);
	shell_print(shell, "");
}

void cmd_cpld_dump(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: test cpld dump <offset> <length>");
		return;
	}

	uint8_t offset = strtoul(argv[1], NULL, 16);
	uint8_t len = strtoul(argv[2], NULL, 10);

	if (!len)
		len = 1;
	uint8_t data[len];
	memset(data, 0, len);

	if (!plat_read_cpld(offset, data, len)) {
		shell_warn(shell, "cpld read 0x%02x fail", offset);
		return;
	}

	shell_hexdump(shell, data, len);
	shell_print(shell, "");
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cpld_cmds, SHELL_CMD(dump, NULL, "cpld dump", cmd_cpld_dump),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_test_cmds, SHELL_CMD(test, NULL, "test command", cmd_test),
			       SHELL_CMD(read_raw, NULL, "read raw data test command",
					 cmd_read_raw),
			       SHELL_CMD(cpld, &sub_cpld_cmds, "cpld commands", NULL),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(test, &sub_test_cmds, "Test commands", NULL);