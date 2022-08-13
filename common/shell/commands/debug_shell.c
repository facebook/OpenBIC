#include "debug_shell.h"
#include <shell/shell.h>
#include "shell_platform.h"
#include <stdio.h>
#include "bic_logging.h"
#include "logging/log_ctrl.h"

/**
 * @brief Function for finding source ID based on source name.
 * 
 * Note: This function isn't available until a later version of Zephyr, so
 * it's copied here for use.
 *
 * @param name Source name
 *
 * @return Source ID.
 */
static int16_t log_source_id_get(const char *name)
{
	for (int16_t i = 0; i < log_src_cnt_get(CONFIG_LOG_DOMAIN_ID); i++) {
		if (strcmp(log_source_name_get(CONFIG_LOG_DOMAIN_ID, i), name) == 0) {
			return i;
		}
	}

	return -1;
}

static int16_t decode_severity_level(const char *level)
{
	if (strcmp(level, "ERROR") == 0) {
		return LOG_LEVEL_ERR;
	} else if (strcmp(level, "WARNING") == 0) {
		return LOG_LEVEL_WRN;
	} else if (strcmp(level, "INFO") == 0) {
		return LOG_LEVEL_INF;
	} else if (strcmp(level, "DEBUG") == 0) {
		return LOG_LEVEL_DBG;
	} else if (strcmp(level, "OFF") == 0) {
		return 0;
	} else {
		return -1;
	}
}

static void print_severity_help(const struct shell *shell, const char *severity_level)
{
	shell_warn(shell, "Unknown severity level: %s", severity_level);
	shell_warn(shell, "Valid severity levels:");
	for (int curr_level = 0; curr_level < SEVERITY_COUNT; ++curr_level) {
		shell_warn(shell, " - %s", SEVERITY_STRING[curr_level]);
	}
}

void cmd_debug_list(const struct shell *shell, size_t argc, char **argv)
{
	for (int curr = 0; curr < LOG_MODULE_COUNT; ++curr) {
		int16_t source_id = log_source_id_get(LOG_MODULE_STRING[curr]);
		uint32_t severity = log_filter_get(shell->log_backend->backend, 0, source_id, true);
		shell_print(shell, "%s - %d", LOG_MODULE_STRING[curr], severity);
	}
}
void cmd_debug_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "help: platform debug set <module_name> <severity_level>");
		return;
	}

	int16_t source_id = log_source_id_get(argv[1]);
	if (source_id == -1) {
		shell_warn(shell, "Module \"%s\" not found...", argv[1]);
		shell_warn(shell, "\"platform debug list\" to list all modules");
		return;
	}

	int16_t severity_level = decode_severity_level(argv[2]);
	if (severity_level == -1) {
		print_severity_help(shell, argv[2]);
		return;
	}

	int16_t set_level = log_filter_set(NULL, 0, source_id, severity_level);
	if (set_level != severity_level) {
		shell_warn(shell, "Attempted to set Severity to: %d", severity_level);
		shell_warn(shell, "However log set to %d", set_level);
	}
}

void cmd_debug_set_all(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "help: platform debug set_all <severity_level>");
		return;
	}

	int16_t severity_level = decode_severity_level(argv[2]);
	if (severity_level == -1) {
		print_severity_help(shell, argv[2]);
		return;
	}

	for (int curr = 0; curr < LOG_MODULE_COUNT; ++curr) {
		int16_t source_id = log_source_id_get(LOG_MODULE_STRING[curr]);
		int16_t set_level = log_filter_set(NULL, 0, source_id, severity_level);
		if (set_level != severity_level) {
			shell_warn(shell, "Attempted to set Severity to: %d", severity_level);
			shell_warn(shell, "However log set to %d", set_level);
		}
	}
}

void cmd_debug_disable(const struct shell *shell, size_t argc, char **argv)
{
	for (int curr = 0; curr < LOG_MODULE_COUNT; ++curr) {
		int16_t source_id = log_source_id_get(LOG_MODULE_STRING[curr]);
		int16_t set_level = log_filter_set(NULL, 0, source_id, 0);
		if (set_level != 0) {
			shell_warn(shell, "Attempted to set log off.");
			shell_warn(shell, "However log set to %d", set_level);
		}
	}
}
