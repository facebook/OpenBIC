/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LOG_SHELL_H
#define LOG_SHELL_H

#include <shell/shell.h>

void cmd_set_event(const struct shell *shell, size_t argc, char **argv);
void cmd_log_dump(const struct shell *shell, size_t argc, char **argv);
void cmd_test_read(const struct shell *shell, size_t argc, char **argv);
void cmd_log_clear(const struct shell *shell, size_t argc, char **argv);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_plat_log_cmd,
			       SHELL_CMD(set_event, NULL, "set_event", cmd_set_event),
			       SHELL_CMD(dump, NULL, "log_dump", cmd_log_dump),
			       SHELL_CMD(clear, NULL, "log_clear", cmd_log_clear),
			       SHELL_CMD(test_read, NULL, "test_read", cmd_test_read),
			       SHELL_SUBCMD_SET_END);

#endif
