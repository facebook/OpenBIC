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

#include <shell/shell.h>
#include <stdlib.h>
#include <stdio.h>

void cmd_echo(struct shell *shell, size_t argc, char **argv)
{
	for (size_t i = 1; i < argc; i++) {
		printf("%s ", argv[i]);
	}
	printf("\n");
}

SHELL_CMD_REGISTER(echo, NULL, "echo command", cmd_echo);
