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

#ifndef PLAT_VOLTAGE_PEAK_SHELL_H
#define PLAT_VOLTAGE_PEAK_SHELL_H

#include <shell/shell.h>

void cmd_get_voltage_peak(const struct shell *shell, size_t argc, char **argv);
void cmd_clear_voltage_peak(const struct shell *shell, size_t argc, char **argv);

#endif
