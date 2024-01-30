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

#include <stdio.h>
#include <string.h>
#include <logging/log.h>
#include "sensor.h"
LOG_MODULE_REGISTER(plat_hook);

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
nct7363_init_arg nct7363_init_args[] = {
    //GPIO setting: Reserved=11, FANINx=10, PWMx=01, GPIOXX=00
    //Fan BD
    [0] = { 
        .is_init = false, 
            .init_pin_config = {
                .GPIO_14_to_17_Pin_Function_Configuration = 0b00001001,
                .GPIO0x_Input_Output_Configuration = 0b11111110,
                .GPIO1x_Input_Output_Configuration = 0b11111100,
                .PWM_8_to_15_Enable = 0b10000000,
                .FANIN_0_to_7_Monitoring_Enable = 0b00100000,
            },
        .fan_poles = 0,
    },	
    // Management BD
    [1] = { 
        .is_init = false,
            .init_pin_config = {
                .GPIO_00_to_03_Pin_Function_Configuration = 0b00101001,
                .GPIO0x_Input_Output_Configuration = 0b11101111,
                .GPIO1x_Input_Output_Configuration = 0b11111111,
                .PWM_0_to_7_Enable = 0b00000001,
                .FANIN_8_to_15_Monitoring_Enable = 0b00000110,
            },
        .fan_poles = 0,
    },
    //Backplane BD
    [2] = { 
        .is_init = false,
            .init_pin_config = {
                .GPIO_00_to_03_Pin_Function_Configuration = 0b10010101,
                .GPIO_04_to_07_Pin_Function_Configuration = 0b00101010,
                .GPIO0x_Input_Output_Configuration = 0b11111111,
                .GPIO1x_Input_Output_Configuration = 0b00000000,
                .PWM_0_to_7_Enable = 0b00000111,
                .FANIN_8_to_15_Monitoring_Enable = 0b01111000,
            },
        .fan_poles = 0,
    },
    //Pump BD
    [3] = { 
        .is_init = false,
            .init_pin_config = {
                .GPIO_00_to_03_Pin_Function_Configuration = 0b00000101,
                .GPIO_04_to_07_Pin_Function_Configuration = 0b00101010,
                .GPIO0x_Input_Output_Configuration = 0b11111111,
                .GPIO1x_Input_Output_Configuration = 0b11111100,
                .PWM_0_to_7_Enable = 0b00000011,
                .FANIN_8_to_15_Monitoring_Enable = 0b01110000,
            },
        .fan_poles = 0,
    },
};
/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/

