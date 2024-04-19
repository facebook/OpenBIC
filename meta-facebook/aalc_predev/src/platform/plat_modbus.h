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

#include <stdbool.h>
#include <stdint.h>
#include <kernel.h>

#define MODBUS_UART_NODE_ADDR 0x0C
#define MODBUS_UART_FP_OFFSET 5000

/*MODBUS Serial Line Parameter*/
#define MODBUS_UART_BAUDRATE_LOW 19200
#define MODBUS_UART_BAUDRATE_HIGH 115200
#define MODBUS_UART_PARITY UART_CFG_PARITY_NONE
#define MODBUS_UART_RESPONSE_T 1000000
//from zephyr/samples/subsys/modbus/rtu_client, client_param: rx_timeout = 1000000(default 50000)

int init_custom_modbus_server(void);
typedef struct _modbus_command_mapping {
	uint16_t addr;
	uint8_t (*wr_fn)(struct _modbus_command_mapping *);
	uint8_t (*rd_fn)(struct _modbus_command_mapping *);
	uint8_t arg0;
	uint8_t arg1;
	uint8_t arg2;
	uint8_t size;
	uint16_t *data;
} modbus_command_mapping;

enum pump_state {
	PUMP_REDUNDENT_SWITCHED,
	MANUAL_CONTROL_PUMP,
	MANUAL_CONTROL_FAN,
	AUTOTUNE_FLOW_CONTROL,
	AUTOTUNE_PRESSURE_BALANCE_CONTROL,
	SYSTEM_STOP,
	RPU_REMOTE_POWER_CYCLE,
	SET_PUMP_GROUP_1,
	SET_PUMP_GROUP_2,
	MANUAL_CONTROL,
	CLEAR_PUMP_RUNNING_TIME,
	CLEAR_LOG,
	PUMP_1_RESET,
	PUMP_2_RESET,
	PUMP_3_RESET,
	PUMP_4_RESET,
};

/* define modbus data address */
#define MODBUS_TEMP_BB_TMP75_ADDR 0x0101
#define MODBUS_TEMP_BPB_TMP75_ADDR 0x0102
#define MODBUS_POWER_RPU_ADDR 0x9999
/* define sensor modbus data address*/
#define MODBUS_BPB_RPU_COOLANT_FLOW_RATE_LPM_ADDR 0x9000
#define MODBUS_BPB_RPU_COOLANT_OUTLET_TEMP_ADDR 0x9001
#define MODBUS_BPB_RPU_COOLANT_INLET_TEMP_ADDR 0x9002
#define MODBUS_BPB_RPU_COOLANT_OUTLET_P_KPA_ADDR 0x9003
#define MODBUS_BPB_RPU_COOLANT_INLET_P_KPA_ADDR 0x9004
#define MODBUS_RPU_PWR_W_ADDR 0x9005
#define MODBUS_AALC_TOTAL_PWR_W_ADDR 0x9006
#define MODBUS_RPU_INPUT_VOLT_V_ADDR 0x9007
#define MODBUS_MB_RPU_AIR_INLET_TEMP_ADDR 0x9008
#define MODBUS_RPU_PUMP_PWM_TACH_PCT_ADDR 0x9009
#define MODBUS_PB_1_PUMP_TACH_RPM_ADDR 0x900A
#define MODBUS_PB_2_PUMP_TACH_RPM_ADDR 0x900B
#define MODBUS_PB_3_PUMP_TACH_RPM_ADDR 0x900C
#define MODBUS_RPU_FAN1_STATUS_ADDR 0x900E
#define MODBUS_RPU_FAN2_STATUS_ADDR 0x900E
#define MODBUS_MB_FAN1_TACH_RPM_ADDR 0x900F
#define MODBUS_MB_FAN2_TACH_RPM_ADDR 0x9010
#define MODBUS_AALC_COOLING_CAPACITY_W_ADDR 0x9011
#define MODBUS_RPU_PUMP1_STATUS_ADDR 0x9012
#define MODBUS_RPU_PUMP2_STATUS_ADDR 0x9013
#define MODBUS_RPU_PUMP3_STATUS_ADDR 0x9014
#define MODBUS_RPU_RESERVOIR_STATUS_ADDR 0x9016
#define MODBUS_RPU_LED_RESERVOIR_STATUS_ADDR 0x9017
#define MODBUS_RPU_LED_LEAKAGE_STATUS_ADDR 0x9017
#define MODBUS_RPU_LED_FAULT_STATUS_ADDR 0x9017
#define MODBUS_RPU_LED_POWER_STATUS_ADDR 0x9017
#define MODBUS_BB_TMP75_TEMP_ADDR 0x9018
#define MODBUS_BPB_RPU_OUTLET_TEMP_ADDR 0x9019
#define MODBUS_PDB_HDC1080DMBR_TEMP_ADDR 0x901A
#define MODBUS_BB_HSC_P48V_TEMP_ADDR 0x901B
#define MODBUS_BPB_HSC_P48V_TEMP_ADDR 0x901C
#define MODBUS_PB_1_HDC1080DMBR_TEMP_ADDR 0x901D
#define MODBUS_PB_2_HDC1080DMBR_TEMP_ADDR 0x901E
#define MODBUS_PB_3_HDC1080DMBR_TEMP_ADDR 0x901F
#define MODBUS_PB_1_HSC_P48V_TEMP_ADDR 0x9020
#define MODBUS_PB_2_HSC_P48V_TEMP_ADDR 0x9021
#define MODBUS_PB_3_HSC_P48V_TEMP_ADDR 0x9022
#define MODBUS_PB_1_HSC_P48V_VIN_VOLT_V_ADDR 0x9023
#define MODBUS_PB_2_HSC_P48V_VIN_VOLT_V_ADDR 0x9024
#define MODBUS_PB_3_HSC_P48V_VIN_VOLT_V_ADDR 0x9025
#define MODBUS_BB_HSC_P51V_VIN_VOLT_V_ADDR 0x9026
#define MODBUS_BPB_HSC_P51V_VIN_VOLT_V_ADDR 0x9027
#define MODBUS_BB_HSC_P51V_IOUT_CURR_A_ADDR 0x9028
#define MODBUS_BPB_HSC_P51V_IOUT_CURR_A_ADDR 0x9029
#define MODBUS_PB_1_HSC_P48V_IOUT_CURR_A_ADDR 0x902A
#define MODBUS_PB_2_HSC_P48V_IOUT_CURR_A_ADDR 0x902B
#define MODBUS_PB_3_HSC_P48V_IOUT_CURR_A_ADDR 0x902C
#define MODBUS_BB_HSC_P51V_PIN_PWR_W_ADDR 0x902D
#define MODBUS_BPB_HSC_P51V_PIN_PWR_W_ADDR 0x902E
#define MODBUS_PB_1_HSC_P48V_PIN_PWR_W_ADDR 0x902F
#define MODBUS_PUMP_1_RUNNING_ADDR 0x9030
#define MODBUS_PUMP_2_RUNNING_ADDR 0x9032
#define MODBUS_PUMP_3_RUNNING_ADDR 0x9034
#define MODBUS_PB_2_HSC_P48V_PIN_PWR_W_ADDR 0x9038
#define MODBUS_PB_3_HSC_P48V_PIN_PWR_W_ADDR 0x9039
#define MODBUS_PB_1_FAN_1_TACH_RPM_ADDR 0x903A
#define MODBUS_PB_1_FAN_2_TACH_RPM_ADDR 0x903B
#define MODBUS_PB_2_FAN_1_TACH_RPM_ADDR 0x903C
#define MODBUS_PB_2_FAN_2_TACH_RPM_ADDR 0x903D
#define MODBUS_PB_3_FAN_1_TACH_RPM_ADDR 0x903E
#define MODBUS_PB_3_FAN_2_TACH_RPM_ADDR 0x903F
#define MODBUS_BPB_RACK_PRESSURE_3_P_KPA_ADDR 0x9040
#define MODBUS_BPB_RACK_PRESSURE_4_P_KPA_ADDR 0x9041
#define MODBUS_BPB_RACK_LEVEL_1_ADDR 0x9042
#define MODBUS_BPB_RACK_LEVEL_2_ADDR 0x9043
#define MODBUS_BPB_CDU_LEVEL_3_ADDR 0x9044
#define MODBUS_MB_HUM_PCT_RH_ADDR 0x9045
#define MODBUS_PDB_HUM_PCT_RH_ADDR 0x9046
#define MODBUS_PB_1_HUM_PCT_RH_ADDR 0x9047
#define MODBUS_PB_2_HUM_PCT_RH_ADDR 0x9048
#define MODBUS_PB_3_HUM_PCT_RH_ADDR 0x9049
#define MODBUS_HEX_FAN_PWM_TACH_PCT_ADDR 0x9100
#define MODBUS_HEX_PWR_W_ADDR 0x9101
#define MODBUS_HEX_INPUT_VOLT_V_ADDR 0x9102
#define MODBUS_HEX_INPUT_CURRENT_V_ADDR 0x9103
#define MODBUS_FB_1_FAN_TACH_RPM_ADDR 0x9104
#define MODBUS_FB_2_FAN_TACH_RPM_ADDR 0x9105
#define MODBUS_FB_3_FAN_TACH_RPM_ADDR 0x9106
#define MODBUS_FB_4_FAN_TACH_RPM_ADDR 0x9107
#define MODBUS_FB_5_FAN_TACH_RPM_ADDR 0x9108
#define MODBUS_FB_6_FAN_TACH_RPM_ADDR 0x9109
#define MODBUS_FB_7_FAN_TACH_RPM_ADDR 0x910A
#define MODBUS_FB_8_FAN_TACH_RPM_ADDR 0x910B
#define MODBUS_FB_9_FAN_TACH_RPM_ADDR 0x910C
#define MODBUS_FB_10_FAN_TACH_RPM_ADDR 0x910D
#define MODBUS_SB_HEX_AIR_OUTLET_1_TEMP_ADDR 0x910E
#define MODBUS_SB_HEX_AIR_OUTLET_2_TEMP_ADDR 0x910F
#define MODBUS_FB_1_HEX_INLET_TEMP_ADDR 0x9110
#define MODBUS_FB_2_HEX_INLET_TEMP_ADDR 0x9111
#define MODBUS_HEX_WATER_INLET_TEMP_C_ADDR 0x9112
#define MODBUS_HEX_BLADDER_LEVEL_STATUS_ADDR 0x9113
#define MODBUS_SB_HEX_AIR_OUTLET_3_TEMP_ADDR 0x9114
#define MODBUS_SB_HEX_AIR_OUTLET_4_TEMP_ADDR 0x9115
#define MODBUS_FB_3_HEX_INLET_TEMP_ADDR 0x9116
#define MODBUS_FB_4_HEX_INLET_TEMP_ADDR 0x9117
#define MODBUS_FB_5_HEX_INLET_TEMP_ADDR 0x9118
#define MODBUS_FB_6_HEX_INLET_TEMP_ADDR 0x9119
#define MODBUS_FB_7_HEX_INLET_TEMP_ADDR 0x911A
#define MODBUS_FB_8_HEX_INLET_TEMP_ADDR 0x911B
#define MODBUS_FB_9_HEX_INLET_TEMP_ADDR 0x911C
#define MODBUS_FB_10_HEX_INLET_TEMP_ADDR 0x911D
#define MODBUS_FB_11_HEX_INLET_TEMP_ADDR 0x911E
#define MODBUS_FB_12_HEX_INLET_TEMP_ADDR 0x911F
#define MODBUS_FB_13_HEX_INLET_TEMP_ADDR 0x9120
#define MODBUS_FB_14_HEX_INLET_TEMP_ADDR 0x9121
#define MODBUS_FB_1_HSC_TEMP_ADDR 0x9122
#define MODBUS_FB_2_HSC_TEMP_ADDR 0x9123
#define MODBUS_FB_3_HSC_TEMP_ADDR 0x9124
#define MODBUS_FB_4_HSC_TEMP_ADDR 0x9125
#define MODBUS_FB_5_HSC_TEMP_ADDR 0x9126
#define MODBUS_FB_6_HSC_TEMP_ADDR 0x9127
#define MODBUS_FB_7_HSC_TEMP_ADDR 0x9128
#define MODBUS_FB_8_HSC_TEMP_ADDR 0x9129
#define MODBUS_FB_9_HSC_TEMP_ADDR 0x912A
#define MODBUS_FB_10_HSC_TEMP_ADDR 0x912B
#define MODBUS_FB_11_HSC_TEMP_ADDR 0x912C
#define MODBUS_FB_12_HSC_TEMP_ADDR 0x912D
#define MODBUS_FB_13_HSC_TEMP_ADDR 0x912E
#define MODBUS_FB_14_HSC_TEMP_ADDR 0x912F
#define MODBUS_FB_1_HSC_P48V_VIN_VOLT_V_ADDR 0x9130
#define MODBUS_FB_2_HSC_P48V_VIN_VOLT_V_ADDR 0x9131
#define MODBUS_FB_3_HSC_P48V_VIN_VOLT_V_ADDR 0x9132
#define MODBUS_FB_4_HSC_P48V_VIN_VOLT_V_ADDR 0x9133
#define MODBUS_FB_5_HSC_P48V_VIN_VOLT_V_ADDR 0x9134
#define MODBUS_FB_6_HSC_P48V_VIN_VOLT_V_ADDR 0x9135
#define MODBUS_FB_7_HSC_P48V_VIN_VOLT_V_ADDR 0x9136
#define MODBUS_FB_8_HSC_P48V_VIN_VOLT_V_ADDR 0x9137
#define MODBUS_FB_9_HSC_P48V_VIN_VOLT_V_ADDR 0x9138
#define MODBUS_FB_10_HSC_P48V_VIN_VOLT_V_ADDR 0x9139
#define MODBUS_FB_11_HSC_P48V_VIN_VOLT_V_ADDR 0x913A
#define MODBUS_FB_12_HSC_P48V_VIN_VOLT_V_ADDR 0x913B
#define MODBUS_FB_13_HSC_P48V_VIN_VOLT_V_ADDR 0x913C
#define MODBUS_FB_14_HSC_P48V_VIN_VOLT_V_ADDR 0x913D
#define MODBUS_FB_1_HSC_P48V_IOUT_CURR_A_ADDR 0x913E
#define MODBUS_FB_2_HSC_P48V_IOUT_CURR_A_ADDR 0x913F
#define MODBUS_FB_3_HSC_P48V_IOUT_CURR_A_ADDR 0x9140
#define MODBUS_FB_4_HSC_P48V_IOUT_CURR_A_ADDR 0x9141
#define MODBUS_FB_5_HSC_P48V_IOUT_CURR_A_ADDR 0x9142
#define MODBUS_FB_6_HSC_P48V_IOUT_CURR_A_ADDR 0x9143
#define MODBUS_FB_7_HSC_P48V_IOUT_CURR_A_ADDR 0x9144
#define MODBUS_FB_8_HSC_P48V_IOUT_CURR_A_ADDR 0x9145
#define MODBUS_FB_9_HSC_P48V_IOUT_CURR_A_ADDR 0x9146
#define MODBUS_FB_10_HSC_P48V_IOUT_CURR_A_ADDR 0x9147
#define MODBUS_FB_11_HSC_P48V_IOUT_CURR_A_ADDR 0x9148
#define MODBUS_FB_12_HSC_P48V_IOUT_CURR_A_ADDR 0x9149
#define MODBUS_FB_13_HSC_P48V_IOUT_CURR_A_ADDR 0x914A
#define MODBUS_FB_14_HSC_P48V_IOUT_CURR_A_ADDR 0x914B
#define MODBUS_FB_1_HSC_P48V_PIN_PWR_W_ADDR 0x914C
#define MODBUS_FB_2_HSC_P48V_PIN_PWR_W_ADDR 0x914D
#define MODBUS_FB_3_HSC_P48V_PIN_PWR_W_ADDR 0x914E
#define MODBUS_FB_4_HSC_P48V_PIN_PWR_W_ADDR 0x914F
#define MODBUS_FB_5_HSC_P48V_PIN_PWR_W_ADDR 0x9150
#define MODBUS_FB_6_HSC_P48V_PIN_PWR_W_ADDR 0x9151
#define MODBUS_FB_7_HSC_P48V_PIN_PWR_W_ADDR 0x9152
#define MODBUS_FB_8_HSC_P48V_PIN_PWR_W_ADDR 0x9153
#define MODBUS_FB_9_HSC_P48V_PIN_PWR_W_ADDR 0x9154
#define MODBUS_FB_10_HSC_P48V_PIN_PWR_W_ADDR 0x9155
#define MODBUS_FB_11_HSC_P48V_PIN_PWR_W_ADDR 0x9156
#define MODBUS_FB_12_HSC_P48V_PIN_PWR_W_ADDR 0x9157
#define MODBUS_FB_13_HSC_P48V_PIN_PWR_W_ADDR 0x9158
#define MODBUS_FB_14_HSC_P48V_PIN_PWR_W_ADDR 0x9159
#define MODBUS_FB_11_FAN_TACH_RPM_ADDR 0x915A
#define MODBUS_FB_12_FAN_TACH_RPM_ADDR 0x915B
#define MODBUS_FB_13_FAN_TACH_RPM_ADDR 0x915C
#define MODBUS_FB_14_FAN_TACH_RPM_ADDR 0x915D
#define MODBUS_SB_HEX_PRESSURE_1_P_KPA_ADDR 0x915E
#define MODBUS_SB_HEX_PRESSURE_2_P_KPA_ADDR 0x915F
#define MODBUS_FB_1_HUM_PCT_RH_ADDR 0x9160
#define MODBUS_FB_2_HUM_PCT_RH_ADDR 0x9161
#define MODBUS_FB_3_HUM_PCT_RH_ADDR 0x9162
#define MODBUS_FB_4_HUM_PCT_RH_ADDR 0x9163
#define MODBUS_FB_5_HUM_PCT_RH_ADDR 0x9164
#define MODBUS_FB_6_HUM_PCT_RH_ADDR 0x9165
#define MODBUS_FB_7_HUM_PCT_RH_ADDR 0x9166
#define MODBUS_FB_8_HUM_PCT_RH_ADDR 0x9167
#define MODBUS_FB_9_HUM_PCT_RH_ADDR 0x9168
#define MODBUS_FB_10_HUM_PCT_RH_ADDR 0x9169
#define MODBUS_FB_11_HUM_PCT_RH_ADDR 0x916A
#define MODBUS_FB_12_HUM_PCT_RH_ADDR 0x916B
#define MODBUS_FB_13_HUM_PCT_RH_ADDR 0x916C
#define MODBUS_FB_14_HUM_PCT_RH_ADDR 0x916D
#define MODBUS_LEAK_RPU_INT_ADDR 0x9202
#define MODBUS_LEAK_RACK_FLOOR_GPO_AND_RELAY_ADDR 0x9202
#define MODBUS_PUMP_SETTING_ADDR 0x9410
//FRU Info
#define FRU_FB_PART_ADDR 0x19C4
#define FRU_MFR_MODEL_ADDR 0x19CC
#define FRU_MFR_DATE_ADDR 0x19D4
#define FRU_MFR_SERIEL_ADDR 0x19D8
#define FRU_WORKORDER_ADDR 0x19E0
#define FRU_HW_REVISION_ADDR 0x19E4
#define FRU_FW_REVISION_ADDR 0x19E8
#define FRU_TOTAL_UP_TIME_ADDR 0x19EC
#define FRU_LAST_ON_TIME_ADDR 0x19EF
#define FRU_HMI_REVISION_ADDR 0x19F2
#define FRU_NOAH_ARK_CONFIG_ADDR 0x19F8
#define FRU_HEATEXCHANGER_CONTROLBOX_FPBN_ADDR 0x1A00
#define FRU_QUANTA_FB_PART_ADDR 0x1A0C
