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

/* platform Event */
#define SENSOR_NUM_SYSTEM_STATUS 0x10
#define SENSOR_NUM_SYS_STA 0x46

#define IPMI_EVENT_OFFSET_SYS_M2PRESENT 0x80
#define IPMI_EVENT_OFFSET_SYS_INA231_PWR_ALERT 0x81
#define IPMI_EVENT_OFFSET_SYS_HSC_PWR_ALERT 0x82
#define IPMI_EVENT_OFFSET_SYS_IRQ_P12V_E1S_FLT 0x83
#define IPMI_EVENT_OFFSET_SYS_IRQ_P3V3_E1S_FLT 0x84
#define IPMI_EVENT_OFFSET_SYS_IRQ_P12V_EDGE_FLT 0x85

#define E1S_BOARD_TYPE 0x07

typedef enum {
	NOSIE_E_M2PRSNT_A = 0,
	NOSIE_E_M2PRSNT_B,
	NOSIE_E_M2PRSNT_C,
	NOSIE_E_M2PRSNT_D,
	NOSIE_E_M2PRSNT_MAX,
} NOSIE_E;

typedef enum {
	DEASSERT_CHK_TYPE_E_INA231_ALERT_0, // INA231 device alert
	DEASSERT_CHK_TYPE_E_INA231_ALERT_1,
	DEASSERT_CHK_TYPE_E_INA231_ALERT_2,
	DEASSERT_CHK_TYPE_E_INA231_ALERT_3,
	DEASSERT_CHK_TYPE_E_MAX
} DEASSERT_CHK_TYPE_E;

void delay_function(uint32_t delay_time, void *func, uint32_t arg1, uint32_t arg2);
uint8_t ignore_noise(uint8_t idx, uint32_t m_sec);
void add_sel(uint8_t sensor_type, uint8_t event_type, uint8_t sensor_number, uint8_t event_data1,
	     uint8_t event_data2, uint8_t event_data3);
void add_sel_work(uint32_t sel_msg_addr);
uint8_t assert_func(DEASSERT_CHK_TYPE_E assert_type);
void deassert_func_handler(DEASSERT_CHK_TYPE_E idx);
void init_sel_sensor_num(void);
uint8_t get_sel_sensor_num(void);
