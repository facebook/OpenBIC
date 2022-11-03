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

#ifndef _PLDM_MONITOR_H
#define _PLDM_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pldm.h"

/* command number of pldm type 0x02 : PLDM for platform monitor and control */
typedef enum pldm_platform_monitor_commands {
	PLDM_MONITOR_CMD_CODE_GET_SENSOR_READING = 0x11,
	PLDM_MONITOR_CMD_CODE_SET_EVENT_RECEIVER = 0x04,
	PLDM_MONITOR_CMD_CODE_PLATFORM_EVENT_MESSAGE = 0x0A,
	PLDM_MONITOR_CMD_CODE_SET_STATE_EFFECTER_STATES = 0x39,
	PLDM_MONITOR_CMD_CODE_GET_STATE_EFFECTER_STATES = 0x3A,
} pldm_platform_monitor_commands_t;

/* define size of request */
#define PLDM_GET_SENSOR_READING_REQ_BYTES 3
#define PLDM_SET_STATE_EFFECTER_REQ_NO_STATE_FIELD_BYTES 3

/* Define size of response */
#define PLDM_GET_STATE_EFFECTER_RESP_NO_STATE_FIELD_BYTES 2

/* The maximum event data size of event type currently support */
#define PLDM_MONITOR_EVENT_DATA_SIZE_MAX 7
/* The maximum event message number in the queue */
#define PLDM_MONITOR_EVENT_QUEUE_MSG_NUM_MAX 15
#define PLDM_MONITOR_SENSOR_SUPPORT_MAX 0xFF
#define PLDM_MONITOR_SENSOR_EVENT_SENSOR_OP_STATE_DATA_LENGTH 2
#define PLDM_MONITOR_SENSOR_EVENT_STATE_SENSOR_STATE_DATA_LENGTH 3
#define PLDM_MONITOR_SENSOR_EVENT_NUMERIC_SENSOR_STATE_MIN_DATA_LENGTH 4
#define PLDM_MONITOR_SENSOR_EVENT_NUMERIC_SENSOR_STATE_MAX_DATA_LENGTH 7

#define PLDM_MONITOR_SENSOR_DATA_SIZE_INT8 1
#define PLDM_MONITOR_SENSOR_DATA_SIZE_INT16 2
#define PLDM_MONITOR_SENSOR_DATA_SIZE_INT32 4

#define PLDM_PLATFORM_OEM_GPIO_EFFECTER_STATE_FIELD_COUNT 2
#define PLDM_PLATFORM_OEM_AST1030_GPIO_PIN_NUM_NAX 167

typedef enum pldm_sensor_readings_data_type {
	PLDM_SENSOR_DATA_SIZE_UINT8,
	PLDM_SENSOR_DATA_SIZE_SINT8,
	PLDM_SENSOR_DATA_SIZE_UINT16,
	PLDM_SENSOR_DATA_SIZE_SINT16,
	PLDM_SENSOR_DATA_SIZE_UINT32,
	PLDM_SENSOR_DATA_SIZE_SINT32
} pldm_sensor_readings_data_type_t;

enum pldm_sensor_operational_state {
	PLDM_SENSOR_ENABLED,
	PLDM_SENSOR_DISABLED,
	PLDM_SENSOR_UNAVAILABLE,
	PLDM_SENSOR_STATUSUNKOWN,
	PLDM_SENSOR_FAILED,
	PLDM_SENSOR_INITIALIZING,
	PLDM_SENSOR_SHUTTINGDOWN,
	PLDM_SENSOR_INTEST
};

enum pldm_effecter_operational_state {
	PLDM_EFFECTER_ENABLED_UPDATEPENDING,
	PLDM_EFFECTER_ENABLED_NOUPDATEPENDING,
	PLDM_EFFECTER_DISABLED,
	PLDM_EFFECTER_UNAVAILABLE,
	PLDM_EFFECTER_STATUSUNKNOWN,
	PLDM_EFFECTER_FAILED,
	PLDM_EFFECTER_INITIALIZING,
	PLDM_EFFECTER_SHUTTINGDOWN,
	PLDM_EFFECTER_INTEST
};

enum pldm_oem_effecter_type {
	OEM_EFFECTER_TYPE_GPIO = 0xFF,
};

enum set_request {
	PLDM_NO_CHANGE = 0x00,
	PLDM_REQUEST_SET = 0x01,
	PLDM_SET_REQUEST_MAX,
};

enum oem_effecter_states_gpio_direction {
	EFFECTER_STATE_GPIO_DIRECTION_UNKNOWN = 0x00,
	EFFECTER_STATE_GPIO_DIRECTION_INPUT = 0x01,
	EFFECTER_STATE_GPIO_DIRECTION_OUTPUT = 0x02,
	EFFECTER_STATE_GPIO_DIRECTION_MAX,
};

enum oem_effecter_states_gpio_value {
	EFFECTER_STATE_GPIO_VALUE_UNKNOWN = 0x00,
	EFFECTER_STATE_GPIO_VALUE_LOW = 0x01,
	EFFECTER_STATE_GPIO_VALUE_HIGH = 0x02,
	EFFECTER_STATE_GPIO_VALUE_MAX
};

enum pldm_sensor_present_state {
	PLDM_SENSOR_UNKNOWN = 0x0,
	PLDM_SENSOR_NORMAL = 0x01,
	PLDM_SENSOR_WARNING = 0x02,
	PLDM_SENSOR_CRITICAL = 0x03,
	PLDM_SENSOR_FATAL = 0x04,
	PLDM_SENSOR_LOWERWARNING = 0x05,
	PLDM_SENSOR_LOWERCRITICAL = 0x06,
	PLDM_SENSOR_LOWERFATAL = 0x07,
	PLDM_SENSOR_UPPERWARNING = 0x08,
	PLDM_SENSOR_UPPERCRITICAL = 0x09,
	PLDM_SENSOR_UPPERFATAL = 0x0a
};

enum pldm_sensor_event_message_enable {
	PLDM_NO_EVENT_GENERATION,
	PLDM_EVENTS_DISABLED,
	PLDM_EVENTS_ENABLED,
	PLDM_OP_EVENTS_ONLY_ENABLED,
	PLDM_STATE_EVENTS_ONLY_ENABLED
};

enum pldm_platform_completion_codes {
	/* GetStateSensorReadings */
	PLDM_PLATFORM_INVALID_SENSOR_ID = 0x80,
	PLDM_PLATFORM_REARM_UNAVAILABLE_IN_PRESENT_STATE = 0x81,

	/* SetEventReceiver */
	PLDM_PLATFORM_INVALID_PROTOCOL_TYPE = 0x80,
	PLDM_PLATFORM_ENABLE_METHOD_NOT_SUPPORTED = 0x81,
	PLDM_PLATFORM_HEARTBEAT_FREQUENCY_TOO_HIGH = 0x82,

	/* SetStateEffecterStates, GetStateEffecterStates */
	PLDM_PLATFORM_INVALID_EFFECTER_ID = 0x80,
	PLDM_PLATFORM_INVALID_STATE_VALUE = 0x81,
	PLDM_PLATFORM_UNSUPPORTED_EFFECTERSTATE = 0x82,

};

enum pldm_oem_platform_completion_codes {
	/* SetStateEffecterStates, GetStateEffecterStates */
	PLDM_OEM_GPIO_UNSUPPORT_RANGE = 0x83,
	PLDM_OEM_GPIO_EFFECTER_INVALID_SET_VALUE = 0x84,
	PLDM_OEM_GPIO_EFFECTER_VALUE_UNKNOWN = 0x85,
};

struct pldm_get_sensor_reading_req {
	uint16_t sensor_id;
	uint8_t rearm_event_state;
} __attribute__((packed));

struct pldm_get_sensor_reading_resp {
	uint8_t completion_code;
	uint8_t sensor_data_size;
	uint8_t sensor_operational_state;
	uint8_t sensor_event_message_enable;
	uint8_t present_state;
	uint8_t previous_state;
	uint8_t event_state;
	uint8_t present_reading[1];
} __attribute__((packed));

enum pldm_event_types {
	PLDM_SENSOR_EVENT = 0x00,
	PLDM_EFFECTER_EVENT = 0x01,
	PLDM_REDFISH_TASK_EXECUTED_EVENT = 0x02,
	PLDM_REDFISH_MESSAGE_EVENT = 0x03,
	PLDM_PDR_REPOSITORY_CHG_EVENT = 0x04,
	PLDM_MESSAGE_POLL_EVENT = 0x05,
	PLDM_HEARTBEAT_TIMER_ELAPSED_EVENT = 0x06
};

typedef enum pldm_sensor_event_class {
	PLDM_SENSOR_OP_STATE,
	PLDM_STATE_SENSOR_STATE,
	PLDM_NUMERIC_SENSOR_STATE
} pldm_sensor_event_class_t;

typedef enum pldm_effecter_event_class { PLDM_EFFECTER_OP_STATE } pldm_effecter_event_class_t;

enum pldm_platform_event_status {
	PLDM_EVENT_NO_LOGGING = 0x00,
	PLDM_EVENT_LOGGING_DISABLED = 0x01,
	PLDM_EVENT_LOG_FULL = 0x02,
	PLDM_EVENT_ACCEPTED_FOR_LOGGING = 0x03,
	PLDM_EVENT_LOGGED = 0x04,
	PLDM_EVENT_LOGGING_REJECTED = 0x05
};

enum pldm_event_message_global_enable {
	PLDM_EVENT_MESSAGE_GLOBAL_DISABLE,
	PLDM_EVENT_MESSAGE_GLOBAL_ENABLE_ASYNC,
	PLDM_EVENT_MESSAGE_GLOBAL_ENABLE_POLLING,
	PLDM_EVENT_MESSAGE_GLOBAL_ENABLE_ASYNC_KEEP_ALIVE
};

struct pldm_platform_event_message_req {
	uint8_t format_version;
	uint8_t tid;
	uint8_t event_class;
	uint8_t event_data[1];
} __attribute__((packed));

struct pldm_platform_event_message_resp {
	uint8_t completion_code;
	uint8_t platform_event_status;
} __attribute__((packed));

struct pldm_sensor_event_data {
	uint16_t sensor_id;
	uint8_t sensor_event_class_type;
	uint8_t event_class_data[1];
} __attribute__((packed));

struct pldm_sensor_event_state_sensor_state {
	uint8_t sensor_offset;
	uint8_t event_state;
	uint8_t previous_event_state;
} __attribute__((packed));

struct pldm_sensor_event_numeric_sensor_state {
	uint8_t event_state;
	uint8_t previous_event_state;
	uint8_t sensor_data_size;
	uint8_t present_reading[1];
} __attribute__((packed));

struct pldm_sensor_event_sensor_op_state {
	uint8_t present_op_state;
	uint8_t previous_op_state;
} __attribute__((packed));

struct pldm_effecter_event_data {
	uint16_t effecter_id;
	uint8_t effecter_event_class;
	uint8_t event_class_data[1];
} __attribute__((packed));

struct pldm_effeter_event_op_state {
	uint8_t present_op_state;
	uint8_t previous_op_state;
} __attribute__((packed));

struct pldm_set_event_receiver_req {
	uint8_t event_message_global_enable;
	uint8_t transport_protocol_type;
	uint8_t event_receiver_address_info;
	uint16_t heartbeat_timer;
} __attribute__((packed));

typedef struct state_field_state_effecter_set {
	uint8_t set_request;
	uint8_t effecter_state;
} __attribute__((packed)) set_effecter_state_field_t;

typedef struct state_field_state_effecter_get {
	uint8_t effecter_op_state;
	uint8_t pending_state;
	uint8_t previous_state;
} __attribute__((packed)) get_effecter_state_field_t;

struct pldm_set_state_effecter_states_req {
	uint16_t effecter_id;
	uint8_t composite_effecter_count;
	set_effecter_state_field_t field[8];
} __attribute__((packed));

struct pldm_get_state_effecter_states_req {
	uint16_t effecter_id;
} __attribute__((packed));

struct pldm_get_state_effecter_states_resp {
	uint8_t completion_code;
	uint8_t composite_effecter_count;
	get_effecter_state_field_t field[8];
} __attribute__((packed));

uint8_t pldm_monitor_handler_query(uint8_t code, void **ret_fn);

uint8_t pldm_platform_event_message_req(void *mctp_inst, mctp_ext_params ext_params,
					uint8_t event_class, const uint8_t *event_data,
					uint8_t event_data_length);

uint16_t pldm_platform_monitor_read(void *mctp_inst, mctp_ext_params ext_params,
				    pldm_platform_monitor_commands_t cmd, uint8_t *req,
				    uint16_t req_len, uint8_t *rbuf, uint16_t rbuf_len);

uint8_t pldm_send_platform_event(uint8_t event_class, uint16_t id, uint8_t ext_class,
				 const uint8_t *event_data, uint8_t event_data_length);

#ifdef __cplusplus
}
#endif

#endif /* _PLDM_MONITOR_H */
