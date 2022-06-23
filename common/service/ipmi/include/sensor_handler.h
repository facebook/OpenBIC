#ifndef SENSOR_HANDLER_H
#define SENSOR_HANDLER_H

#include "ipmi.h"

#define SENSOR_THRESHOLD_STATUS 0xC0

void SENSOR_GET_SENSOR_READING(ipmi_msg *msg);

void IPMI_SENSOR_handler(ipmi_msg *msg);

#endif
