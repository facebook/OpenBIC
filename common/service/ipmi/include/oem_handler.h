#ifndef OEM_HANDLER_H
#define OEM_HANDLER_H

#include "ipmi.h"

void OEM_NM_SENSOR_READ(ipmi_msg *msg);

#ifdef CONFIG_ESPI
void OEM_SET_SYSTEM_GUID(ipmi_msg *msg);
#endif

#ifdef ENABLE_FAN
void OEM_SET_FAN_DUTY_MANUAL(ipmi_msg *msg);
void OEM_GET_SET_FAN_CTRL_MODE(ipmi_msg *msg);
#endif

void OEM_GET_MB_INDEX(ipmi_msg *msg);
void OEM_CABLE_DETECTION(ipmi_msg *msg);
void IPMI_OEM_handler(ipmi_msg *msg);

#endif
