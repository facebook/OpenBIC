#ifndef OEM_HANDLER_H
#define OEM_HANDLER_H

#include "ipmi.h"

void OEM_NM_SENSOR_READ(ipmi_msg *msg);

#ifdef CONFIG_ESPI
void OEM_SET_SYSTEM_GUID(ipmi_msg *msg);
#endif

void IPMI_OEM_handler(ipmi_msg *msg);

#endif
