#ifndef STORAGE_HANDLER_H
#define STORAGE_HANDLER_H

#include "ipmi.h"

void STORAGE_GET_FRUID_INFO(ipmi_msg *msg);
void STORAGE_READ_FRUID_DATA(ipmi_msg *msg);
void STORAGE_WRITE_FRUID_DATA(ipmi_msg *msg);
void STORAGE_RSV_SDR(ipmi_msg *msg);
void STORAGE_GET_SDR(ipmi_msg *msg);
void STORAGE_ADD_SEL(ipmi_msg *msg);

void IPMI_Storage_handler(ipmi_msg *msg);

#endif
