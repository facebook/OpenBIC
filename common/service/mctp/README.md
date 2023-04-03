<h1> Implement MCTP and PLDM </h1>

## Introduction
------------------
Add common code of the Management Component Transport Protocol (MCTP) and the Platform Level Data Model (PLDM) in OpenBIC project.

The currently supported feature are the following:
- PLDM over MCTP over SMBus 
- Asynchronous command response function invoke 
- Initiative send MCTP request command 
- MCTP service
    - Bridge command to other device by endpoint ID
    - Control command (message type 0x00)
        - Get Endpoint ID (common code 0x02)
- PLDM service (message type 0x01)
    - Base command
        -  GetTid (common code 0x02)
    - OEM command
        - Echo command for testing
        - Support IPMI command

## Core API
---------------------------
- `mctp_init()`
    - Allocate memory for the structure of MCTP instance use.
- `mctp_set_medium_confiure()`
    - Set the medium type.
- `mctp_reg_endpoint_resolve_func()`
    - Assign route rule function by user to find corresponding MCTP instance by endpoint id.
- `mctp_reg_msg_rx_func()`
    - Assign MCTP message handle function to process the message of MCTP receive.
- `mctp_start()`
    - Start MCTP service.

## Usage
Include MCTP and PLDM related header file in the platform code

```c
#include "mctp.h"
#include "mctp_ctrl.h"
#include "pldm.h"
```
Add source and include path in CMakeLists.txt
```cmake
FILE(GLOB common_sources ${common_path}/mctp/*.c ${common_path}/pldm/*.c)

target_include_directories(app PRIVATE ${common_path}/mctp ${common_path/pldm})
```
---
Initial the MCTP instance, `mctp_init()` also allocate the memory for the MCTP instance.
```c
mctp *p = mctp_init();
```
Configure mctp handle with specific medium type
```c
uint8_t ret = mctp_set_medium_configure(p, MCTP_MEDIUM_TYPE_SMBUS, medium_configuration)
```
Assign a function use the MCTP instance as one parameter to find a suitable instance by endpoint ID for routing the MCTP message
```c
static uint8_t get_mctp_route_info(uint8_t dest_endpoint, void **mctp_inst, mctp_ext_params *ext_params);

mctp_reg_endpoint_resolve_func(p, get_mctp_route_info)
```
Assign a function to handle when the MCTP message receive
```c
static uint8_t mctp_msg_recv(void *mctp_p, uint8_t *buf, uin32_t len, mctp_ext_params ext_params);

mctp_reg_msg_rx_func(p, mctp_msg_recv);
```
Call the `mctp_start()` to start the MCTP service on the specific interface

```c
mctp_start(p);
```
## Endpoint ID
------------------
Endpoint ID use for identifying which controller to receive the MCTP package. To prevent EID conflicts, define below table for reference:
| Platform Code | Device | EID | Remark |
| ------------ | ------------ | ------------ | ------------ |
| All Platform | BIC | 0x00 | BIC common reserve |
| All Platform | BMC | 0x08 | BMC common default |
| Cascade Creek | BIC | 0x0A | BIC common default |
| Moose Creek | BIC | 0x0B |   |
| Colter Bay | BIC | 0x0C |   |
| Cascade Creek | NIC0 | 0x10 |   |
| Cascade Creek | NIC1 | 0x11 |   |
| Cascade Creek | NIC2 | 0x12 |   |
| Cascade Creek | NIC3 | 0x13 |   |
| Cascade Creek | NIC4 | 0x14 |   |
| Cascade Creek | NIC5 | 0x15 |   |
| Cascade Creek | NIC6 | 0x16 |   |
| Cascade Creek | NIC7 | 0x17 |   |
| Crafter Lake | BIC | 0x20 |   |
| Y35 Baseboard | BIC | 0x21 |   |
| Great Lake | BIC | 0x22 |   |
| Halfdome | BIC | 0x23 |   |
| Rainball Falls | BIC | 0x24 |   |
| Delta Lake | BIC | 0x2A |   |
| Vernal Falls | BIC | 0x2B |   |
| Rainball Falls | CXL | 0x2E |   |
| Moose Creek | CXL | 0x2F |   |
| Olmsted Point A | BIC | 0x30 |   |
| Olmsted Point B | BIC | 0x31 |   |
| Waimea Canyon MB | BIC | 0x38 |   |
## Acknowledgments
-----
- [Management Component Transport Protocol (MCTP) Base Specification (DSP0236)](https://www.dmtf.org/sites/default/files/standards/documents/DSP0236_1.3.0.pdf)
- [Platform Level Data Model (PLDM) for Platform
Monitoring and Control Specification (DSP2048)](https://www.dmtf.org/sites/default/files/standards/documents/DSP0248_1.2.0.pdf)
- [Management Component Transport Protocol(MCTP) SMBus/I2C Transport Binding Specification (DSP0237)](https://www.dmtf.org/sites/default/files/standards/documents/DSP0237_1.2.0.pdf)