#include <modbus/modbus.h>

#define MODBUS_UART_BAUDRATE_LOW 19200
#define MODBUS_UART_BAUDRATE_HIGH 115200
#define MODBUS_UART_PARITY UART_CFG_PARITY_EVEN
#define MODBUS_UART_RESPONSE_T 1000000 
//from zephyr/samples/subsys/modbus/rtu_client, client_param: rx_timeout = 1000000(default 50000)

enum {
	MODBUS_READ_WRITE_REGISTER_SUCCESS,
	MODBUS_FUNCCODE_NOT_SUPPORT,
	MODBUS_NOT_IN_REGISTER_VAL_RANGE,
	MODBUS_ADDR_NOT_DEFINITION,
	MODBUS_READ_WRITE_REGISTER_FAIL,
};

int init_modbus_server(const char iface_name, struct modbus_iface_param server_param);