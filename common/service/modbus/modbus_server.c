#include <modbus/modbus.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(modbus_server);

int init_modbus_server(const char *iface_name, struct modbus_iface_param server_param)
{
	//const uint32_t mb_rtu_br = MB_TEST_BAUDRATE_HIGH;
	//const char iface_name[] = {DT_PROP(DT_INST(1, zephyr_modbus_serial), label)};
	// int iface;
	// printk("S_iface %s/n", iface_name);
	int iface = modbus_iface_get_by_name(iface_name);

	if (iface < 0) {
		LOG_ERR("Failed to get iface index for %s", iface_name);
		return iface;
	}

	return modbus_init_server(iface, server_param);
}
