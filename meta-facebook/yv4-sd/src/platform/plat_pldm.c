#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>
#include <logging/log_ctrl.h>
#include <stdlib.h>
#include <stdio.h>
#include "plat_mctp.h"
#include "plat_pldm.h"
#include "hal_i2c.h"
#include "util_sys.h"

LOG_MODULE_REGISTER(plat_pldm);

uint8_t plat_pldm_get_tid()
{
	// Set TID as EID
	return plat_get_eid();
}

uint8_t plat_pldm_get_http_boot_data(uint8_t *httpBootData, uint16_t *httpBootDataLen)
{
	pldm_msg pmsg = { 0 };

	uint8_t bmc_bus = I2C_BUS_BMC;
	uint8_t bmc_interface = pal_get_bmc_interface();
	pmsg.ext_params.ep = MCTP_EID_BMC;

	switch (bmc_interface) {
	case BMC_INTERFACE_I3C:
		bmc_bus = I3C_BUS_BMC;
		pmsg.ext_params.type = MCTP_MEDIUM_TYPE_TARGET_I3C;
		pmsg.ext_params.i3c_ext_params.addr = I3C_STATIC_ADDR_BMC;
		break;
	case BMC_INTERFACE_I2C:
		bmc_bus = I2C_BUS_BMC;
		pmsg.ext_params.type = MCTP_MEDIUM_TYPE_SMBUS;
		pmsg.ext_params.smbus_ext_params.addr = I2C_ADDR_BMC;
		break;
	default:
		return PLDM_ERROR_INVALID_DATA;
	}

	pmsg.hdr.rq = PLDM_REQUEST;
	pmsg.hdr.pldm_type = PLDM_TYPE_OEM;
	pmsg.hdr.cmd = PLDM_OEM_READ_FILE_IO;

	struct pldm_oem_read_file_io_req ptr = { 0 };
	ptr.cmd_code = HTTP_BOOT;
	ptr.data_length = BMC_PLDM_DATA_MAXIMUM - sizeof(struct pldm_oem_read_file_io_req);
	ptr.transfer_flag = PLDM_START;
	ptr.highOffset = 0;
	ptr.lowOffset = 0;

	pmsg.buf = (uint8_t *)&ptr;
	pmsg.len = sizeof(struct pldm_oem_read_file_io_req);

	uint16_t resp_len = BMC_PLDM_DATA_MAXIMUM;

	struct pldm_oem_read_file_io_resp *rbuf =
		(struct pldm_oem_read_file_io_resp *)malloc(sizeof(uint8_t) * resp_len);
	if (rbuf == NULL) {
		LOG_ERR("Failed to allocate response buffer");
		return PLDM_ERROR;
	}

	uint16_t offset = 0;
	while (true) {
		if (!mctp_pldm_read(find_mctp_by_bus(bmc_bus), &pmsg, (uint8_t *)rbuf, resp_len)) {
			LOG_ERR("Fail to send OEM HTTP BOOT DATA");
			free(rbuf);
			return PLDM_ERROR_INVALID_DATA;
		}

		memcpy(httpBootData + offset, rbuf->messages, rbuf->data_length);

		ptr.transfer_flag = rbuf->transfer_flag;
		ptr.highOffset = rbuf->highOffset;
		ptr.lowOffset = rbuf->lowOffset;
		pmsg.buf = (uint8_t *)&ptr;

		if (rbuf->transfer_flag == PLDM_END || rbuf->transfer_flag == PLDM_START_AND_END) {
			*httpBootDataLen = offset + rbuf->data_length;
			break;
		}

		offset = ((uint16_t)ptr.highOffset << 8) | ptr.lowOffset;
	}

	free(rbuf);
	return PLDM_SUCCESS;
}
