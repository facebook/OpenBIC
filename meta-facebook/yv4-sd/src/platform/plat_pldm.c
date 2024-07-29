#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>
#include <logging/log_ctrl.h>
#include <stdlib.h>
#include <stdio.h>
#include "libutil.h"
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

uint8_t plat_pldm_get_http_boot_attr(uint8_t length, uint8_t *httpBootattr)
{
	CHECK_NULL_ARG_WITH_RETURN(httpBootattr, PLDM_ERROR);

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

	struct pldm_oem_read_file_io_req *ptr = (struct pldm_oem_read_file_io_req *)malloc(
		sizeof(struct pldm_oem_read_file_io_req) +
		sizeof(uint8_t) /* Minimum requried length */);
	if (ptr == NULL) {
		LOG_ERR("Fail to allocate ptr for reading file IO request: get http attr");
		return PLDM_ERROR;
	}

	ptr->cmd_code = HTTP_BOOT;
	ptr->read_option = READ_FILE_ATTR;
	ptr->read_info_length = 1;
	ptr->read_info[0] = 0x00;

	pmsg.buf = (uint8_t *)ptr;
	pmsg.len = sizeof(struct pldm_oem_read_file_io_req) + sizeof(uint8_t);
	uint16_t resp_len = BMC_PLDM_DATA_MAXIMUM;
	struct pldm_oem_read_file_io_resp *rbuf =
		(struct pldm_oem_read_file_io_resp *)malloc(sizeof(uint8_t) * resp_len);
	if (rbuf == NULL) {
		LOG_ERR("Fail to allocate rbuf for getting http boot attribute");
		SAFE_FREE(ptr);
		return PLDM_ERROR;
	}

	resp_len = mctp_pldm_read(find_mctp_by_bus(bmc_bus), &pmsg, (uint8_t *)rbuf, resp_len);
	if (resp_len == 0) {
		LOG_ERR("Fail to send command to get http boot attribute");
		SAFE_FREE(ptr);
		SAFE_FREE(rbuf);
		return PLDM_ERROR_INVALID_DATA;
	}

	SAFE_FREE(ptr);
	if (rbuf->completion_code != PLDM_SUCCESS) {
		LOG_ERR("Read http attribute fail, cc: 0x%x", rbuf->completion_code);
		SAFE_FREE(rbuf);
		return PLDM_ERROR;
	}

	if (rbuf->read_info_length > length) {
		LOG_ERR("Read info length exceeded buffer length, read length: 0x%x, buffer length: 0x%x",
			rbuf->read_info_length, length);
		SAFE_FREE(rbuf);
		return PLDM_ERROR_INVALID_DATA;
	}

	memcpy(httpBootattr, rbuf->read_info, rbuf->read_info_length);
	SAFE_FREE(rbuf);
	return PLDM_SUCCESS;
}

uint8_t plat_pldm_get_http_boot_data(uint16_t offset, uint8_t *read_length, uint8_t buffer_length,
				     uint8_t *httpBootData)
{
	CHECK_NULL_ARG_WITH_RETURN(httpBootData, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(read_length, PLDM_ERROR);

	if (*read_length > buffer_length) {
		LOG_ERR("Invalid read length: 0x%x, buffer length: 0x%x", *read_length,
			buffer_length);
		return PLDM_ERROR;
	}

	pldm_msg pmsg = { 0 };
	uint8_t bmc_bus = I2C_BUS_BMC;
	uint8_t bmc_interface = pal_get_bmc_interface();
	uint8_t hdr_req_len = sizeof(pldm_hdr) + sizeof(struct pldm_oem_read_file_io_req);
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

	struct pldm_oem_read_file_io_req *ptr = (struct pldm_oem_read_file_io_req *)malloc(
		sizeof(struct pldm_oem_read_file_io_req) +
		sizeof(struct pldm_oem_read_file_data_info));
	if (ptr == NULL) {
		LOG_ERR("Fail to allocate ptr for reading file IO request: get http data");
		return PLDM_ERROR;
	}

	ptr->cmd_code = HTTP_BOOT;
	ptr->read_option = READ_FILE_DATA;
	ptr->read_info_length = sizeof(struct pldm_oem_read_file_data_info);

	struct pldm_oem_read_file_data_info data_info = { 0 };
	data_info.data_length = (*read_length > (BMC_PLDM_DATA_MAXIMUM - hdr_req_len) ?
					 (BMC_PLDM_DATA_MAXIMUM - hdr_req_len) :
					 *read_length);
	data_info.transfer_flag = PLDM_START;
	data_info.highOffset = (offset >> 8) & 0xFF;
	data_info.lowOffset = offset & 0xFF;
	memcpy(ptr->read_info, &data_info, sizeof(struct pldm_oem_read_file_data_info));

	pmsg.buf = (uint8_t *)ptr;
	pmsg.len = sizeof(struct pldm_oem_read_file_io_req) +
		   sizeof(struct pldm_oem_read_file_data_info);
	uint16_t resp_len = BMC_PLDM_DATA_MAXIMUM;
	struct pldm_oem_read_file_io_resp *rbuf =
		(struct pldm_oem_read_file_io_resp *)malloc(sizeof(uint8_t) * resp_len);
	if (rbuf == NULL) {
		LOG_ERR("Fail to allocate rbuf for getting http boot data");
		SAFE_FREE(ptr);
		return PLDM_ERROR;
	}

	uint8_t total_length = 0;
	while (true) {
		resp_len =
			mctp_pldm_read(find_mctp_by_bus(bmc_bus), &pmsg, (uint8_t *)rbuf, resp_len);
		if (resp_len == 0) {
			LOG_ERR("Fail to send command to get http boot data");
			SAFE_FREE(ptr);
			SAFE_FREE(rbuf);
			return PLDM_ERROR_INVALID_DATA;
		}

		if (rbuf->completion_code != PLDM_SUCCESS) {
			LOG_ERR("Read http data fail, cc: 0x%x", rbuf->completion_code);
			SAFE_FREE(ptr);
			SAFE_FREE(rbuf);
			return PLDM_ERROR;
		}

		memcpy(&data_info, rbuf->read_info, sizeof(struct pldm_oem_read_file_data_info));
		if ((total_length + data_info.data_length) > buffer_length) {
			LOG_ERR("Total data length exceeded buffer length, total length: 0x%x, buffer length: 0x%x",
				total_length + data_info.data_length, buffer_length);
			SAFE_FREE(ptr);
			SAFE_FREE(rbuf);
			return PLDM_ERROR_INVALID_DATA;
		}

		memcpy(httpBootData + total_length,
		       &rbuf->read_info[sizeof(struct pldm_oem_read_file_data_info)],
		       data_info.data_length);
		total_length += data_info.data_length;

		if (total_length >= *read_length || data_info.transfer_flag == PLDM_END ||
		    data_info.transfer_flag == PLDM_START_AND_END) {
			// Correct read_length
			*read_length = total_length;
			break;
		}

		data_info.data_length =
			((*read_length - total_length) > (BMC_PLDM_DATA_MAXIMUM - hdr_req_len) ?
				 (BMC_PLDM_DATA_MAXIMUM - hdr_req_len) :
				 (*read_length - total_length));
		memcpy(ptr->read_info, &data_info, sizeof(struct pldm_oem_read_file_data_info));
		pmsg.buf = (uint8_t *)ptr;
		pmsg.len = sizeof(struct pldm_oem_read_file_io_req) +
			   sizeof(struct pldm_oem_read_file_data_info);
	}

	SAFE_FREE(ptr);
	SAFE_FREE(rbuf);
	return PLDM_SUCCESS;
}
