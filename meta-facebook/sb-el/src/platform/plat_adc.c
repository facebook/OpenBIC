#include <drivers/adc.h>
#include <drivers/spi.h>
#include "libutil.h"
#include <logging/log.h>
#include "plat_gpio.h"
#include "plat_adc.h"
#include "plat_class.h"
#include "plat_cpld.h"
#include <device.h>
#include "util_sys.h"
#include "plat_i2c_target.h"
#include "plat_pldm_sensor.h"

LOG_MODULE_REGISTER(plat_adc);

#define BUFFER_SIZE 2
#define ADC_STACK_SIZE 1024

#define NUWA0_ADC_CHANNEL 8
#define NUWA1_ADC_CHANNEL 6

K_THREAD_STACK_DEFINE(adc_electra_thread_stack, ADC_STACK_SIZE);
struct k_thread adc_electra_poll_thread;

static bool adc_poll_flag = true;
uint8_t adc_idx_read = 0;
float ad4058_val_0 = 0;
float ad4058_val_1 = 0;
float ads7066_val_0 = 0;
float ads7066_val_1 = 0;
const float ads7066_vref = 2.5;
const float ad4058_vref = 2.5;
static uint8_t adc_good_status[2] = { 0xFF, 0xFF };

static const struct device *spi_dev;

bool adc_get_poll_flag()
{
	return adc_poll_flag;
}

int ads7066_read_reg(uint8_t reg, uint8_t idx, uint8_t *out_data)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}
	// Set GPIO73 as CS control pin SPI_ADC_CS0_N
	struct spi_cs_control cs_ctrl = {
		.gpio_dev = device_get_binding("GPIO_7"),
		.gpio_pin = 3,
		.gpio_dt_flags = GPIO_ACTIVE_LOW,
		.delay = 0, // No delay
	};
	switch (idx) {
	case ADC_EL_IDX_NUWA0:
		// do nothing
		break;
	case ADC_EL_IDX_NUWA1:
		// Set GPIO73 as CS control pin SPI_ADC_CS1_N
		cs_ctrl.gpio_dev = device_get_binding("GPIO_C");
		cs_ctrl.gpio_pin = 1;
		cs_ctrl.gpio_dt_flags = GPIO_ACTIVE_LOW;
		cs_ctrl.delay = 0; // No delay
		break;
	default:
		LOG_ERR("Invalid ADC index %d", idx);
		break;
	}

	const struct spi_config spi_cfg = {
		.frequency = 6000000, // 6MHz
		.operation =
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
		.slave = 0,
		.cs = &cs_ctrl,
	};

	uint8_t tx_buf[3] = { 0x10, reg, 0x00 }; // bit15=1: read
	uint8_t rx_buf[3] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	int ret = spi_write(spi_dev, &spi_cfg, &tx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		return ret;
	}
	ret = spi_read(spi_dev, &spi_cfg, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI read failed: %d", ret);
		return ret;
	}

	*out_data = rx_buf[0];
	LOG_INF("nuwa%d ADS7066 read reg 0x%02x: 0x%02x 0x%02x 0x%02x", idx, reg, rx_buf[0],
		rx_buf[1], rx_buf[2]);
	return 0;
}
int ads7066_write_reg(uint8_t reg, uint8_t write_val, uint8_t idx)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}
	// Set GPIO73 as CS control pin SPI_ADC_CS0_N
	struct spi_cs_control cs_ctrl = {
		.gpio_dev = device_get_binding("GPIO_7"),
		.gpio_pin = 3,
		.gpio_dt_flags = GPIO_ACTIVE_LOW,
		.delay = 0, // No delay
	};
	switch (idx) {
	case ADC_EL_IDX_NUWA0:
		// do nothing
		break;
	case ADC_EL_IDX_NUWA1:
		// Set GPIO73 as CS control pin SPI_ADC_CS1_N
		cs_ctrl.gpio_dev = device_get_binding("GPIO_C");
		cs_ctrl.gpio_pin = 1;
		cs_ctrl.gpio_dt_flags = GPIO_ACTIVE_LOW;
		cs_ctrl.delay = 0; // No delay
		break;
	default:
		LOG_ERR("Invalid ADC index %d", idx);
		break;
	}

	const struct spi_config spi_cfg = {
		.frequency = 6000000, // 6MHz
		.operation =
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
		.slave = 0,
		.cs = &cs_ctrl,
	};

	uint8_t tx_buf[3] = { 0x08, reg, write_val };

	struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	int ret = spi_write(spi_dev, &spi_cfg, &tx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		return ret;
	}
	LOG_INF("nuwa%d ADS7066 write reg 0x%02x", idx, reg);
	return 0;
}

void read_adc_info()
{
	uint8_t adc_idx = 0;
	// plat_read_cpld(CPLD_OFFSET_ADC_IDX, &adc_idx, 1);
	// Seems that read from CPLD is not necessary
	LOG_WRN("ADC index read from CPLD not implement, waiting for CPLD target\n");
	adc_idx_read = adc_idx;

	/* read VENDOR_L to determine*/
	uint8_t value = 0;
	ad4058_write_reg(0xA8, 0x00, 0);
	ad4058_read_reg(0x0C, 0, &value);
	if (value == 0x56) {
		adc_idx_read = 0;
	} else {
		adc_idx_read = 1;
	}
}
static void ads7066_read_voltage(uint8_t idx)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
		return;
	}
	// Set GPIO73 as CS control pin SPI_ADC_CS0_N
	struct spi_cs_control cs_ctrl = {
		.gpio_dev = device_get_binding("GPIO_7"),
		.gpio_pin = 3,
		.gpio_dt_flags = GPIO_ACTIVE_LOW,
		.delay = 0, // No delay
	};
	switch (idx) {
	case ADC_EL_IDX_NUWA0:
		// do nothing
		break;
	case ADC_EL_IDX_NUWA1:
		// Set GPIOC1 as CS control pin SPI_ADC_CS1_N
		cs_ctrl.gpio_dev = device_get_binding("GPIO_C");
		cs_ctrl.gpio_pin = 1;
		cs_ctrl.gpio_dt_flags = GPIO_ACTIVE_LOW;
		cs_ctrl.delay = 0; // No delay
		break;
	default:
		LOG_ERR("Invalid ADC index %d", idx);
		break;
	}

	const struct spi_config spi_cfg = {
		.frequency = 6000000, // 6MHz
		.operation =
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
		.slave = 0,
		.cs = &cs_ctrl,
	};

	uint8_t tx_buf[3] = { 0x00, 0x00, 0x00 };
	uint8_t rx_buf[3] = { 0 };
	uint8_t out_buf[3] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	memset(rx_buf, 0, 3);
	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI failed: %d", ret);
		return;
	}

	memcpy(out_buf, rx_buf, 3);

	LOG_HEXDUMP_DBG(out_buf, 3, "ads7066_read_voltage_value");
	uint16_t raw_value = out_buf[0] << 8 | out_buf[1];
	if (idx == ADC_EL_IDX_NUWA0) {
		ads7066_val_0 = ((float)raw_value / 65536) * ads7066_vref;
		// update_adc_info(raw_value, ADC_EL_IDX_NUWA0, ads7066_vref);
		LOG_INF("NUWA0 ads7066 value: 0x%04x", raw_value);
	} else if (idx == ADC_EL_IDX_NUWA1) {
		ads7066_val_1 = ((float)raw_value / 65536) * ads7066_vref;
		// update_adc_info(raw_value, ADC_EL_IDX_NUWA1, ads7066_vref);
		LOG_INF("NUWA1 ads7066 value: 0x%04x", raw_value);
	}

	return;
}

int ad4058_read_reg(uint8_t reg, uint8_t idx, uint8_t *out_data)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}

	uint8_t cnv_pin = 0;
	// Set GPIO73 as CS control pin SPI_ADC_CS0_N
	struct spi_cs_control cs_ctrl = {
		.gpio_dev = device_get_binding("GPIO_7"),
		.gpio_pin = 3,
		.gpio_dt_flags = GPIO_ACTIVE_LOW,
		.delay = 0, // No delay
	};
	switch (idx) {
	case ADC_EL_IDX_NUWA0:
		// do nothing
		cnv_pin = NUWA0_CNV;
		break;
	case ADC_EL_IDX_NUWA1:
		// Set GPIO73 as CS control pin SPI_ADC_CS1_N
		cs_ctrl.gpio_dev = device_get_binding("GPIO_C");
		cs_ctrl.gpio_pin = 1;
		cs_ctrl.gpio_dt_flags = GPIO_ACTIVE_LOW;
		cs_ctrl.delay = 0; // No delay
		cnv_pin = NUWA1_CNV;
		break;
	default:
		LOG_ERR("Invalid ADC index %d", idx);
		break;
	}

	const struct spi_config spi_cfg = {
		.frequency = 6000000, // 6MHz
		.operation =
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
		.slave = 0,
		.cs = &cs_ctrl,
	};

	uint8_t tx_buf[3] = { 0x80, 0x00, 0x00 };
	uint8_t rx_buf[3] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = 3 };
	struct spi_buf rx = { .buf = rx_buf, .len = 3 };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	tx_buf[0] += reg;

	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		return ret;
	}

	*out_data = rx_buf[1];
	LOG_HEXDUMP_INF(rx_buf, 3, "ad4058_read_reg");
	return 0;
}

int ad4058_write_reg(uint8_t reg, uint8_t write_val, uint8_t idx)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}
	// Set GPIO73 as CS control pin SPI_ADC_CS0_N
	struct spi_cs_control cs_ctrl = {
		.gpio_dev = device_get_binding("GPIO_7"),
		.gpio_pin = 3,
		.gpio_dt_flags = GPIO_ACTIVE_LOW,
		.delay = 0, // No delay
	};
	switch (idx) {
	case ADC_EL_IDX_NUWA0:
		// do nothing
		break;
	case ADC_EL_IDX_NUWA1:
		// Set GPIO73 as CS control pin SPI_ADC_CS1_N
		cs_ctrl.gpio_dev = device_get_binding("GPIO_C");
		cs_ctrl.gpio_pin = 1;
		cs_ctrl.gpio_dt_flags = GPIO_ACTIVE_LOW;
		cs_ctrl.delay = 0; // No delay
		break;
	default:
		LOG_ERR("Invalid ADC index %d", idx);
		break;
	}

	const struct spi_config spi_cfg = {
		.frequency = 6000000, // 6MHz
		.operation =
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
		.slave = 0,
		.cs = &cs_ctrl,
	};

	uint8_t tx_buf[2] = { reg, write_val };
	uint8_t rx_buf[2] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = 2 };
	struct spi_buf rx = { .buf = rx_buf, .len = 2 };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI write failed: %d", ret);
		return ret;
	}

	LOG_HEXDUMP_DBG(tx_buf, 2, "ad4058_write_reg");
	return 0;
}

static void ad4058_read_voltage(uint8_t idx)
{
	spi_dev = device_get_binding("SPIP");
	if (!spi_dev) {
		LOG_ERR("SPI device not find");
	}

	uint8_t cnv_pin = 0;
	// Set GPIO73 as CS control pin SPI_ADC_CS0_N
	struct spi_cs_control cs_ctrl = {
		.gpio_dev = device_get_binding("GPIO_7"),
		.gpio_pin = 3,
		.gpio_dt_flags = GPIO_ACTIVE_LOW,
		.delay = 0, // No delay
	};
	switch (idx) {
	case ADC_EL_IDX_NUWA0:
		// do nothing
		cnv_pin = NUWA0_CNV;
		break;
	case ADC_EL_IDX_NUWA1:
		// Set GPIO73 as CS control pin SPI_ADC_CS1_N
		cs_ctrl.gpio_dev = device_get_binding("GPIO_C");
		cs_ctrl.gpio_pin = 1;
		cs_ctrl.gpio_dt_flags = GPIO_ACTIVE_LOW;
		cs_ctrl.delay = 0; // No delay
		cnv_pin = NUWA1_CNV;
		break;
	default:
		LOG_ERR("Invalid ADC index %d", idx);
		break;
	}

	const struct spi_config spi_cfg = {
		.frequency = 6000000, // 6MHz
		.operation =
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
		.slave = 0,
		.cs = &cs_ctrl,
	};

	uint8_t tx_buf[2] = { 0 };
	uint8_t rx_buf[3] = { 0 };
	uint8_t out_buf[3] = { 0 };

	struct spi_buf tx = { .buf = tx_buf, .len = 2 };
	struct spi_buf rx = { .buf = rx_buf, .len = 3 };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };
	// set cnv_pin to low
	gpio_set(cnv_pin, 0);

	memset(rx_buf, 0, 3);

	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI failed: %d", ret);
		return;
	}

	memcpy(out_buf, rx_buf, 3);
	/*
	(Read_back 16 bits data / 0xFFFF ) * Vref 
	example: 0b 62 83
	get 0xb628
	0xb628 / 0xffff = (0xb628 / 0xffff) * 3.3
	*/
	uint8_t high = (uint8_t)((out_buf[0] << 4) | (out_buf[1] >> 4));
	uint8_t low = (uint8_t)(((out_buf[1] & 0x0F) << 4) | (out_buf[2] >> 4));

	uint16_t raw_value = (uint16_t)((high << 8) | low);
	if (idx == ADC_EL_IDX_NUWA0) {
		ad4058_val_0 = (float)raw_value / 65536 * ad4058_vref;
		LOG_INF("NUWA0 ad4058 value: 0x%04x", raw_value);
	} else if (idx == ADC_EL_IDX_NUWA1) {
		ad4058_val_1 = (float)raw_value / 65536 * ad4058_vref;
		LOG_INF("NUWA1 ad4058 value: 0x%04x", raw_value);
	}

	// set cnv_pin to high
	gpio_set(cnv_pin, 1);

	return;
}

void ads7066_mode_init()
{
	//set auto-sequence mode
	// nuwa0 & nuwa1
	for (int i = 0; i < ADC_EL_IDX_MAX; i++) {
		ads7066_write_reg(0, 0x1, i);
		// disable internal Volt reference
		ads7066_write_reg(0x1, 0x2, i);

		ads7066_write_reg(0x12, 0x1, i);
		ads7066_write_reg(0x3, 0x6, i);

		//check and update good status
		uint8_t value = 0;
		ads7066_read_reg(0x3, i, &value);
		adc_good_status[i] = (value & 0x07) == 0x06 ? 0 : 0xFF;

		ads7066_write_reg(0x4, 0x8, i);
		ads7066_write_reg(0x10, 0x11, i);
		ads7066_write_reg(0x2, 0x10, i);
	}
	LOG_INF("ads7066 mode init done");
}

void ad4058_mode_init()
{
	/*
		set ad4058 to burst averaging mode 
		sample rate: 300KHz = 3.33us
		Averaging ratio: 256
		3.33us * 256 = 0.8ms per sample
	*/
	for (int i = 0; i < ADC_EL_IDX_MAX; i++) {
		// exit to config mode, check product id
		ad4058_write_reg(0xA8, 0x00, i);
		uint8_t value = 0;
		ad4058_read_reg(0x03, i, &value);
		adc_good_status[i] = (value & 0x0F) == 0x07 ? 0 : 0xFF;

		ad4058_write_reg(0x27, 0x20, i);
		ad4058_write_reg(0x23, 0x7, i);
		ad4058_write_reg(0x21, 0x1, i);
		ad4058_write_reg(0x20, 0x1, i);
	}
	LOG_INF("ad4058 mode init done");
}

void adc_electra_polling_handler(void *p1, void *p2, void *p3)
{
	read_adc_info();
	LOG_INF("adc index is %d", adc_idx_read);
	if (adc_idx_read == ADI_AD4058)
		ad4058_mode_init();
	else if (adc_idx_read == TIC_ADS7066)
		ads7066_mode_init();
	else
		LOG_ERR("Invalid ADC index %d", adc_idx_read);

	while (1) {
			switch (adc_idx_read) {
			case ADI_AD4058:
				ad4058_read_voltage(ADC_EL_IDX_NUWA0);
				ad4058_read_voltage(ADC_EL_IDX_NUWA1);
				break;
			case TIC_ADS7066:
				ads7066_read_voltage(ADC_EL_IDX_NUWA0);
				ads7066_read_voltage(ADC_EL_IDX_NUWA1);
				break;
			default:
				LOG_DBG("Invalid ADC index %d", adc_idx_read);
				break;
			}
		k_msleep(2000);
	}
}

void plat_adc_electra_init(void)
{
	k_thread_create(&adc_electra_poll_thread, adc_electra_thread_stack, ADC_STACK_SIZE,
			adc_electra_polling_handler, NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY,
			0, K_NO_WAIT);

	k_thread_name_set(&adc_electra_poll_thread, "platform adc(electra) read");

	LOG_INF("ADC(electra) polling thread started...\n");
}