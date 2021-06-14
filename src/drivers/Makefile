-include $(KCONFIG_CONFIG)

obj-y += scu/
obj-y += pinctrl/
obj-y += clk/
obj-y += timer/
obj-y += cache/
obj-y += reset_aspeed.o irq_aspeed.o
obj-$(CONFIG_DEVICE_SERIAL) += serial/
obj-$(CONFIG_DEVICE_GPIO) += gpio/
obj-$(CONFIG_DEVICE_SGPIOM) += sgpiom/
obj-$(CONFIG_DEVICE_I3C) += i3c/
obj-$(CONFIG_DEVICE_I2C) += i2c/
obj-$(CONFIG_DEVICE_IPI) += ipi_ast2600.o
obj-$(CONFIG_DEVICE_ESPI) += espi/
obj-$(CONFIG_DEVICE_PECI) += peci/
obj-$(CONFIG_DEVICE_PWM_TACH) += pwm_tach/
obj-$(CONFIG_DEVICE_ADC) += adc/
obj-$(CONFIG_DEVICE_JTAG) += jtag/
obj-$(CONFIG_DEVICE_KCS) += lpc/aspeed_kcs.o
obj-$(CONFIG_DEVICE_BT) += lpc/aspeed_bt.o
obj-$(CONFIG_DEVICE_SNOOP) += lpc/aspeed_snoop.o
obj-$(CONFIG_DEVICE_PCC) += lpc/aspeed_pcc.o
obj-$(CONFIG_DEVICE_USB) += usb/
obj-$(CONFIG_DEVICE_MAC) += mac_aspeed.o
obj-$(CONFIG_DEVICE_MDIO) += mdio/
obj-$(CONFIG_DEVICE_FMC_SPI) += fmc_spi/
obj-$(CONFIG_DEVICE_CRYPTO) += crypto/

