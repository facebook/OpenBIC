ifneq ("$(wildcard .config)","")
include .config
KCONFIG_CONFIG ?= $(TOP_DIR)/.config
export KCONFIG_CONFIG
endif

VENDOR = $(subst ",,$(CONFIG_SOC_VENDOR))
CPU=$(subst ",,$(CONFIG_CPU_ID))
ARCH=$(subst ",,$(CONFIG_CPU_ARCH))
FPU=$(subst ",,$(CONFIG_FPU_SELECT))
ifeq ($(CONFIG_AST2600_SERIES), y)
SOC = ast2600
else
SOC = ast1030
endif

OS = FreeRTOS-Kernel
ifeq ($(CPU), cortex-m3)
OS_PORT = ARM_CM3
else ifeq ($(CPU), cortex-m4)
OS_PORT = ARM_CM4F
endif
export OS_PORT

CC 		:= $(CROSS_COMPILE)gcc
AS		:= $(CROSS_COMPILE)as
LD 		:= $(CROSS_COMPILE)ld
OBJCOPY := $(CROSS_COMPILE)objcopy
SIZE    := $(CROSS_COMPILE)size
NM		:= $(CROSS_COMPILE)nm
export CC AS LD OBJCOPY SIZE NM

TOP_DIR := $(shell pwd)
BIN_DIR := $(TOP_DIR)/bin
COM_DIR := $(TOP_DIR)/../common
PROJ_DIR := $(TOP_DIR)/../meta-facebook/$(subst ",,$(CONFIG_BOARD_NAME))
export TOP_DIR BIN_DIR

DEV_DIR = $(TOP_DIR)/arch/arm/$(VENDOR)/$(SOC)
APP_DIR = $(TOP_DIR)/app
UTIL_DIR = $(TOP_DIR)/util
DRV_DIR = $(TOP_DIR)/drivers
LIB_DIR = $(TOP_DIR)/libraries
BOARD_DIR = $(TOP_DIR)/board
OS_DIR = $(TOP_DIR)/os
OS_PORT_DIR = $(OS_DIR)/$(OS)/portable/GCC/$(OS_PORT)

INC += 	-I $(TOP_DIR)/include \
		-I $(TOP_DIR)/arch/arm/cmsis_core \
		-I $(TOP_DIR)/drivers/include
INC += 	-I $(DEV_DIR)/include
INC +=  -I $(COM_DIR)/ \
		-I $(COM_DIR)/util \
		-I $(COM_DIR)/usb \
		-I $(COM_DIR)/host \
		-I $(COM_DIR)/IPMB \
		-I $(COM_DIR)/IPMB/include \
		-I $(COM_DIR)/sensor \
		-I $(COM_DIR)/sensor/dev 
INC +=  -I $(PROJ_DIR)/ \
		-I $(PROJ_DIR)/sensor \
		-I $(PROJ_DIR)/IPMB \
		-I $(PROJ_DIR)/IPMB/include \
		-I $(PROJ_DIR)/platform 
INC += 	-I $(LIB_DIR)/aspeed/include \
		-I $(LIB_DIR)/freertos_plus/freertos_plus_cli
INC +=  -I $(LIB_DIR)/log/include
INC +=  -I $(LIB_DIR)/CmBacktrace/cm_backtrace
INC +=  -I $(LIB_DIR)/clib/include
INC += 	-I $(OS_DIR)/$(OS)/include
INC += -I $(OS_DIR)/cmsis
INC += 	-I $(OS_PORT_DIR)
INC += -I $(BOARD_DIR)/$(subst ",,$(CONFIG_BOARD_NAME))/include
INC +=  -I $(INC_PATH)
export INC

obj-y += $(DEV_DIR)/
obj-y += $(COM_DIR)/
obj-y += $(PROJ_DIR)/
obj-y += $(DRV_DIR)/
obj-y += $(BOARD_DIR)/
obj-y += $(LIB_DIR)/	  
obj-y += $(OS_DIR)/
obj-y += $(APP_DIR)/

TARGET_ELF := $(SOC).elf
TARGET_BIN := $(subst ",,$(CONFIG_IMG_NAME))

LDSCRIPTS := $(DEV_DIR)/gcc/$(SOC).ld

CFLAGS 	:= -Wall -mcpu=$(CPU) -mthumb -march=$(ARCH) $(FPU) -g -Og \
	   -nostdlib -std=c99 -fno-builtin -Wformat=0 \
	   -Wno-stringop-overflow \
	   -Wno-nonnull-compare \
	   -Wno-discarded-qualifiers
LDFLAGS = -T $(LDSCRIPTS) -EL -Map $(SOC).lst --print-memory-usage
LDFLAGS += --wrap=printf \
		--wrap=sprintf \
		--wrap=snprintf \
		--wrap=vprintf \
		--wrap=vsnprintf \
		--wrap=malloc \
		--wrap=free
export CFLAGS LDFLAGS

.PHONY: all clean menuconfig %_defconfig image secure_boot otp venv

ifeq ($(CONFIG_SB_ENABLE), y)
all: image secure_boot
else
all: image
endif

image: check
	make -f $(TOP_DIR)/tools/Makefile.build
	$(LD) $(LDFLAGS) -o $(TARGET_ELF) built-in.o
	$(OBJCOPY) -O binary $(TARGET_ELF) $(TARGET_BIN)
	$(SIZE) -A -d -x $(TARGET_ELF)
	$(NM) --print-size --size-sort -r --radix=d -l $(TARGET_ELF) > memory_analyze.log
	mkdir -p $(BIN_DIR)
	mv $(TARGET_ELF) $(TARGET_BIN) $(SOC).lst $(BIN_DIR)
	$(TOP_DIR)/tools/gen_uart_booting_image.sh $(BIN_DIR)/$(TARGET_BIN) $(BIN_DIR)/uart_$(TARGET_BIN)
	@if [ -f "$(BIN_DIR)/sec_$(TARGET_BIN)" ]; then \
		$(TOP_DIR)/tools/gen_uart_booting_image.sh $(BIN_DIR)/sec_$(TARGET_BIN) $(BIN_DIR)/uart_sec_$(TARGET_BIN); \
	fi
#	scp $(BIN_DIR)/$(TARGET_BIN) wsl:/mnt/d/tftpboot
#	cp $(BIN_DIR)/$(TARGET_BIN) /mnt/d/tftpboot

ifeq ($(CONFIG_SB_ENABLE), y)
secure_boot: otp venv
	@SIGNING_HELPER_PATH="";\
	SIGNING_HELPER_WF_PATH="";\
	RSA_KEY_PATH="";\
	GCM_KEY_PATH="";\
	AES_KEY_PATH="";\
	RSA_AES_KEY_PATH="";\
	if [ "$(SIGNING_HELPER)" != "" ] ; then \
		SIGNING_HELPER_PATH=$$(realpath $(SIGNING_HELPER)); \
	fi; \
	if [ "$(SIGNING_HELPER_WITH_FILES)" != "" ] ; then \
		SIGNING_HELPER_WF_PATH=$$(realpath $(SIGNING_HELPER_WITH_FILES)); \
	fi; \
	if [ "$(CONFIG_SB_RSA_KEY)" != "" ] ; then \
		RSA_KEY_PATH=$$(realpath $(CONFIG_SB_RSA_KEY)); \
	fi;\
	if [ "$(CONFIG_SB_GCM_AES_KEY)" != "" ] ; then \
		GCM_KEY_PATH=$$(realpath $(CONFIG_SB_GCM_AES_KEY)); \
	fi;\
	if [ "$(CONFIG_SB_AES_KEY)" != "" ] ; then \
		AES_KEY_PATH=$$(realpath $(CONFIG_SB_AES_KEY)); \
	fi;\
	if [ "$(CONFIG_SB_RSA_AES_KEY)" != "" ] ; then \
		RSA_AES_KEY_PATH=$$(realpath $(CONFIG_SB_RSA_AES_KEY)); \
	fi;\
	if [ "$(CONFIG_SB_MODE_GCM)" == "y" ] ; then \
		.venv/bin/socsec\
			make_secure_bl1_image\
			--soc 1030\
			--algorithm AES_GCM\
			--bl1_image $(BIN_DIR)/$(TARGET_BIN)\
			--output $(BIN_DIR)/sec_$(TARGET_BIN)\
			--gcm_aes_key $$GCM_KEY_PATH\
			--signing_helper $$SIGNING_HELPER_PATH\
			--signing_helper_with_files $$SIGNING_HELPER_WF_PATH;\
	elif [ "$(CONFIG_SB_MODE_2)" == "y" ] ; then \
		MODE_2_ALG="";\
		if [ "$(CONFIG_SB_MODE_2_AES)" == "y" ] ; then \
			MODE_2_ALG="AES_";\
		fi;\
		if [ "$(CONFIG_SB_RSA_1024)" == "y" ] ; then \
			MODE_2_ALG=$$MODE_2_ALG"RSA1024";\
		elif [ "$(CONFIG_SB_RSA_2048)" == "y" ] ; then \
			MODE_2_ALG=$$MODE_2_ALG"RSA2048";\
		elif [ "$(CONFIG_SB_RSA_3072)" == "y" ] ; then \
			MODE_2_ALG=$$MODE_2_ALG"RSA3072";\
		elif [ "$(CONFIG_SB_RSA_4096)" == "y" ] ; then \
			MODE_2_ALG=$$MODE_2_ALG"RSA4096";\
		fi;\
		if [ "$(CONFIG_SB_SHA_224)" == "y" ] ; then \
			MODE_2_ALG=$$MODE_2_ALG\_"SHA224";\
		elif [ "$(CONFIG_SB_SHA_256)" == "y" ] ; then \
			MODE_2_ALG=$$MODE_2_ALG\_"SHA256";\
		elif [ "$(CONFIG_SB_SHA_384)" == "y" ] ; then \
			MODE_2_ALG=$$MODE_2_ALG\_"SHA384";\
		elif [ "$(CONFIG_SB_SHA_512)" == "y" ] ; then \
			MODE_2_ALG=$$MODE_2_ALG\_"SHA512";\
		fi;\
		echo $$MODE_2_ALG;\
		if [ "$(CONFIG_SB_MODE_2_O1)" == "y" ] ; then \
			.venv/bin/socsec\
				make_secure_bl1_image\
				--soc 1030\
				--algorithm $$MODE_2_ALG\
				--bl1_image $(BIN_DIR)/$(TARGET_BIN)\
				--output $(BIN_DIR)/sec_$(TARGET_BIN)\
				--rsa_sign_key $$RSA_KEY_PATH\
				--key_in_otp\
				--aes_key $$AES_KEY_PATH\
				--signing_helper $$SIGNING_HELPER_PATH\
				--signing_helper_with_files $$SIGNING_HELPER_WF_PATH;\
		elif [ "$(CONFIG_SB_MODE_2_O2)" == "y" ] ; then \
			.venv/bin/socsec\
				make_secure_bl1_image\
				--soc 1030\
				--algorithm $$MODE_2_ALG\
				--bl1_image $(BIN_DIR)/$(TARGET_BIN)\
				--output $(BIN_DIR)/sec_$(TARGET_BIN)\
				--rsa_sign_key $$RSA_KEY_PATH\
				--aes_key $$AES_KEY_PATH\
				--rsa_aes $$RSA_AES_KEY_PATH\
				--signing_helper $$SIGNING_HELPER_PATH\
				--signing_helper_with_files $$SIGNING_HELPER_WF_PATH;\
		else \
			.venv/bin/socsec\
				make_secure_bl1_image\
				--soc 1030\
				--algorithm $$MODE_2_ALG\
				--bl1_image $(BIN_DIR)/$(TARGET_BIN)\
				--output $(BIN_DIR)/sec_$(TARGET_BIN)\
				--rsa_sign_key $$RSA_KEY_PATH\
				--signing_helper $$SIGNING_HELPER_PATH\
				--signing_helper_with_files $$SIGNING_HELPER_WF_PATH;\
		fi \
	fi
	@if [ "$(CONFIG_OTP_MEM)" == "y" ] ; then \
		.venv/bin/socsec verify --soc 1030 --sec_image $(BIN_DIR)/sec_$(TARGET_BIN) --otp_image bin/otp/otp-all.image; \
	fi

	@if [ "$(CONFIG_OTP_FPGA)" == "y" ] ; then \
		tools/fpga_sb_gen.py $(BIN_DIR)/sec_$(TARGET_BIN) bin/otp/otp-conf.bin bin/otp/otp-data.bin; \
	fi

	$(TOP_DIR)/tools/gen_uart_booting_image.sh $(BIN_DIR)/sec_$(TARGET_BIN) $(BIN_DIR)/uart_sec_$(TARGET_BIN)


else
secure_boot:
	@echo "secure boot not enable"
endif

ifeq ($(CONFIG_OTP_MEM), y)
otp: venv
	@OTP_KEY_PATH="";\
	OTP_DATA_PATH="";\
	mkdir -p bin/otp; \
	if [ "$(CONFIG_OTP_KEY_FOLDER)" != "" ] ; then \
		OTP_KEY_PATH=$$(realpath $(CONFIG_OTP_KEY_FOLDER)); \
	fi; \
	if [ "$(CONFIG_OTP_USER_DATA_FOLDER)" != "" ] ; then \
		OTP_DATA_PATH=$$(realpath $(CONFIG_OTP_USER_DATA_FOLDER)); \
	fi; \
	.venv/bin/otptool \
		$(CONFIG_OTP_CONFIG) \
		--key_folder $$OTP_KEY_PATH \
		--user_data_folder $$OTP_DATA_PATH \
		--output_folder bin/otp;
else
otp:
	@echo "otp not enable"
endif

venv:
	@if [ ! -f ".venv/bin/socsec" ]; then \
		virtualenv .venv;\
		. .venv/bin/activate;\
		cd $(TOP_DIR)/tools/socsec;\
		pip3 install -r requirements.txt;\
		python3 setup.py install; \
	fi; \

check:
ifeq ("$(wildcard .config)","")
	$(error ".config not found. run make xxxx_defconfig first")
endif

clean:
	@rm -f $(shell find . -path ./toolchain -prune -o -name "*.o" -print)

dist-clean: clean
	@rm -rf $(BIN_DIR)

cleanconfig:
	@rm -f $(TOP_DIR)/.config*
	@rm -f $(TOP_DIR)/include/config.h

menuconfig: check
	tools/Kconfiglib/menuconfig.py
	make gencfg

savedefconfig:
	tools/Kconfiglib/savedefconfig.py

loadcfg:
	tools/Kconfiglib/defconfig.py $(TOP_DIR)/configs/$(MAKECMDGOALS)

gencfg:
	tools/Kconfiglib/genconfig.py --header-path $(TOP_DIR)/include/config.h
#	mv config.h include/

%_defconfig: loadcfg gencfg clean
	@echo "include/config.h is generated"
