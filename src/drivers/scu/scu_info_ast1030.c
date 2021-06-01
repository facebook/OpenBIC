#include <stdint.h>
#include <stdio.h>
#include "common.h"
#include "device.h"
#include "log.h"
#include "scu_info_aspeed.h"

/* SoC mapping Table */

#define ASPEED_REVISION_ID0	0x04
#define ASPEED_REVISION_ID1	0x14

#define SOC_ID(str, rev) { .name = str, .rev_id = rev, }

struct soc_id {
        const char *name;
        uint64_t rev_id;
};

static struct soc_id soc_map_table[] = {
        SOC_ID("AST1030-A0", 0x8000000080000000),
};

void aspeed_print_soc_id(void)
{
        int i;
        uint64_t rev_id;

        rev_id = readl(SCU_BASE +ASPEED_REVISION_ID0);
        rev_id = ((uint64_t)readl(SCU_BASE +ASPEED_REVISION_ID1) << 32) | rev_id;

        for (i = 0; i < ARRAY_SIZE(soc_map_table); i++) {
                if (rev_id == soc_map_table[i].rev_id)
                        break;
        }
        if (i == ARRAY_SIZE(soc_map_table))
                printf("UnKnow-SOC: %llx\n", rev_id);
        else
                printf("SOC: %4s \n",soc_map_table[i].name);
}

#define ASPEED_SYS_RESET_EVENT0		0x74
	
#define SYS_WDT4_SW_RESET	BIT(31)
#define SYS_WDT4_ARM_RESET	BIT(30)
#define SYS_WDT4_FULL_RESET	BIT(29)
#define SYS_WDT4_SOC_RESET	BIT(28)
#define SYS_WDT3_SW_RESET	BIT(27)
#define SYS_WDT3_ARM_RESET	BIT(26)
#define SYS_WDT3_FULL_RESET	BIT(25)
#define SYS_WDT3_SOC_RESET	BIT(24)
#define SYS_WDT2_SW_RESET	BIT(23)
#define SYS_WDT2_ARM_RESET	BIT(22)
#define SYS_WDT2_FULL_RESET	BIT(21)
#define SYS_WDT2_SOC_RESET	BIT(20)
#define SYS_WDT1_SW_RESET	BIT(19)
#define SYS_WDT1_ARM_RESET	BIT(18)
#define SYS_WDT1_FULL_RESET	BIT(17)
#define SYS_WDT1_SOC_RESET	BIT(16)

#define SYS_LPC_RESET		BIT(5)
#define SYS_ESPI_RESET		BIT(4)
		
#define SYS_FLASH_ABR_RESET	BIT(2)
#define SYS_EXT_RESET		BIT(1)
#define SYS_PWR_RESET_FLAG	BIT(0)

#define BIT_WDT_SOC(x)	SYS_WDT ## x ## _SOC_RESET
#define BIT_WDT_FULL(x)	SYS_WDT ## x ## _FULL_RESET
#define BIT_WDT_ARM(x)	SYS_WDT ## x ## _ARM_RESET
#define BIT_WDT_SW(x)	SYS_WDT ## x ## _SW_RESET

#define HANDLE_WDTx_RESET(x, event_log, event_log_reg) \
	if (event_log & (BIT_WDT_SOC(x) | BIT_WDT_FULL(x) | BIT_WDT_ARM(x) | BIT_WDT_SW(x))) { \
		printf("RST: WDT%d ", x); \
		if (event_log & BIT_WDT_SOC(x)) { \
			printf("SOC "); \
			SCU_WR(event_log_reg, BIT_WDT_SOC(x)); \
		} \
		if (event_log & BIT_WDT_FULL(x)) { \
			printf("FULL "); \
			SCU_WR(event_log_reg, BIT_WDT_FULL(x)); \
		} \
		if (event_log & BIT_WDT_ARM(x)) { \
			printf("ARM "); \
			SCU_WR(event_log_reg, BIT_WDT_ARM(x)); \
		} \
		if (event_log & BIT_WDT_SW(x)) { \
			printf("SW "); \
			SCU_WR(event_log_reg, BIT_WDT_SW(x)); \
		} \
		printf("\n"); \
	} \
	(void)(x)


#define ASPEED_SYS_RESET_EVENT1		0x78

void aspeed_print_sysrst_info(void)
{
	uint32_t rest = readl(SCU_BASE +ASPEED_SYS_RESET_EVENT0);
//	uint32_t rest1 = readl(SCU_BASE +ASPEED_SYS_RESET_EVENT1);

	if (rest & SYS_PWR_RESET_FLAG) {
		printf("RST: Power On \n");
		SCU_WR(ASPEED_SYS_RESET_EVENT0, rest);
	} else {
		HANDLE_WDTx_RESET(4, rest, ASPEED_SYS_RESET_EVENT0);
		HANDLE_WDTx_RESET(3, rest, ASPEED_SYS_RESET_EVENT0);
		HANDLE_WDTx_RESET(2, rest, ASPEED_SYS_RESET_EVENT0);
		HANDLE_WDTx_RESET(1, rest, ASPEED_SYS_RESET_EVENT0);

	
		if (rest & SYS_LPC_RESET) {
			printf("LPCRST# ");
		}

		if (rest & SYS_ESPI_RESET) {
			printf("ESPIRST# ");
		}

		if (rest & SYS_FLASH_ABR_RESET) {
			printf("RST: SYS_FLASH_ABR_RESET \n");
			SCU_WR(ASPEED_SYS_RESET_EVENT0, SYS_FLASH_ABR_RESET);		
		}
		if (rest & SYS_EXT_RESET) {
			printf("RST: External \n");
			SCU_WR(ASPEED_SYS_RESET_EVENT0, SYS_EXT_RESET);
		}	
	}
}

#define ASPEED_SB_STS		0x14
#define ASPEED_OTP_QSR		0x40

void aspeed_print_security_info(void)
{
	uint32_t qsr = readl(SECBOOT_BASE + ASPEED_OTP_QSR);
	uint32_t sb_sts = readl(SECBOOT_BASE + ASPEED_SB_STS);
	uint32_t hash;
	uint32_t rsa;
	char alg[20];

	if (!(sb_sts & BIT(6)))
		return;
	printf("Secure Boot: ");
	if (qsr & BIT(7)) {
		hash = (qsr >> 10) & 3;
		rsa = (qsr >> 12) & 3;

		if (qsr & BIT(27)) {
			sprintf(alg + strlen(alg), "AES_");
		}
		switch (rsa) {
		case 0:
			sprintf(alg + strlen(alg), "RSA1024_");
			break;
		case 1:
			sprintf(alg + strlen(alg), "RSA2048_");
			break;
		case 2:
			sprintf(alg + strlen(alg), "RSA3072_");
			break;
		default:
			sprintf(alg + strlen(alg), "RSA4096_");
			break;
		}
		switch (hash) {
		case 0:
			sprintf(alg + strlen(alg), "SHA224");
			break;
		case 1:
			sprintf(alg + strlen(alg), "SHA256");
			break;
		case 2:
			sprintf(alg + strlen(alg), "SHA384");
			break;
		default:
			sprintf(alg + strlen(alg), "SHA512");
			break;
		}
		printf("Mode_2, %s\n", alg);
	} else {
		printf("Mode_GCM\n");
		return;
	}
}

#define ASPEED_HW_STRAP2	0x510
#define ASPEED_HW_STRAP1	0x500


#define ASPEED_FMC_WDT2		0x7e620064

void aspeed_print_2nd_wdt_mode(void)
{
	/* ABR enable */
	if (readl(SCU_BASE +ASPEED_HW_STRAP2) & BIT(11)) {
		printf("FMC 2nd Boot (ABR): Enable");
		if (readl(SCU_BASE +ASPEED_HW_STRAP2) & BIT(12))
			printf(", Single flash");
		else
			printf(", Dual flashes");

		printf(", Source: %s", \
				REG_RD(ASPEED_FMC_WDT2) & BIT(4) ? "Alternate" : "Primary");

		if (readl(SCU_BASE +ASPEED_HW_STRAP2) & GENMASK(15, 13))
			printf(", bspi_size: %ld MB", \
				BIT((readl(SCU_BASE +ASPEED_HW_STRAP2) >> 13) & 0x7));

		printf("\n");
	}
}

#define ASPEED_GPIO_YZ_DATA	0x6e7801e0

void aspeed_print_fmc_aux_ctrl(void)
{

	if (readl(SCU_BASE +ASPEED_HW_STRAP2) & BIT(22)) {
		printf("FMC aux control: Enable");
		/* gpioY6 : BSPI_ABR */
		if (REG_RD(ASPEED_GPIO_YZ_DATA) & BIT(6))
			printf(", Force Alt boot");

		/* gpioY7 : BSPI_WP_N */
		if (!(REG_RD(ASPEED_GPIO_YZ_DATA) & BIT(7)))
			printf(", BSPI_WP: Enable");

		if (!(REG_RD(ASPEED_GPIO_YZ_DATA) & BIT(7)) && \
			(readl(SCU_BASE +ASPEED_HW_STRAP2) & GENMASK(24, 23)) != 0) {
			printf(", FMC HW CRTM: Enable, size: %ld KB", \
					BIT((readl(SCU_BASE +ASPEED_HW_STRAP2) >> 23) & 0x3) * 128);
		}

		printf("\n");
	}
}

#define ASPEED_SPI1_BOOT_CTRL	0x6e630064

void aspeed_print_spi1_abr_mode(void)
{
	if (readl(SCU_BASE +ASPEED_HW_STRAP2) & BIT(16)) {
		printf("SPI1 ABR: Enable");
		if(REG_RD(ASPEED_SPI1_BOOT_CTRL) & BIT(6))
			printf(", Single flash");
		else
			printf(", Dual flashes");

		printf(", Source : %s", \
				REG_RD(ASPEED_SPI1_BOOT_CTRL) & BIT(4) ? "Alternate" : "Primary");

		if (REG_RD(ASPEED_SPI1_BOOT_CTRL) & GENMASK(3, 1))
			printf(", hspi_size : %ld MB", \
				BIT((REG_RD(ASPEED_SPI1_BOOT_CTRL) >> 1) & 0x7));

		printf("\n");
	}

	if (readl(SCU_BASE +ASPEED_HW_STRAP2) & BIT(17)) {
		printf("SPI1 select pin: Enable");
		/* gpioZ1 : HSPI_ABR */
		if (REG_RD(ASPEED_GPIO_YZ_DATA) & BIT(9))
			printf(", Force Alt boot");

		printf("\n");
	}
}

void aspeed_print_spi1_aux_ctrl(void)
{
	if (readl(SCU_BASE +ASPEED_HW_STRAP2) & BIT(27)) {
		printf("SPI1 aux control: Enable");
		/* gpioZ1 : HSPI_ABR */
		if (REG_RD(ASPEED_GPIO_YZ_DATA) & BIT(9))
			printf(", Force Alt boot");

		/* gpioZ2: BSPI_WP_N */
		if (!(REG_RD(ASPEED_GPIO_YZ_DATA) & BIT(10)))
			printf(", HPI_WP: Enable");

		if (!(REG_RD(ASPEED_GPIO_YZ_DATA) & BIT(10)) && \
			(readl(SCU_BASE +ASPEED_HW_STRAP2) & GENMASK(26, 25)) != 0) {
			printf(", SPI1 HW CRTM: Enable, size: %ld KB", \
					BIT((readl(SCU_BASE +ASPEED_HW_STRAP2) >> 25) & 0x3) * 128);
		}

		printf("\n");
	}
}

void aspeed_print_spi_strap_mode(void)
{
	if(readl(SCU_BASE +ASPEED_HW_STRAP2) & BIT(10))
		printf("SPI: 3/4 byte mode auto detection \n");
}

void aspeed_print_espi_mode(void)
{
	int espi_mode = 0;
	int sio_disable = 0;
	uint32_t sio_addr = 0x2e;

	if (readl(SCU_BASE +ASPEED_HW_STRAP2) & BIT(6))
		espi_mode = 0;
	else
		espi_mode = 1;

	if (readl(SCU_BASE +ASPEED_HW_STRAP2) & BIT(2))
		sio_addr = 0x4e;

	if (readl(SCU_BASE +ASPEED_HW_STRAP2) & BIT(3))
		sio_disable = 1;

	if (espi_mode)
		printf("eSPI Mode: SIO:%s ", sio_disable ? "Disable" : "Enable");
	else
		printf("LPC Mode: SIO:%s ", sio_disable ? "Disable" : "Enable");

	if (!sio_disable)
		printf(": SuperIO-%02x\n", sio_addr);
	else
		printf("\n");
}

