/* AST2600 register base address */
#define PWM_TACH_BASE		0x7e610000
#define MDIO0_BASE			0x7e650000
#define MDIO1_BASE			0x7e650008
#define MDIO2_BASE			0x7e650010
#define MDIO3_BASE			0x7e650018
#define I3CDMA_BASE			0x7e651000
#define MAC0_BASE			0x7e660000
#define MAC1_BASE			0x7e680000
#define MAC2_BASE			0x7e670000
#define MAC3_BASE			0x7e690000
#define VIC_BASE			0x7e6c0000
#define MMC_BASE			0x7e6e0000
#define SCU_BASE			0x7e6e2000
#define ADC0_BASE			0x7e6e9000
#define ADC1_BASE			0x7e6e9100
#define JTAG0_BASE			0x7e6e4000
#define JTAG1_BASE			0x7e6e4100
#define ESPI_BASE			0x7e6ee000
#define SECBOOT_BASE		0x7e6f2000
#define GPIO_BASE			0x7e780000
#define SGPIOM0_BASE		0x7e780500
#define SGPIOM1_BASE		0x7e780600
#define GPIO1P8_BASE		0x7e780800
#define TMC_BASE			0x7e782000
#define UART0_BASE			0x7e783000
#define UART1_BASE			0x7e78d000
#define UART2_BASE			0x7e78e000
#define UART3_BASE			0x7e78f000
#define UART4_BASE			0x7e784000
#define UART5_BASE			0x7e790000
#define UART6_BASE			0x7e790100
#define UART7_BASE			0x7e790200
#define UART8_BASE			0x7e790300
#define UART9_BASE			0x7e790400
#define UART10_BASE			0x7e790500
#define UART11_BASE			0x7e790600
#define UART12_BASE			0x7e790700
#define UDMA_BASE			0x7e79e000

#define WDT0_BASE			0x7e785000

#define LPC_BASE			0x7e789000

#define I2C_GLOBAL_BASE 	0x7e78a000
#define I2C0_BASE			0x7e78a080
#define I2C1_BASE			0x7e78a100
#define I2C2_BASE			0x7e78a180
#define I2C3_BASE			0x7e78a200
#define I2C4_BASE			0x7e78a280
#define I2C5_BASE			0x7e78a300
#define I2C6_BASE			0x7e78a380
#define I2C7_BASE			0x7e78a400
#define I2C8_BASE			0x7e78a480
#define I2C9_BASE			0x7e78a500
#define I2C10_BASE			0x7e78a580
#define I2C11_BASE			0x7e78a600
#define I2C12_BASE			0x7e78a680
#define I2C13_BASE			0x7e78a700
#define I2C14_BASE			0x7e78a780
#define I2C15_BASE			0x7e78a800

#define I2C0_BUFF_BASE		0x7e78ac00
#define I2C1_BUFF_BASE		0x7e78ac20
#define I2C2_BUFF_BASE		0x7e78ac40
#define I2C3_BUFF_BASE		0x7e78ac60
#define I2C4_BUFF_BASE		0x7e78ac80
#define I2C5_BUFF_BASE		0x7e78aca0
#define I2C6_BUFF_BASE		0x7e78acc0
#define I2C7_BUFF_BASE		0x7e78ace0
#define I2C8_BUFF_BASE		0x7e78ad00
#define I2C9_BUFF_BASE		0x7e78ad20
#define I2C10_BUFF_BASE		0x7e78ad40
#define I2C11_BUFF_BASE		0x7e78ad60
#define I2C12_BUFF_BASE		0x7e78ad80
#define I2C13_BUFF_BASE		0x7e78ada0
#define I2C14_BUFF_BASE		0x7e78adc0
#define I2C15_BUFF_BASE		0x7e78ade0

#define PECI_BASE			0x7e78b000

#define USB_BASE			0x7e6a2000

#define I2CSEC_GLOBAL_BASE 	0x7e7a8000
#define I2CSEC0_BASE		0x7e7a8080
#define I2CSEC1_BASE		0x7e7a8100
#define I2CSEC2_BASE		0x7e7a8180
#define I2CSEC3_BASE		0x7e7a8200

#define I2CSEC0_BUFF_BASE	0x7e7a8c00
#define I2CSEC1_BUFF_BASE	0x7e7a8c20
#define I2CSEC2_BUFF_BASE	0x7e7a8c40
#define I2CSEC3_BUFF_BASE	0x7e7a8c60

#define I3C_GLOBAL_BASE 	0x7e7a0000
#define I3C0_BASE			0x7e7a2000
#define I3C1_BASE			0x7e7a3000
#define I3C2_BASE			0x7e7a4000
#define I3C3_BASE			0x7e7a5000
#define I3C4_BASE			0x7e7a6000
#define I3C5_BASE			0x7e7a7000

#define FMC_BASE			0x7e620000
#define SPI1_BASE			0x7e630000
#define SPI2_BASE			0x7e631000

#define HACE_BASE			0x7e6d0000
#define SEC_BASE			0x7e6f2000

/* macros to access registers */
#define REG_RD(addr) 	*(volatile uint32_t *)(addr)
#define REG_WR(addr, value) (*(volatile uint32_t *)(addr) = (uint32_t)value)
#define SCU_RD(offset) 	REG_RD(SCU_BASE + offset)
#define SCU_WR(offset, value) REG_WR(SCU_BASE + offset, value)
#define LPC_RD(offset)	REG_RD(LPC_BASE + offset)
#define LPC_WR(offset, value)	REG_WR(LPC_BASE + offset, value)

#define PHY_DRAM_ADDR		SCU_RD(0xa04)
#define TO_PHY_ADDR(addr)	(PHY_DRAM_ADDR + (uint32_t)(addr))
#define TO_VIR_ADDR(addr)	((addr) - PHY_DRAM_ADDR)
