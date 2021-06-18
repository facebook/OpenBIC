#include "device.h"

extern aspeed_device_t uart0;
extern aspeed_device_t uart4;
extern aspeed_device_t uart6;
extern aspeed_device_t uart9;
extern aspeed_device_t uart10;

extern aspeed_device_t gpio0;
extern aspeed_device_t gpio1;

extern aspeed_device_t sgpiom0;
extern aspeed_device_t sgpiom1;

extern aspeed_device_t timer0;

extern aspeed_device_t i2c_global;
extern aspeed_device_t i2c0;
extern aspeed_device_t i2c1;
extern aspeed_device_t i2c2;
extern aspeed_device_t i2c3;
extern aspeed_device_t i2c4;
extern aspeed_device_t i2c5;
extern aspeed_device_t i2c6;
extern aspeed_device_t i2c7;
extern aspeed_device_t i2c8;
extern aspeed_device_t i2c9;
extern aspeed_device_t i2c10;
extern aspeed_device_t i2c11;
extern aspeed_device_t i2c12;
extern aspeed_device_t i2c13;
extern aspeed_device_t i2c14;
extern aspeed_device_t i2c15;

extern aspeed_device_t i2csec_global;
extern aspeed_device_t i2csec0;
extern aspeed_device_t i2csec1;
extern aspeed_device_t i2csec2;
extern aspeed_device_t i2csec3;

extern aspeed_device_t i3c0;
extern aspeed_device_t i3c1;
extern aspeed_device_t i3c2;
extern aspeed_device_t i3c3;
extern aspeed_device_t i3cdma;
extern aspeed_device_t i3c_global;
extern aspeed_device_t ipi;

extern aspeed_device_t peci;

extern aspeed_device_t g_pwm_tach;

extern aspeed_device_t pwm0;
extern aspeed_device_t pwm1;
extern aspeed_device_t pwm2;
extern aspeed_device_t pwm3;
extern aspeed_device_t pwm4;
extern aspeed_device_t pwm5;
extern aspeed_device_t pwm6;
extern aspeed_device_t pwm7;
extern aspeed_device_t pwm8;
extern aspeed_device_t pwm9;
extern aspeed_device_t pwm10;
extern aspeed_device_t pwm11;
extern aspeed_device_t pwm12;
extern aspeed_device_t pwm13;
extern aspeed_device_t pwm14;
extern aspeed_device_t pwm15;

extern aspeed_device_t tach0;
extern aspeed_device_t tach1;
extern aspeed_device_t tach2;
extern aspeed_device_t tach3;
extern aspeed_device_t tach4;
extern aspeed_device_t tach5;
extern aspeed_device_t tach6;
extern aspeed_device_t tach7;
extern aspeed_device_t tach8;
extern aspeed_device_t tach9;
extern aspeed_device_t tach10;
extern aspeed_device_t tach11;
extern aspeed_device_t tach12;
extern aspeed_device_t tach13;
extern aspeed_device_t tach14;
extern aspeed_device_t tach15;

extern aspeed_device_t g_adc;

extern aspeed_device_t adc0;
extern aspeed_device_t adc1;

extern aspeed_device_t jtag0;
extern aspeed_device_t jtag1;

extern aspeed_device_t fmc_dev;
extern aspeed_device_t spi1_dev;
extern aspeed_device_t spi2_dev;

#define STDIO_UART_DEVICE			CONCAT(uart, CONFIG_STDIO_UART)
#if CONFIG_DEVICE_TIMER
#define DELAY_TIMER_DEVICE			CONCAT(timer, CONFIG_DEVICE_DELAY_TIMER)
#endif

extern aspeed_device_t mdio0;
extern aspeed_device_t mdio1;
extern aspeed_device_t mdio2;
extern aspeed_device_t mdio3;

extern aspeed_device_t mac0;
extern aspeed_device_t mac1;
extern aspeed_device_t mac2;
extern aspeed_device_t mac3;