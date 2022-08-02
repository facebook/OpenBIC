#include <stdint.h>

#define SSD_TURN_OFF 0x00
#define SSD_TURN_ON 0x01
#define SSD_START_BLINK 0x02
#define SSD_STOP_BLINK 0x03

void SSDLEDSet(uint8_t idx, uint8_t behaviour);
uint8_t SSDLEDCtrl(uint8_t idx, uint8_t ctrl);
uint8_t GetAmberLEDStat(uint8_t idx);
void SSDLEDInit(void);