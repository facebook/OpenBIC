enum {
	ADC_IDX_MEDHA0_1 = 0, // 20ms
	ADC_IDX_MEDHA1_1, // 60ms
	ADC_IDX_MEDHA0_2, // 600ms
	ADC_IDX_MEDHA1_2, // 800ms
	ADC_IDX_MAX,
};

#define ADC_AVERGE_TIMES_MIN 2
#define ADC_AVERGE_TIMES_MAX 200

void plat_adc_init(void);
void adc_set_averge_times(uint8_t idx, uint16_t time);
void adc_set_poll_flag(uint8_t onoff);
bool adc_get_poll_flag();
uint16_t get_adc_averge_val(uint8_t idx);
uint16_t *get_adc_buf(uint8_t idx);
uint16_t get_adc_averge_times(uint8_t idx);