

#ifndef ADC_H
#define ADC_H

#include <stdint.h>


//
// Function prototypes
//

void adc_start(void);

void adc_select_chanel(uint32_t chanel);

uint32_t adc_read(void);

#endif
