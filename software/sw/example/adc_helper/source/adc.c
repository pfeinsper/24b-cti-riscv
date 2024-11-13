

#include "neorv32.h"


/**********************************************************************//**
 * Check if GPIO unit was synthesized.
 *
 * @return 0 if GPIO was not synthesized, 1 if GPIO is available.
 **************************************************************************/
void adc_start(void) {
    neorv32_gpio_pin_set(31, 1);
    return;
}

void adc_select_chanel(uint32_t chanel) {
    // set 3 bits of the gpio output (30 downto 28) to the chanel
    neorv32_gpio_port_set(chanel << 28);
    return;
}

uint32_t adc_read(void) {
    // pick the value of the last 12 bits of the gpio input (31 downto 20)
    uint32_t gpio_in = neorv32_gpio_port_get();
    // ADC = 12 most significant bits of the gpio input (31 downto 20)
    uint32_t adc = gpio_in >> 20;
    return adc;
}