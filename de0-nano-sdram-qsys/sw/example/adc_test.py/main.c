// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //


/**********************************************************************//**
 * @file demo_blink_led/main.c
 * @author Stephan Nolting
 * @brief Minimal blinking LED demo program using the lowest 8 bits of the GPIO.output port.
 **************************************************************************/
#include <neorv32.h>

/** UART BAUD rate */
#define BAUD_RATE 19200


/**********************************************************************//**
 * Main function; shows an incrementing 8-bit counter on GPIO.output(7:0).
 *
 * @note This program requires the GPIO controller to be synthesized.
 *
 * @return Will never return.
 **************************************************************************/
int main() {

    // setup NEORV32 runtime environment (for trap handling)
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

    // Intro
  neorv32_uart0_puts("ADC functions test.\n\n");

  // clear GPIO output (set all bits to 0)
  neorv32_gpio_port_set(0);

  neorv32_gpio_pin_set(32, 1);

  int cnt = 0;

  while (1) {
    // pick the value of the last 12 bits of the gpio input (32 downto 21)
    uint8_t bit_12 = neorv32_gpio_pin_get(32);
    uint8_t bit_11 = neorv32_gpio_pin_get(30);
    uint8_t bit_10 = neorv32_gpio_pin_get(29);
    uint8_t bit_9 = neorv32_gpio_pin_get(28);
    uint8_t bit_8 = neorv32_gpio_pin_get(27);
    uint8_t bit_7 = neorv32_gpio_pin_get(26);
    uint8_t bit_6 = neorv32_gpio_pin_get(25);
    uint8_t bit_5 = neorv32_gpio_pin_get(24);
    uint8_t bit_4 = neorv32_gpio_pin_get(23);
    uint8_t bit_3 = neorv32_gpio_pin_get(22);
    uint8_t bit_2 = neorv32_gpio_pin_get(21);
    uint8_t bit_1 = neorv32_gpio_pin_get(20);
    uint8_t bit_0 = neorv32_gpio_pin_get(19);

    // print the value of the last 12 bits of the gpio input
    neorv32_uart0_printf("ADC value: %d %d %d %d %d %d %d %d %d %d %d %d\n", bit_12, bit_11, bit_10, bit_9, bit_8, bit_7, bit_6, bit_5, bit_4, bit_3, bit_2, bit_1, bit_0);

    // wait a little
    for (volatile int i=0; i<1000000; i++) { }
    
  }

  // this should never be reached
  return 0;
}
