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

  neorv32_gpio_pin_set(31, 1);


  while (1) {
    // pick the value of the last 12 bits of the gpio input (31 downto 20)
    uint32_t gpio_in = neorv32_gpio_port_get();
    // ADC = 12 most significant bits of the gpio input (31 downto 20)
    uint32_t adc = gpio_in >> 20;

    // print the value of the last 12 bits of the gpio input
    neorv32_uart0_printf("ADC: %u\n", adc);

    // wait a little
    for (volatile int i=0; i<1000000; i++) { }
    
  }

  // this should never be reached
  return 0;
}
