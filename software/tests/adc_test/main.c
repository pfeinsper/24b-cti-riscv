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
#include <float.h>
#include <math.h>
#include <stdio.h>
#include "neorv32_zfinx_extension_intrinsics.h"

/** UART BAUD rate */
#define BAUD_RATE 19200

const float_conv_t conversion_factor = {.float_value = 3.3f / (1 << 12)};


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

  // print the conversion factor
  neorv32_uart0_printf("Conversion factor: %u\n", conversion_factor.float_value);

    // Intro
  neorv32_uart0_puts("ADC functions test.\n\n");

  // clear GPIO output (set all bits to 0)
  neorv32_gpio_port_set(0);

  adc_start();
  adc_select_chanel(0);
 

  while (1) {
    // pick the value of the last 12 bits of the gpio input (31 downto 20)
    uint32_t adc = riscv_intrinsic_fmuls(adc_read(), conversion_factor.float_value);

    // print the value of the last 12 bits of the gpio input
    neorv32_uart0_printf("ADC: %u\n", adc);

    // wait a little
    for (volatile int i=0; i<1000000; i++) { }
    
  }

  // this should never be reached
  return 0;
}
