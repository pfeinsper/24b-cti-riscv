// #################################################################################################
// # << NEORV32 - General Purpose Timer (GPTMR) Demo Program >>                                    #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file demo_gptmr/main.c
 * @author Stephan Nolting
 * @brief Simple GPTMR usage example.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
#define IN1 0
#define IN2 1
#define IN3 2
#define EN1 3
#define EN2 4
#define EN3 5
/**@}*/

// uint8_t led_state[6] = {0, 1, 0, 1, 0, 1};

uint8_t in_seq[6][3] = {{1, 0, 0}, {0, 1, 0}, {0, 1, 0},
                        {0, 0, 1}, {0, 0, 1}, {1, 0, 0}};
uint8_t en_seq[6][3] = {{1, 0, 1}, {0, 1, 1}, {1, 1, 0},
                        {1, 0, 1}, {0, 1, 1}, {1, 1, 0}};

volatile uint32_t counter = 0;
volatile uint8_t is_moving = 0;


// Prototypes
void gptmr_firq_handler(void);


/**********************************************************************//**
 * This program blinks an LED at GPIO.output(0) at 1Hz using the general purpose timer interrupt.
 *
 * @note This program requires the GPTMR unit to be synthesized (and UART0 and GPIO).
 *
 * @return Should not return;
 **************************************************************************/
int main() {
  
  // setup NEORV32 runtime environment (for trap handling)
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);


  // check if GPTMR unit is implemented at all
  if (neorv32_gptmr_available() == 0) {
    neorv32_uart0_puts("ERROR! General purpose timer not implemented!\n");
    return 1;
  }

  // Intro
  neorv32_uart0_puts("Basic Motor Spinning.\n"
                     "Activates the control interruption at 100Hz using the GPTMR interrupt.\n\n");


  // clear GPIO output port
  neorv32_gpio_port_set(0);


  // install GPTMR interrupt handler
  neorv32_rte_handler_install(GPTMR_RTE_ID, gptmr_firq_handler);

  // configure timer for 1Hz ticks in continuous mode (with clock divisor = 8)
  neorv32_gptmr_setup(CLK_PRSC_8, neorv32_sysinfo_get_clk() / (8 * 2), 1);

  // enable interrupt
  neorv32_cpu_csr_clr(CSR_MIP, 1 << GPTMR_FIRQ_PENDING);  // make sure there is no GPTMR IRQ pending already
  neorv32_cpu_csr_set(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);   // enable GPTMR FIRQ channel
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts


  // go to sleep mode and wait for interrupt
  while(1) {
    neorv32_cpu_sleep();
  }

  return 0;
}


/**********************************************************************//**
 * GPTMR FIRQ handler.
 *
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void gptmr_firq_handler(void) {

  neorv32_gptmr_irq_ack(); // clear/ack pending FIRQ

  neorv32_gpio_pin_set(IN1, in_seq[counter][0]);
  neorv32_gpio_pin_set(IN2, in_seq[counter][1]);
  neorv32_gpio_pin_set(IN3, in_seq[counter][2]);
  neorv32_gpio_pin_set(EN1, en_seq[counter][0]);
  neorv32_gpio_pin_set(EN2, en_seq[counter][1]);
  neorv32_gpio_pin_set(EN3, en_seq[counter][2]);

  counter = (counter + 1) % 6;

}