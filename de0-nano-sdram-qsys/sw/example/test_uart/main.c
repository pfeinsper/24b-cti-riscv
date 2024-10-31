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

void gptmr_firq_handler(void);





/**********************************************************************//**
 * This testes everything that we will use in the project, from the PWM to the GPIO, ADC, GPTMR and XIRQ.
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
  neorv32_uart0_puts("The Golden Top.\n"
                     "Test everything that we will use.\n\n");


  // install GPTMR interrupt handler
  neorv32_rte_handler_install(GPTMR_RTE_ID, gptmr_firq_handler);

  // configure timer for 0.5Hz in continuous mode (with clock divisor = 8)
  neorv32_gptmr_setup(CLK_PRSC_8, neorv32_sysinfo_get_clk() / (8 * 2), 1);

  // enable interrupt
  neorv32_cpu_csr_clr(CSR_MIP, 1 << GPTMR_FIRQ_PENDING);  // make sure there is no GPTMR IRQ pending already
  neorv32_cpu_csr_set(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);   // enable GPTMR FIRQ channel
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

  // go to sleep mode and wait for interrupt
  while(1) {
  neorv32_cpu_sleep();
}

}

/**********************************************************************//**
 * GPTMR FIRQ handler.
 *
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void gptmr_firq_handler(void) {

  neorv32_gptmr_irq_ack(); // clear/ack pending FIRQ
  // scan for UART input
  int c = neorv32_uart_getc();
  // print the char
  neorv32_uart0_printf("Received value: %i\n", c);
}