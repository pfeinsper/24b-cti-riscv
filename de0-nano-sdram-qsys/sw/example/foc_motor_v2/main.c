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
#include <float.h>
#include <math.h>
#include <stdio.h>
#include "neorv32_zfinx_extension_intrinsics.h"


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

// config pwm values
volatile uint8_t timer_status = 0;
volatile uint8_t encoder_status = 0;
volatile uint8_t step_index = 0;
/**@}*/

typedef struct {
  float_conv_t cur_a;
  float_conv_t cur_b;
} current_ab;

typedef struct {
  float_conv_t cur_alpha;
  float_conv_t cur_beta;
} current_qd;

current_ab three_phase;
current_qd quadrature;

volatile float_conv_t current_angle = { .float_value = 0.0 };

// things for the motor control
uint8_t in_seq[6][3] = {{1, 0, 0}, {0, 1, 0}, {0, 1, 0},
                        {0, 0, 1}, {0, 0, 1}, {1, 0, 0}};
uint8_t en_seq[6][3] = {{1, 0, 1}, {0, 1, 1}, {1, 1, 0},
                        {1, 0, 1}, {0, 1, 1}, {1, 1, 0}};
/**@{*/

const float_conv_t conversion_factor = { .float_value =  0.0008056640625}; // still need to be calculated!!!!!!!!

/** Maximum PWM output intensity (8-bit) */
#define PWM_MAX 255
/** Number of PWM channels to modulate simultaneously */
#define NUM_PWM_CHANNELS 4
/**@}*/


// Prototypes
void gptmr_firq_handler(void);
void xirq_handler_ch0(void);
void align_rotor();
void move_clockwise();

current_ab get_current_ab() {
  current_ab res;
  adc_select_chanel(0);
  res.cur_a.float_value = riscv_intrinsic_fmuls(adc_read(), conversion_factor.float_value);
  adc_select_chanel(1);
  res.cur_b.float_value = riscv_intrinsic_fmuls(adc_read(), conversion_factor.float_value);
  return res;
}

current_qd get_clark_transform(current_ab cur_ab){
  current_qd res;
  res.cur_alpha.float_value = cur_ab.cur_a.float_value;
  res.cur_beta.float_value = riscv_intrinsic_fdivs(cur_ab.cur_a.float_value,riscv_intrinsic_fadds(sqrt(3) , riscv_intrinsic_fmuls(2, riscv_intrinsic_fdivs(cur_ab.cur_b.float_value, sqrt(3)))));
  return res;
}




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

    // check if Zfinx extension is implemented at all
  if ((neorv32_cpu_csr_read(CSR_MXISA) & (1<<CSR_MXISA_ZFINX)) == 0) {
    neorv32_uart0_puts("Error! <Zfinx> extension not synthesized!\n");
    return 1;
  }


  // Disable compilation by default
#ifndef RUN_CHECK
  #warning Program HAS NOT BEEN COMPILED! Use >>make USER_FLAGS+=-DRUN_CHECK clean_all exe<< to compile it.

  // inform the user if you are actually executing this
  neorv32_uart0_printf("ERROR! Program has not been compiled. Use >>make USER_FLAGS+=-DRUN_CHECK clean_all exe<< to compile it.\n");

  return 1;
#endif

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);


  // check if GPTMR unit is implemented at all
  if (neorv32_gptmr_available() == 0) {
    neorv32_uart0_puts("ERROR! General purpose timer not implemented!\n");
    return 1;
  }

  // Check if PWM unit is implemented
  if (neorv32_pwm_available() == 0) {
    if (neorv32_uart0_available()) {
      neorv32_uart0_printf("ERROR: PWM module not implemented!\n");
    }
    return 1;
  }

  // check if XIRQ unit is implemented at all
  if (neorv32_xirq_available() == 0) {
    neorv32_uart0_printf("XIRQ not synthesized!\n");
    return 1;
  }

  int err_cnt = 0;

  // initialize XIRQ controller
  // this will disable all XIRQ channels and will also clear any pending external interrupts
  // (details: this will register the XIRQ's second-level interrupt handler in the NEORV32 RTE)
  err_cnt = neorv32_xirq_setup();

  // check if setup went fine
  if (err_cnt) {
    neorv32_uart0_printf("Error during XIRQ setup!\n");
    return 1;
  }

  // configure per-channel trigger type
  neorv32_xirq_setup_trigger(0, XIRQ_TRIGGER_EDGE_RISING); // rising-edge for channel 0

  // install handler functions for XIRQ channel 0. note that these functions are "normal" functions!
  // (details: these are "third-level" interrupt handlers)
  // neorv32_xirq_install() also enables the specified XIRQ channel and clears any pending interrupts
  err_cnt = 0;
  err_cnt += neorv32_xirq_install(0, xirq_handler_ch0); // handler function for channel 0

  // check if installation went fine
  if (err_cnt) {
    neorv32_uart0_printf("Error during XIRQ install!\n");
    return 1;
  }

  // enable XIRQ channels
  neorv32_xirq_channel_enable(0);

  // allow XIRQ to trigger CPU interrupt
  neorv32_xirq_global_enable();

  int num_pwm_channels = neorv32_pmw_get_num_channels();

  // Intro
  neorv32_uart0_puts("The Golden Top.\n"
                     "Test everything that we will use.\n\n");

  // Check number of PWM channels
  if (neorv32_uart0_available()) {
    neorv32_uart0_printf("Implemented PWM channels: %i\n\n", num_pwm_channels);
  }

  // Deactivate all PWM channels initially
  for (int i = 0; i < num_pwm_channels; i++) {
    neorv32_pwm_set(i, 0);
  }

  // Configure and enable PWM
  neorv32_pwm_setup(CLK_PRSC_64);


  // clear GPIO output port
  neorv32_gpio_port_set(0);


  // install GPTMR interrupt handler
  neorv32_rte_handler_install(GPTMR_RTE_ID, gptmr_firq_handler);

  // configure timer for 0.5Hz in continuous mode (with clock divisor = 8)
  neorv32_gptmr_setup(CLK_PRSC_8, neorv32_sysinfo_get_clk() / (8 * 2), 1);

  // config ADC
  adc_start();

  // align the motor
  align_rotor();

  // enable interrupt
  neorv32_cpu_csr_clr(CSR_MIP, 1 << GPTMR_FIRQ_PENDING);  // make sure there is no GPTMR IRQ pending already
  neorv32_cpu_csr_set(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);   // enable GPTMR FIRQ channel
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

  // Main loop
  while (1) {

    // update current read
    three_phase = get_current_ab();

    // Move the motor based on the timer callback
    move_clockwise();

    // Encoder handling
    if (encoder_status) {
      current_angle.float_value = riscv_intrinsic_fadds(current_angle.float_value, 1.8);

      // Reset the angle if it exceeds 360 degrees
      if (!(riscv_intrinsic_flts(360, current_angle.float_value))) {
        current_angle.float_value = riscv_intrinsic_fsubs(current_angle.float_value, 360.0);
      }

      encoder_status = 0;
    }
  }

}

// Motor movement function based on the timer flag
void move_clockwise() {
  if (timer_status) {
    // Set motor pins based on the current step
    neorv32_gpio_pin_set(IN1, in_seq[step_index][0]);
    neorv32_gpio_pin_set(IN2, in_seq[step_index][1]);
    neorv32_gpio_pin_set(IN3, in_seq[step_index][2]);
    neorv32_gpio_pin_set(EN1, en_seq[step_index][0]);
    neorv32_gpio_pin_set(EN2, en_seq[step_index][1]);
    neorv32_gpio_pin_set(EN3, en_seq[step_index][2]);

    // Increment and wrap around the step index
    step_index = (step_index + 1) % 6;

    // Reset the timer flag
    timer_status = 0;
  }
}

/**********************************************************************//**
 * GPTMR FIRQ handler.
 *
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void gptmr_firq_handler(void) {
  neorv32_gptmr_irq_ack(); // clear/ack pending FIRQ
  timer_status = 1;
}

// Handler for the external interrupt channel 0 (where we will check the Hall sensor)
void xirq_handler_ch0(void) {
  encoder_status = 1;
}

void align_rotor() {
  // align the motor
    neorv32_gpio_pin_set(IN1, in_seq[5][0]);
    neorv32_gpio_pin_set(IN2, in_seq[5][1]);
    neorv32_gpio_pin_set(IN3, in_seq[5][2]);
    neorv32_gpio_pin_set(EN1, en_seq[5][0]);
    neorv32_gpio_pin_set(EN2, en_seq[5][1]);
    neorv32_gpio_pin_set(EN3, en_seq[5][2]);

    // delay for 0.5 second
    neorv32_cpu_delay_ms(500);

    // disable the motor
    neorv32_gpio_pin_set(EN1, 0);
    neorv32_gpio_pin_set(EN2, 0);
    neorv32_gpio_pin_set(EN3, 0);

    // set position to 30 degrees
    current_angle.float_value = 330.0;
}