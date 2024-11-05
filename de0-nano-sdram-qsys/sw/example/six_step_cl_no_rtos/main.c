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

/* Standard includes. */
#include <string.h>
#include <unistd.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
// config gpio pins
#define BAUD_RATE 19200
#define IN1 0
#define IN2 1
#define IN3 2
#define EN1 3
#define EN2 4
#define EN3 5

/** Maximum PWM output intensity (8-bit) */
#define PWM_RES 255

// config pwm values
volatile uint8_t update_motor = 0;
volatile uint8_t update_constants = 0;
volatile uint8_t step_index = 0;
/**@}*/

volatile float_conv_t current_angle = { .float_value = 0.0 };

// things for the motor control
uint8_t in_seq[6][3] = {{1, 0, 0}, {0, 1, 0}, {0, 1, 0},
                        {0, 0, 1}, {0, 0, 1}, {1, 0, 0}};
uint8_t en_seq[6][3] = {{1, 0, 1}, {0, 1, 1}, {1, 1, 0},
                        {1, 0, 1}, {0, 1, 1}, {1, 1, 0}};
/**@{*/

const float_conv_t conversion_factor = {.float_value = 3.3f / (1 << 12)}; // this is not the right way to calculate the conversion factor but works for now




uint32_t last_count = 0;
volatile uint32_t encoder_count = 0;

volatile uint32_t update_constants_time = 1; // in ms
volatile uint32_t motor_move_time = 5; // in ms
volatile float_conv_t motor_speed = {.float_value = 0.0};
volatile uint8_t voltage_divider = 1;
volatile uint8_t sector = 0;
volatile uint8_t open_loop = 1;
float_conv_t target_speed = {.float_value = 1000.0};
float_conv_t error = {.float_value = 1000.0};

/**@}*/


// Prototypes
void gptmr_firq_handler(void);
void mtime_irq_handler(void);
void move_clockwise();
void move_clockwise_pwm();
void update_angle();
void PID_control();
void listemUART();





/**********************************************************************//**
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

  // Check if PWM unit is implemented
  if (neorv32_pwm_available() == 0) {
    if (neorv32_uart0_available()) {
      neorv32_uart0_printf("ERROR: PWM module not implemented!\n");
    }
    return 1;
  }

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
    // install MTIME interrupt handler to RTE
  neorv32_rte_handler_install(RTE_TRAP_MTI, mtime_irq_handler);

  // configure MTIME timer's first interrupt to trigger after 1 second starting from now
  neorv32_mtime_set_timecmp(neorv32_mtime_get_time() + neorv32_sysinfo_get_clk());

  // configure timer for 0.5Hz in continuous mode (with clock divisor = 8)
  neorv32_gptmr_setup(CLK_PRSC_8, neorv32_sysinfo_get_clk() / (8 * 2), 1);

  // enable interrupt
  neorv32_cpu_csr_clr(CSR_MIP, 1 << GPTMR_FIRQ_PENDING);  // make sure there is no GPTMR IRQ pending already
  neorv32_cpu_csr_set(CSR_MIE, (1 << CSR_MIE_MTIE) | (1 << CSR_MIE_FIRQ12E));   // enable GPTMR FIRQ channel and mtime channel
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

  // config ADC
  adc_start();
  adc_select_chanel(0);

  // go to sleep mode and wait for interrupt
  while(1) {

  if (update_constants){
    listemUART();
  }

  if (open_loop){
    if (update_constants){
        update_angle();
        update_constants = 0;
      }
    if (!(riscv_intrinsic_flts(100, motor_speed.float_value))) { // if the speed is greater than 100 degrees per second
        open_loop = 0;
      }
    move_clockwise();
    //move_clockwise_pwm();
  }

  else {
      if (update_constants){
        update_angle();
        PID_control();
        update_constants = 0;
      }
      move_clockwise();
      //move_clockwise_pwm();
  }
}

}


/**********************************************************************//**
 * GPTMR FIRQ handler.
 *
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void gptmr_firq_handler(void) {
  update_motor = 1;
}

/**********************************************************************//**
 * MTIME IRQ handler.
 *
 * @warning This function has to be of type "void xyz(void)" and must not use any interrupt attributes!
 **************************************************************************/
void mtime_irq_handler(void) {
  update_constants = 1;
  // configure MTIME timer's next interrupt to trigger after 1 second starting from now
  neorv32_mtime_set_timecmp(neorv32_mtime_get_timecmp() + neorv32_sysinfo_get_clk());
  // print an warning
  neorv32_uart0_puts("MTIME IRQ handler.\n");
}


// Motor movement function based on the timer flag
void move_clockwise() {
  if (update_motor) {
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
    update_motor = 0;
  }
}

// move clockwise with pwm instead of gpio
void move_clockwise_pwm() {
  if (update_motor) {
    // Set motor pins based on the current step
    neorv32_pwm_set(IN1, in_seq[step_index][0]*PWM_RES/voltage_divider);
    neorv32_pwm_set(IN2, in_seq[step_index][1]*PWM_RES/voltage_divider);
    neorv32_pwm_set(IN3, in_seq[step_index][2]*PWM_RES/voltage_divider);
    neorv32_gpio_pin_set(EN1, en_seq[step_index][0]);
    neorv32_gpio_pin_set(EN2, en_seq[step_index][1]);
    neorv32_gpio_pin_set(EN3, en_seq[step_index][2]);

    // Increment and wrap around the step index
    step_index = (step_index + 1) % 6;

    // Reset the timer flag
    update_motor = 0;
  }
}


void update_angle() {

  encoder_count = neorv32_counter_get();
  //each tick is 1.8 degrees
  uint32_t diff = encoder_count - last_count;
  current_angle.float_value = riscv_intrinsic_fadds(current_angle.float_value, riscv_intrinsic_fmuls(1.8, diff));
  last_count = encoder_count;
  if (!(riscv_intrinsic_flts(360, current_angle.float_value))) { // if the angle is greater than 360 degrees
    current_angle.float_value = riscv_intrinsic_fsubs(current_angle.float_value, 360.0);
  }
  // find out the sector (0-5) -> uint8_t sector = (uint8_t)floor(current_angle/60);
  sector = (uint8_t)floorf(riscv_emulate_fdivs(current_angle.float_value, 60.0));
  neorv32_uart0_printf("sector: %u\n", sector);
  // calculate speed
  // speed = diff * 1.8 / time_between_measurements
  float_conv_t time_in_seconds = { .float_value = riscv_emulate_fdivs(update_constants_time, 1000) };
  motor_speed.float_value = riscv_intrinsic_fmuls(1.8, riscv_emulate_fdivs(diff, time_in_seconds.float_value)); // in degrees per second
  // print the speed  
  neorv32_uart0_printf("Speed: %u\n", motor_speed.binary_value);

}

void PID_control() {
  float_conv_t error;
  // print an warning
  neorv32_uart0_puts("PID control.\n");
}


void listemUART () {
  // scan for the uart input
  char buffer[32];
  neorv32_uart_scan(NEORV32_UART0, buffer, 32, 0);
  // save the value of the new target speed to a variable
  target_speed.float_value = atof(buffer);
  error.float_value =  riscv_intrinsic_fsubs(target_speed.float_value, motor_speed.float_value);
}