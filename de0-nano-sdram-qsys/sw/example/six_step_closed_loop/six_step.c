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

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
// config gpio pins
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

const float_conv_t conversion_factor = {.float_value = 3.3f / (1 << 12)}; // this is not the right way to calculate the conversion factor but works for now

/* Priorities used by the tasks. */
#define mainMotorTask_PRIORITY    ( tskIDLE_PRIORITY + 1 )



// create timers
static void vTimerUpdateConstants(TimerHandle_t xTimer);
static void vTimerMotorMove(TimerHandle_t xTimer);

// config tasks
static void prvMotorTask(void *pvParameters);
static void vOpenLoopMotorTask(void *pvParameters);

uint32_t last_count = 0;
volatile uint32_t encoder_count = 0;

volatile uint32_t update_constants_time = 1000; // in ms
volatile uint32_t motor_move_time = 5; // in ms
volatile uint32_t speed = 0;


/**@}*/


// Prototypes
void align_rotor();
void move_clockwise();
void move_clockwise_pwm();
void read_and_convert_current();
void createCurTimer();
void createprvMotorTask();
void createOpenLoopMotorTask();
void createMotorMoveTimer();
void createTimers();
int six_step();
void update_angle();

current_ab get_current_ab() {
  current_ab res;
  adc_select_chanel(0);
  // wait  for stabilization
  for (volatile int i=0; i<75; i++) { }
  res.cur_a.float_value = riscv_intrinsic_fmuls(adc_read(), conversion_factor.float_value);
  //uint32_t test = riscv_intrinsic_fmuls(adc_read(), conversion_factor.float_value);
  adc_select_chanel(1);
  // wait  for stabilization
  for (volatile int i=0; i<75; i++) { }
  res.cur_b.float_value = riscv_intrinsic_fmuls(adc_read(), conversion_factor.float_value);
  // print the value of the last 12 bits of the gpio input
  //neorv32_uart0_printf("ADC 0: %u\n", test);
  //neorv32_uart0_printf("Current A: %u\n", res.cur_a.float_value);
  //neorv32_uart0_printf("Current B: %u\n", res.cur_b.float_value);
  return res;
}

current_qd get_clark_transform(current_ab cur_ab){
  current_qd res;
  res.cur_alpha.float_value = cur_ab.cur_a.float_value;
  res.cur_beta.float_value = riscv_emulate_fdivs(cur_ab.cur_a.float_value,riscv_intrinsic_fadds(sqrt(3) , riscv_intrinsic_fmuls(2, riscv_emulate_fdivs(cur_ab.cur_b.float_value, sqrt(3)))));
  // res.cur_beta.float_value = riscv_intrinsic_fdivs(cur_ab.cur_a.float_value,riscv_intrinsic_fadds(sqrt(3) , riscv_intrinsic_fmuls(2, riscv_intrinsic_fdivs(cur_ab.cur_b.float_value, sqrt(3)))));
  return res;
}



/**********************************************************************//**
 *
 * @note This program requires the GPTMR unit to be synthesized (and UART0 and GPIO).
 *
 * @return Should not return;
 **************************************************************************/
int six_step() {

  // config ADC
  adc_start();

  // align the motor
  align_rotor();

  // create the motor task
  //createprvMotorTask();
  createOpenLoopMotorTask();

  // create the timers
  createTimers();

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();

  neorv32_uart0_puts("If the scheduler started, this warning should not be printed\n");

  // Main loop
  while (1) {
  }

}

void createTimers(void)
{
    // Create both timers
    TimerHandle_t xCurTimer, xMotorMoveTimer;

    // Create CurTimer
    neorv32_uart0_puts("Creating Cur timer.\n");
    const TickType_t xCurTimerPeriod = pdMS_TO_TICKS(update_constants_time);
    xCurTimer = xTimerCreate(
        "CurTimer",          // Timer name
        xCurTimerPeriod,     // Timer period (10ms)
        pdTRUE,              // Auto-reload timer
        (void *)0,           // Timer ID
        vTimerUpdateConstants         // Callback function for CurTimer
    );

    if (xCurTimer != NULL)
    {
        if (xTimerStart(xCurTimer, 0) != pdPASS)
        {
            neorv32_uart0_puts("Failed to start Cur timer.\n");
        }
    }

    // Create MotorMoveTimer
    neorv32_uart0_puts("Creating motor move timer.\n");
    const TickType_t xMotorMoveTimerPeriod = pdMS_TO_TICKS(motor_move_time);
    xMotorMoveTimer = xTimerCreate(
        "MotorMoveTimer",       // Timer name
        xMotorMoveTimerPeriod,  // Timer period (50ms)
        pdTRUE,                 // Auto-reload timer
        (void *)0,              // Timer ID
        vTimerMotorMove         // Callback function for MotorMoveTimer
    );

    if (xMotorMoveTimer != NULL)
    {
        if (xTimerStart(xMotorMoveTimer, 0) != pdPASS)
        {
            neorv32_uart0_puts("Failed to start MotorMove timer.\n");
        }
    }
}


void createprvMotorTask(void)
{
  // print an warning
  neorv32_uart0_puts("Creating motor task.\n");
  // Create the task
  xTaskCreate(prvMotorTask, "MotorTask", configMINIMAL_STACK_SIZE, NULL, mainMotorTask_PRIORITY, NULL);
  // print an warning
  neorv32_uart0_puts("Motor task created.\n");
}

void createOpenLoopMotorTask(void)
{
  // print an warning
  neorv32_uart0_puts("Creating open loop motor task.\n");
  // Create the task
  xTaskCreate(vOpenLoopMotorTask, "OpenLoopMotorTask", configMINIMAL_STACK_SIZE, NULL, mainMotorTask_PRIORITY, NULL);
  // print an warning
  neorv32_uart0_puts("Open loop motor task created.\n");
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

    last_count = neorv32_counter_get();

    // print a warning
    neorv32_uart0_puts("Motor aligned.\n");
}

// Function to be called when the timer expires
void vTimerUpdateConstants(TimerHandle_t xTimer)
{
  update_constants = 1;
}

void vTimerMotorMove(TimerHandle_t xTimer)
{
  update_motor = 1;
}

void vOpenLoopMotorTask(void *pvParameters)
{
    // print a warning
    neorv32_uart0_puts("Open loop motor task started.\n");
    // Loop indefinitely
    while (1)
    {
      if (update_constants){
        update_angle();
        update_constants = 0;
      }
      if (speed > 100) { // still need to figure out the right value
        // create the motor move task
        createprvMotorTask();
        // delete the open loop motor task
        vTaskDelete(NULL);
      }
      move_clockwise();
      //move_clockwise_pwm();
    }
}

void prvMotorTask(void *pvParameters)
{
    // print a warning
    neorv32_uart0_puts("Motor task started.\n");
    // Loop indefinitely
    while (1)
    {
      if (update_constants){
        update_angle();
        read_and_convert_current();
        update_constants = 0;
      }
      move_clockwise();
      //move_clockwise_pwm();
    }
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
    neorv32_pwm_set(IN1, in_seq[step_index][0]*PWM_RES);
    neorv32_pwm_set(IN2, in_seq[step_index][1]*PWM_RES);
    neorv32_pwm_set(IN3, in_seq[step_index][2]*PWM_RES);
    neorv32_gpio_pin_set(EN1, en_seq[step_index][0]);
    neorv32_gpio_pin_set(EN2, en_seq[step_index][1]);
    neorv32_gpio_pin_set(EN3, en_seq[step_index][2]);

    // Increment and wrap around the step index
    step_index = (step_index + 1) % 6;

    // Reset the timer flag
    update_motor = 0;
  }
}


void read_and_convert_current() {
    // Get the current values
    three_phase = get_current_ab();
    quadrature = get_clark_transform(three_phase);
}

void update_angle() {

  encoder_count = neorv32_counter_get();
  //each tick is 1.8 degrees
  uint32_t diff = encoder_count - last_count;
  current_angle.float_value = riscv_intrinsic_fadds(current_angle.float_value, riscv_intrinsic_fmuls(1.8, diff));
  last_count = encoder_count;
  if (!(riscv_intrinsic_flts(360, current_angle.float_value))) {
    current_angle.float_value = riscv_intrinsic_fsubs(current_angle.float_value, 360.0);
  }
  // calculate speed
  // speed = diff * 1.8 / time_between_measurements
  float_conv_t time_in_seconds = { .float_value = riscv_emulate_fdivs(update_constants_time, 1000) };
  speed = riscv_intrinsic_fmuls(1.8, riscv_emulate_fdivs(diff, time_in_seconds.float_value)); // in degrees per second
  // print the speed  
  neorv32_uart0_printf("Speed: %u\n", speed);

}