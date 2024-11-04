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

/* Priorities used by the tasks. */
#define mainMotorTask_PRIORITY    ( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue.  The 500ms value is converted
 * to ticks using the pdMS_TO_TICKS() macro. */
#define mainQUEUE_SEND_FREQUENCY_MS             pdMS_TO_TICKS( 500 )

/* The maximum number items the queue can hold.*/
#define mainQUEUE_LENGTH                        ( 1 )



// create timers
static void vTimerUpdateConstants(TimerHandle_t xTimer);
static void vTimerMotorMove(TimerHandle_t xTimer);

// config tasks
static void prvMotorTask(void *pvParameters);
static void vOpenLoopMotorTask(void *pvParameters);
static void vListemUARTTask(void *pvParameters);

// config queue
static QueueHandle_t xQueue = NULL;

uint32_t last_count = 0;
volatile uint32_t encoder_count = 0;

volatile uint32_t update_constants_time = 1000; // in ms
volatile uint32_t motor_move_time = 5; // in ms
volatile float_conv_t motor_speed = {.float_value = 0.0};
volatile uint8_t voltage_divider = 1;
volatile uint8_t sector = 0;

/**@}*/


// Prototypes
void align_rotor();
void move_clockwise();
void move_clockwise_pwm();
void createInitialTasks();
void createprvMotorTask();
void createOpenLoopMotorTask();
void createTimers();
int six_step();
void update_angle();
void PID_control();



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

  // create queue
  xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( float ) );

  // create the motor task
  //createprvMotorTask();
  createInitialTasks();

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

void createInitialTasks(void)
{
  // Create the task
  xTaskCreate(vOpenLoopMotorTask, "OpenLoopMotorTask", configMINIMAL_STACK_SIZE, NULL, mainMotorTask_PRIORITY, NULL);
  // print an warning
  neorv32_uart0_puts("Open loop motor task created.\n");
  // Create the task
  xTaskCreate(vListemUARTTask, "UARTTask", configMINIMAL_STACK_SIZE, NULL, mainMotorTask_PRIORITY, NULL);
  // print an warning
  neorv32_uart0_puts("UART task created.\n");
}

void createprvMotorTask(void)
{
  // Create the task
  xTaskCreate(prvMotorTask, "MotorTask", configMINIMAL_STACK_SIZE, NULL, mainMotorTask_PRIORITY, NULL);
  // print an warning
  neorv32_uart0_puts("Motor task created.\n");
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
      if (!(riscv_intrinsic_flts(100, motor_speed.float_value))) { // if the speed is greater than 100 degrees per second
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
        PID_control();
        update_constants = 0;
      }
      move_clockwise();
      //move_clockwise_pwm();
    }
}

void vListemUARTTask(void *pvParameters)
{
    // print a warning
    neorv32_uart0_puts("UART task started.\n");
    // Loop indefinitely
    while (1)
    {
      // scan for the uart input
      char buffer[32];
      neorv32_uart_scan(NEORV32_UART0, buffer, 32, 0);
      // save the value of the new target speed to a variable
      float_conv_t target_speed = { .float_value = atof(buffer) };
      float_conv_t error = { .float_value = riscv_intrinsic_fsubs(target_speed.float_value, motor_speed.float_value) };
      // send the error to the queue
      xQueueSend(xQueue, &error.float_value, 0);
      // delay for 10 ms
      neorv32_cpu_delay_ms(10);
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
  // findout the sector (0-5) -> uint8_t sector = (uint8_t)floor(current_angle/60);
  sector = (uint8_t)floorf(current_angle.float_value/60);
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
  if (xQueueReceive(xQueue, &error.float_value, 0) == pdTRUE) {
    // print the error
    neorv32_uart0_printf("Error: %u\n", error.binary_value);
  }
}