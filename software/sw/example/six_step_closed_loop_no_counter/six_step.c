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
#define EN1 0
#define EN2 1
#define EN3 2

/** Maximum PWM output intensity (8-bit) */
#define PWM_RES 255

// config pwm values
volatile uint8_t update_motor = 0;
volatile uint8_t update_constants = 0;
volatile uint8_t step_index = 0;
volatile uint8_t sector_index = 0;
/**@}*/

// define pi
#define PI 3.14159265358979323846

volatile float_conv_t current_angle = { .float_value = 0.0 };

// things for the motor control
const uint8_t in_seq[6][3] = {{0, 1, 0}, {0, 1, 0}, {0, 0, 1},
                              {0, 0, 1}, {1, 0, 0}, {1, 0, 0}};

const uint8_t en_seq[6][3] = {{0, 1, 1}, {1, 1, 0}, {1, 0, 1},
                              {0, 1, 1}, {1, 1, 0}, {1, 0, 1}};
/**@{*/

const float_conv_t conversion_factor = {.float_value = 3.3f / (1 << 12)}; // this is not the right way to calculate the conversion factor but works for now

/* Priorities used by the tasks. */
#define mainMotorTask_PRIORITY    ( tskIDLE_PRIORITY + 3 ) 
#define mainUARTTask_PRIORITY    ( tskIDLE_PRIORITY + 1 ) 
#define mainUARTWriteTask_PRIORITY    ( tskIDLE_PRIORITY + 3 )

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
static void vListemUARTTask(void *pvParameters);
static void vWriteUARTTask(void *pvParameters);

// config queue
static QueueHandle_t xQueue = NULL;

volatile uint32_t last_sector = 0;

volatile uint32_t update_constants_time = 1; // in ms
volatile uint32_t motor_move_time = 5; // in ms
volatile float_conv_t motor_speed = {.float_value = 0.0};
volatile float_conv_t duty_cycle = {.float_value = 0.5};
volatile float_conv_t PWM_VALUE = {.float_value = 0.0};
volatile float_conv_t acummulated_time = {.float_value = 0.0};

/**@}*/


// Prototypes
void align_rotor();
void move_clockwise();
void move_clockwise_pwm(uint8_t direction);
void createTasks();
void createprvMotorTask();
void createTimers();
int six_step();
void update_angle();
void PID_control();
void get_sector();



/**********************************************************************//**
 *
 * @note This program requires the GPTMR unit to be synthesized (and UART0 and GPIO).
 *
 * @return Should not return;
 **************************************************************************/
int six_step() {

  // config ADC
  //adc_start();

  // align the motor
  align_rotor();

  // create queue
  xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( float ) );

  // create the motor task
  //createprvMotorTask();
  createTasks();

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

void createTasks(void)
{
  // Create the task
  xTaskCreate(prvMotorTask, "MotorTask", configMINIMAL_STACK_SIZE, NULL, mainMotorTask_PRIORITY, NULL);
  // print an warning
  neorv32_uart0_puts("Motor task created.\n");
  // Create the task
  xTaskCreate(vListemUARTTask, "UARTTask", configMINIMAL_STACK_SIZE, NULL, mainUARTTask_PRIORITY, NULL);
  // print an warning
  neorv32_uart0_puts("UART task created.\n");
  // Create the task
  xTaskCreate(vWriteUARTTask, "UARTWriteTask", configMINIMAL_STACK_SIZE, NULL, mainUARTWriteTask_PRIORITY, NULL);
  // print an warning
  neorv32_uart0_puts("UART write task created.\n");
}

void align_rotor() {
  // align the motor
    PWM_VALUE.float_value = riscv_intrinsic_fmuls(PWM_RES, duty_cycle.float_value);
    neorv32_pwm_set(IN1, riscv_intrinsic_fmuls(in_seq[0][0], PWM_VALUE.float_value));
    neorv32_pwm_set(IN2, riscv_intrinsic_fmuls(in_seq[0][1], PWM_VALUE.float_value));
    neorv32_pwm_set(IN3, riscv_intrinsic_fmuls(in_seq[0][2], PWM_VALUE.float_value));
    neorv32_gpio_pin_set(EN1, en_seq[0][0]);
    neorv32_gpio_pin_set(EN2, en_seq[0][1]);
    neorv32_gpio_pin_set(EN3, en_seq[0][2]);

    // delay for 0.5 second
    neorv32_cpu_delay_ms(500);

    // disable the motor
    neorv32_gpio_pin_set(EN1, 0);
    neorv32_gpio_pin_set(EN2, 0);
    neorv32_gpio_pin_set(EN3, 0);

    // set position to 30 degrees
    current_angle.float_value = 30.0;

    // print the angle
    neorv32_uart0_printf("Angle init: %u\n", current_angle.binary_value);

    get_sector();

    last_sector = sector_index;

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

void prvMotorTask(void *pvParameters)
{
    // print a warning
    neorv32_uart0_puts("Motor task started.\n");
    // Loop indefinitely
    while (1)
    {
      if (update_constants){
        update_angle();
        get_sector();
        //PID_control();
        update_constants = 0;
      }
      if (update_motor) {
        //move_clockwise();
        move_clockwise_pwm(1);
        update_motor = 0;
      }
    }
}

void vListemUARTTask(void *pvParameters)
{
    // print a warning
    neorv32_uart0_puts("UART listem task started.\n");
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
      // print the speed
      xQueueSend(xQueue, &error.float_value, 0);
      // make the task sleep for 1 second
      vTaskDelay(pdMS_TO_TICKS(400));
    }
}

void vWriteUARTTask(void *pvParameters)
{
    // print a warning
    neorv32_uart0_puts("UART write task started.\n");
    // Loop indefinitely
    while (1)
    {
      // print the speed
      uint32_t motor_speed_int = riscv_intrinsic_fcvt_wus(motor_speed.float_value);
      neorv32_uart0_printf("Speed: %u\n", motor_speed_int);
      // make the task sleep for 1 second
      vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Motor movement function based on the timer flag
void move_clockwise() {
    // Set motor pins based on the current step
    neorv32_gpio_pin_set(IN1, in_seq[step_index][0]);
    neorv32_gpio_pin_set(IN2, in_seq[step_index][1]);
    neorv32_gpio_pin_set(IN3, in_seq[step_index][2]);
    neorv32_gpio_pin_set(EN1, en_seq[step_index][0]);
    neorv32_gpio_pin_set(EN2, en_seq[step_index][1]);
    neorv32_gpio_pin_set(EN3, en_seq[step_index][2]);

    // Increment and wrap around the step index
    step_index = (step_index + 1) % 6;
}

// move clockwise with pwm instead of gpio
void move_clockwise_pwm(uint8_t direction) {
    step_index = (sector_index + direction) % 6;
    // print the step index
    //neorv32_uart0_printf("Step index: %u\n", step_index);
    // Set motor pins based on the current step
    PWM_VALUE.float_value = riscv_intrinsic_fmuls(PWM_RES, duty_cycle.float_value);
    neorv32_pwm_set(IN1, riscv_intrinsic_fmuls(in_seq[step_index][0], PWM_VALUE.float_value));
    neorv32_pwm_set(IN2, riscv_intrinsic_fmuls(in_seq[step_index][1], PWM_VALUE.float_value));
    neorv32_pwm_set(IN3, riscv_intrinsic_fmuls(in_seq[step_index][2], PWM_VALUE.float_value));
    neorv32_gpio_pin_set(EN1, en_seq[step_index][0]);
    neorv32_gpio_pin_set(EN2, en_seq[step_index][1]);
    neorv32_gpio_pin_set(EN3, en_seq[step_index][2]);
}


void update_angle() {

  get_sector();
  uint32_t diff = sector_index - last_sector;
  if (diff < 0) {
    diff = 6 + diff;
  }
  last_sector = sector_index;
  current_angle.float_value = riscv_intrinsic_fadds(current_angle.float_value, riscv_intrinsic_fmuls(60, diff));
  
  if ((riscv_intrinsic_flts(360.0, current_angle.float_value))) { // if the angle is greater than 360 degrees
    current_angle.float_value = riscv_intrinsic_fsubs(current_angle.float_value, 360.0);
  }

  // speed = ((diff * pi/3) / time_between_measurements)
  float_conv_t time_in_seconds = { .float_value = riscv_emulate_fdivs(update_constants_time, 1000) };
  if (diff == 0) {
    acummulated_time.float_value = riscv_intrinsic_fadds(acummulated_time.float_value, time_in_seconds.float_value);
  } else{
    acummulated_time.float_value = riscv_intrinsic_fadds(acummulated_time.float_value, time_in_seconds.float_value);
    float_conv_t ang_rad = { .float_value = riscv_emulate_fdivs(PI, 3) };
    motor_speed.float_value = riscv_emulate_fdivs(riscv_intrinsic_fmuls(diff, ang_rad.float_value), acummulated_time.float_value);
    acummulated_time.float_value = 0.0;
  }

}

void PID_control() {
  float_conv_t error;
  if (xQueueReceive(xQueue, &error.float_value, 0) == pdTRUE) {
    // print the error
    neorv32_uart0_printf("Error: %u\n", error.binary_value);
  }
}


void get_sector() {

  uint8_t hall_0 = neorv32_gpio_pin_get(0);
  uint8_t hall_1 = neorv32_gpio_pin_get(1);
  uint8_t hall_2 = neorv32_gpio_pin_get(2);
  if (hall_0 && !hall_1 && !hall_2) {
    sector_index = 0;
  } else if (hall_0 && hall_1 && !hall_2) {
    sector_index = 1;
  } else if (!hall_0 && hall_1 && !hall_2) {
    sector_index = 2;
  } else if (!hall_0 && hall_1 && hall_2) {
    sector_index = 3;
  } else if (!hall_0 && !hall_1 && hall_2) {
    sector_index = 4;
  } else if (hall_0 && !hall_1 && hall_2) {
    sector_index = 5;
  }
}