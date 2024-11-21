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
#define mainUpdatePIDTask_PRIORITY    ( tskIDLE_PRIORITY + 3 )
#define mainUARTTask_PRIORITY    ( tskIDLE_PRIORITY + 3 ) 
#define mainUARTWriteTask_PRIORITY    ( tskIDLE_PRIORITY + 3 )

/* The rate at which data is sent to the queue.  The 500ms value is converted
 * to ticks using the pdMS_TO_TICKS() macro. */
#define mainQUEUE_SEND_FREQUENCY_MS             pdMS_TO_TICKS( 500 )

/* The maximum number items the queue can hold.*/
#define mainQUEUE_LENGTH                        ( 1 )



// create timers
static void vTimerUpdateConstants(TimerHandle_t xTimer);
static void vTimerMotorMove(TimerHandle_t xTimer);
static void vTimerChangeMode(TimerHandle_t xTimer);

// config tasks
static void vUpdatePIDTask(void *pvParameters);
static void vListemUARTTask(void *pvParameters);
static void vWriteUARTTask(void *pvParameters);

// config queue
static QueueHandle_t xQueue = NULL;

volatile uint32_t last_sector = 0;

volatile uint32_t update_constants_time = 1; // in ms

volatile uint32_t motor_move_time = 1; // in ms
volatile uint32_t change_mode_time = 1000; // in ms
volatile float_conv_t motor_speed = {.float_value = 0.0};
volatile float_conv_t duty_cycle = {.float_value = 0.0};
volatile float_conv_t PWM_VALUE = {.float_value = 0.0};

volatile float_conv_t rad_60 = { .float_value = 0.0 };

volatile float_conv_t last_time = {.float_value = 0.0};
volatile float_conv_t last_update = {.float_value = 0.0};


/* pi contoler constants */
const float_conv_t Kp = {.float_value = 0.0000362};
const float_conv_t Ki = {.float_value = 0.0015001};

// PI Controller structure
typedef struct {
  float_conv_t integral; // Integral accumulator
  float_conv_t max_duty; // Maximum duty cycle (1.0 for 100%)
  float_conv_t min_duty; // Minimum duty cycle (0.0 for 0%)
} PI_Controller;

volatile float_conv_t target_speed = {.float_value = 0.0};
PI_Controller pi;

float_conv_t debug_var = {.float_value = 0.0};

volatile uint8_t noise_counter = 0;

/**@}*/


// Prototypes
//void align_rotor();
void move_clockwise();
void move_clockwise_pwm(uint8_t direction);
void createTasks();
void createTimers();
int six_step();
void update_angle();
void PID_control();
void get_sector();
void PI_Init(PI_Controller *pi, float_conv_t max_duty, float_conv_t min_duty);
void PI_Update(PI_Controller *pi, float_conv_t desired_speed,
                float_conv_t actual_speed);




/**********************************************************************//**
 *
 * @note This program requires the GPTMR unit to be synthesized (and UART0 and GPIO).
 *
 * @return Should not return;
 **************************************************************************/
int six_step() {
  
  // initialize constants
  rad_60.float_value = riscv_emulate_fdivs(PI, 3.0);
  target_speed.float_value = 10000.0;

  // initialize the PI controller
  PI_Init(&pi, (float_conv_t){.float_value = 1.0}, (float_conv_t){.float_value = 0.0});

  // create queue
  xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( float ) );

  // create the motor task
  createTasks();

  // create the timers
  createTimers();

  last_time.float_value = neorv32_mtime_get_time();

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
    TimerHandle_t xConstUpdate, xMotorMoveTimer, xChangeModeTimer;

    // Create CurTimer
    neorv32_uart0_puts("Creating Cur timer.\n");
    const TickType_t xConstUpdatePeriod = pdMS_TO_TICKS(update_constants_time);
    xConstUpdate = xTimerCreate(
        "CurTimer",          // Timer name
        xConstUpdatePeriod,     // Timer period (10ms)
        pdTRUE,              // Auto-reload timer
        (void *)0,           // Timer ID
        vTimerUpdateConstants         // Callback function for CurTimer
    );

    if (xConstUpdate != NULL)
    {
        if (xTimerStart(xConstUpdate, 0) != pdPASS)
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
    // Create ChangeModeTimer (activates only once)
    neorv32_uart0_puts("Creating change mode timer.\n");
    const TickType_t xChangeModeTimerPeriod = pdMS_TO_TICKS(change_mode_time);
    xChangeModeTimer = xTimerCreate(
        "ChangeModeTimer",       // Timer name
        xChangeModeTimerPeriod,  // Timer period (50ms)
        pdFALSE,                 // Auto-reload timer
        (void *)0,               // Timer ID
        vTimerChangeMode         // Callback function for ChangeModeTimer
    );

    if (xChangeModeTimer != NULL)
    {
        if (xTimerStart(xChangeModeTimer, 0) != pdPASS)
        {
            neorv32_uart0_puts("Failed to start ChangeMode timer.\n");
        }
    }
}

void createTasks(void)
{
  // Create the task
  xTaskCreate(vUpdatePIDTask, "UpdatePIDTask", configMINIMAL_STACK_SIZE, NULL, mainUpdatePIDTask_PRIORITY, NULL);
  // print an warning
  neorv32_uart0_puts("Update PID task created.\n");
  // Create the task
  xTaskCreate(vListemUARTTask, "UARTTask", configMINIMAL_STACK_SIZE, NULL, mainUARTTask_PRIORITY, NULL);
  // print an warning
  neorv32_uart0_puts("UART task created.\n");
  // Create the task
  xTaskCreate(vWriteUARTTask, "UARTWriteTask", configMINIMAL_STACK_SIZE, NULL, mainUARTWriteTask_PRIORITY, NULL);
  // print an warning
  neorv32_uart0_puts("UART write task created.\n");
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

void vTimerChangeMode(TimerHandle_t xTimer)
{
  // print a warning
  neorv32_uart0_puts("Change mode timer expired.\n");
  // change the duty cycle
  //duty_cycle.float_value = 1.0;
}

void vUpdatePIDTask(void *pvParameters)
{
    // print a warning
    neorv32_uart0_puts("Update PID task started.\n");
    // Loop indefinitely
    while (1)
    {
      if (update_constants) {
        PI_Update(&pi, target_speed, motor_speed);
        update_constants = 0;
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
      //char buffer[32];
      //neorv32_uart_scan(NEORV32_UART0, buffer, 32, 0);
      // save the value of the new target speed to a variable
      //target_speed.float_value = atof(buffer);
      // make the task sleep for 1 second
      vTaskDelay(pdMS_TO_TICKS(800));
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
      if (motor_speed_int < 300) {
        motor_speed_int = 0;
      }
      neorv32_uart0_printf("Speed: %u\n", motor_speed_int);
      neorv32_uart1_printf("Speed: %u\n", motor_speed_int);
      // print the debug variable
      //debug_var.float_value = sector_index;
      //int debug_int = riscv_intrinsic_fcvt_ws((debug_var.float_value*1000));
      //neorv32_uart0_printf("Debug: %i\n", debug_int);
      // make the task sleep for 1 second
      vTaskDelay(pdMS_TO_TICKS(2));
    }
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
  int diff = sector_index - last_sector;
  if (diff < 0) {
    diff = 6 + diff;
  }
  last_sector = sector_index;
  //debug_var.float_value = diff;

  // speed = ((diff * pi/3) / time_between_measurements)

  if (diff > 0){
    float_conv_t current_time = {.float_value = riscv_emulate_fdivs(neorv32_mtime_get_time(), ((float)neorv32_sysinfo_get_clk()))};
    float_conv_t  time_diff = {.float_value = riscv_intrinsic_fsubs(current_time.float_value, last_time.float_value)};
    motor_speed.float_value = riscv_emulate_fdivs(riscv_intrinsic_fmuls(1, rad_60.float_value), time_diff.float_value);
    // convert to rpm
    motor_speed.float_value = riscv_emulate_fdivs(riscv_intrinsic_fmuls(motor_speed.float_value, 60), (2*PI));
    last_time.float_value = current_time.float_value;
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

  sector_index = neorv32_sector_get();
}

/******************************************************************************
 * Handle NEORV32-/application-specific interrupts.
 ******************************************************************************/
void freertos_risc_v_application_interrupt_handler(void) {

  // mcause identifies the cause of the interrupt
  uint32_t mcause = neorv32_cpu_csr_read(CSR_MCAUSE);

  if (mcause == GPTMR_TRAP_CODE) { // is GPTMR interrupt
    neorv32_gptmr_irq_ack(); // clear GPTMR timer-match interrupt
    update_angle();
    move_clockwise_pwm(1);
  }
  else { // undefined interrupt cause
    neorv32_uart0_printf("\n<NEORV32-IRQ> Unexpected IRQ! cause=0x%x </NEORV32-IRQ>\n", mcause); // debug output
  }
}

void PI_Init(PI_Controller *pi, float_conv_t max_duty, float_conv_t min_duty) {
  pi->integral.float_value = 0.0;
  pi->max_duty.float_value = max_duty.float_value;
  pi->min_duty.float_value = min_duty.float_value;
}

void PI_Update(PI_Controller *pi, float_conv_t desired_speed,
                float_conv_t actual_speed) {

  float_conv_t current_time = {.float_value = riscv_emulate_fdivs(neorv32_mtime_get_time(), ((float)neorv32_sysinfo_get_clk()))};
  float_conv_t dt = {.float_value = riscv_intrinsic_fsubs(current_time.float_value, last_update.float_value)};
  last_update.float_value = current_time.float_value;
  //debug_var.float_value = dt.float_value;

  if (noise_counter < 5){
    noise_counter++;
    return;
  }

  float_conv_t error = {.float_value =
                            riscv_intrinsic_fsubs(desired_speed.float_value,
                                                  actual_speed.float_value)};
  float_conv_t aux = {.float_value = riscv_intrinsic_fmuls(error.float_value, dt.float_value)};

  pi->integral.float_value = riscv_intrinsic_fadds(
      pi->integral.float_value,
      aux.float_value);

  //debug_var.float_value = riscv_intrinsic_fmuls(error.float_value, dt.float_value);

  // compute the duty_cycle, the output
  float_conv_t new_duty_cycle = {.float_value = riscv_intrinsic_fadds(
      riscv_intrinsic_fmuls(Kp.float_value, error.float_value),
      riscv_intrinsic_fmuls(Ki.float_value, pi->integral.float_value))};

  //debug_var.float_value = new_duty_cycle.float_value;

  // limit the duty cycle
  if (new_duty_cycle.float_value > pi->max_duty.float_value) {
    new_duty_cycle.float_value = pi->max_duty.float_value;
  } else if (new_duty_cycle.float_value < pi->min_duty.float_value) {
    new_duty_cycle.float_value = pi->min_duty.float_value;
  }
  //debug_var.float_value = new_duty_cycle.float_value;
  duty_cycle.float_value = new_duty_cycle.float_value;
}
