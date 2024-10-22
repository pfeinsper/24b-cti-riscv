#include "utils.h"

current_ab three_phase;
current_clark quadrature;
current_park rotated;
voltage_pi reference_voltage;
voltage_clark quadrature_voltage;
space_vector duty_cycle;
pwm_config_space_vector pwm_a, pwm_b, pwm_c;

// Function to be called when the timer expires
void vTimerGetCur(TimerHandle_t xTimer)
{
    // Code to execute at 100Hz frequency
    // This function will be called every 10ms
    // print a warning
    //neorv32_uart0_puts("Timer C expired.\n");
    read_current = 1;
}

void vTimerMotorMove(TimerHandle_t xTimer)
{
  // neorv32_uart0_puts("Timer M expired.\n");
  // Set the timer flag
  update_motor = 1;
}

// Handler for the external interrupt channel 0 (where we will check the Hall sensor)
void xirq_handler_ch0(void) {
  encoder_status = 1;
  // print a warning
  neorv32_uart0_puts("Encoder status updated.\n");
}

// encoder handling
void encoder_handler() {
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

void read_and_convert_current() {
  if (read_current) {
    // Get the current values
    three_phase = get_current_ab();
    quadrature = get_clark_transform(three_phase);
    rotated = get_park_transform(quadrature);
    reference_voltage = update_control(rotated);
    quadrature_voltage = get_inverse_park_transform(rotated);
    duty_cycle = get_space_vector(quadrature_voltage);
    motor_control(duty_cycle, pwm_a, pwm_b, pwm_c);
    // Reset the flag
    read_current = 0;
  }
}

void prvMotorTask(void *pvParameters)
{
    // print a warning
    neorv32_uart0_puts("Motor task started.\n");
    // Loop indefinitely
    while (1)
    {
      // Move the motor based on the timer callback
      move_clockwise();
      // Encoder handling
      encoder_handler();
      // Read and convert the current
      read_and_convert_current();
    }
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

    // print a warning
    neorv32_uart0_puts("Motor aligned.\n");
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


void createTimers(void)
{
    // Create both timers
    TimerHandle_t xCurTimer, xMotorMoveTimer;

    // Create CurTimer
    neorv32_uart0_puts("Creating Cur timer.\n");
    const TickType_t xCurTimerPeriod = pdMS_TO_TICKS(100); // 100Hz
    xCurTimer = xTimerCreate(
        "CurTimer",          // Timer name
        xCurTimerPeriod,     // Timer period (10ms)
        pdTRUE,              // Auto-reload timer
        (void *)0,           // Timer ID
        vTimerGetCur         // Callback function for CurTimer
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
    const TickType_t xMotorMoveTimerPeriod = pdMS_TO_TICKS(200); // 50Hz
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

void createMotorTask(void)
{
  // print an warning
  neorv32_uart0_puts("Creating motor task.\n");
  // Create the task
  xTaskCreate(prvMotorTask, "MotorTask", configMINIMAL_STACK_SIZE, NULL, mainMotorTask_PRIORITY, NULL);
  // print an warning
  neorv32_uart0_puts("Motor task created.\n");
}

void setup_xirq(void) {

  if (neorv32_xirq_available() == 0) {
    neorv32_uart0_printf("XIRQ not synthesized!\n");
  }

  int err_cnt = 0;

  // initialize XIRQ controller
  // this will disable all XIRQ channels and will also clear any pending external interrupts
  // (details: this will register the XIRQ's second-level interrupt handler in the NEORV32 RTE)
  err_cnt = neorv32_xirq_setup();

  // check if setup went fine
  if (err_cnt) {
    neorv32_uart0_printf("Error during XIRQ setup!\n");
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
  }

  // enable XIRQ channels
  neorv32_xirq_channel_enable(0);

  // allow XIRQ to trigger CPU interrupt
  neorv32_xirq_global_enable();
}



// ----------------------------------------------------------------
// ----------------- Motor control functions ----------------------
// ----------------------------------------------------------------

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

current_clark get_clark_transform(current_ab cur_ab){
  current_clark res;
  res.cur_alpha.float_value = cur_ab.cur_a.float_value;
  res.cur_beta.float_value = riscv_emulate_fdivs(cur_ab.cur_a.float_value,
  riscv_intrinsic_fadds(sqrt(3), 
  riscv_intrinsic_fmuls(2, 
  riscv_emulate_fdivs(cur_ab.cur_b.float_value, 
  sqrt(3))
  )
  )
  );
  // res.cur_beta.float_value = riscv_intrinsic_fdivs(cur_ab.cur_a.float_value,riscv_intrinsic_fadds(sqrt(3) , riscv_intrinsic_fmuls(2, riscv_intrinsic_fdivs(cur_ab.cur_b.float_value, sqrt(3)))));
  return res;
}


current_park get_park_transform(current_clark cur_clark) {
  current_park res;

  res.cur_d.float_value = riscv_intrinsic_fadds(riscv_intrinsic_fmuls(cos(current_angle.float_value), cur_clark.cur_alpha.float_value),
              riscv_intrinsic_fmuls(sin(current_angle.float_value), cur_clark.cur_beta.float_value));
  res.cur_q.float_value = riscv_intrinsic_fadds(riscv_intrinsic_fmuls(-sin(current_angle.float_value), cur_clark.cur_alpha.float_value),
              riscv_intrinsic_fmuls(cos(current_angle.float_value), cur_clark.cur_beta.float_value));
  return res;
}

voltage_pi update_control(current_park cur_park) {
  voltage_pi res;
  id_error.float_value = riscv_intrinsic_fsubs(id_ref.float_value, cur_park.cur_d.float_value);
  iq_error.float_value = riscv_intrinsic_fsubs(iq_ref.float_value, cur_park.cur_q.float_value);

  id_integrator.float_value = riscv_intrinsic_fadds(id_integrator.float_value, id_error.float_value);
  iq_integrator.float_value = riscv_intrinsic_fadds(iq_integrator.float_value, iq_error.float_value);

  res.v_d.float_value = riscv_intrinsic_fadds(riscv_intrinsic_fmuls(kp.float_value, id_error.float_value),
              riscv_intrinsic_fmuls(ki.float_value, id_integrator.float_value));
  res.v_q.float_value = riscv_intrinsic_fadds(riscv_intrinsic_fmuls(kp.float_value, iq_error.float_value),
              riscv_intrinsic_fmuls(ki.float_value, iq_integrator.float_value));

  // clamp id integrator
  if (!riscv_intrinsic_flts(id_integrator.float_value, integrator_max.float_value)) {
    id_integrator.float_value = integrator_max.float_value;
  } else if (riscv_intrinsic_flts(id_integrator.float_value, -integrator_max.float_value)) {
    id_integrator.float_value = -integrator_max.float_value;
  }


  // clamp iq integrator
  if (!riscv_intrinsic_flts(iq_integrator.float_value, integrator_max.float_value)) {
      iq_integrator.float_value = integrator_max.float_value;
  } else if (riscv_intrinsic_flts(iq_integrator.float_value, -integrator_max.float_value)) {
      iq_integrator.float_value = -integrator_max.float_value;
  }


  return res;
}

voltage_clark get_inverse_park_transform(current_park cur_park) {
  voltage_clark res;
  res.v_alpha.float_value = riscv_emulate_fsubs(
    riscv_emulate_fmuls(cos(current_angle.float_value), cur_park.cur_d.float_value),
    riscv_emulate_fmuls(sin(current_angle.float_value), cur_park.cur_q.float_value)
);

res.v_beta.float_value = riscv_emulate_fadds(
    riscv_emulate_fmuls(sin(current_angle.float_value), cur_park.cur_d.float_value),
    riscv_emulate_fmuls(cos(current_angle.float_value), cur_park.cur_q.float_value)
);

  return res;
}

space_vector get_space_vector(voltage_clark cur_clark) {
  space_vector res;
  float_conv_t v_ref;
  float_conv_t theta_ref;
  float_conv_t ts;
  float_conv_t angle_in_sector;
  float_conv_t t1, t2, t0;
  float_conv_t duty_a, duty_b, duty_c;

  // Step 1: Calculate V_ref and theta_ref (in radians)
  v_ref.float_value = riscv_emulate_fsqrts(
      riscv_emulate_fadds(
          riscv_emulate_fmuls(cur_clark.v_alpha.float_value, cur_clark.v_alpha.float_value),
          riscv_emulate_fmuls(cur_clark.v_beta.float_value, cur_clark.v_beta.float_value)
      )
  );                 
  theta_ref.float_value = atan2(cur_clark.v_beta.float_value, cur_clark.v_alpha.float_value);

  // Step 2: Determine the sector (radians divided by pi/3, which is 60 degrees)
  //uint8_t sector = (uint8_t)(floor(theta_ref.float_value / (M_PI / 3))) + 1;
  uint8_t sector = (uint8_t)(floor(
    riscv_emulate_fdivs(theta_ref.float_value, (float)(M_PI / 3))
  )) + 1;
  if (sector > 6) {
    sector = 6; // Sector must be between 1 and 6
  }

  // Step 3: Calculate switching times T1, T2, t0.float_value based on the sector
  ts.float_value = 1.0 / PWM_FREQ; // PWM period (1 MHz frequency as you specified)
  //angle_in_sector.float_value =
      //theta_ref.float_value - (sector - 1) * (M_PI / 3); // Relative angle within sector
  angle_in_sector.float_value = riscv_emulate_fsubs(
    theta_ref.float_value,
    riscv_emulate_fmuls(
        (float)(sector - 1),
        (float)(M_PI / 3)
    )
  );

t1.float_value = riscv_emulate_fdivs(
    riscv_emulate_fmuls(
        ts.float_value,
        riscv_emulate_fmuls(
            v_ref.float_value,
            sin((M_PI / 3) - angle_in_sector.float_value)
        )
    ),
    MOTOR_VOLTAGE
  ); // Time for first active vector
t2.float_value = riscv_emulate_fdivs(
    riscv_emulate_fmuls(
        ts.float_value,
        riscv_emulate_fmuls(
            v_ref.float_value,
            sin(angle_in_sector.float_value)
        )
    ),
    MOTOR_VOLTAGE
  ); // Time for second active vector
  
  t0.float_value = riscv_emulate_fsubs(
    ts.float_value,
    riscv_emulate_fadds(t1.float_value, t2.float_value)
  ); // Zero vector time (remaining time)

  // Step 4: Calculate duty cycles for phases A, B, C
  switch (sector) {
  case 1:
    duty_a.float_value = riscv_emulate_fdivs(
    riscv_emulate_fadds(
        riscv_emulate_fadds(t1.float_value, t2.float_value),
        riscv_emulate_fdivs(t0.float_value, 2)
    ),
    ts.float_value
  );
    duty_b.float_value = riscv_emulate_fdivs(
    riscv_emulate_fadds(
        t2.float_value,
        riscv_emulate_fdivs(t0.float_value, 2)
    ),
    ts.float_value
  );
    duty_c.float_value = riscv_emulate_fdivs(
    t0.float_value,
    riscv_emulate_fmuls(2, ts.float_value)
  );
    break;
  case 2:
    duty_a.float_value = riscv_emulate_fdivs(
    riscv_emulate_fadds(
        t2.float_value,
        riscv_emulate_fdivs(t0.float_value, 2)
    ),
    ts.float_value
  );
    duty_b.float_value = riscv_emulate_fdivs(
    riscv_emulate_fadds(
        riscv_emulate_fadds(t1.float_value, t2.float_value),
        riscv_emulate_fdivs(t0.float_value, 2)
    ),
    ts.float_value
  );
    duty_c.float_value = riscv_emulate_fdivs(
    t0.float_value,
    riscv_emulate_fmuls(2, ts.float_value)
  );
    break;
  case 3:
    duty_a.float_value = riscv_emulate_fdivs(
    t0.float_value,
    riscv_emulate_fmuls(2, ts.float_value)
  );
    duty_b.float_value = riscv_emulate_fdivs(
    riscv_emulate_fadds(
        riscv_emulate_fadds(t1.float_value, t2.float_value),
        riscv_emulate_fdivs(t0.float_value, 2)
    ),
    ts.float_value
  );
    duty_c.float_value = riscv_emulate_fdivs(
    riscv_emulate_fadds(
        t2.float_value,
        riscv_emulate_fdivs(t0.float_value, 2)
    ),
    ts.float_value
  );
    break;
  case 4:
    duty_a.float_value = riscv_emulate_fdivs(
    t0.float_value,
    riscv_emulate_fmuls(2, ts.float_value)
  );
    duty_b.float_value = riscv_emulate_fdivs(
    t0.float_value,
    riscv_emulate_fmuls(2, ts.float_value)
  );
    duty_c.float_value = riscv_emulate_fdivs(
    riscv_emulate_fadds(
        riscv_emulate_fadds(t1.float_value, t2.float_value),
        riscv_emulate_fdivs(t0.float_value, 2)
    ),
    ts.float_value
  );
    break;
  case 5:
    duty_a.float_value = riscv_emulate_fdivs(
    riscv_emulate_fadds(
        t2.float_value,
        riscv_emulate_fdivs(t0.float_value, 2)
    ),
    ts.float_value
  );
    duty_b.float_value = riscv_emulate_fdivs(
    t0.float_value,
    riscv_emulate_fmuls(2, ts.float_value)
  );
    duty_c.float_value = riscv_emulate_fdivs(
    riscv_emulate_fadds(
        riscv_emulate_fadds(t1.float_value, t2.float_value),
        riscv_emulate_fdivs(t0.float_value, 2)
    ),
    ts.float_value
  );
    break;
  case 6:
    duty_a.float_value = riscv_emulate_fdivs(
    riscv_emulate_fadds(
        riscv_emulate_fadds(t1.float_value, t2.float_value),
        riscv_emulate_fdivs(t0.float_value, 2)
    ),
    ts.float_value
  );
    duty_b.float_value = riscv_emulate_fdivs(
    t0.float_value,
    riscv_emulate_fmuls(2, ts.float_value)
  );
    duty_c.float_value = riscv_emulate_fdivs(
    riscv_emulate_fadds(
        t2.float_value,
        riscv_emulate_fdivs(t0.float_value, 2)
    ),
    ts.float_value
  );
    break;
  default:
    duty_a.float_value = 0;
    duty_b.float_value = 0;
    duty_c.float_value = 0;
    break;
  }

  // Step 5: Store the results in the structure
  res.duty_a.float_value = duty_a.float_value;
  res.duty_b.float_value = duty_b.float_value;
  res.duty_c.float_value = duty_c.float_value;

  return res;
}

void motor_control(space_vector duty_cycle, pwm_config_space_vector pwm_a,
                   pwm_config_space_vector pwm_b,
                   pwm_config_space_vector pwm_c) {
}