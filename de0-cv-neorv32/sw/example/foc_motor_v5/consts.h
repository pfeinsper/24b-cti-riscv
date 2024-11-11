#ifndef CONSTS_H
#define CONSTS_H

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

// config gpio pins
#define IN1 0
#define IN2 1
#define IN3 2
#define EN1 3
#define EN2 4
#define EN3 5

// PWM frequency
#define PWM_FREQ 1e6
#define PWM_RES 255

// motor voltage
#define MOTOR_VOLTAGE 12

/* Priorities used by the tasks. */
#define mainMotorTask_PRIORITY    ( tskIDLE_PRIORITY + 1 )


extern const float_conv_t conversion_factor;

extern volatile uint8_t update_motor;
extern volatile uint8_t read_current;
extern volatile uint8_t init_control;
extern volatile uint8_t encoder_status;
extern volatile uint8_t step_index;

extern volatile float_conv_t current_angle;

extern uint8_t in_seq[6][3];
extern uint8_t en_seq[6][3];

extern float_conv_t id_ref;
extern float_conv_t iq_ref;

extern float_conv_t kp;
extern float_conv_t ki;

extern float_conv_t id_error, iq_error;
extern float_conv_t id_integrator;
extern float_conv_t iq_integrator;
extern float_conv_t id_output, iq_output;

extern float_conv_t integrator_max;


#endif // CONSTS_H 