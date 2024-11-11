#ifndef UTILS_H
#define UTILS_H

#include "consts.h"


typedef struct {
  float_conv_t cur_a;
  float_conv_t cur_b;
} current_ab;

typedef struct {
  float_conv_t cur_alpha;
  float_conv_t cur_beta;
} current_qd;

typedef struct {
  float_conv_t cur_alpha;
  float_conv_t cur_beta;
} current_clark;

typedef struct {
  float_conv_t cur_d;
  float_conv_t cur_q;
} current_park;

typedef struct {
  float_conv_t v_q;
  float_conv_t v_d;
} voltage_pi;

typedef struct {
  float_conv_t v_alpha;
  float_conv_t v_beta;
} voltage_clark;

typedef struct {
  float_conv_t duty_a;
  float_conv_t duty_b;
  float_conv_t duty_c;
} space_vector;

typedef struct {
  uint32_t slice_num;
  uint32_t chan_num;
} pwm_config_space_vector;


/**@}*/

// timers
void vTimerGetCur(TimerHandle_t xTimer);
void vTimerMotorMove(TimerHandle_t xTimer);
void vTimerInitControl(TimerHandle_t xTimer);

// tasks
void prvMotorTask(void *pvParameters);

/**@}*/

// Prototypes
void setup_xirq(void);
void xirq_handler_ch0(void);
void align_rotor();
void move_clockwise();
void move_clockwise_pwm();
void read_and_convert_current();
void createCurTimer();
void createMotorTask();
void createTimers();
current_ab get_current_ab();
current_clark get_clark_transform(current_ab cur_ab);
current_park get_park_transform(current_clark cur_clark);
voltage_pi update_control(current_park cur_park);
voltage_clark get_inverse_park_transform(current_park cur_park);
space_vector get_space_vector(voltage_clark cur_clark);
void motor_control(space_vector duty_cycle, pwm_config_space_vector pwm_a,
                   pwm_config_space_vector pwm_b,
                   pwm_config_space_vector pwm_c);





#endif // UTILS_H