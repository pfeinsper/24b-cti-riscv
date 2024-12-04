#include "consts.h"


const float_conv_t conversion_factor = {.float_value = 3.3f / (1 << 12)}; // this is not the right way to calculate the conversion factor but works for now

volatile uint8_t update_motor = 0;
volatile uint8_t read_current = 0;
volatile uint8_t init_control = 0;
volatile uint8_t encoder_status = 0;
volatile uint8_t step_index = 0;

volatile float_conv_t current_angle = { .float_value = 0.0 };

// things for the motor control
uint8_t in_seq[6][3] = {{1, 0, 0}, {0, 1, 0}, {0, 1, 0},
                        {0, 0, 1}, {0, 0, 1}, {1, 0, 0}};
uint8_t en_seq[6][3] = {{1, 0, 1}, {0, 1, 1}, {1, 1, 0},
                        {1, 0, 1}, {0, 1, 1}, {1, 1, 0}};


float_conv_t id_ref = { .float_value = 0.0 };
float_conv_t iq_ref = { .float_value = 0.5 };

float_conv_t kp = { .float_value = 0.05 };
float_conv_t ki = { .float_value = 200.0 };

float_conv_t id_error, iq_error;
float_conv_t id_integrator = { .float_value = 0.0 };
float_conv_t iq_integrator = { .float_value = 0.0 };
float_conv_t id_output, iq_output;

float_conv_t integrator_max = { .float_value = 10 };