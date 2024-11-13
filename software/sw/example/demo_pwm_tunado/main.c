/**********************************************************************//**
 * @file demo_pwm/main.c
 * @author Stephan Nolting
 * @brief Enhanced PWM demo program with multi-channel modulation.
 **************************************************************************/

#include <neorv32.h>

/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/** Maximum PWM output intensity (8-bit) */
#define PWM_MAX 255
/** Number of PWM channels to modulate simultaneously */
#define NUM_PWM_CHANNELS 4
/**@}*/

/**********************************************************************//**
 * This program generates an enhanced dimming sequence for PWM channels 0 to 3
 * with varying speeds for each channel.
 *
 * @note This program requires the PWM controller to be synthesized (the UART is optional).
 *
 * @return !=0 if error.
 **************************************************************************/
int main() {

  // Capture all exceptions and give debug info via UART
  neorv32_rte_setup();

  // Use UART0 if implemented
  if (neorv32_uart0_available()) {
    // Setup UART at default baud rate, no interrupts
    neorv32_uart0_setup(BAUD_RATE, 0);
    // Say hello
    neorv32_uart0_printf("<<< Enhanced PWM demo program >>>\n");
  }

  // Check if PWM unit is implemented
  if (neorv32_pwm_available() == 0) {
    if (neorv32_uart0_available()) {
      neorv32_uart0_printf("ERROR: PWM module not implemented!\n");
    }
    return 1;
  }

  int num_pwm_channels = neorv32_pmw_get_num_channels();

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

  uint8_t pwm[NUM_PWM_CHANNELS] = {0, 64, 128, 192}; // Start with different initial values
  uint8_t direction[NUM_PWM_CHANNELS] = {1, 1, 1, 1}; // 1 for increasing, 0 for decreasing
  uint8_t speed[NUM_PWM_CHANNELS] = {2, 3, 4, 5}; // Different speeds for each channel

  // Animate!
  while (1) {
    for (int ch = 0; ch < NUM_PWM_CHANNELS; ch++) {
      // Update duty cycle
      if (direction[ch]) {
        pwm[ch] += speed[ch];
        if (pwm[ch] >= PWM_MAX) {
          pwm[ch] = PWM_MAX;
          direction[ch] = 0;
        }
      } else {
        pwm[ch] -= speed[ch];
        if (pwm[ch] <= 0) {
          pwm[ch] = 0;
          direction[ch] = 1;
        }
      }
      // Set PWM output for the channel
      neorv32_pwm_set(ch, pwm[ch]);
    }

    neorv32_cpu_delay_ms(10); // Adjust the delay to control the overall speed
  }

  return 0;
}
