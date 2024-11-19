#include <neorv32.h>

/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200

#define IN1 0
#define IN2 1
#define IN3 2
#define EN1 3
#define EN2 4
#define EN3 5

// things for the motor control
uint8_t in_seq[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
volatile uint32_t counter_led = 0;
volatile uint32_t counter = 0;

void gptmr_firq_handler(void);

int main() {

// setup NEORV32 runtime environment (for trap handling)
  neorv32_rte_setup();

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

    // Intro
  neorv32_uart0_puts("test.\n"
                     "Test everything that we will use.\n\n");

  // clear GPIO output port
  neorv32_gpio_port_set(0);

    // install GPTMR interrupt handler
  neorv32_rte_handler_install(GPTMR_RTE_ID, gptmr_firq_handler);

  // configure timer for 0.5Hz in continuous mode (with clock divisor = 8)
  neorv32_gptmr_setup(CLK_PRSC_8, neorv32_sysinfo_get_clk() / (8 * 2), 1);

  // enable interrupt
  neorv32_cpu_csr_clr(CSR_MIP, 1 << GPTMR_FIRQ_PENDING);  // make sure there is no GPTMR IRQ pending already
  neorv32_cpu_csr_set(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);   // enable GPTMR FIRQ channel
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

  // go to sleep mode and wait for interrupt
  while(1) {
  neorv32_cpu_sleep();
}

}

void gptmr_firq_handler(void) {

  neorv32_gptmr_irq_ack(); // clear/ack pending FIRQ

  counter = neorv32_sector_get();

  neorv32_uart0_printf("Counter: %i\n", counter);

}