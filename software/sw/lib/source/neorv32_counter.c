#include "neorv32.h"

uint32_t neorv32_counter_get(void) {

  uint32_t data;

  data = NEORV32_COUNTER->COUNT;

  return data;
}
