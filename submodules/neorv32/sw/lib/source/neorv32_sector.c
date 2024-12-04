#include "neorv32.h"

uint32_t neorv32_sector_get(void) {

  uint32_t data;

  data = NEORV32_SECTOR->SECTOR;

  return data;
}
