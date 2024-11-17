#ifndef neorv32_sector_h
#define neorv32_sector_h

#include <stdint.h>


/**********************************************************************//**
 * @name HALL SECTOR
 **************************************************************************/
/**@{*/
/** SECTOR module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  const uint32_t SECTOR;  /**< offset 0: parallel input port, read-only */
} neorv32_sector_t;

/** SECTOR module hardware access (#neorv32_sector_t) */
#define NEORV32_SECTOR ((neorv32_sector_t*) (NEORV32_SECTOR_BASE))
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
uint32_t neorv32_sector_get(void);
/**@}*/


#endif // neorv32_sector_h
