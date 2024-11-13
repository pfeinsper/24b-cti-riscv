#ifndef neorv32_counter_h
#define neorv32_counter_h

#include <stdint.h>


/**********************************************************************//**
 * @name IO Device: General Purpose Input/Output Port Unit (COUNTER)
 **************************************************************************/
/**@{*/
/** COUNTER module prototype */
typedef volatile struct __attribute__((packed,aligned(4))) {
  const uint32_t COUNT;  /**< offset 0: parallel input port, read-only */
} neorv32_counter_t;

/** COUNTER module hardware access (#neorv32_counter_t) */
#define NEORV32_COUNTER ((neorv32_counter_t*) (NEORV32_COUNTER_BASE))
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
uint32_t neorv32_counter_get(void);
/**@}*/


#endif // neorv32_counter_h
