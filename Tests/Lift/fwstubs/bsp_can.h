#ifndef DSH_FWCHECK_BSP_CAN_H
#define DSH_FWCHECK_BSP_CAN_H

#include <stdbool.h>
#include <stdint.h>

/* Host compile-check stub of Bsp/Inc/bsp_can.h (unused by the lift chain). */

bool bsp_can_send(void *handle, uint32_t id, const uint8_t *data, uint8_t len);

#endif
