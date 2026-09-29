#ifndef CAN_TX_SERVICE_H
#define CAN_TX_SERVICE_H

#include "can_rx_service.h"

#include <stdint.h>

typedef enum
{
    CAN_TX_SERVICE_OK = 0,
    CAN_TX_SERVICE_BUSY,
    CAN_TX_SERVICE_INVALID_ARGUMENT,
    CAN_TX_SERVICE_HAL_ERROR
} can_tx_service_status_t;

typedef struct
{
    uint32_t sent_count;
    uint32_t busy_count;
    uint32_t invalid_count;
    uint32_t error_count;
} can_tx_service_stats_t;

void can_tx_service_init(void);

can_tx_service_status_t can_tx_service_send(
    can_bus_t bus,
    uint16_t std_id,
    const uint8_t *data,
    uint8_t dlc);

void can_tx_service_get_stats(can_tx_service_stats_t *stats);

#endif

