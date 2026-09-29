#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"

#include <stdint.h>

typedef enum
{
    BSP_CAN_OK = 0,
    BSP_CAN_INVALID_ARGUMENT,
    BSP_CAN_FILTER_ERROR,
    BSP_CAN_START_ERROR,
    BSP_CAN_NOTIFICATION_ERROR
} bsp_can_status_t;

typedef enum
{
    BSP_CAN_TX_OK = 0,
    BSP_CAN_TX_BUSY,
    BSP_CAN_TX_INVALID_ARGUMENT,
    BSP_CAN_TX_HAL_ERROR
} bsp_can_tx_status_t;

typedef struct
{
    volatile uint32_t error_count[2];
    volatile uint32_t bus_off_count[2];
    volatile uint32_t last_error[2];
} bsp_can_error_stats_t;

extern bsp_can_error_stats_t g_bsp_can_error_stats;

bsp_can_status_t bsp_can_init(void);

bsp_can_tx_status_t bsp_can_send(
    CAN_HandleTypeDef *hcan,
    uint16_t std_id,
    const uint8_t *data,
    uint8_t dlc);

#endif
