#ifndef CAN_RX_SERVICE_H
#define CAN_RX_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CAN_BUS_1 = 0,
    CAN_BUS_2
} can_bus_t;

typedef struct
{
    can_bus_t bus;
    uint16_t std_id;
    uint8_t dlc;
    uint8_t data[8];
} can_rx_frame_t;

typedef struct
{
    volatile uint32_t total_count[2];
    volatile uint16_t last_std_id[2];
    volatile uint8_t last_dlc[2];
    volatile uint8_t last_data[2][8];
    volatile uint32_t id_001_count[2];
    volatile uint32_t id_011_count[2];
    volatile uint32_t dji_id_count[2][8];
} can_rx_debug_t;

extern can_rx_debug_t g_can_rx_debug;

bool can_rx_service_init(void);
bool can_rx_service_task_create(void);
bool can_rx_service_push_from_isr(const can_rx_frame_t *frame);

uint32_t can_rx_service_get_received_count(void);
uint32_t can_rx_service_get_dropped_count(void);

/*
 * 弱实现，后续由 can_dispatch.c 提供同名强实现。
 */
void can_rx_service_dispatch(const can_rx_frame_t *frame);

#endif
