#ifndef AUTO_AIM_LINK_H
#define AUTO_AIM_LINK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    volatile bool online;
    volatile bool frame_valid;
    volatile bool fire_requested;
    volatile bool target_present_heuristic;
    volatile float pitch_deg;
    volatile float yaw_deg;
    volatile uint32_t frame_number;
    volatile uint32_t last_update_tick;
    volatile uint32_t valid_frame_count;
    volatile uint32_t crc_error_count;
    volatile uint32_t format_error_count;
    volatile uint32_t overflow_count;
    volatile uint32_t sequence_gap_count;
    volatile uint32_t timeout_count;
} auto_aim_status_t;

extern auto_aim_status_t g_auto_aim_status;

void auto_aim_link_init(void);
bool auto_aim_link_task_create(void);
void auto_aim_link_feed(const uint8_t *data, uint32_t length);
uint16_t auto_aim_crc16_modbus(const uint8_t *data, uint32_t length);

#endif
