#ifndef DBUS_REMOTE_H
#define DBUS_REMOTE_H
#include <stdbool.h>
#include <stdint.h>
#define DBUS_SWITCH_UP 1U
#define DBUS_SWITCH_DOWN 2U
#define DBUS_SWITCH_MID 3U
typedef struct { int16_t channel[5]; uint8_t switch_right; uint8_t switch_left; int16_t mouse_x,mouse_y,mouse_z; bool mouse_left,mouse_right; uint16_t keyboard; uint32_t last_update_tick; uint32_t frame_count; uint32_t invalid_count; } dbus_remote_state_t;
extern volatile dbus_remote_state_t g_dbus_remote;
bool dbus_remote_init(void);
bool dbus_remote_is_online(uint32_t timeout_ms);
void dbus_remote_get_snapshot(dbus_remote_state_t *snapshot);
#endif
