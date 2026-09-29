#ifndef SYSTEM_TASK_H
#define SYSTEM_TASK_H

#include <stdint.h>

void StartSystemTask(void *argument);
void board_led_set(uint16_t red, uint16_t green, uint16_t blue);
void board_led_startup_self_test(void);
void board_led_show_error(void);

#endif
