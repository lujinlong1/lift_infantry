#ifndef DSH_TEST_STUB_FREERTOS_H
#define DSH_TEST_STUB_FREERTOS_H

#include <stdint.h>

/* Host-test stub of the FreeRTOS types/macros used by lift_jog_task.c. */

typedef uint32_t TickType_t;
typedef uint32_t StackType_t;

typedef struct
{
    void    *dummy;
} StaticTask_t;

#define portTICK_PERIOD_MS   1U
#define pdMS_TO_TICKS(ms)    ((TickType_t)(ms))

#endif
