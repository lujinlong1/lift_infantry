#ifndef DSH_FWCHECK_FREERTOS_H
#define DSH_FWCHECK_FREERTOS_H

#include <stdint.h>

/* Host compile-check stub of the FreeRTOS types/macros the lift task uses. */

typedef uint32_t TickType_t;
typedef uint32_t StackType_t;
typedef uint32_t UBaseType_t;

typedef struct
{
    void *dummy;
} StaticTask_t;

#define portTICK_PERIOD_MS   1U
#define pdMS_TO_TICKS(ms)    ((TickType_t)(ms))

#define taskENTER_CRITICAL()   do { } while (0)
#define taskEXIT_CRITICAL()    do { } while (0)

#endif
