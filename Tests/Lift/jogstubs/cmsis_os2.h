#ifndef DSH_TEST_STUB_CMSIS_OS2_H
#define DSH_TEST_STUB_CMSIS_OS2_H

#include <stddef.h>
#include <stdint.h>

/* Host-test stub of the tiny slice of CMSIS-RTOS2 that lift_jog_task.c uses.
 * osThreadNew returns a dummy non-NULL handle; the task body is never run. */

typedef void *osThreadId_t;

typedef enum
{
    osPriorityNone = 0,
    osPriorityLow,
    osPriorityBelowNormal,
    osPriorityNormal
} osPriority_t;

typedef struct
{
    const char   *name;
    void         *cb_mem;
    uint32_t      cb_size;
    void         *stack_mem;
    uint32_t      stack_size;
    osPriority_t  priority;
} osThreadAttr_t;

osThreadId_t osThreadNew(void (*func)(void *), void *argument,
                         const osThreadAttr_t *attr);
uint32_t osKernelGetTickCount(void);

#endif
