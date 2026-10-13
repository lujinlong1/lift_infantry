#ifndef DSH_FWCHECK_CMSIS_OS2_H
#define DSH_FWCHECK_CMSIS_OS2_H

#include <stddef.h>
#include <stdint.h>

/*
 * Host compile-check stub of the tiny slice of CMSIS-RTOS2 that the lift task
 * uses. Nothing is linked or run by compile_check.ps1; this only has to be
 * type-correct.
 */

typedef void *osThreadId_t;

typedef enum
{
    osPriorityNone = 0,
    osPriorityLow,
    osPriorityBelowNormal,
    osPriorityNormal,
    osPriorityHigh
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
