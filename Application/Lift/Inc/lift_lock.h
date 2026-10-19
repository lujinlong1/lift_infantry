#ifndef LIFT_LOCK_H
#define LIFT_LOCK_H
#include <stdint.h>
/* Short, nestable single-core exclusion, including before scheduler startup.
 * Never hold over controller math, a history copy, HAL TX, or a blocking call.
 * Save/restore PRIMASK: nesting must not enable previously masked interrupts. */
#if defined(__arm__) || defined(__thumb__)
static inline uint32_t lift_lock_enter(void)
{
    uint32_t key;
    __asm volatile ("mrs %0, primask\n cpsid i" : "=r"(key) :: "memory");
    return key;
}
static inline void lift_lock_exit(uint32_t key)
{
    __asm volatile ("dmb\n msr primask, %0" :: "r"(key) : "memory");
}
#else
/* The host harness uses a recursive atomic lock, never a no-op RTOS stub. */
uint32_t lift_lock_enter(void);
void lift_lock_exit(uint32_t key);
#endif
#endif
