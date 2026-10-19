#include "lift_safety.h"
#include "lift_lock.h"
#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <sched.h>
static atomic_int inside, start, waiting, wrote;
static void *contender(void *argument)
{
    lift_owner_t owner = (lift_owner_t)(uintptr_t)argument;
    while (!atomic_load(&start)) sched_yield();
    for (unsigned i=0; i<40000; ++i) {
        if (!lift_owner_claim(owner,1000)) continue;
        assert(atomic_fetch_add(&inside,1)==0);
        assert(lift_owner_current()==owner);
        lift_owner_release(owner==LIFT_OWNER_CONTROL ? LIFT_OWNER_JOG : LIFT_OWNER_CONTROL);
        assert(lift_owner_current()==owner);
        assert(atomic_fetch_sub(&inside,1)==1);
        lift_owner_release(owner);
    }
    return NULL;
}
static void *preempt(void *unused)
{
    lift_owner_t previous;
    (void)unused; atomic_store(&waiting,1);
    assert(lift_owner_claim_override(LIFT_OWNER_CAN_TEST,2,&previous));
    assert(atomic_load(&wrote)==1 && previous==LIFT_OWNER_NONE);
    lift_owner_release(LIFT_OWNER_CAN_TEST);
    return NULL;
}
int main(void)
{
    pthread_t threads[3], competitor;
    lift_owner_t previous;
    lift_owner_reset();
    for(unsigned i=0;i<3;i++) assert(!pthread_create(&threads[i],NULL,contender,(void *)(uintptr_t)(i+1)));
    atomic_store(&start,1);
    for(unsigned i=0;i<3;i++) assert(!pthread_join(threads[i],NULL));
    assert(lift_owner_current()==LIFT_OWNER_NONE && atomic_load(&inside)==0);
    assert(!lift_owner_claim((lift_owner_t)99,1));
    assert(lift_owner_claim(LIFT_OWNER_CONTROL,1));
    assert(lift_owner_claim_override(LIFT_OWNER_JOG,2,&previous) && previous==LIFT_OWNER_CONTROL);
    lift_owner_release(LIFT_OWNER_CONTROL);lift_owner_release(LIFT_OWNER_CONTROL);
    assert(lift_owner_current()==LIFT_OWNER_JOG);
    assert(lift_owner_claim_override(LIFT_OWNER_CAN_TEST,3,&previous) && previous==LIFT_OWNER_JOG);
    assert(!lift_owner_claim_override(LIFT_OWNER_JOG,4,&previous));
    lift_owner_release(LIFT_OWNER_JOG);assert(lift_owner_current()==LIFT_OWNER_CAN_TEST);
    lift_owner_release(LIFT_OWNER_CAN_TEST);lift_owner_release(LIFT_OWNER_CAN_TEST);
    assert(lift_owner_current()==LIFT_OWNER_NONE);
    lift_owner_set_timeout_ms(50);
    assert(lift_owner_claim(LIFT_OWNER_JOG,UINT32_MAX-10U));
    lift_owner_tick(39);assert(lift_owner_current()==LIFT_OWNER_JOG); /* exactly 50ms */
    lift_owner_tick(40);assert(lift_owner_current()==LIFT_OWNER_NONE);
    assert(lift_owner_claim(LIFT_OWNER_JOG,100));
    assert(lift_owner_claim(LIFT_OWNER_JOG,160));
    lift_owner_tick(150);assert(lift_owner_current()==LIFT_OWNER_JOG); /* stale sampled now */
    lift_owner_tick(210);assert(lift_owner_current()==LIFT_OWNER_JOG);
    lift_owner_tick(211);assert(lift_owner_current()==LIFT_OWNER_NONE);
    lift_owner_set_timeout_ms(0);assert(lift_owner_claim(LIFT_OWNER_CAN_TEST,1));
    lift_owner_tick(100000);assert(lift_owner_current()==LIFT_OWNER_CAN_TEST);
    lift_owner_reset();
    {
        uint32_t key=lift_lock_enter();
        assert(lift_owner_claim(LIFT_OWNER_CONTROL,1));
        assert(!pthread_create(&competitor,NULL,preempt,NULL));
        while(!atomic_load(&waiting)) sched_yield();
        assert(lift_owner_current()==LIFT_OWNER_CONTROL);
        atomic_store(&wrote,1);
        lift_owner_release(LIFT_OWNER_CONTROL);
        lift_lock_exit(key);
        assert(!pthread_join(competitor,NULL));
    }
    puts("PASS: 120000 competing claims, wrong/repeat release, atomic handover, timeout/wrap/stale tick, nested cache transaction");
    return 0;
}
