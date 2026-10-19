#include "lift_diagnostics.h"
#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <sched.h>
static atomic_int completed;
static atomic_uint snapshots, records, transports;
static void check_frame(const lift_diag_frame_t *f)
{
    assert(f->cycle_id!=0);
    assert(f->tick_ms==f->cycle_id*5U);
    for(unsigned ch=0;ch<4;ch++) {
        assert(f->axis[ch].position_ecd==(int32_t)(f->cycle_id*10U+ch));
        assert(f->axis[ch].reference_ecd==(float)(f->cycle_id*10U+ch));
        assert(f->parameter_version==f->cycle_id);
    }
}
static void *publisher(void *unused)
{
    lift_diag_frame_t f={0};(void)unused;
    for(unsigned i=1;i<=16000;i++) {
        f.cycle_id=f.parameter_version=i;f.tick_ms=i*5;
        f.reasons=i%97U==0 ? LIFT_DIAG_BIT(LIFT_DIAG_STALL) : 0;
        for(unsigned ch=0;ch<4;ch++) f.axis[ch].reference_ecd=(float)(f.axis[ch].position_ecd=(int32_t)(i*10+ch));
        lift_diag_publish(&f);
        if(i%19==0)sched_yield();
    }
    atomic_fetch_add(&completed,1);return NULL;
}
static void *cache_writer(void *unused)
{
    (void)unused;
    for(unsigned i=1;i<=16000;i++) {
        int16_t raw[4]={(int16_t)i,-(int16_t)i,123,-123};
        lift_diag_cache(raw,i,i,LIFT_DIAG_SOURCE_CONTROL);
    }
    atomic_fetch_add(&completed,1);return NULL;
}
static void *tx_writer(void *unused)
{
    (void)unused;
    for(unsigned i=1;i<=16000;i++) {
        int16_t raw[4]={(int16_t)i,-(int16_t)i,456,-456};
        lift_diag_tx(raw,i,i,i,(uint8_t)(i%4),false,false);
    }
    atomic_fetch_add(&completed,1);return NULL;
}
static void *reader(void *unused)
{
    lift_diag_frame_t f;lift_diag_transport_t t;
    unsigned n=0;(void)unused;
    do {
        if(lift_diag_get_snapshot(&f)) {check_frame(&f);atomic_fetch_add(&snapshots,1);}
        if(lift_diag_get_record((lift_diag_read_t)(n%4),(uint16_t)(n%101),&f)) {
            check_frame(&f);atomic_fetch_add(&records,1);
        }
        if(lift_diag_get_transport(&t)) {
            if(t.cache_generation) {
                assert(t.cache_generation==t.cache_cycle_id && t.cache_tick_ms==t.cache_cycle_id);
                assert(t.cache_raw[0]==(int16_t)t.cache_cycle_id && t.cache_raw[1]==-(int16_t)t.cache_cycle_id);
            }
            if(t.tx_count) {
                assert(t.tx_count==t.tx_cycle_id && t.tx_tick_ms==t.tx_cycle_id);
                assert(t.accepted_count+t.failed_count==t.tx_count);
                assert(t.packed_raw[0]==(int16_t)t.tx_cycle_id && t.packed_raw[1]==-(int16_t)t.tx_cycle_id);
            }
            atomic_fetch_add(&transports,1);
        }
        if(++n%127U==0) lift_diag_request_rearm();
    } while(atomic_load(&completed)<3 || n<1000);
    return NULL;
}
int main(void)
{
    pthread_t p,c,t,r[2];lift_diag_init();
    assert(!pthread_create(&r[0],NULL,reader,NULL));assert(!pthread_create(&r[1],NULL,reader,NULL));
    assert(!pthread_create(&p,NULL,publisher,NULL));assert(!pthread_create(&c,NULL,cache_writer,NULL));
    assert(!pthread_create(&t,NULL,tx_writer,NULL));
    assert(!pthread_join(p,NULL));assert(!pthread_join(c,NULL));assert(!pthread_join(t,NULL));
    assert(!pthread_join(r[0],NULL));assert(!pthread_join(r[1],NULL));
    assert(atomic_load(&snapshots)>0 && atomic_load(&records)>0 && atomic_load(&transports)>0);
    printf("PASS: concurrent live/history/rearm/cache/TX snapshots: %u/%u/%u accepted\n",
        atomic_load(&snapshots),atomic_load(&records),atomic_load(&transports));
    return 0;
}
