#include "lift_diagnostics.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static lift_diag_frame_t frame, readback;
static void publish(uint32_t cycle, uint32_t ms, uint32_t reasons)
{
    frame.cycle_id=cycle; frame.tick_ms=ms; frame.reasons=reasons;
    frame.axis[0].position_ecd=(int32_t)cycle;
    frame.axis[3].position_ecd=-(int32_t)cycle;
    lift_diag_publish(&frame);
}
static void ring_and_fault(void)
{
    uint32_t n;
    lift_diag_init(); memset(&frame,0,sizeof(frame));
    assert(!lift_diag_get_snapshot(&readback));
    assert(!lift_diag_get_snapshot(NULL));
    assert(!lift_diag_get_transport(NULL));
    for(n=1;n<=600;n++) publish(n, n*5U, 0U);
    assert(g_lift_blackbox.count==101 && g_lift_blackbox.head<101);
    assert(lift_diag_get_record(LIFT_DIAG_READ_HISTORY,0,&readback) && readback.cycle_id==197);
    assert(lift_diag_get_record(LIFT_DIAG_READ_HISTORY,100,&readback) && readback.cycle_id==597);
    assert(!lift_diag_get_record(LIFT_DIAG_READ_HISTORY,101,&readback));
    assert(!lift_diag_get_record(LIFT_DIAG_READ_HISTORY,UINT16_MAX,&readback));
    assert(!lift_diag_get_record(LIFT_DIAG_READ_TRIGGER,0,&readback));
    assert(!lift_diag_get_record(LIFT_DIAG_READ_POST,0,&readback));
    assert(!lift_diag_get_record((lift_diag_read_t)99,0,&readback));
    assert(!lift_diag_get_record(LIFT_DIAG_READ_HISTORY,0,NULL));
    {
        uint32_t oldest=(g_lift_blackbox.head+101-g_lift_blackbox.count)%101;
        uint32_t newest=(g_lift_blackbox.head+100)%101;
        assert(g_lift_blackbox.history[newest].tick_ms-
               g_lift_blackbox.history[oldest].tick_ms==2000U);
        assert(g_lift_blackbox.history[newest].cycle_id==597U);
        assert(g_lift_blackbox.history[oldest].cycle_id==197U);
    }
    frame.axis[0].demand_current=16000.0F;
    frame.axis[0].limit_current=2000.0F;
    frame.axis[0].guard_current=0;
    frame.axis[0].protected_current=0;
    frame.axis[0].submitted_raw=0;
    frame.reason_axis[LIFT_DIAG_STALL]=2;
    publish(601,3005,LIFT_DIAG_BIT(LIFT_DIAG_STALL));
    assert(g_lift_blackbox.frozen && !g_lift_blackbox.post_valid);
    assert(lift_diag_get_record(LIFT_DIAG_READ_BEFORE,0,&readback) && readback.cycle_id==600);
    assert(g_lift_blackbox.count==101 && g_lift_blackbox.trigger_cycle_id==601);
    assert(g_lift_blackbox.trigger.axis[0].demand_current==16000.0F);
    assert(g_lift_blackbox.trigger.axis[0].guard_current==0);
    assert(g_lift_blackbox.trigger.reason_axis[LIFT_DIAG_STALL]==2);
    publish(602,3010,LIFT_DIAG_BIT(LIFT_DIAG_STALL));
    assert(g_lift_blackbox.post_valid && g_lift_blackbox.post.cycle_id==602);
    assert(lift_diag_get_record(LIFT_DIAG_READ_TRIGGER,0,&readback) && readback.cycle_id==601);
    assert(lift_diag_get_record(LIFT_DIAG_READ_POST,0,&readback) && readback.cycle_id==602);
    g_lift_blackbox.sequence|=1U;
    assert(!lift_diag_get_record(LIFT_DIAG_READ_HISTORY,0,&readback));
    ++g_lift_blackbox.sequence;
    for(n=603;n<800;n++) publish(n,n*5U,LIFT_DIAG_BIT(LIFT_DIAG_STALL));
    assert(g_lift_diag.event_count[LIFT_DIAG_STALL]==1);
    assert(g_lift_blackbox.trigger_cycle_id==601 && g_lift_blackbox.post.cycle_id==602);
    publish(800,4000,0); publish(801,4005,LIFT_DIAG_BIT(LIFT_DIAG_STALL));
    assert(g_lift_diag.event_count[LIFT_DIAG_STALL]==2);
    assert(g_lift_blackbox.latch_count==1);
    g_lift_diag_rearm=true; publish(802,4010,0);
    assert(!g_lift_blackbox.frozen && g_lift_blackbox.count==1);
    assert(!g_lift_blackbox.post_valid && !g_lift_diag_rearm && !g_lift_blackbox.before_valid);
    assert(g_lift_diag.event_count[LIFT_DIAG_STALL]==2);
    for(n=0;n<LIFT_DIAG_REASON_COUNT;n++) {
        publish(803+n*2U,4015+n*10U,0);
        publish(804+n*2U,4020+n*10U,LIFT_DIAG_BIT(n));
        assert(g_lift_diag.event_count[n]>0);
    }
    g_lift_diag.event_count[LIFT_DIAG_STALL]=UINT32_MAX;
    publish(900,4500,0);publish(901,4505,LIFT_DIAG_BIT(LIFT_DIAG_STALL));
    assert(g_lift_diag.event_count[LIFT_DIAG_STALL]==UINT32_MAX);
    assert(lift_diag_get_snapshot(&readback));
    assert(readback.axis[0].position_ecd==901 && readback.axis[3].position_ecd==-901);
    g_lift_diag.sequence|=1U;
    assert(!lift_diag_get_snapshot(&readback)); /* bounded, high-priority reader */
    ++g_lift_diag.sequence;
    g_lift_diag.sequence=UINT32_MAX-1U;
    publish(902,4510,0);
    assert(g_lift_diag.sequence==0U && lift_diag_get_snapshot(&readback));
    lift_diag_init();
    publish(1,UINT32_MAX-9U,0);publish(2,5,0);assert(g_lift_blackbox.count==1);
    publish(3,10,0);assert(g_lift_blackbox.count==2); /* 20 ms across wrap */
}
static void transport(void)
{
    lift_diag_transport_t tx;
    const int16_t cache[4]={123,-456,32767,-32768};
    const int16_t packed[4]={123,-456,16000,-16000};
    lift_diag_init();
    assert(lift_diag_get_transport(&tx) && tx.send_result==255 && !tx.ever_accepted);
    lift_diag_cache(cache,77,100,LIFT_DIAG_SOURCE_API);
    lift_diag_tx(packed,1,77,102,1,false,false);
    lift_diag_tx(packed,1,77,104,0,false,false);
    assert(lift_diag_get_transport(&tx));
    assert(tx.cache_cycle_id==77 && tx.tx_cycle_id==77 && tx.tx_cache_generation==1);
    assert(tx.cache_raw[3]==-32768 && tx.packed_raw[3]==-16000);
    assert(tx.failed_count==1 && tx.tx_count==2 && tx.accepted_count==1);
    assert(tx.consecutive_failures==0 && tx.last_success_ms==104);
    assert(tx.last_failure_cycle==77 && tx.last_failure_ms==102 && tx.last_failure_result==1);
    lift_diag_tx(packed,1,77,106,2,true,false);
    lift_diag_tx(packed,1,77,108,3,true,false);
    assert(lift_diag_get_transport(&tx) && tx.failed_count==3 && tx.consecutive_failures==2);
    assert(tx.send_result==3 && tx.timed_out && tx.last_success_ms==104);
    g_lift_diag_transport.failed_count=UINT32_MAX;
    lift_diag_tx(packed,1,0,110,3,false,true);
    assert(lift_diag_get_transport(&tx) && tx.failed_count==UINT32_MAX && tx.test_override);
    g_lift_diag_transport_sequence|=1U;
    assert(!lift_diag_get_transport(&tx));
    ++g_lift_diag_transport_sequence;
    g_lift_diag_transport.cache_generation=UINT32_MAX;
    lift_diag_cache(cache,78,112,LIFT_DIAG_SOURCE_API);
    assert(lift_diag_get_transport(&tx) && tx.cache_generation==0);
    printf("frame=%zu history=%zu bytes\n",sizeof(frame),sizeof(g_lift_blackbox));
}
int main(void)
{
    ring_and_fault();transport();
    puts("PASS: history/wrap/latch/post/rearm/all reasons/saturation/bounded snapshots/transport");
    return 0;
}
