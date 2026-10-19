/* Execute the real motor cache, packing, override and unique TX task. */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "../../Devices/RobotMotors/Src/robot_motor_commands.c"
static uint32_t now_ms=1000;
static jmp_buf tx_end;
static can_tx_service_status_t next_result;
static uint8_t last_wire[8];
uint32_t osKernelGetTickCount(void) {return now_ms;}
TickType_t xTaskGetTickCount(void) {return now_ms;}
osThreadId_t osThreadNew(void (*f)(void *), void *a, const osThreadAttr_t *attr)
{(void)f;(void)a;(void)attr;return (void *)1;}
void vTaskDelayUntil(TickType_t *t, TickType_t inc)
{*t+=inc;longjmp(tx_end,1);}
can_tx_service_status_t can_tx_service_send(can_bus_t bus,uint16_t id,const uint8_t *data,uint8_t dlc)
{
    assert(dlc==8);
    if(bus==CAN_BUS_1 && id==0x200) {memcpy(last_wire,data,8);return next_result;}
    return CAN_TX_SERVICE_OK;
}
static void run_tx(void) {if(setjmp(tx_end)==0)robot_motor_tx_task(NULL);}
static int16_t wire_current(unsigned ch)
{return (int16_t)(((uint16_t)last_wire[ch*2]<<8)|last_wire[ch*2+1]);}
int main(void)
{
    lift_diag_transport_t tx;
    robot_motor_commands_t cache;
    lift_diag_init();robot_motor_commands_init();lift_owner_reset();
    robot_lift_set_currents_diagnostic(32767,-32768,1234,-1234,42);
    robot_motor_commands_get_snapshot(&cache);
    assert(cache.lift.motor[0]==ROBOT_3508_CURRENT_LIMIT);
    assert(cache.lift.motor[1]==-ROBOT_3508_CURRENT_LIMIT);
    assert(cache.lift.diagnostic_cycle_id==42 && cache.lift.diagnostic_generation==1);
    next_result=CAN_TX_SERVICE_OK; run_tx();
    assert(lift_diag_get_transport(&tx));
    assert(tx.tx_count==1 && tx.send_result==0 && tx.ever_accepted);
    assert(tx.tx_cycle_id==42 && tx.tx_cache_generation==1);
    for(unsigned ch=0;ch<4;ch++)assert(tx.packed_raw[ch]==wire_current(ch));
    assert(tx.packed_raw[0]==cache.lift.motor[0] && tx.cache_raw[3]==-1234);
    now_ms+=2;next_result=CAN_TX_SERVICE_BUSY;run_tx();
    now_ms+=2;next_result=CAN_TX_SERVICE_HAL_ERROR;run_tx();
    now_ms+=2;next_result=CAN_TX_SERVICE_OK;run_tx();
    assert(lift_diag_get_transport(&tx) && tx.tx_count==4 && tx.failed_count==2);
    assert(tx.last_failure_cycle==42 && tx.last_failure_ms==1004 && tx.last_failure_raw[2]==1234);
    assert(tx.accepted_count==2 && tx.last_success_ms==1006);
    now_ms=1200;run_tx();
    assert(lift_diag_get_transport(&tx) && tx.timed_out);
    for(unsigned ch=0;ch<4;ch++) assert(tx.packed_raw[ch]==0 && tx.cache_raw[ch]==cache.lift.motor[ch]);
    robot_motor_commands_get_snapshot(&cache);assert(cache.lift.motor[2]==1234);
    assert(lift_owner_claim(LIFT_OWNER_JOG,now_ms));
    robot_lift_set_currents(12,-34,56,-78);
    lift_owner_release(LIFT_OWNER_JOG);
    assert(lift_diag_get_transport(&tx) && tx.cache_cycle_id==0 && tx.cache_source==LIFT_DIAG_SOURCE_JOG);
    g_dji_can_test_command.can_bus=1;g_dji_can_test_command.feedback_id=0x202;
    g_dji_can_test_command.test_current=-321;g_dji_can_test_command.duration_ms=50;
    g_dji_can_test_command.enabled=true;run_tx();
    assert(lift_diag_get_transport(&tx) && tx.test_override && tx.packed_raw[1]==-321 && tx.tx_cycle_id==0);
    assert(tx.cache_raw[1]==-34);
    assert(lift_owner_current()==LIFT_OWNER_CAN_TEST);
    lift_owner_release(LIFT_OWNER_CONTROL);lift_owner_release(LIFT_OWNER_JOG);
    assert(lift_owner_current()==LIFT_OWNER_CAN_TEST);
    now_ms+=55;run_tx();
    assert(lift_diag_get_transport(&tx) && tx.test_override);
    for(unsigned ch=0;ch<4;ch++)assert(tx.packed_raw[ch]==0);
    assert(lift_owner_current()==LIFT_OWNER_CAN_TEST);
    now_ms+=ROBOT_DJI_TEST_ZERO_HOLD_MS+1;run_tx();
    assert(lift_diag_get_transport(&tx) && !tx.test_override && tx.timed_out);
    assert(lift_owner_current()==LIFT_OWNER_NONE);
    assert(lift_owner_claim(LIFT_OWNER_JOG,now_ms));
    g_dji_can_test_command.can_bus=2;g_dji_can_test_command.enabled=true;run_tx();
    assert(lift_owner_current()==LIFT_OWNER_JOG); /* other bus must not steal lift */
    lift_owner_release(LIFT_OWNER_JOG);
    /* Fault latch blocks active/new CAN tests and old JOG cache, while the
     * main executor retains ownership of its explicitly computed stop output. */
    now_ms+=6000;g_dji_can_test_command.enabled=false;run_tx();
    now_ms+=ROBOT_DJI_TEST_ZERO_HOLD_MS+1;run_tx();
    robot_lift_set_currents(111,222,333,444);
    lift_owner_inhibit_tests(true);
    g_dji_can_test_command.can_bus=1;g_dji_can_test_command.enabled=true;run_tx();
    assert(!g_dji_can_test_command.enabled && !g_dji_can_test_status.running);
    for(unsigned ch=0;ch<4;ch++) assert(wire_current(ch)==0);
    assert(!lift_owner_claim(LIFT_OWNER_JOG,now_ms));
    assert(!lift_owner_may_write(LIFT_OWNER_JOG));
    assert(!lift_owner_may_write(LIFT_OWNER_CAN_TEST));
    assert(!lift_owner_claim_override(LIFT_OWNER_CAN_TEST,now_ms,NULL));
    robot_lift_set_currents_diagnostic(12,-34,56,-78,43);run_tx();
    assert(wire_current(0)==12 && wire_current(1)==-34);
    lift_owner_inhibit_tests(false);run_tx();
    assert(!g_dji_can_test_status.running); /* no pending test after recovery */
    g_dji_can_test_command.enabled=true;run_tx();
    assert(g_dji_can_test_status.running && wire_current(1)==-321);
    lift_owner_inhibit_tests(true);run_tx();
    for(unsigned ch=0;ch<4;ch++) assert(wire_current(ch)==((const int16_t[4]){12,-34,56,-78})[ch]);
    assert(!g_dji_can_test_status.running && !g_dji_can_test_command.enabled);
    lift_owner_inhibit_tests(false);
    puts("PASS: P1B bench fault latch, stale cache rejection, active cancellation and explicit restart");
    puts("PASS: real cache clamp/provenance/packet bytes/send failures/timeout/jog/CAN override/zero hold");
    return 0;
}
