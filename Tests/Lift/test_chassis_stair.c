#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include "../../Application/Chassis/Src/chassis_control.c"

static uint32_t now=1000;
static jmp_buf task_end;
static robot_motor_state_t feedback;
static int16_t sent[4];
chassis_spin_command_t g_chassis_spin_command;
void chassis_spin_init(void) { memset(&g_chassis_spin_command,0,sizeof(g_chassis_spin_command)); }
float chassis_spin_update(uint32_t tick) { (void)tick; return 10.0F; }
uint32_t osKernelGetTickCount(void) { return now; }
TickType_t xTaskGetTickCount(void) { return now; }
osThreadId_t osThreadNew(void (*f)(void *), void *a, const osThreadAttr_t *b)
{ (void)f; (void)a; (void)b; return (void *)1; }
bool dji_motor_feedback_is_online(const dji_motor_feedback_t *f,uint32_t t,uint32_t age)
{ (void)t; (void)age; return f->initialized; }
void robot_motors_get_snapshot(robot_motor_state_t *s) { *s=feedback; }
void robot_chassis_set_currents(int16_t a,int16_t b,int16_t c,int16_t d)
{ sent[0]=a; sent[1]=b; sent[2]=c; sent[3]=d; }
void vTaskDelayUntil(TickType_t *t,TickType_t d)
{ *t+=d; now+=d; longjmp(task_end,1); }
static void cycle(void)
{ if(setjmp(task_end)==0) chassis_control_task(NULL); }
static void assert_zero(void)
{ for(unsigned ch=0;ch<4;ch++) assert(sent[ch]==0); }
int main(void)
{
    chassis_control_init();
    for(unsigned ch=0;ch<4;ch++) feedback.chassis.wheel[ch].initialized=true;
    chassis_control_stair_velocity(0.05F);
    /* Remote off does not cancel ownership. Remote spin and lateral velocity
     * cannot contaminate the forward command. */
    g_chassis_control_command.enabled=false;
    g_chassis_spin_command.enabled=true;
    chassis_control_set_velocity(5.0F,3.0F,2.0F);
    cycle();
    assert(g_chassis_control_status.running);
    for(unsigned ch=0;ch<4;ch++)
        assert(g_chassis_control_status.target_rpm[ch]==
            g_chassis_control_debug.motor_direction[ch]*120);
    now+=101;
    g_chassis_control_command.enabled=true;
    cycle();
    assert_zero();
    assert(stair_owned && !g_chassis_control_status.running);
    chassis_control_stair_velocity(0.05F);
    feedback.chassis.wheel[1].initialized=false;
    cycle();
    assert_zero();
    feedback.chassis.wheel[1].initialized=true;
    chassis_control_stair_velocity(0.0F);
    cycle();
    assert_zero();
    chassis_control_stair_release();
    assert(!stair_owned && !g_chassis_control_command.enabled);
    puts("stair ownership, spin isolation, zero command, stale lease and offline stop passed");
    return 0;
}
