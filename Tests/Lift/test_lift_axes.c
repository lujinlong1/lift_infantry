/* Exercise the actual task + top + axis controller, not a copied controller. */
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <setjmp.h>
#include <stdio.h>
#include "../../Application/Lift/Src/lift_control_task.c"

static uint32_t fake_now = 1000U;
static unsigned cycles_left, writes, rear_nonzero_writes, front_retract_writes;
static jmp_buf task_end;
static robot_motor_state_t feedback;
static int16_t sent[4];
chassis_control_debug_t g_chassis_control_debug;
static float stair_sent_vx;
void chassis_control_stair_velocity(float vx) { stair_sent_vx = vx; }
void chassis_control_stair_release(void) { stair_sent_vx = 0.0F; }
dji_can_test_status_t g_dji_can_test_status;
lift_jog_status_t g_lift_jog_status;
uint32_t osKernelGetTickCount(void) { return fake_now; }
TickType_t xTaskGetTickCount(void) { return fake_now; }
osThreadId_t osThreadNew(void (*f)(void *), void *a, const osThreadAttr_t *b)
{ (void)f; (void)a; (void)b; return (void *)1; }
bool dji_motor_feedback_is_online(const dji_motor_feedback_t *f, uint32_t t, uint32_t age)
{ (void)t; (void)age; return f->initialized; }
void robot_motors_get_snapshot(robot_motor_state_t *s) { *s = feedback; }
void robot_lift_set_currents(int16_t a, int16_t b, int16_t c, int16_t d)
{ sent[0]=a; sent[1]=b; sent[2]=c; sent[3]=d; writes++;
  if(c != 0 || d != 0) rear_nonzero_writes++;
  if(a > 0 || b < 0) front_retract_writes++; }
void vTaskDelayUntil(TickType_t *t, TickType_t increment)
{
    *t += increment; fake_now += increment;
    if (--cycles_left == 0U) longjmp(task_end, 1);
}
static void run_task(unsigned cycles)
{
    cycles_left = cycles;
    if (setjmp(task_end) == 0) lift_control_task(NULL);
}
static void setup(void)
{
    unsigned ch;
    lift_top_input_t in = {0};
    lift_control_command_t cmd = {0};
    int32_t zero[4] = {0};
    fake_now = 1000U; writes = 0; rear_nonzero_writes = 0;
    front_retract_writes = 0;
    memset(&feedback,0,sizeof(feedback));
    memset(&g_dji_can_test_status,0,sizeof(g_dji_can_test_status));
    memset(&g_lift_jog_status,0,sizeof(g_lift_jog_status));
    lift_control_task_init();
    assert(lift_configuration_applied && lift_kin_ok);
    assert(lift_top.config.control.axis_control_enabled);
    assert(lift_top.config.control.pid[0].ki == 0.0F);
    lift_top_set_zero_now(&lift_top,zero);
    in.command_fresh = true;
    for(ch=0;ch<4;ch++)
    {
        feedback.lift.motor[ch].initialized = true;
        feedback.lift.motor[ch].total_ecd = lift_top.config.axis_direction[ch]*15000;
        in.position_ecd[ch] = feedback.lift.motor[ch].total_ecd;
        in.online[ch] = true;
    }
    lift_top_update(&lift_top,&in,&cmd,&lift_last_status);
}
static void test_task_routing(void)
{
    lift_seq_status_t seq = {0};
    lift_top_input_t in = {0};
    lift_control_command_t cmd = {0};
    unsigned ch;
    setup();
    /* Every conversion uses that axis' measured calibration. */
    g_lift_control_debug.axis_counts_per_mm[1] = 800.0F;
    g_lift_control_debug.apply_gains = 1;
    lift_handle_light_commands();
    seq.state = LIFT_SEQ_STATE_PRECHECK;
    for(ch=0;ch<4;ch++) in.position_ecd[ch] = feedback.lift.motor[ch].total_ecd;
    assert(lift_route_sequence(&seq,&in,&cmd));
    assert(cmd.axis_targets_valid && cmd.axis_target_ecd[0] == 15000);
    seq.command_valid = seq.command_enabled = true;
    seq.axis_target_mm[0] = 40; seq.axis_target_mm[1] = 50;
    seq.axis_target_mm[2] = 180; seq.axis_target_mm[3] = 190;
    assert(lift_route_sequence(&seq,&in,&cmd));
    assert(cmd.axis_target_ecd[0] == 30000);
    assert(cmd.axis_target_ecd[1] == 40000);
    assert(cmd.axis_target_ecd[2] == 135000);
    assert(cmd.axis_target_ecd[3] == 142500);
    assert(in.command_fresh);
    {
        lift_control_t ctl;
        lift_control_status_t st;
        int32_t p[4]={15000,16000,15000,15000}, out[4];
        int16_t rpm[4]={0};
        lift_control_command_t manual={0};
        manual.enabled=true; manual.target_height_ecd=122000; /* 40mm * sum(cpm) */
        lift_control_init(&ctl,&lift_top.config.control);
        lift_control_update_axes(&ctl,p,rpm,&manual,.005F,out,&st);
        assert(st.axis_target_ecd[0]==30000 && st.axis_target_ecd[1]==32000);
    }
    seq.state = LIFT_SEQ_STATE_IDLE; seq.command_valid = false;
    assert(!lift_route_sequence(&seq,&in,&cmd) && !cmd.enabled);
    setup();
    g_lift_control_debug.sequence_layout_confirmed = true;
    g_lift_control_debug.sequence_start = true;
    run_task(4);
    assert(g_lift_seq_status.seq_state == LIFT_SEQ_STATE_PRECHECK);
    for(ch=0;ch<4;ch++) assert(g_lift_control_status.axis_target_ecd[ch] == 15000);
    /* Continue beyond the manual 100ms timeout: sequence remains fresh. */
    run_task(30);
    assert(g_lift_control_status.command_fresh);
    assert(g_lift_seq_status.seq_state == LIFT_SEQ_STATE_LIFT_CLEARANCE);
    assert(g_lift_control_status.guard_state != LIFT_GUARD_STATE_DESCENDING);
    for(ch=0;ch<4;ch++) assert(g_lift_control_status.axis_target_ecd[ch] == 148500);
    g_lift_jog_status.running = true;
    {
        unsigned before = writes;
        run_task(3);
        assert(writes == before);
        assert(!lift_top.control.axis_initialized);
        for(ch=0;ch<4;ch++)
        {
            assert(lift_top.control.axis_pid[1][ch].integral == 0.0F);
            assert(g_lift_control_status.axis_final_current[ch] == 0.0F);
        }
    }
    setup();
    lift_control_set_target(120000,0,0); lift_control_enable(true);
    run_task(2);
    assert(writes == 2);
    assert(sent[0] <= 0 && sent[1] >= 0 && sent[2] <= 0 && sent[3] >= 0);
    for(ch=0;ch<4;ch++) assert(g_lift_control_status.axis_target_ecd[ch] == 30000);
    setup();
    g_lift_control_debug.sequence_start=true; /* layout deliberately unconfirmed */
    run_task(30);
    assert(g_lift_seq_status.seq_state==LIFT_SEQ_STATE_FAULT);
    assert(g_lift_control_status.state==LIFT_TOP_STATE_FAULT);
    assert(g_lift_control_status.fault==LIFT_TOP_FAULT_SEQUENCE);
    assert(!g_lift_control_status.stop_capable &&
        g_lift_control_status.stop_action==LIFT_STOP_CUT);
    assert(!lift_top.control.axis_initialized);
    setup();
    g_lift_control_debug.target_accel_mm_s2=NAN;
    g_lift_control_debug.apply_gains=1;
    lift_handle_light_commands();
    assert(lift_configuration_rejected);
    assert(lift_top.config.control.target_accel_mm_s2==20);
    printf("task routing, PRECHECK, calibration, freshness and bench gate passed\n");
}
static void test_pid_antiwindup(void)
{
    pid_config_t cfg = {2.0F,1.0F,0.0F,10000.0F,16000.0F};
    pid_controller_t p;
    unsigned n;
    pid_init(&p,&cfg);
    for(n=0;n<2000;n++) assert(pid_calculate_limited(&p,1000,0,.005F,-100,100)==100);
    assert(p.integral == 0);
    p.integral = 2000;
    (void)pid_calculate_limited(&p,-10,0,.005F,-100,100);
    assert(p.integral < 2000); /* unwind even while still high-side saturated */
    printf("conditional integration and unwind passed\n");
}
static void test_axis_limits_and_sync(void)
{
    lift_control_t ctl;
    lift_control_config_t cfg;
    lift_control_status_t st;
    lift_control_command_t cmd = {0};
    int32_t pos[4] = {15000,15000,75000,75000}, out[4];
    int16_t rpm[4] = {0}, applied[4];
    unsigned ch,n;
    setup(); cfg = lift_top.config.control;
    cfg.current_limit_start = 0;
    cfg.axis_load_current[0]=1500; cfg.axis_load_current[1]=1800;
    cfg.axis_load_current[2]=3500; cfg.axis_load_current[3]=4000;
    cmd.enabled = cmd.axis_targets_valid = true;
    for(ch=0;ch<4;ch++) cmd.axis_target_ecd[ch]=pos[ch];
    lift_control_init(&ctl,&cfg);
    for(n=0;n<30;n++)
    {
        float previous[4];
        for(ch=0;ch<4;ch++) previous[ch]=ctl.axis_current[ch];
        lift_control_update_axes(&ctl,pos,rpm,&cmd,.005F,out,&st);
        assert(st.sync_scale==1); /* deliberately unequal front/rear targets */
        for(ch=0;ch<4;ch++)
        {
            assert(fabsf((float)out[ch]-previous[ch])<=301);
            assert(abs(out[ch])<=cfg.axis_current_limit);
            applied[ch]=(int16_t)out[ch];
        }
        lift_control_track_axes(&ctl,applied,&st);
    }
    assert(st.at_target);
    for(ch=0;ch<4;ch++) assert(out[ch]==(int32_t)cfg.axis_load_current[ch]);
    /* HOLD must keep the speed integral instead of resetting supporting force. */
    ctl.axis_pid[1][2].integral = 2000;
    lift_control_update_axes(&ctl,pos,rpm,&cmd,.005F,out,&st);
    assert(ctl.axis_pid[1][2].integral==2000 && out[2]>3500);
    /* Quantization of current must not silently reject every integral update. */
    ctl.axis_holding[0]=false; rpm[0]=-1;
    lift_control_update_axes(&ctl,pos,rpm,&cmd,.005F,out,&st);
    {
        float before=ctl.axis_pid[1][0].integral;
        for(ch=0;ch<4;ch++) applied[ch]=(int16_t)out[ch];
        lift_control_track_axes(&ctl,applied,&st);
        assert(ctl.axis_pid[1][0].integral==before);
    }
    /* External protection clamps an axis: roll back its attempted integral. */
    applied[0]=0;
    lift_control_track_axes(&ctl,applied,&st);
    assert(ctl.axis_pid[1][0].integral==ctl.axis_integral_before[0]);
    assert(st.axis_final_current[0]==0);
    {
        lift_control_t saturated;
        lift_control_config_t limited=cfg;
        limited.axis_current_limit=1000;
        limited.current_limit_start=0;
        for(ch=0;ch<4;ch++)
        {
            limited.axis_load_current[ch]=0;
            cmd.axis_target_ecd[ch]=140000;
        }
        lift_control_init(&saturated,&limited);
        for(n=0;n<2000;n++)
        {
            lift_control_update_axes(&saturated,pos,rpm,&cmd,.005F,out,&st);
            for(ch=0;ch<4;ch++)
            {
                assert(abs(out[ch])<=1000);
                assert(fabsf(saturated.axis_pid[1][ch].integral)<1000);
            }
        }
        /* FF changes must still respect the combined current slew/ceiling. */
        saturated.config.axis_load_current[0]=1000;
        {
            float previous=saturated.axis_current[0];
            lift_control_update_axes(&saturated,pos,rpm,&cmd,.005F,out,&st);
            assert(fabsf((float)out[0]-previous)<=301 && abs(out[0])<=1000);
        }
        for(ch=0;ch<4;ch++) cmd.axis_target_ecd[ch]=pos[ch];
    }
    /* A stalled axis pauses common reference progression, not a second torque loop. */
    rpm[0]=0; pos[0]-=10000;
    lift_control_update_axes(&ctl,pos,rpm,&cmd,.005F,out,&st);
    assert(st.sync_scale==0);
    assert(st.axis_reference_ecd[0]==15000);
    /* Already outside the upper stop: gravity FF cannot push farther. */
    pos[2]=150001; cmd.axis_target_ecd[2]=200000;
    lift_control_update_axes(&ctl,pos,rpm,&cmd,.005F,out,&st);
    assert(out[2]<=0 && st.axis_target_ecd[2]==148500 && st.limit_active);
    cmd.enabled=false;
    lift_control_update_axes(&ctl,pos,rpm,&cmd,.005F,out,&st);
    for(ch=0;ch<4;ch++) assert(out[ch]==0 && ctl.axis_pid[1][ch].integral==0);
    printf("axis hold, asymmetric feedforward, current slew, bounds and sync passed\n");
}
static void test_loaded_plant_and_reversal(void)
{
    lift_control_t ctl;
    lift_control_status_t st;
    lift_control_config_t cfg;
    lift_control_command_t cmd = {0};
    float q[4]={15000,15000,15000,15000}, v[4]={0};
    float weight[4]={1500,1800,3500,4000};
    int32_t pos[4], out[4];
    int16_t rpm[4], applied[4];
    unsigned ch,n;
    setup(); cfg=lift_top.config.control;
    cfg.current_limit_start=0;
    /* No load feedforward: the retained PI must learn different supporting
     * currents. First-order velocity response with finite inertia and RPM
     * quantization; no position snapping or artificial motion clamps. */
    cmd.enabled=cmd.axis_targets_valid=true;
    for(ch=0;ch<4;ch++) cmd.axis_target_ecd[ch]=30000;
    lift_control_init(&ctl,&cfg);
    for(n=0;n<16000;n++)
    {
        for(ch=0;ch<4;ch++)
        {
            pos[ch]=(int32_t)q[ch];
            rpm[ch]=(int16_t)lroundf(v[ch]*60.0F/8192.0F);
        }
        lift_control_update_axes(&ctl,pos,rpm,&cmd,.005F,out,&st);
        for(ch=0;ch<4;ch++)
        {
            assert(abs(out[ch])<=cfg.axis_current_limit);
            assert(fabsf(st.axis_target_speed_ecd_s[ch])<=11250);
            v[ch] += .005F/.06F * (.5F*((float)out[ch]-weight[ch])-v[ch]);
            q[ch] += v[ch]*.005F;
            applied[ch]=(int16_t)out[ch];
        }
        lift_control_track_axes(&ctl,applied,&st);
    }
    for(ch=0;ch<4;ch++)
    {
        assert(fabsf(q[ch]-30000)<350);
        assert(fabsf(v[ch])<375);
        assert(fabsf((float)out[ch]-weight[ch])<400);
        printf("axis%u hold error %.1f ecd, velocity %.1f, supporting current %ld\n",
            ch,q[ch]-30000,v[ch],(long)out[ch]);
    }
    /* Reverse a moving target: each reference velocity must decelerate
     * through zero instead of instantly changing +8 to -8 mm/s. */
    for(ch=0;ch<4;ch++) cmd.axis_target_ecd[ch]=45000;
    for(n=0;n<120;n++)
    {
        for(ch=0;ch<4;ch++) { pos[ch]=(int32_t)ctl.axis_reference[ch]; rpm[ch]=0; }
        lift_control_update_axes(&ctl,pos,rpm,&cmd,.005F,out,&st);
    }
    for(ch=0;ch<4;ch++) cmd.axis_target_ecd[ch]=15000;
    for(n=0;n<200;n++)
    {
        float before[4];
        for(ch=0;ch<4;ch++) { before[ch]=ctl.axis_reference_speed[ch]; pos[ch]=(int32_t)ctl.axis_reference[ch]; }
        lift_control_update_axes(&ctl,pos,rpm,&cmd,.005F,out,&st);
        for(ch=0;ch<4;ch++)
        {
            assert(fabsf(ctl.axis_reference_speed[ch]-before[ch])<=76);
            assert(fabsf(ctl.axis_reference_speed[ch])<=6001);
        }
    }
    assert(ctl.axis_reference_speed[0]<0);
    puts("unequal load plant and reference reversal passed");
}
static void setup_unhomed(void)
{
    lift_top_config_t cfg;
    setup();
    cfg = lift_top.config;
    lift_top_init(&lift_top, &cfg);
    memset(&lift_last_status, 0, sizeof(lift_last_status));
    /* Deliberately nonzero, mirrored raw encoders at the rear datum. */
    feedback.lift.motor[0].total_ecd = -5000;
    feedback.lift.motor[1].total_ecd = 5000;
    feedback.lift.motor[2].total_ecd = -12345;
    feedback.lift.motor[3].total_ecd = 23456;
    assert(lift_top.config.homing_known_zero_mask == ROBOT_LIFT_REAR_MASK);
}

static void test_front_homing_rear_datum(void)
{
    setup_unhomed();
    /* A Watch confirmation must survive the delay before writing the pulse. */
    g_lift_task_command.rear_at_minimum_confirmed = true;
    run_task(2);
    assert(g_lift_task_command.rear_at_minimum_confirmed);
    g_lift_task_command.homing_request = true;
    run_task(1);
    assert(!g_lift_task_command.rear_at_minimum_confirmed);
    assert(!g_lift_task_command.homing_request);
    assert(!lift_top.homed && lift_top.state == LIFT_TOP_STATE_HOMING);
    assert(lift_top.homing[0].state == LIFT_HOMING_STATE_SEEK);
    assert(lift_top.homing[1].state == LIFT_HOMING_STATE_SEEK);
    assert(lift_top.homing[2].state == LIFT_HOMING_STATE_DONE);
    assert(lift_top.homing[3].state == LIFT_HOMING_STATE_DONE);
    assert(lift_top.homing[2].zero_ecd == 12345);
    assert(lift_top.homing[3].zero_ecd == 23456);
    assert(sent[0] < 0 && sent[1] > 0); /* both extend in logical coordinates */
    assert(sent[2] == 0 && sent[3] == 0);
    /* Extend to a candidate contact, without any retract or backoff motion. */
    feedback.lift.motor[0].total_ecd = -7000;
    feedback.lift.motor[1].total_ecd = 7000;
    run_task(70);
    assert(lift_top.homing[0].state == LIFT_HOMING_STATE_CONTACT_WAIT);
    assert(lift_top.homing[1].state == LIFT_HOMING_STATE_CONTACT_WAIT);
    assert(sent[0] == 0 && sent[1] == 0 && !lift_top.homed);
    /* Stalled motion alone must not assert chassis contact. */
    run_task(100);
    assert(!lift_top.homed);
    g_lift_task_command.front_contact_confirmed_mask = 1U;
    run_task(1);
    assert(g_lift_task_command.front_contact_confirmed_mask == 0U);
    run_task(70);
    assert(lift_top.homing[0].state == LIFT_HOMING_STATE_DONE);
    assert(lift_top.homing[1].state == LIFT_HOMING_STATE_CONTACT_WAIT);
    assert(!lift_top.homed); /* each physical contact needs its own confirmation */
    g_lift_task_command.front_contact_confirmed_mask = 2U;
    run_task(71);
    assert(lift_top.homed && lift_top.state == LIFT_TOP_STATE_READY);
    assert(lift_top.zero_ecd[0] == -7000 && lift_top.zero_ecd[1] == 7000);
    assert(lift_top.zero_ecd[2] == -12345 && lift_top.zero_ecd[3] == 23456);
    assert(g_lift_control_status.homed_position_ecd[0] == 0);
    assert(g_lift_control_status.homed_position_ecd[1] == 0);
    assert(g_lift_control_status.homed_position_ecd[2] == 0);
    assert(g_lift_control_status.homed_position_ecd[3] == 0);
    assert(rear_nonzero_writes == 0);
    assert(front_retract_writes == 0);
    /* Without any issued motion command, homing must not start a descent. */
    run_task(2000);
    assert(lift_top.state == LIFT_TOP_STATE_READY);
    assert(lift_top.guard.state == LIFT_GUARD_STATE_READY);
    assert(!g_lift_control_status.command_fresh);
    assert(sent[0] == 0 && sent[1] == 0 && sent[2] == 0 && sent[3] == 0);
    assert(front_retract_writes == 0 && rear_nonzero_writes == 0);
    g_lift_task_command.enabled = true;
    g_lift_task_command.command_update_tick = 0U; /* stale flag is not an enable */
    run_task(2);
    assert(!lift_top.motion_command_seen);
    assert(sent[0] == 0 && sent[1] == 0 && sent[2] == 0 && sent[3] == 0);
    /* After a real enabled command, timeout protection must still descend. */
    for(unsigned ch=0; ch<4; ch++)
        feedback.lift.motor[ch].total_ecd = lift_top.zero_ecd[ch] +
            lift_top.config.axis_direction[ch]*7500;
    g_lift_task_command.enabled = true;
    g_lift_task_command.target_height_ecd = 30000;
    g_lift_task_command.command_update_tick = fake_now;
    run_task(1);
    assert(lift_top.motion_command_seen);
    fake_now += 200U;
    run_task(1);
    assert(lift_top.guard.state == LIFT_GUARD_STATE_DESCENDING);
    assert(sent[0] > 0 && sent[1] < 0 && sent[2] > 0 && sent[3] < 0);
    g_lift_task_command.enabled = false;
    rear_nonzero_writes = front_retract_writes = 0;
    /* Clearing datum must not reuse a confirmation from the first request. */
    g_lift_task_command.clear_homing_request = true;
    run_task(1);
    g_lift_task_command.homing_request = true;
    run_task(1);
    assert(lift_top.state == LIFT_TOP_STATE_FAULT);
    assert(lift_top.fault == LIFT_TOP_FAULT_ZERO_UNCONFIRMED);
    assert(!lift_top.homed && rear_nonzero_writes == 0);

    setup_unhomed();
    g_lift_task_command.homing_request = true;
    run_task(1);
    assert(lift_top.fault == LIFT_TOP_FAULT_ZERO_UNCONFIRMED);
    assert(sent[0] == 0 && sent[1] == 0 && sent[2] == 0 && sent[3] == 0);
    g_lift_task_command.reset_request = true;
    run_task(1);
    g_lift_task_command.rear_at_minimum_confirmed = true;
    g_lift_task_command.homing_request = true;
    run_task(1);
    assert(lift_top.state == LIFT_TOP_STATE_HOMING);
    /* A rear axis marked DONE is still required to stay online. */
    feedback.lift.motor[3].initialized = false;
    run_task(1);
    assert(lift_top.state == LIFT_TOP_STATE_FAULT);
    assert(lift_top.homing[3].fault == LIFT_HOMING_FAULT_OFFLINE);
    assert(sent[0] == 0 && sent[1] == 0 && rear_nonzero_writes == 0);

    setup_unhomed();
    feedback.lift.motor[2].speed_rpm = -100;
    g_lift_task_command.rear_at_minimum_confirmed = true;
    g_lift_task_command.homing_request = true;
    run_task(1);
    assert(lift_top.fault == LIFT_TOP_FAULT_ZERO_MOVING);
    assert(sent[0] == 0 && sent[1] == 0 && rear_nonzero_writes == 0);

    setup_unhomed();
    g_lift_task_command.rear_at_minimum_confirmed = true;
    g_lift_task_command.homing_request = true;
    run_task(1);
    run_task(70); /* Not moving pauses at an unconfirmed candidate, not DONE. */
    assert(lift_top.homing[0].state == LIFT_HOMING_STATE_CONTACT_WAIT);
    assert(!lift_top.homed);
    fake_now += 61000U;
    run_task(1);
    assert(lift_top.fault == LIFT_TOP_FAULT_HOMING_FAILED);
    assert(lift_top.homing[0].fault == LIFT_HOMING_FAULT_CONTACT_TIMEOUT);
    assert(lift_top.homing[2].state == LIFT_HOMING_STATE_DONE);
    assert(!lift_top.homed && rear_nonzero_writes == 0);
    setup_unhomed();
    feedback.lift.motor[0].speed_rpm = -100;
    feedback.lift.motor[1].speed_rpm = 100;
    g_lift_task_command.rear_at_minimum_confirmed = true;
    g_lift_task_command.homing_request = true;
    run_task(1);
    fake_now += 9000U;
    run_task(1);
    assert(lift_top.homing[0].fault == LIFT_HOMING_FAULT_SEEK_TIMEOUT);
    assert(lift_top.fault == LIFT_TOP_FAULT_HOMING_FAILED);
    assert(rear_nonzero_writes == 0 && front_retract_writes == 0);

    /* A front already at real contact needs no artificial minimum travel. */
    setup_unhomed();
    g_lift_task_command.rear_at_minimum_confirmed = true;
    g_lift_task_command.homing_request = true;
    run_task(71);
    assert(lift_top.homing[0].state == LIFT_HOMING_STATE_CONTACT_WAIT);
    assert(!lift_top.homed);
    g_lift_task_command.front_contact_confirmed_mask = 3U;
    run_task(71);
    assert(lift_top.homed);
    assert(lift_top.zero_ecd[0] == -5000 && lift_top.zero_ecd[1] == 5000);
    assert(front_retract_writes == 0 && rear_nonzero_writes == 0);
    puts("front extension/contact datum, rear zero and watchdog arming passed");
}

static void test_contact_zero_start_recovery(void)
{
    const struct {
        int32_t position[4];
        float front1_cpm;
        bool contact_datum;
        int32_t fault_axis;
    } cases[] = {
        {{-576,0,0,23},750.0F,true,-1}, /* actual Keil failure */
        {{-1000,-1000,-1000,-1000},750.0F,true,-1},
        {{-1001,0,0,0},750.0F,true,0},
        {{0,-1000,0,0},800.0F,true,-1},
        {{0,-1001,0,0},800.0F,true,1},
        {{0,0,-1,0},750.0F,true,-1},
        {{0,0,0,-1},750.0F,true,-1}, /* actual rear startup failure */
        {{0,0,-1001,0},750.0F,true,2},
        {{0,0,0,-1001},750.0F,true,3},
        {{150001,0,0,0},750.0F,true,0}, /* upper limit stays strict */
        {{-1000,0,0,0},750.0F,false,-1} /* same tolerance for hard-stop zero */
    };
    for (unsigned n=0;n<sizeof(cases)/sizeof(cases[0]);n++)
    {
        setup();
        g_lift_control_debug.axis_counts_per_mm[1]=cases[n].front1_cpm;
        g_lift_control_debug.apply_gains=1;
        lift_handle_light_commands();
        if (!cases[n].contact_datum) lift_top.config.homing_contact_mask=0;
        for (unsigned ch=0;ch<4;ch++)
            feedback.lift.motor[ch].total_ecd =
                lift_top.config.axis_direction[ch]*cases[n].position[ch];
        run_task(1);
        g_lift_sequence.config.clearance_mm=10;
        g_lift_sequence.config.contact_mm=8;
        g_lift_sequence.config.retracted_mm=2;
        g_lift_control_debug.sequence_layout_confirmed=true;
        g_lift_control_debug.sequence_start=true;
        run_task(30);
        if (cases[n].fault_axis<0)
        {
            assert(g_lift_seq_status.seq_state==LIFT_SEQ_STATE_LIFT_CLEARANCE);
            assert(g_lift_seq_status.seq_fault==LIFT_SAFETY_FAULT_NONE);
            assert(g_lift_control_status.command_fresh);
            for (unsigned ch=0;ch<4;ch++)
            {
                assert(lift_top.zero_ecd[ch]==0); /* keep confirmed contact */
                assert(g_lift_control_status.limit_height_min_ecd[ch]==0);
                assert(g_lift_control_status.limit_height_max_ecd[ch]==150000);
                assert(g_lift_control_status.axis_final_current[ch]>=0);
                assert(g_lift_control_status.axis_target_ecd[ch]==
                    (int32_t)(10*lift_top.config.control.counts_per_mm[ch]));
            }
        }
        else
        {
            assert(g_lift_seq_status.seq_state==LIFT_SEQ_STATE_FAULT);
            assert(g_lift_seq_status.seq_fault==LIFT_SAFETY_FAULT_LIMIT);
            assert(g_lift_seq_status.seq_fault_axis==cases[n].fault_axis);
            assert(sent[0]==0 && sent[1]==0 && sent[2]==0 && sent[3]==0);
        }
    }
    puts("all-axis -1000 count startup boundary and unchanged output limits passed");
}

static void test_one_click_lift(void)
{
    setup();
    for (unsigned ch=0;ch<4;ch++) feedback.lift.motor[ch].total_ecd=0;
    run_task(1);
    g_lift_control_debug.sequence_drive_done=true;
    g_lift_control_debug.sequence_confirm=true;
    g_lift_control_debug.test_start=true;
    run_task(30);
    assert(!g_lift_control_debug.test_start);
    assert(g_lift_control_debug.test_start_result==1);
    assert(!g_lift_control_debug.sequence_drive_done);
    assert(!g_lift_control_debug.sequence_confirm);
    assert(g_lift_seq_status.seq_state==LIFT_SEQ_STATE_LIFT_CLEARANCE);
    for (unsigned ch=0;ch<4;ch++)
        assert(g_lift_control_status.axis_target_ecd[ch]==37500);
    g_lift_control_debug.test_start=true;
    run_task(1);
    assert(g_lift_control_debug.test_start_result==3); /* no restart */

    setup();
    feedback.lift.motor[3].total_ecd=-1001;
    for (unsigned ch=0;ch<3;ch++) feedback.lift.motor[ch].total_ecd=0;
    run_task(1);
    g_lift_control_debug.test_start=true;
    run_task(30);
    assert(g_lift_seq_status.seq_state==LIFT_SEQ_STATE_FAULT);
    assert(g_lift_seq_status.seq_fault_axis==3);
    feedback.lift.motor[3].total_ecd=-1;
    run_task(1);
    g_lift_control_debug.test_start=true;
    run_task(30);
    assert(g_lift_seq_status.seq_state==LIFT_SEQ_STATE_LIFT_CLEARANCE);
    assert(g_lift_control_debug.test_start_result==1);

    setup();
    lift_last_status.homed=false;
    g_lift_control_debug.test_auto_zero=false;
    g_lift_control_debug.test_start=true;
    run_task(1);
    assert(g_lift_control_debug.test_start_result==2);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_IDLE);

    setup();
    g_lift_jog_status.running=true;
    g_lift_control_debug.test_start=true;
    run_task(1);
    assert(g_lift_control_debug.test_start_result==5);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_IDLE);

    setup();
    g_lift_sequence.state=LIFT_SEQ_STATE_FAULT;
    g_lift_sequence.fault=LIFT_SAFETY_FAULT_ABORTED;
    g_lift_sequence.fault_detail="abort_request";
    g_lift_control_debug.test_start=true;
    run_task(1);
    assert(g_lift_control_debug.test_start_result==4);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_FAULT);
    setup();
    for (unsigned ch=0;ch<4;ch++) feedback.lift.motor[ch].total_ecd=0;
    run_task(1);
    g_lift_control_debug.test_height_mm=10;
    g_lift_control_debug.test_start=true;
    run_task(30);
    assert(g_lift_seq_status.seq_state==LIFT_SEQ_STATE_LIFT_CLEARANCE);
    for (unsigned ch=0;ch<4;ch++)
        assert(g_lift_control_status.axis_target_ecd[ch]==7500);
    setup();
    g_lift_control_debug.test_height_mm=201;
    g_lift_control_debug.test_start=true;
    run_task(1);
    assert(g_lift_control_debug.test_start_result==6);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_IDLE);
    puts("one-click lift, startup fault retry and homing/busy/fault gates passed");
}

static void test_fault_snapshot(void)
{
    lift_fault_snapshot_t saved;
    setup();
    assert(!g_lift_fault_snapshot.valid);
    for (unsigned ch=0;ch<4;ch++) feedback.lift.motor[ch].total_ecd=0;
    run_task(1);
    g_lift_sequence.config.lift_timeout_ms=200;
    for (unsigned ch=0;ch<4;ch++)
    {
        feedback.lift.motor[ch].given_current=(int16_t)(100*(ch+1));
        feedback.lift.motor[ch].speed_rpm=(int16_t)(20*(ch+1));
    }
    g_lift_control_debug.test_start=true;
    run_task(70);
    assert(g_lift_seq_status.seq_fault==LIFT_SAFETY_FAULT_TIMEOUT);
    assert(g_lift_fault_snapshot.valid && g_lift_fault_snapshot.count==1);
    assert(g_lift_fault_snapshot.stage==LIFT_SEQ_STATE_LIFT_CLEARANCE);
    assert(strcmp((const char *)g_lift_fault_snapshot.stage_name,"LIFT_CLEARANCE")==0);
    assert(strcmp((const char *)g_lift_fault_snapshot.detail,"stage timeout")==0);
    assert(g_lift_fault_snapshot.stage_elapsed_ms==200);
    for (unsigned ch=0;ch<4;ch++)
    {
        assert(g_lift_fault_snapshot.position_ecd[ch]==0);
        assert(g_lift_fault_snapshot.target_mm[ch]==50);
        assert(g_lift_fault_snapshot.target_ecd[ch]==37500);
        assert(g_lift_fault_snapshot.feedback_current[ch]==
            lift_top.config.axis_direction[ch]*(int16_t)(100*(ch+1)));
        assert(g_lift_fault_snapshot.speed_rpm[ch]==(int16_t)(20*(ch+1)));
        assert(g_lift_fault_snapshot.online[ch]);
    }
    saved=g_lift_fault_snapshot;
    feedback.lift.motor[1].total_ecd=7582;
    run_task(10);
    assert(memcmp(&saved,&g_lift_fault_snapshot,sizeof(saved))==0);
    g_lift_control_debug.sequence_reset=true;
    g_lift_task_command.reset_request=true;
    run_task(2);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_IDLE);
    assert(memcmp(&saved,&g_lift_fault_snapshot,sizeof(saved))==0);

    /* A second fault replaces the snapshot; waiting for advance is distinct
     * from failing to reach the lifting height. */
    g_lift_sequence.state=LIFT_SEQ_STATE_APPROACH_STEP;
    g_lift_sequence.step_start_ms=fake_now;
    g_lift_sequence.config.advance_timeout_ms=100;
    g_lift_sequence.targets_valid=true;
    for (unsigned ch=0;ch<4;ch++)
    {
        g_lift_sequence.axis_target_mm[ch]=50;
        feedback.lift.motor[ch].total_ecd=
            lift_top.config.axis_direction[ch]*37500;
    }
    run_task(24);
    assert(g_lift_fault_snapshot.count==2);
    assert(g_lift_fault_snapshot.stage==LIFT_SEQ_STATE_APPROACH_STEP);
    assert(g_lift_fault_snapshot.stage_elapsed_ms==100);
    assert(g_lift_fault_snapshot.fault==LIFT_SAFETY_FAULT_TIMEOUT);
    for (unsigned ch=0;ch<4;ch++)
        assert(g_lift_fault_snapshot.position_ecd[ch]==37500);
    lift_control_task_init();
    assert(!g_lift_fault_snapshot.valid && g_lift_fault_snapshot.count==0);
    puts("fault snapshot retains pre-stop targets/stage across refresh/reset and replaces on next fault");
}

static void test_manual_lift_controls(void)
{
    setup();
    for (unsigned ch=0;ch<4;ch++) feedback.lift.motor[ch].total_ecd=0;
    run_task(1);
    g_lift_control_debug.test_height_mm=200;
    g_lift_control_debug.test_start=true;
    run_task(25);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_LIFT_CLEARANCE);
    for (unsigned ch=0;ch<4;ch++)
    {
        assert(g_lift_control_status.axis_target_ecd[ch]==(ch<2 ? 171750 : 187500));
        feedback.lift.motor[ch].total_ecd=
            lift_top.config.axis_direction[ch]*(12345+(int32_t)ch*10);
    }
    g_lift_control_debug.test_pause=true;
    run_task(2);
    assert(!g_lift_control_debug.test_pause);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_TEST_PAUSED);
    for (unsigned ch=0;ch<4;ch++)
        assert(g_lift_control_status.axis_target_ecd[ch]==12345+(int32_t)ch*10);
    run_task(12005); /* >60s: paused hold has no advance timeout */
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_TEST_PAUSED);
    assert(g_lift_control_status.command_fresh);
    assert(!g_lift_fault_snapshot.valid);
    g_lift_control_debug.test_resume=true;
    run_task(1);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_LIFT_CLEARANCE);
    for (unsigned ch=0;ch<4;ch++)
    {
        assert(g_lift_control_status.axis_target_ecd[ch]==(ch<2 ? 171750 : 187500));
        feedback.lift.motor[ch].total_ecd=lift_top.config.axis_direction[ch]*(ch<2 ? 171750 : 187500);
    }
    run_task(70);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_TEST_HOLD);
    g_lift_control_debug.sequence_drive_done=true;
    g_lift_control_debug.sequence_confirm=true;
    run_task(12005);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_TEST_HOLD);
    assert(!g_lift_seq_status.seq_drive_request);
    assert(g_lift_control_status.command_fresh);
    g_lift_control_debug.test_lower=true;
    run_task(1);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_TEST_LOWER);
    for (unsigned ch=0;ch<4;ch++)
        assert(g_lift_control_status.axis_target_ecd[ch]==(ch<2 ? 1733 : 1890));
    g_lift_control_debug.test_pause=true;
    run_task(1);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_TEST_PAUSED);
    g_lift_control_debug.test_resume=true;
    run_task(1);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_TEST_LOWER);
    for (unsigned ch=0;ch<4;ch++)
        feedback.lift.motor[ch].total_ecd=lift_top.config.axis_direction[ch]*(ch<2 ? 2599 : 2835);
    feedback.lift.motor[0].total_ecd=lift_top.config.axis_direction[0]*3465;
    run_task(1);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_TEST_LOWER); /* one axis at 4mm */
    feedback.lift.motor[0].total_ecd=lift_top.config.axis_direction[0]*2599;
    run_task(1);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_TEST_PARKED); /* all at 3mm */
    for (unsigned ch=0;ch<4;ch++) assert(sent[ch]==0);
    run_task(70);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_TEST_PARKED);
    run_task(12005);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_TEST_PARKED);
    assert(g_lift_control_status.command_fresh);
    for (unsigned ch=0;ch<4;ch++) assert(sent[ch]==0);
    /* Feedback motion after cut cannot re-arm low-position compensation. */
    feedback.lift.motor[0].total_ecd=-100;
    run_task(10);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_TEST_PARKED);
    for (unsigned ch=0;ch<4;ch++) assert(sent[ch]==0);
    /* Safety faults still latch in powered hold and snapshot identifies it. */
    feedback.lift.motor[0].initialized=false;
    run_task(3);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_FAULT);
    assert(g_lift_fault_snapshot.valid);
    assert(g_lift_fault_snapshot.stage==LIFT_SEQ_STATE_TEST_PARKED);
    puts("200mm lift-only test: exact pause, resume, >60s hold, descent and offline fault passed");
}

static void test_software_start_zero(void)
{
    setup();
    lift_control_task_init();
    for (unsigned ch=0;ch<4;ch++)
        feedback.lift.motor[ch].total_ecd=(int32_t)(1000*(ch+1));
    g_lift_control_debug.test_start=true;
    run_task(30);
    assert(lift_top.homed);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_LIFT_CLEARANCE);
    for (unsigned ch=0;ch<4;ch++)
    {
        assert(lift_top.zero_ecd[ch]==(int32_t)(1000*(ch+1)));
        assert(g_lift_control_status.homed_position_ecd[ch]==0);
    }
    setup();
    lift_control_task_init();
    feedback.lift.motor[0].speed_rpm=3;
    g_lift_control_debug.test_start=true;
    run_task(1);
    assert(!lift_top.homed && g_lift_control_debug.test_start_result==7);
    setup();
    lift_control_task_init();
    feedback.lift.motor[0].initialized=false;
    g_lift_control_debug.test_start=true;
    run_task(1);
    assert(!lift_top.homed && g_lift_control_debug.test_start_result==7);
    puts("stationary software startup datum and moving/offline rejection passed");
}

static void test_stall_holds_instead_of_cut(void)
{
    setup();
    for (unsigned ch=0;ch<4;ch++) feedback.lift.motor[ch].total_ecd=0;
    run_task(1);
    g_lift_control_debug.test_height_mm=200;
    g_lift_control_debug.test_start=true;
    run_task(340); /* stationary feedback: >=1.5s movement watchdog */
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_TEST_PAUSED);
    assert(g_lift_seq_status.test_stall_axis==0);
    assert(g_lift_seq_status.seq_fault==LIFT_SAFETY_FAULT_NONE);
    assert(!g_lift_fault_snapshot.valid);
    assert(g_lift_control_status.command_fresh);
    for (unsigned ch=0;ch<4;ch++)
    {
        assert(g_lift_control_status.axis_target_ecd[ch]==0);
        assert(lift_top.control.axis_reference[ch]==0.0F);
        assert(g_lift_control_debug.position_max_ecd[ch]==(ch<2 ? 173250 : 189000));
    }
    run_task(12005);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_TEST_PAUSED);
    g_lift_control_debug.test_resume=true;
    run_task(1);
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_LIFT_CLEARANCE);
    assert(g_lift_seq_status.test_stall_axis==-1);
    for (unsigned ch=0;ch<4;ch++)
        assert(g_lift_control_status.axis_target_ecd[ch]==(ch<2 ? 171750 : 187500));
    g_lift_control_debug.test_lower=true;
    run_task(1); /* already low -> cut, lower takes priority */
    assert(g_lift_sequence.state==LIFT_SEQ_STATE_TEST_PARKED);
    for (unsigned ch=0;ch<4;ch++) assert(sent[ch]==0);
    puts("provisional front/rear travel expansion and stalled-motion powered pause passed");
}

static void test_stair_debug(void)
{
    lift_seq_input_t in = {0};
    lift_seq_status_t out;
    unsigned ch;
    setup();
    g_chassis_control_debug.rpm_per_mps = 2404.946F;
    for (ch=0; ch<4; ch++)
    {
        g_chassis_control_debug.motor_direction[ch] = ch==1 || ch==2 ? -1 : 1;
        feedback.chassis.wheel[ch].initialized = true;
    }
    g_lift_control_debug.stair_start = true;
    lift_handle_test_start(false);
    assert(g_lift_control_debug.test_start_result==8 && !lift_stair_owned);
    for (ch=0; ch<3; ch++) g_lift_control_debug.stair_distance_mm[ch]=100;
    feedback.chassis.wheel[2].speed_rpm=3;
    g_lift_control_debug.stair_start=true;
    lift_handle_test_start(false);
    assert(g_lift_control_debug.test_start_result==9 && !lift_stair_owned);
    feedback.chassis.wheel[2].speed_rpm=0;
    g_lift_control_debug.stair_start=true;
    lift_handle_test_start(false);
    assert(g_lift_control_debug.test_start_result==1 && lift_stair_owned);
    assert(!g_lift_sequence.config.lift_test_only && g_lift_sequence.config.stair_debug_run);
    assert(stair_sent_vx==0.0F);
    g_lift_sequence.state=LIFT_SEQ_STATE_APPROACH_STEP;
    assert(!lift_stair_drive(&feedback, fake_now));
    assert(stair_sent_vx>0.0F);
    for (ch=0; ch<4; ch++) feedback.chassis.wheel[ch].total_ecd =
        stair_direction[ch]*(int32_t)(stair_counts_per_mm*40.0F);
    fake_now+=100;
    assert(!lift_stair_drive(&feedback,fake_now));
    assert(g_lift_control_debug.stair_drive_progress_mm>=39);
    g_lift_sequence.state=LIFT_SEQ_STATE_TEST_PAUSED;
    fake_now+=10000;
    assert(!lift_stair_drive(&feedback,fake_now) && stair_sent_vx==0.0F);
    g_lift_sequence.state=LIFT_SEQ_STATE_APPROACH_STEP;
    assert(!lift_stair_drive(&feedback,fake_now));
    assert(g_lift_control_debug.stair_drive_progress_mm>=39);
    for (ch=0; ch<4; ch++) feedback.chassis.wheel[ch].total_ecd =
        stair_direction[ch]*(int32_t)(stair_counts_per_mm*101.0F);
    assert(!lift_stair_drive(&feedback,fake_now));
    assert(stair_sent_vx==0.0F);
    fake_now+=100;
    assert(lift_stair_drive(&feedback,fake_now));
    g_lift_sequence.state=LIFT_SEQ_STATE_ADVANCE_MIDDLE;
    assert(!lift_stair_drive(&feedback,fake_now));
    assert(g_lift_control_debug.stair_drive_progress_mm==0);
    fake_now+=2000;
    assert(!lift_stair_drive(&feedback,fake_now));
    assert(g_lift_control_debug.test_pause && g_lift_control_debug.stair_drive_stop_reason==2);
    assert(stair_sent_vx==0.0F);

    /* Unequal front/rear pose is intentional while retracting. Pause captures
     * exact encoders; waiting does not consume stage timeout or confirmations. */
    g_lift_sequence.state=LIFT_SEQ_STATE_RETRACT_FRONT;
    g_lift_sequence.step_start_ms=1000;
    g_lift_sequence.run_start_ms=1000;
    g_lift_sequence.targets_valid=true;
    for(ch=0;ch<4;ch++)
    {
        in.axis_online[ch]=true;
        in.limit_max_ecd[ch]=150000;
        in.position_ecd[ch]=ch<2 ? 15000 : 37500;
        in.position_mm[ch]=ch<2 ? 20 : 50;
        g_lift_sequence.axis_target_mm[ch]=ch<2 ? 2 : 50;
    }
    in.now_ms=1500;
    in.sync_error_mm=30;
    in.deviation_max_mm=30;
    in.tracking_deviation_valid=true;
    in.test_pause_request=true;
    lift_seq_update(&g_lift_sequence,&in,&out);
    assert(out.state==LIFT_SEQ_STATE_TEST_PAUSED && out.command_ecd_valid);
    assert(out.axis_target_ecd[0]==15000 && out.axis_target_ecd[2]==37500);
    in.test_pause_request=false;
    in.now_ms=100000;
    lift_seq_update(&g_lift_sequence,&in,&out);
    assert(out.state==LIFT_SEQ_STATE_TEST_PAUSED && out.fault==LIFT_SAFETY_FAULT_NONE);
    in.test_resume_request=true;
    lift_seq_update(&g_lift_sequence,&in,&out);
    assert(out.state==LIFT_SEQ_STATE_RETRACT_FRONT);
    assert(out.axis_target_mm[0]==2 && out.axis_target_mm[2]==50);
    assert(out.step_elapsed_ms==500);
    /* Front axes near the retracted target are cut independently; rear
     * support must still receive the controller's holding current. */
    g_lift_sequence.state=LIFT_SEQ_STATE_RETRACT_FRONT;
    g_lift_sequence.step_start_ms=fake_now;
    g_lift_sequence.stall_active[0]=g_lift_sequence.stall_active[1]=false;
    g_lift_control_debug.test_pause=false;
    g_lift_control_debug.sequence_start=false;
    g_lift_control_debug.stair_retracted_mask=0;
    g_lift_control_debug.gravity_feedforward_current=1000.0F;
    g_lift_control_debug.apply_gains=1;
    for(ch=0;ch<4;ch++)
        feedback.lift.motor[ch].total_ecd=lift_top.config.axis_direction[ch]*(ch<2 ? 1500 : 37500);
    run_task(1);
    assert(g_lift_control_debug.stair_retracted_mask==3);
    assert(sent[0]==0 && sent[1]==0);
    assert(sent[2]!=0 || sent[3]!=0);
    puts("stair startup, per-leg odometry, stop/settle, stall and powered resume passed");
}

int main(void)
{
    test_pid_antiwindup();
    test_task_routing();
    test_axis_limits_and_sync();
    test_loaded_plant_and_reversal();
    test_front_homing_rear_datum();
    test_contact_zero_start_recovery();
    test_one_click_lift();
    test_fault_snapshot();
    test_manual_lift_controls();
    test_software_start_zero();
    test_stall_holds_instead_of_cut();
    test_stair_debug();
    puts("all active axis-chain tests passed");
    return 0;
}
