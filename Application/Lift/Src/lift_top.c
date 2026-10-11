#include "lift_top.h"

#include <stddef.h>

/** int32 饱和到 int16，避免越界转换。 */
static int16_t top_sat_i16(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }
    return (int16_t)value;
}

/** 按每轴方向把原始坐标或速度转换为逻辑抬升坐标。 */
static int32_t top_to_logical(int8_t direction, int32_t value)
{
    return direction < 0 ? -value : value;
}

/**
 * @brief 由【原始编码器位置】与【原始域零点】算出已找零坐标（逻辑域）。
 *
 * 存放约定（两处必须一致，否则方向 -1 的轴会拿错基准）：
 *   self->zero_ecd[ch]        原始编码器域
 *   in->position_ecd[ch]      原始编码器域
 *   homed_position_ecd[ch]    逻辑域（正 = 上升）
 */
static int32_t top_homed_from_raw(
    int8_t direction,
    int32_t raw_position,
    int32_t raw_zero)
{
    return top_to_logical(direction, raw_position) -
           top_to_logical(direction, raw_zero);
}

/** 把模态量清零（模态结构体没有单条清零函数，逐字段更直白）。 */
static void top_clear_modes(lift_sync_modes_t *modes)
{
    modes->height = 0;
    modes->roll = 0;
    modes->pitch = 0;
    modes->warp = 0;
}

void lift_top_init(lift_top_t *self, const lift_top_config_t *config)
{
    uint32_t ch;

    if ((self == NULL) || (config == NULL))
    {
        return;
    }

    *self = (lift_top_t){0};
    self->config = *config;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        if (self->config.axis_direction[ch] > 0)
        {
            self->config.axis_direction[ch] = 1;
        }
        else if (self->config.axis_direction[ch] < 0)
        {
            self->config.axis_direction[ch] = -1;
        }
        else
        {
            self->config.axis_direction[ch] = 1;
        }

        /* top 已把所有通道归一化到逻辑坐标，guard 只处理逻辑正方向。 */
        self->config.guard.direction[ch] = 1;
        lift_homing_init(&self->homing[ch], &self->config.homing);
    }
    lift_guard_init(&self->guard, &self->config.guard);
    lift_control_init(&self->control, &self->config.control);

    self->state = LIFT_TOP_STATE_IDLE;
    self->fault = LIFT_TOP_FAULT_NONE;
    self->homed = false;
}

void lift_top_set_zero_now(
    lift_top_t *self,
    const int32_t raw_position[LIFT_CHANNEL_COUNT])
{
    uint32_t ch;

    if ((self == NULL) || (raw_position == NULL))
    {
        return;
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        const int8_t direction = self->config.axis_direction[ch];

        self->zero_ecd[ch] = raw_position[ch];
        lift_homing_force_done(
            &self->homing[ch], top_to_logical(direction, raw_position[ch]));
    }

    self->homed = true;
    self->state = LIFT_TOP_STATE_READY;
    self->fault = LIFT_TOP_FAULT_NONE;
    lift_control_reset(&self->control);
    lift_guard_init(&self->guard, &self->config.guard);
}

void lift_top_reload_control(
    lift_top_t *self,
    const lift_top_config_t *config)
{
    if ((self == NULL) || (config == NULL))
    {
        return;
    }

    /* 只换控制与安全闸的参数，**保留** zero_ecd / homed / state。 */
    self->config.control = config->control;
    self->config.guard = config->guard;
    lift_control_init(&self->control, &self->config.control);
    lift_guard_init(&self->guard, &self->config.guard);
}

bool lift_top_is_operational(const lift_top_t *self)
{
    if (self == NULL)
    {
        return false;
    }
    return self->homed && (self->state == LIFT_TOP_STATE_READY);
}

void lift_top_update(
    lift_top_t *self,
    const lift_top_input_t *in,
    const lift_control_command_t *command,
    lift_top_status_t *status)
{
    bool homing_rising;
    bool clear_rising;
    bool reset_rising;
    int16_t homing_cur[LIFT_CHANNEL_COUNT];
    uint32_t ch;

    if ((self == NULL) || (in == NULL) || (command == NULL) || (status == NULL))
    {
        return;
    }

    /* 每个周期完整重建状态，避免找零/故障分支残留上一周期数据。 */
    *status = (lift_top_status_t){0};

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        const int8_t direction = self->config.axis_direction[ch];

        status->output_current[ch] = 0;
        status->zero_ecd[ch] = self->zero_ecd[ch];
        status->homed_position_ecd[ch] =
            top_to_logical(
                direction,
                in->position_ecd[ch] - self->zero_ecd[ch]);
        status->homing_state[ch] = self->homing[ch].state;
        status->homing_fault[ch] = self->homing[ch].fault;
        homing_cur[ch] = 0;
    }
    status->at_target = false;
    status->guard_state = self->guard.state;
    status->guard_fault = self->guard.fault;
    /*
     * 找零支路不走控制层，这些量先清零，避免残留上一周期的值——
     * 一个只写不读的字段不会报错，但会让 Watch 窗口撒谎。
     */
    top_clear_modes(&status->actual_modes);
    top_clear_modes(&status->actual_modes_speed);
    top_clear_modes(&status->ramped_target_ecd);
    top_clear_modes(&status->target_speed_ecd_s);
    status->position_loop_height = 0.0F;
    status->position_loop_roll = 0.0F;
    status->position_loop_pitch = 0.0F;
    status->speed_loop_height = 0.0F;
    status->speed_loop_roll = 0.0F;
    status->speed_loop_pitch = 0.0F;

    homing_rising = in->homing_request && !self->homing_last;
    clear_rising = in->clear_homing_request && !self->clear_last;
    reset_rising = in->reset_request && !self->reset_last;
    self->homing_last = in->homing_request;
    self->clear_last = in->clear_homing_request;
    self->reset_last = in->reset_request;

    if (reset_rising)
    {
        self->fault = LIFT_TOP_FAULT_NONE;
        if (self->state == LIFT_TOP_STATE_FAULT)
        {
            self->state = self->homed ? LIFT_TOP_STATE_READY : LIFT_TOP_STATE_IDLE;
        }
    }

    /* 使找零结果失效。故障态必须先 reset，不允许用 clear 绕过。 */
    if (clear_rising && (self->state != LIFT_TOP_STATE_FAULT))
    {
        self->homed = false;
        self->state = LIFT_TOP_STATE_IDLE;
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            self->zero_ecd[ch] = 0;
            lift_homing_init(&self->homing[ch], &self->config.homing);
        }
    }

    /* 顶层故障锁存：只有 reset 上升沿能解。 */
    if (self->state == LIFT_TOP_STATE_FAULT)
    {
        goto emit;
    }

    /* 启动找零：只在未找零时响应，禁止在已找零状态下直接重新找零。 */
    if (homing_rising && !self->homed)
    {
        self->state = LIFT_TOP_STATE_HOMING;
        self->fault = LIFT_TOP_FAULT_NONE;
    }

    if (!self->homed)
    {
        bool all_done = true;
        bool any_fault = false;

        if (self->state != LIFT_TOP_STATE_HOMING)
        {
            /* 既没找零也没在找零：不输出任何电流。 */
            goto emit;
        }

        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            lift_homing_input_t hin;
            lift_homing_output_t hout;
            const int8_t direction = self->config.axis_direction[ch];

            hin.position_ecd = top_to_logical(
                direction, in->position_ecd[ch]);
            hin.speed_rpm = top_sat_i16(top_to_logical(
                direction, (int32_t)in->speed_rpm[ch]));
            hin.online = in->online[ch];
            hin.start_request = homing_rising;   /* 单周期脉冲 */
            hin.abort_request = false;
            hin.now_ms = in->now_ms;

            lift_homing_update(&self->homing[ch], &hin, &hout);

            status->homing_state[ch] = hout.state;
            status->homing_fault[ch] = hout.fault;
            homing_cur[ch] = top_sat_i16(top_to_logical(
                direction, (int32_t)hout.commanded_current));

            if (hout.state == LIFT_HOMING_STATE_FAULT)
            {
                any_fault = true;
            }
            if (hout.state != LIFT_HOMING_STATE_DONE)
            {
                all_done = false;
            }
        }

        if (any_fault)
        {
            /* 只成功三个通道也不行：第四个没有位置基准，整车不得进入运动状态。 */
            self->state = LIFT_TOP_STATE_FAULT;
            self->fault = LIFT_TOP_FAULT_HOMING_FAILED;
            goto emit;
        }

        if (all_done)
        {
            for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            {
                const int8_t direction = self->config.axis_direction[ch];

                /*
                 * ⚠️ 这里必须做一次方向折算，不能直接赋值。
                 *
                 * lift_homing 全程工作在【逻辑域】（它的 position_ecd 就是
                 * top_to_logical 之后的量），所以 homing[ch].zero_ecd 是逻辑
                 * 域的死点。而 self->zero_ecd 存的是【原始编码器域】的值，
                 * 以便与 in->position_ecd（原始域）直接相减。
                 *
                 * 直接赋值的话，direction = -1 的轴零点符号会反过来：
                 * 该轴于是拿着一个错误基准进入闭环，一抬升就等于往死点里顶。
                 */
                self->zero_ecd[ch] = top_to_logical(
                    direction, self->homing[ch].zero_ecd);
                status->zero_ecd[ch] = self->zero_ecd[ch];
                /* 已找零坐标 = 原始位置（也折到逻辑域）− 逻辑零点。 */
                status->homed_position_ecd[ch] = top_homed_from_raw(
                    direction, in->position_ecd[ch], self->zero_ecd[ch]);
            }
            self->homed = true;
            self->state = LIFT_TOP_STATE_READY;
            lift_control_reset(&self->control);
            lift_guard_init(&self->guard, &self->config.guard);
            /* 本周期不再输出：找零刚结束，让控制支路从下一周期接管。 */
            goto emit;
        }

        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            status->output_current[ch] = homing_cur[ch];
        }
        goto emit;
    }

    /* ---- 已找零：正常控制链路 ---- */
    {
        lift_sync_effort_t effort;
        lift_control_status_t ctl_status;
        int32_t cur32[LIFT_CHANNEL_COUNT];
        int16_t logical_speed[LIFT_CHANNEL_COUNT];
        lift_guard_input_t gin;
        lift_guard_output_t gout;

        /*
         * 模态限幅与重力前馈都在 lift_control 内部完成，这里传【已找零坐标】
         * 供限幅判断。下游 guard 的逐轴限位只是兜底，不再承担协作职责。
         *
         * ⚠️ 已找零坐标必须【每周期重新计算】。之前这一支直接沿用 status 里
         * 上一拍的值，而 status 是调用方的局部量——于是"找零完成后第一拍"
         * 会带着 0（因为 top 当时还没算），之后又永远慢一拍。位置环吃的是
         * 这个量，慢一拍就是相位滞后。
         */
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            const int8_t direction = self->config.axis_direction[ch];

            status->zero_ecd[ch] = self->zero_ecd[ch];
            status->homed_position_ecd[ch] = top_homed_from_raw(
                direction, in->position_ecd[ch], self->zero_ecd[ch]);
            logical_speed[ch] = top_sat_i16(top_to_logical(
                direction, (int32_t)in->speed_rpm[ch]));
        }

        lift_control_update(
            &self->control, status->homed_position_ecd, logical_speed,
            command, &effort, &ctl_status);
        lift_sync_distribute(&self->control.config.sync, &effort, cur32);

        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            gin.position_ecd[ch] = status->homed_position_ecd[ch];
            gin.speed_rpm[ch] = logical_speed[ch];
            gin.requested_current[ch] = top_sat_i16(cur32[ch]);
            gin.online[ch] = in->online[ch];
        }
        gin.homed = true;
        gin.command_fresh = in->command_fresh;
        gin.reset_request = reset_rising;
        gin.now_ms = in->now_ms;

        lift_guard_update(&self->guard, &gin, &gout);

        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            status->output_current[ch] = top_sat_i16(top_to_logical(
                self->config.axis_direction[ch],
                (int32_t)gout.output_current[ch]));
            status->limit_blocked[ch] = gout.limit_blocked[ch];
        }
        status->guard_state = gout.state;
        status->guard_fault = gout.fault;
        status->at_target = ctl_status.at_target;
        status->control = ctl_status;
        status->control_effort_height = effort.height;
        status->actual_modes = ctl_status.actual;
        status->actual_modes_speed = ctl_status.actual_speed;
        status->ramped_target_ecd.height = ctl_status.ramped_height_ecd;
        status->ramped_target_ecd.roll = ctl_status.ramped_roll_ecd;
        status->ramped_target_ecd.pitch = ctl_status.ramped_pitch_ecd;
        status->target_speed_ecd_s.height = ctl_status.target_speed_height;
        status->target_speed_ecd_s.roll = ctl_status.target_speed_roll;
        status->target_speed_ecd_s.pitch = ctl_status.target_speed_pitch;
        status->position_loop_height =
            ctl_status.pid_output[LIFT_CONTROL_LOOP_POSITION]
                                 [LIFT_CONTROL_MODE_HEIGHT];
        status->position_loop_roll =
            ctl_status.pid_output[LIFT_CONTROL_LOOP_POSITION]
                                 [LIFT_CONTROL_MODE_ROLL];
        status->position_loop_pitch =
            ctl_status.pid_output[LIFT_CONTROL_LOOP_POSITION]
                                 [LIFT_CONTROL_MODE_PITCH];
        status->speed_loop_height =
            ctl_status.pid_output[LIFT_CONTROL_LOOP_SPEED]
                                 [LIFT_CONTROL_MODE_HEIGHT];
        status->speed_loop_roll =
            ctl_status.pid_output[LIFT_CONTROL_LOOP_SPEED]
                                 [LIFT_CONTROL_MODE_ROLL];
        status->speed_loop_pitch =
            ctl_status.pid_output[LIFT_CONTROL_LOOP_SPEED]
                                 [LIFT_CONTROL_MODE_PITCH];

        if (gout.state == LIFT_GUARD_STATE_FAULT)
        {
            self->state = LIFT_TOP_STATE_FAULT;
            self->fault = LIFT_TOP_FAULT_GUARD;
        }
    }

emit:
    status->state = self->state;
    status->fault = self->fault;
    status->homed = self->homed;

    if (self->state == LIFT_TOP_STATE_FAULT)
    {
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            status->output_current[ch] = 0;
        }
        status->at_target = false;
    }
}
