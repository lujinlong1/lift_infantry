#include "lift_control.h"

#include <stddef.h>
#include <math.h>
#ifdef LIFT_LIMIT_TRACE
#include <stdio.h>
#endif

/* speed_rpm -> ecd/s 与 rpm -> deg/s 共用的换算常数：360 / 60。 */
#define CONTROL_RPM_TO_DEG_S 6.0F

/* 下面按"每个模态"跑同一套链，写全名太长，起三个短别名。 */
#define MODE_HEIGHT LIFT_CONTROL_MODE_HEIGHT
#define MODE_ROLL   LIFT_CONTROL_MODE_ROLL
#define MODE_PITCH  LIFT_CONTROL_MODE_PITCH

static float axis_clamp(float value, float lo, float hi)
{
    return value < lo ? lo : (value > hi ? hi : value);
}

/* The modal inverse is used only to accept the existing manual command API.
 * Sequence targets remain four independent logical encoder positions. */
void lift_control_update_axes(lift_control_t *self,
    const int32_t position[LIFT_CHANNEL_COUNT],
    const int16_t speed_rpm[LIFT_CHANNEL_COUNT],
    const lift_control_command_t *command, float dt_s,
    int32_t current[LIFT_CHANNEL_COUNT], lift_control_status_t *status)
{
    float delta[LIFT_CHANNEL_COUNT], error_mm[LIFT_CHANNEL_COUNT];
    float max_distance = 0.0F, min_error = 0.0F, max_error = 0.0F;
    float max_speed, mean_error = 0.0F, counts_sum = 0.0F;
    int32_t ref32[LIFT_CHANNEL_COUNT], speed32[LIFT_CHANNEL_COUNT];
    lift_sync_modes_t modes;
    uint32_t ch;
    const lift_control_config_t *cfg;
    if (!self || !position || !speed_rpm || !command || !current || !status)
        return;
    cfg = &self->config;
    *status = (lift_control_status_t){0};
    status->limit_scale = 1.0F;
    status->sync_scale = 1.0F;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++) current[ch] = 0;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        counts_sum += cfg->counts_per_mm[ch];
    if (counts_sum <= 0.0F) { lift_control_reset(self); return; }
    if (dt_s <= 0.0F) dt_s = cfg->period_s;
    /* Long task pauses must not cause an unbounded reference/current jump. */
    dt_s = axis_clamp(dt_s, 0.001F, 0.020F);
    lift_sync_forward(&cfg->sync, position, &status->actual);
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        float target, cpm = cfg->counts_per_mm[ch];
        float lo = (float)cfg->limit_height_min_ecd[ch];
        float hi = (float)cfg->limit_height_max_ecd[ch] - cfg->limit_margin_ecd;
        float speed = lift_control_speed_to_ecd_s(cfg, speed_rpm[ch]);
        if (cpm <= 0.0F || hi < lo) { lift_control_reset(self); return; }
        if (command->axis_targets_valid)
            target = (float)command->axis_target_ecd[ch];
        else
            target = (cfg->sync.w_height[ch] * (float)command->target_height_ecd +
                      cfg->sync.w_roll[ch] * (float)command->target_roll_ecd +
                      cfg->sync.w_pitch[ch] * (float)command->target_pitch_ecd) *
                cpm / counts_sum;
        status->axis_target_ecd[ch] = (int32_t)axis_clamp(target, lo, hi);
        if ((float)status->axis_target_ecd[ch] != target) status->limit_active = true;
        status->axis_position_error[ch] =
            (float)status->axis_target_ecd[ch] - (float)position[ch];
        status->axis_speed_ecd_s[ch] = speed;
        speed32[ch] = (int32_t)speed;
        current[ch] = 0;
        if (!self->axis_initialized)
        {
            self->axis_reference[ch] = (float)position[ch];
            self->axis_reference_speed[ch] = 0.0F;
            self->axis_filtered_speed[ch] = speed;
            self->axis_current[ch] = 0.0F;
            self->axis_holding[ch] = false;
            self->axis_open_limit[ch] = cfg->current_limit_start > 0 &&
                cfg->current_limit_ramp_per_s > 0 ?
                (float)cfg->current_limit_start : (float)cfg->axis_current_limit;
        }
        delta[ch] = ((float)status->axis_target_ecd[ch] - self->axis_reference[ch]) / cpm;
        if (fabsf(delta[ch]) > max_distance) max_distance = fabsf(delta[ch]);
        error_mm[ch] = (self->axis_reference[ch] - (float)position[ch]) / cpm;
        mean_error += error_mm[ch] * 0.25F;
        if (ch == 0U || error_mm[ch] < min_error) min_error = error_mm[ch];
        if (ch == 0U || error_mm[ch] > max_error) max_error = error_mm[ch];
    }
    lift_sync_forward(&cfg->sync, speed32, &status->actual_speed);
    if (!command->enabled)
    {
        lift_control_reset(self);
        return;
    }
    self->axis_initialized = true;
    /* Supervise mismatch relative to the moving references. A commanded
     * front/rear height difference is not a synchronization error. */
    status->sync_spread_mm = max_error - min_error;
    if (cfg->sync_stop_mm > cfg->sync_slow_mm &&
        status->sync_spread_mm > cfg->sync_slow_mm)
        status->sync_scale = axis_clamp((cfg->sync_stop_mm - status->sync_spread_mm) /
            (cfg->sync_stop_mm - cfg->sync_slow_mm), 0.0F, 1.0F);
    max_speed = fminf(cfg->target_rate_mm_s,
        sqrtf(2.0F * cfg->target_accel_mm_s2 * max_distance));
    self->planner_speed_mm_s += axis_clamp(max_speed - self->planner_speed_mm_s,
        -cfg->target_accel_mm_s2 * dt_s, cfg->target_accel_mm_s2 * dt_s);
    status->at_target = true;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        float cpm = cfg->counts_per_mm[ch];
        float fraction = max_distance > 0.0F ? fminf(1.0F,
            self->planner_speed_mm_s * status->sync_scale * dt_s / max_distance) : 0.0F;
        float step = delta[ch] * fraction * cpm;
        float ref_speed = step / dt_s;
        float final_error = status->axis_position_error[ch];
        float tolerance = (float)cfg->settle_tolerance_ecd;
        float velocity_limit = cfg->axis_pid[LIFT_CONTROL_LOOP_POSITION][ch].output_limit;
        float ff, lower, upper, limit, output, pos_output, desired_speed;
        float alpha = cfg->speed_filter_tau_ms > 0 ?
            dt_s / (dt_s + (float)cfg->speed_filter_tau_ms * 0.001F) : 1.0F;
        pid_controller_t *speed_pid = &self->axis_pid[LIFT_CONTROL_LOOP_SPEED][ch];
        /* Preserve acceleration limits across target changes/reversals too.
         * Synchronization STOP overrides the trajectory immediately. */
        if (status->sync_scale <= 0.0F)
            ref_speed = 0.0F;
        else
            ref_speed = self->axis_reference_speed[ch] + axis_clamp(
                ref_speed - self->axis_reference_speed[ch],
                -cfg->target_accel_mm_s2 * cpm * dt_s,
                cfg->target_accel_mm_s2 * cpm * dt_s);
        step = ref_speed * dt_s;
        if (step * delta[ch] >= 0.0F && fabsf(step) > fabsf(delta[ch] * cpm))
            step = delta[ch] * cpm;
        {
            float bounded = axis_clamp(self->axis_reference[ch] + step,
                (float)cfg->limit_height_min_ecd[ch],
                (float)cfg->limit_height_max_ecd[ch] - cfg->limit_margin_ecd);
            step = bounded - self->axis_reference[ch];
        }
        ref_speed = step / dt_s;
        self->axis_reference_speed[ch] = ref_speed;
        self->axis_reference[ch] += step;
        self->axis_filtered_speed[ch] += alpha *
            (status->axis_speed_ecd_s[ch] - self->axis_filtered_speed[ch]);
        /* Position P only: no gravity integral in the outer loop. */
        pos_output = cfg->axis_pid[LIFT_CONTROL_LOOP_POSITION][ch].kp *
            (self->axis_reference[ch] - (float)position[ch]);
        pos_output = axis_clamp(pos_output, -velocity_limit, velocity_limit);
        desired_speed = pos_output + ref_speed;
        /* Apply the speed ceiling AFTER adding reference velocity. */
        desired_speed = axis_clamp(desired_speed, -velocity_limit, velocity_limit);
        /* Braking distance caps the final approach in either direction. */
        {
            float brake_limit = sqrtf(2.0F * cfg->target_accel_mm_s2 *
                fabsf(final_error) / cpm) * cpm;
            desired_speed = axis_clamp(desired_speed, -brake_limit, brake_limit);
        }
        if (fabsf(final_error) > tolerance * 2.0F || fabsf(delta[ch] * cpm) > tolerance * 2.0F)
            self->axis_holding[ch] = false;
        if (fabsf(final_error) <= tolerance && fabsf(delta[ch] * cpm) <= tolerance)
            self->axis_holding[ch] = true;
        if (self->axis_holding[ch])
        {
            desired_speed = 0.0F;
            status->axis_hold_state[ch] = fabsf(status->axis_speed_ecd_s[ch]) <=
                cfg->hold_speed_ecd_s ? LIFT_CONTROL_HOLD_HOLD : LIFT_CONTROL_HOLD_BRAKE;
        }
        if (fabsf(final_error) > tolerance ||
            fabsf(status->axis_speed_ecd_s[ch]) > cfg->hold_speed_ecd_s ||
            fabsf(delta[ch] * cpm) > tolerance)
            status->at_target = false;
        /* HOLD continues PI: retain learned supporting current, avoid the
         * old HOLD->zero PI->fall->APPROACH switching cycle. */
        ff = cfg->axis_load_current[ch];
        if (cfg->feedforward_min_position_ecd > 0 &&
            position[ch] <= cfg->feedforward_min_position_ecd) ff = 0.0F;
        self->axis_open_limit[ch] = fminf((float)cfg->axis_current_limit,
            self->axis_open_limit[ch] + cfg->current_limit_ramp_per_s * dt_s);
        limit = fminf(self->axis_open_limit[ch], speed_pid->config.output_limit);
        lower = -limit;
        upper = limit;
        if (cfg->current_ramp_per_s > 0)
        {
            lower = fmaxf(lower, self->axis_current[ch] - cfg->current_ramp_per_s * dt_s);
            upper = fminf(upper, self->axis_current[ch] + cfg->current_ramp_per_s * dt_s);
        }
        /* Hard travel protection takes priority over current slew. */
        if (position[ch] >= cfg->limit_height_max_ecd[ch]) upper = fminf(upper, 0.0F);
        if (position[ch] <= cfg->limit_height_min_ecd[ch]) lower = fmaxf(lower, 0.0F);
        if (lower > upper) lower = upper = 0.0F;
        self->axis_integral_before[ch] = speed_pid->integral;
        output = pid_calculate_limited(speed_pid, desired_speed,
            self->axis_filtered_speed[ch], dt_s, lower - ff, upper - ff);
        self->axis_current[ch] = axis_clamp(output + ff, lower, upper);
        current[ch] = (int32_t)self->axis_current[ch];
        status->axis_reference_ecd[ch] = self->axis_reference[ch];
        status->axis_target_speed_ecd_s[ch] = desired_speed;
        status->axis_speed_error[ch] = speed_pid->error;
        status->axis_filtered_speed_ecd_s[ch] = self->axis_filtered_speed[ch];
        status->axis_position_output[ch] = pos_output;
        status->axis_speed_output[ch] = output;
        status->axis_feedforward_current[ch] = ff;
        status->axis_final_current[ch] = (float)current[ch];
        status->axis_speed_integral[ch] = speed_pid->integral;
        status->axis_limit_now[ch] = limit;
        status->axis_current_lower[ch] = lower;
        status->axis_current_upper[ch] = upper;
        status->axis_current_demand[ch] = speed_pid->p_output + speed_pid->i_output +
            speed_pid->d_output + ff;
        status->axis_sync_error_mm[ch] = error_mm[ch] - mean_error;
        ref32[ch] = (int32_t)self->axis_reference[ch];
        speed32[ch] = (int32_t)desired_speed;
    }
    lift_sync_forward(&cfg->sync, ref32, &modes);
    status->ramped_height_ecd = modes.height;
    status->ramped_roll_ecd = modes.roll;
    status->ramped_pitch_ecd = modes.pitch;
    lift_sync_forward(&cfg->sync, speed32, &modes);
    status->target_speed_height = modes.height;
    status->target_speed_roll = modes.roll;
    status->target_speed_pitch = modes.pitch;
    status->error_height_ecd = command->target_height_ecd - status->actual.height;
    status->limit_active = status->limit_active || status->sync_scale < 1.0F;
}

void lift_control_track_axes(lift_control_t *self,
    const int16_t applied[LIFT_CHANNEL_COUNT], lift_control_status_t *status)
{
    uint32_t ch;
    if (!self || !applied || !status) return;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        pid_controller_t *pid = &self->axis_pid[LIFT_CONTROL_LOOP_SPEED][ch];
        if (pid->error * (status->axis_final_current[ch] - (float)applied[ch]) > 0.0F)
        {
            pid->integral = self->axis_integral_before[ch];
            pid->i_output = pid->config.ki * pid->integral;
        }
        self->axis_current[ch] = (float)applied[ch];
        status->axis_final_current[ch] = (float)applied[ch];
        status->axis_speed_integral[ch] = pid->integral;
    }
}

/** 取 32 位绝对值。 */
static int32_t control_abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

/** 把 current 朝 target 移动最多 step；step<=0 表示不限制。 */
static int32_t control_ramp_toward(int32_t current, int32_t target, int32_t step)
{
    int32_t delta;

    if (step <= 0)
    {
        return target;
    }

    delta = target - current;
    if (delta > step)
    {
        return current + step;
    }
    if (delta < -step)
    {
        return current - step;
    }
    return target;
}

/** float 转 int32 并饱和，避免越界转换是未定义行为。 */
static int32_t control_f32_to_i32_sat(float value)
{
    if (value > 2000000000.0F)
    {
        return 2000000000;
    }
    if (value < -2000000000.0F)
    {
        return -2000000000;
    }
    return (int32_t)value;
}

/*
 * ---- 平滑链的四个小工具（2026-10-11 从台架模块移植）--------------------
 *
 * 台架模块（lift_jog_task.c）在实机上调出来的一套"怎么让它丝滑到位"，
 * 这里按模态复用同样的四个动作：斜坡速率、接近减速、逐级开放、电流斜坡。
 */

/**
 * @brief 本周期目标斜坡的实际速率（模态 ecd/s），供跟踪前馈用。
 * @note 斜坡关掉（step <= 0）时目标是一步到位的阶跃，速率算出来是"无穷大"
 *       （host 实测 2e7 ecd/s），会把内环一拍拍到满电流 —— 所以必须返回 0。
 */
static int32_t control_ramp_rate(int32_t delta, int32_t step, float dt)
{
    if ((step <= 0) || (dt <= 0.0F))
    {
        return 0;
    }
    return control_f32_to_i32_sat((float)delta / dt);
}

/** 一阶低通一步：alpha = dt/(tau+dt)；alpha >= 1 时直通。 */
static float control_low_pass(float previous, float input, float alpha)
{
    if (alpha >= 1.0F)
    {
        return input;
    }
    return previous + (alpha * (input - previous));
}

/** 限幅到 ±min(逐级开放上限, 硬限幅)；两个上限都为负时按 0。 */
static int32_t control_clamp_i32(
    int32_t value,
    int32_t open_limit,
    int32_t full_limit)
{
    int32_t limit = (open_limit < full_limit) ? open_limit : full_limit;

    if (limit < 0)
    {
        limit = 0;
    }
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

/*
 * 逐级开放的电流上限：使能后从 current_limit_start 起步，按
 * current_limit_ramp_per_s 放开到 full_limit。
 *
 * 为什么需要：一使能就给满电流，位置环还在零误差附近，机构会"窜一下"；
 * 而且这是用户实测"空载大幅摆动"的解药之一。
 *
 * 顺序注意：**先返回本拍允许的值，再把下一拍推进**（反过来会让第一拍
 * 就变成 start + rate·dt）。
 */
static int32_t control_open_limit(lift_control_t *self, uint32_t mode)
{
    const int32_t start = (self->config.current_limit_start > 0)
        ? self->config.current_limit_start : 0;
    const int32_t rate = (self->config.current_limit_ramp_per_s > 0)
        ? self->config.current_limit_ramp_per_s : 0;
    int32_t open = self->limit_open[mode];

    if (rate <= 0)
    {
        /* 不逐级开放：直接用满限幅（由输出限幅那一层兜住） */
        open = 2000000000;
    }
    if (open < start)
    {
        open = start;
    }

    self->limit_open[mode] = control_f32_to_i32_sat(
        (float)open + ((float)rate * self->config.period_s));
    return open;
}

/*
 * 接近目标时的目标速度上限（模态 ecd/s）。返回 < 0 = 不减速。
 *
 *   |误差| >= approach_zone            → 不减速（用位置环自己的限幅）
 *   deadband < |误差| < approach_zone  → 线性收窄到 approach_min_speed_ecd_s
 *   |误差| <= deadband                 → 0（交给三态机刹停）
 *
 * ⚠️ 两个坑（都是台架 host 仿真抓出来的，别再踩）：
 *   ① 收窄【电流上限】→ 机构停在半路推不过去（上限刚好等于重力+静摩擦）；
 *   ② 速度指令一路收到 0 → 最后几十个计数推不动，卡在死区外进不去。
 * 所以收窄的是速度指令，而且**只收到一个非零下限**。
 */
static int32_t control_approach_vmax(
    const lift_control_config_t *config,
    int32_t error,
    int32_t deadband)
{
    const int32_t zone = config->approach_zone_ecd;
    const int32_t magnitude = control_abs_i32(error);
    const int32_t vmin = (config->approach_min_speed_ecd_s > 0)
        ? config->approach_min_speed_ecd_s : 0;
    const int32_t vmax = control_f32_to_i32_sat(
        config->pid[LIFT_CONTROL_LOOP_POSITION].output_limit);

    if ((zone <= 0) || (magnitude >= zone) || (zone <= deadband))
    {
        return -1;   /* 不减速 */
    }
    if (magnitude <= deadband)
    {
        return 0;    /* 刹停 */
    }
    if (vmax <= vmin)
    {
        return vmax;
    }
    return vmin + ((vmax - vmin) * (magnitude - deadband) / (zone - deadband));
}

/*
 * 电流斜坡（di/dt 限幅）：把上一拍真正放出去的模态控制量朝目标挪一小步。
 *
 * 为什么主链也需要：这是"一给目标就一瞬间到 16000"的直接解药（护齿轮、护 C620）。
 * ⚠️ 但它**不是越小越安全**：太小会让电流跟不上控制器的刹车请求，过冲反而更大
 *    （台架实测 20000 → 过冲 18.8%，60000 → 1.9%）。两个方向都别调过头。
 */
static int32_t control_ramp_current(
    lift_control_t *self,
    uint32_t mode,
    int32_t desired,
    float dt)
{
    const float rate = (float)self->config.current_ramp_per_s;
    float value = self->current_ramped[mode];

    if ((rate <= 0.0F) || (dt <= 0.0F))
    {
        value = (float)desired;
    }
    else
    {
        const float step = rate * dt;
        const float delta = (float)desired - value;

        if (delta > step)
        {
            value += step;
        }
        else if (delta < -step)
        {
            value -= step;
        }
        else
        {
            value = (float)desired;
        }
    }

    self->current_ramped[mode] = value;
    return control_f32_to_i32_sat(value);
}

/** 取三者较小值。 */
static float control_min3(float a, float b, float c)
{
    float m = (a < b) ? a : b;
    return (m < c) ? m : c;
}

/** 某轴在一个区间里的余量：可继续向上 / 继续向下各走多少计数。 */
typedef struct
{
    int32_t up;     /* 还能往正方向走多少 */
    int32_t down;   /* 还能往负方向走多少 */
} control_room_t;

/*
 * 两个方向【必须分开算】。
 *
 * 踩过的坑：一开始把它们取成同一个最小值，结果车停在地面时
 * "到下限余量 ≈ 0"，连【向上】的出力也被压成 0，车根本抬不起来。
 * 下限紧不影响往上走，这两件事没有关系。
 */
static control_room_t control_room(
    int32_t position,
    int32_t lower,
    int32_t upper,
    int32_t margin)
{
    control_room_t r;

    r.up = upper - position - margin;
    r.down = position - lower - margin;

    /* 负余量统一夹到 0：已经贴着界了，就没有往那边走的余量。 */
    if (r.up < 0)
    {
        r.up = 0;
    }
    if (r.down < 0)
    {
        r.down = 0;
    }
    return r;
}

/*
 * 模态限幅：算一个统一比例 s（0~1），把三个模态控制量同时乘以 s，
 * 使得【没有任何一根轴会越过自己的行程区间】。
 *
 * 为什么逐轴清零是错的：一根轴上的电流是「高度 + 横滚 + 俯仰」三个分量
 * 之和，其中一部分是【在帮别的轴调平】。逐轴判决看不见这一点，一旦触界就
 * 把整轴清零，等于把这根轴从协同里踢出去，车身再也拉不平，而且会反复触发。
 *
 * 这里改成：谁快要越界，就按比例压制"让它继续往外走"的那部分模态出力，
 * 其余模态对它的贡献保留下来。
 *
 * 判据用的是【位置余量 + 本周期还有多少冲量】：
 *   本周期位移 ≈ m_i × dt，m_i 是第 i 个模态的当前出力。
 * 于是"还能放行多少比例"= 余量 / 该模态对本轴的贡献。
 * 三个模态各算一个 s，取最小 —— 保守，但保证不越界。
 */
static float control_limit_scale(
    const lift_control_config_t *cfg,
    const int32_t position[LIFT_CHANNEL_COUNT],
    const lift_sync_effort_t *effort,
    const lift_sync_effort_t *feedforward)
{
    const int32_t margin = (cfg->limit_margin_ecd > 0)
                               ? cfg->limit_margin_ecd : 0;
    /*
     * 电流 -> 本周期位移 的换算因子。
     *
     *   每轴位移 = 每轴电流 × K × dt，其中 K 是执行器增益。
     *   模态量是按"四轴之和"定义的（正变换不除 4），而这里算的是【每轴】
     *   位移，所以不再出现那个 4 倍——effort 经反变换后每根轴拿到的就是
     *   ±effort 本身。
     *
     * actuator_gain 未标定时为 0：此时不做预测限幅，交给 guard 兜底。
     */
    const float gain = cfg->actuator_gain_ecd_s_per_current;
    const float disp_per_current = gain * cfg->period_s;
    const float e_height = (float)(effort->height + feedforward->height);
    const float e_roll = (float)(effort->roll + feedforward->roll);
    const float e_pitch = (float)(effort->pitch + feedforward->pitch);
    float scale = 1.0F;
    uint32_t ch;

    if (disp_per_current <= 0.0F)
    {
        return 1.0F;
    }

#ifdef LIFT_LIMIT_TRACE
    (void)printf("[lim] dpc=%.4f eH=%.0f eR=%.0f eP=%.0f |", 
                 (double)disp_per_current, (double)e_height,
                 (double)e_roll, (double)e_pitch);
#endif

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        const int32_t w_h = (int32_t)cfg->sync.w_height[ch];
        const int32_t w_r = (int32_t)cfg->sync.w_roll[ch];
        const int32_t w_p = (int32_t)cfg->sync.w_pitch[ch];
        const float c_h = e_height * (float)w_h;
        const float c_r = e_roll * (float)w_r;
        const float c_p = e_pitch * (float)w_p;
        const float c = c_h + c_r + c_p;
        const float disp = c * disp_per_current;
        const control_room_t room = control_room(
            position[ch],
            cfg->limit_height_min_ecd[ch],
            cfg->limit_height_max_ecd[ch],
            margin);

        if (disp > 0.0F)
        {
            /* 往正方向走：只看"还能往上多少" */
            if (disp > (float)room.up)
            {
                const float s = (float)room.up / disp;
                scale = control_min3(scale, s, 1.0F);
            }
        }
        else if (disp < 0.0F)
        {
            /* 往负方向走：只看"还能往下多少"。下限紧不影响往上走。 */
            if (-disp > (float)room.down)
            {
                const float s = (float)room.down / (-disp);
                scale = control_min3(scale, s, 1.0F);
            }
        }
        else
        {
            /* 该轴本周期不打算动。 */
        }
#ifdef LIFT_LIMIT_TRACE
        (void)printf(" c%d=%.0f d=%.2f up=%d dn=%d s=%.3f |",
                     (int)ch, (double)c, (double)disp,
                     (int)room.up, (int)room.down, (double)scale);
#endif
    }

    if (scale < 0.0F)
    {
        scale = 0.0F;
    }
    if (scale > 1.0F)
    {
        scale = 1.0F;
    }
    return scale;
}

float lift_control_speed_to_ecd_s(
    const lift_control_config_t *config,
    int16_t speed_rpm)
{
    float value;

    if ((config == NULL) || (config->ecd_per_rev <= 0) ||
        (config->gear_ratio <= 0.0F))
    {
        return 0.0F;
    }

    value = (float)speed_rpm * (float)config->ecd_per_rev *
            config->gear_ratio * CONTROL_RPM_TO_DEG_S / 360.0F;

    /* 反馈是转子侧转速时，先除减速比还原到输出侧。 */
    if (config->speed_is_rotor)
    {
        value /= config->gear_ratio;
    }

    return value;
}

/**
 * @brief 把四轴 speed_rpm 换算为四轴 ecd/s。
 * @note 转速语义由 speed_is_rotor 决定（LIFT-007）。
 *       转子侧解释比输出侧解释大整整一个减速比，3508 上约 19 倍，
 *       所以这个开关翻了之后速度环增益必须跟着重标。
 */
static void control_speed_rpm_to_ecd_s(
    const lift_control_config_t *config,
    const int16_t speed_rpm[LIFT_CHANNEL_COUNT],
    int32_t speed_ecd_s[LIFT_CHANNEL_COUNT])
{
    uint32_t ch;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        speed_ecd_s[ch] =
            control_f32_to_i32_sat(lift_control_speed_to_ecd_s(
                config, speed_rpm[ch]));
    }
}

/** 四条环、三个模态全部复位。 */
static void control_reset_all_pid(lift_control_t *self)
{
    uint32_t loop;
    uint32_t mode;

    for (loop = 0U; loop < (uint32_t)LIFT_CONTROL_LOOP_COUNT; loop++)
    {
        for (mode = 0U; mode < (uint32_t)LIFT_CONTROL_MODE_COUNT; mode++)
        {
            pid_reset(&self->pid[loop][mode]);
        }
    }
}

void lift_control_reset(lift_control_t *self)
{
    if (self == NULL)
    {
        return;
    }

    control_reset_all_pid(self);

    /*
     * 下次进入使能时，斜坡从【当前位置】起步、两条环的积分从零开始，
     * 而不是从 0 或旧目标跳过去。
     */
    self->ramp_initialized = false;
    self->axis_initialized = false;
    self->planner_speed_mm_s = 0.0F;
    {
        uint32_t loop, ch;
        for (loop = 0U; loop < LIFT_CONTROL_LOOP_COUNT; loop++)
            for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
                pid_reset(&self->axis_pid[loop][ch]);
    }
}

void lift_control_init(lift_control_t *self, const lift_control_config_t *config)
{
    uint32_t loop;
    uint32_t mode;

    if ((self == NULL) || (config == NULL))
    {
        return;
    }

    *self = (lift_control_t){0};
    self->config = *config;

    if (self->config.period_s <= 0.0F)
    {
        self->config.period_s = 0.005F;
    }
    if (self->config.ecd_per_rev <= 0)
    {
        self->config.ecd_per_rev = 8192;
    }
    if (self->config.gear_ratio <= 0.0F)
    {
        self->config.gear_ratio = 1.0F;
    }
    if (self->config.max_target_step_ecd < 0)
    {
        self->config.max_target_step_ecd = -self->config.max_target_step_ecd;
    }
    if (self->config.settle_tolerance_ecd < 0)
    {
        self->config.settle_tolerance_ecd = -self->config.settle_tolerance_ecd;
    }
    if (self->config.limit_margin_ecd < 0)
    {
        self->config.limit_margin_ecd = -self->config.limit_margin_ecd;
    }
    if (self->config.gravity_feedforward_current < 0.0F)
    {
        /* 配置填的是"保持电流大小"，取绝对值；方向由模块内部固定为【抬升】。 */
        self->config.gravity_feedforward_current =
            -self->config.gravity_feedforward_current;
    }
    if (self->config.feedforward_min_position_ecd < 0)
    {
        self->config.feedforward_min_position_ecd =
            -self->config.feedforward_min_position_ecd;
    }
    /* 平滑链的参数都是"大小"，取绝对值；0 有各自明确的含义（关/不开放） */
    if (self->config.approach_zone_ecd < 0)
    {
        self->config.approach_zone_ecd = -self->config.approach_zone_ecd;
    }
    if (self->config.approach_min_speed_ecd_s < 0)
    {
        self->config.approach_min_speed_ecd_s =
            -self->config.approach_min_speed_ecd_s;
    }
    if (self->config.hold_speed_ecd_s < 0)
    {
        self->config.hold_speed_ecd_s = -self->config.hold_speed_ecd_s;
    }
    if (self->config.current_ramp_per_s < 0)
    {
        self->config.current_ramp_per_s = -self->config.current_ramp_per_s;
    }
    if (self->config.current_limit_start < 0)
    {
        self->config.current_limit_start = -self->config.current_limit_start;
    }
    if (self->config.current_limit_ramp_per_s < 0)
    {
        self->config.current_limit_ramp_per_s =
            -self->config.current_limit_ramp_per_s;
    }
    if (self->config.speed_filter_tau_ms < 0)
    {
        self->config.speed_filter_tau_ms = -self->config.speed_filter_tau_ms;
    }

    /*
     * ---- 限幅区间规范化 ----
     *
     * max <= min 表示【该模态不启用限幅】，必须在这里显式展开成一个宽区间。
     *
     * 踩过的坑：本工程习惯用 memset(c,0,...) 初始化配置，此时 min=max=0。
     * 若不处理，"区间"就是空的、两个方向余量恒为 0，限幅会把所有出力压成 0
     * —— 表现为车一动不动，而配置看起来"什么都没设错"。
     * 这与 limit_roll/pitch 用 0/0 表示不启用是同一套约定，必须一致。
     */
    {
        const int32_t wide = 1000000;   /* 1e6 计数远大于任何实际行程 */
        uint32_t ch;

        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            if (self->config.limit_height_max_ecd[ch] <=
                self->config.limit_height_min_ecd[ch])
            {
                self->config.limit_height_min_ecd[ch] = -wide;
                self->config.limit_height_max_ecd[ch] = wide;
            }
        }
    }

    /* 权重不合法时回落到标准矩形布局，避免整条链路静默输出垃圾。 */
    if (!lift_sync_config_is_orthogonal(&self->config.sync))
    {
        lift_sync_config_rect(&self->config.sync);
    }

    for (loop = 0U; loop < (uint32_t)LIFT_CONTROL_LOOP_COUNT; loop++)
    {
        for (mode = 0U; mode < (uint32_t)LIFT_CONTROL_MODE_COUNT; mode++)
        {
            pid_init(&self->pid[loop][mode], &self->config.pid[loop]);
        }
        for (mode = 0U; mode < LIFT_CHANNEL_COUNT; mode++)
            pid_init(&self->axis_pid[loop][mode], &self->config.axis_pid[loop][mode]);
    }

    self->ramp_initialized = false;
}

void lift_control_update(
    lift_control_t *self,
    const int32_t position[LIFT_CHANNEL_COUNT],
    const int16_t speed_rpm[LIFT_CHANNEL_COUNT],
    const lift_control_command_t *command,
    lift_sync_effort_t *effort,
    lift_control_status_t *status)
{
    lift_sync_modes_t modes;
    lift_sync_modes_t modes_speed;
    lift_sync_effort_t feedforward;
    int32_t speed_ecd_s[LIFT_CHANNEL_COUNT];
    float position_output[LIFT_CONTROL_MODE_COUNT];
    float speed_output[LIFT_CONTROL_MODE_COUNT];
    int32_t target_speed[LIFT_CONTROL_MODE_COUNT];
    int32_t actual[LIFT_CONTROL_MODE_COUNT];
    int32_t ramped[LIFT_CONTROL_MODE_COUNT];
    int32_t final_target[LIFT_CONTROL_MODE_COUNT];
    int32_t ramp_rate[LIFT_CONTROL_MODE_COUNT];
    int32_t pid_effort[LIFT_CONTROL_MODE_COUNT];
    int32_t limit_now[LIFT_CONTROL_MODE_COUNT];
    float limit_scale;
    uint32_t m;
    uint32_t ch;

    if ((self == NULL) || (position == NULL) || (speed_rpm == NULL) ||
        (command == NULL) || (effort == NULL) || (status == NULL))
    {
        return;
    }

    /* 先清零，保证任何提前返回都不会把上一周期的控制量留在总线上。 */
    effort->height = 0;
    effort->roll = 0;
    effort->pitch = 0;
    for (m = 0U; m < (uint32_t)LIFT_CONTROL_MODE_COUNT; m++)
    {
        status->pid_output[LIFT_CONTROL_LOOP_POSITION][m] = 0.0F;
        status->pid_output[LIFT_CONTROL_LOOP_SPEED][m] = 0.0F;
        status->hold_state[m] = (uint8_t)LIFT_CONTROL_HOLD_APPROACH;
        status->limit_open_current[m] = 0;
    }
    status->target_speed_height = 0;
    status->target_speed_roll = 0;
    status->target_speed_pitch = 0;
    status->limit_scale = 1.0F;
    status->limit_active = false;
    status->at_target = false;

    /*
     * 重力/保持前馈：本机构不自锁，停在行程中间时托住车身的力【只能】由电机出。
     * 配置值填的是"保持电流大小"（正数），这里按【抬升方向（逻辑正）】施加。
     *
     * ⚠️ 2026-10-11 修正：此前这里是 `-gravity_feedforward_current`，按"向下"
     *    施加——方向是反的。实测确认的机构关系是"正电流 = 齿条伸出 = 把车
     *    顶起来"，要托住车必须给正向电流；而且台架模块的 hold_feedforward_
     *    current 一直是正向相加，同一固件里两处符号相反本身就是信号。
     *    旧方向若真的填了标定值，现象是"越给保持电流车掉得越快"。
     *
     * 施加门槛：四轴里【最低】的那根高过 feedforward_min_position_ecd 才施加。
     * 车压在最低端（零点）时重量由地面/挡块承担，再给正向电流只会白烧电机。
     *
     * 若不给这份前馈，保持电流就只能靠位置环积分去凑，而"要出力就必须有
     * 误差"，于是位置永远收敛不到目标——仿真里表现为目标 20000、
     * 稳态停在 18974（差 5.1%），三个模态看起来却都"收敛了"。
     */
    {
        int32_t lowest = position[0];

        for (ch = 1U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            if (position[ch] < lowest)
            {
                lowest = position[ch];
            }
        }

        if (lowest > self->config.feedforward_min_position_ecd)
        {
            feedforward.height = (int32_t)self->config.gravity_feedforward_current;
        }
        else
        {
            feedforward.height = 0;
        }
    }
    feedforward.roll = 0;
    feedforward.pitch = 0;

    lift_sync_forward(&self->config.sync, position, &modes);
    status->actual = modes;
    control_speed_rpm_to_ecd_s(&self->config, speed_rpm, speed_ecd_s);
    lift_sync_forward_speed(&self->config.sync, speed_ecd_s, &modes_speed);
    status->actual_speed = modes_speed;

    if (!command->enabled)
    {
        lift_control_reset(self);
        status->ramped_height_ecd = modes.height;
        status->ramped_roll_ecd = modes.roll;
        status->ramped_pitch_ecd = modes.pitch;
        status->error_height_ecd = 0;
        status->error_roll_ecd = 0;
        status->error_pitch_ecd = 0;
        return;
    }

    /* 首次进入使能：斜坡吸附到当前位置，两环从零积分、斜坡缓存清零。 */
    if (!self->ramp_initialized)
    {
        /*
         * 速度反馈低通的状态吸附到【当前实测速度】，不是 0：
         * 从 0 起步会让第一拍的低通输出只有真值的一小部分（α≈0.2 时只剩 1/5），
         * 速度环于是以为"还没动"而多给一拍电流 —— 一个凭空的启动冲击。
         * 同理电流斜坡缓存清 0：让保持电流也从 0 平滑加上去，而不是第一拍跳变。
         */
        self->ramped_height_ecd = modes.height;
        self->ramped_roll_ecd = modes.roll;
        self->ramped_pitch_ecd = modes.pitch;
        self->speed_filtered[LIFT_CONTROL_MODE_HEIGHT] = (float)modes_speed.height;
        self->speed_filtered[LIFT_CONTROL_MODE_ROLL] = (float)modes_speed.roll;
        self->speed_filtered[LIFT_CONTROL_MODE_PITCH] = (float)modes_speed.pitch;
        for (m = 0U; m < (uint32_t)LIFT_CONTROL_MODE_COUNT; m++)
        {
            self->current_ramped[m] = 0.0F;
            self->limit_open[m] = self->config.current_limit_start;
            self->hold_state[m] = (uint8_t)LIFT_CONTROL_HOLD_APPROACH;
        }
        self->ramp_initialized = true;
        control_reset_all_pid(self);
    }

    /* ---- ① 目标斜坡（同时算出本周期实际速率，供跟踪前馈）---- */
    {
        const int32_t step = self->config.max_target_step_ecd;
        int32_t before;

        before = self->ramped_height_ecd;
        self->ramped_height_ecd = control_ramp_toward(
            before, command->target_height_ecd, step);
        ramp_rate[LIFT_CONTROL_MODE_HEIGHT] = control_ramp_rate(
            self->ramped_height_ecd - before, step, self->config.period_s);

        before = self->ramped_roll_ecd;
        self->ramped_roll_ecd = control_ramp_toward(
            before, command->target_roll_ecd, step);
        ramp_rate[LIFT_CONTROL_MODE_ROLL] = control_ramp_rate(
            self->ramped_roll_ecd - before, step, self->config.period_s);

        before = self->ramped_pitch_ecd;
        self->ramped_pitch_ecd = control_ramp_toward(
            before, command->target_pitch_ecd, step);
        ramp_rate[LIFT_CONTROL_MODE_PITCH] = control_ramp_rate(
            self->ramped_pitch_ecd - before, step, self->config.period_s);
    }

    /* ---- 模态量装进数组，下面按模态跑同一套链 ---- */
    actual[MODE_HEIGHT] = modes.height;
    actual[MODE_ROLL] = modes.roll;
    actual[MODE_PITCH] = modes.pitch;
    final_target[MODE_HEIGHT] = command->target_height_ecd;
    final_target[MODE_ROLL] = command->target_roll_ecd;
    final_target[MODE_PITCH] = command->target_pitch_ecd;
    ramped[MODE_HEIGHT] = self->ramped_height_ecd;
    ramped[MODE_ROLL] = self->ramped_roll_ecd;
    ramped[MODE_PITCH] = self->ramped_pitch_ecd;

    /*
     * 前馈只加在高度模态（重力是单向的；横滚/俯仰没有恒定偏载），
     * 并且统一在下面的限幅块里加 —— 这样它既参与模态限幅的比例计算，
     * 也受逐级开放与电流上限约束。分装在各状态分支里加过一次就重复了。
     */

    /* 速度反馈一阶低通（C620 的 rpm 是整数，1 rpm 台阶在内环会被放大） */
    {
        const float tau_s = (float)self->config.speed_filter_tau_ms / 1000.0F;
        const float alpha = (tau_s > 0.0F)
            ? (self->config.period_s / (tau_s + self->config.period_s)) : 1.0F;

        self->speed_filtered[MODE_HEIGHT] = control_low_pass(
            self->speed_filtered[MODE_HEIGHT], (float)modes_speed.height, alpha);
        self->speed_filtered[MODE_ROLL] = control_low_pass(
            self->speed_filtered[MODE_ROLL], (float)modes_speed.roll, alpha);
        self->speed_filtered[MODE_PITCH] = control_low_pass(
            self->speed_filtered[MODE_PITCH], (float)modes_speed.pitch, alpha);
    }

    /*
     * ---- ②③④ 每个模态：位置环 → 减速区 → 跟踪前馈 → 状态机 → 速度环 ----
     *
     * 这一段与台架模块（lift_jog_task.c 的 lift_test_cascade_axis）同构，
     * 只是把"每根轴"换成"每个模态"。台架那套是 host 仿真 + 实机验证过的，
     * 搬过来的理由逐条写在配置结构（lift_control_config_t）里。
     */
    for (m = 0U; m < (uint32_t)LIFT_CONTROL_MODE_COUNT; m++)
    {
        const int32_t deadband = self->config.settle_tolerance_ecd;
        const int32_t error = ramped[m] - actual[m];
        const int32_t speed = control_f32_to_i32_sat(self->speed_filtered[m]);
        const bool settled = (ramped[m] == final_target[m]);
        int32_t value;

        /* 逐级开放的电流上限（先取本拍的值，再推进下一拍） */
        limit_now[m] = control_open_limit(self, m);

        /*
         * 三态机：APPROACH / BRAKE / HOLD。
         *
         * ⚠️ 转 BRAKE 必须同时满足 target_settled（斜坡走到头）：
         * 目标还在斜坡上时位置误差本来就只有几十计数（跟踪前馈压住了），
         * 只看误差就刹车会变成"目标还在走、车却停了"的抖振。
         */
        if ((control_abs_i32(error) > deadband) || !settled)
        {
            if (self->hold_state[m] != (uint8_t)LIFT_CONTROL_HOLD_APPROACH)
            {
                /*
                 * 从 BRAKE/HOLD 退回 APPROACH：先清两条环的历史。
                 * HOLD 期间根本没调用 PID，previous_error 还是进去之前那一拍的，
                 * 直接接着算微分会得到一个巨大的假导数（退出瞬间踢一脚）。
                 */
                pid_reset(&self->pid[LIFT_CONTROL_LOOP_POSITION][m]);
                pid_reset(&self->pid[LIFT_CONTROL_LOOP_SPEED][m]);
            }
            self->hold_state[m] = (uint8_t)LIFT_CONTROL_HOLD_APPROACH;
        }
        else if (self->hold_state[m] == (uint8_t)LIFT_CONTROL_HOLD_APPROACH)
        {
            self->hold_state[m] = (uint8_t)LIFT_CONTROL_HOLD_BRAKE;
        }

        if ((self->hold_state[m] == (uint8_t)LIFT_CONTROL_HOLD_BRAKE) &&
            (self->config.hold_speed_ecd_s > 0) &&
            (control_abs_i32(speed) <= self->config.hold_speed_ecd_s))
        {
            self->hold_state[m] = (uint8_t)LIFT_CONTROL_HOLD_HOLD;
        }

        if (self->hold_state[m] == (uint8_t)LIFT_CONTROL_HOLD_HOLD)
        {
            /*
             * HOLD：两条环退出，只给保持前馈把车托住。
             * 本机构不自锁，这一项是 0 就会自己落下去（除非车压在最低端，
             * 那时前馈本身被 feedforward_min_position_ecd 关掉）。
             *
             * 注意这里 pid_effort 只装【PID 那一份】= 0；前馈由下面的限幅块
             * 统一加（那里还要参与模态限幅与电流上限），不能在这儿重复加。
             */
            position_output[m] = 0.0F;
            speed_output[m] = 0.0F;
            target_speed[m] = 0;
            value = 0;
        }
        else if (self->hold_state[m] == (uint8_t)LIFT_CONTROL_HOLD_BRAKE)
        {
            /* 刹车：速度指令强制 0，让速度环主动把速度拉下来（不是断力） */
            position_output[m] = 0.0F;
            target_speed[m] = 0;
            speed_output[m] = pid_calculate(
                &self->pid[LIFT_CONTROL_LOOP_SPEED][m], 0.0F,
                self->speed_filtered[m], self->config.period_s);
            value = control_f32_to_i32_sat(speed_output[m]);
        }
        else
        {
            const int32_t approach_vmax =
                control_approach_vmax(&self->config, error, deadband);

            position_output[m] = pid_calculate(
                &self->pid[LIFT_CONTROL_LOOP_POSITION][m],
                (float)ramped[m], (float)actual[m], self->config.period_s);
            target_speed[m] = control_f32_to_i32_sat(position_output[m]);

            /* 减速区：只压上限，而且**只压到非零下限**（收到 0 会卡在死区外） */
            if (approach_vmax >= 0)
            {
                if (target_speed[m] > approach_vmax)
                {
                    target_speed[m] = approach_vmax;
                }
                else if (target_speed[m] < -approach_vmax)
                {
                    target_speed[m] = -approach_vmax;
                }
            }

            /*
             * ★ 跟踪前馈：目标以速率 R 走时，理想速度指令就是 R。
             * 不给它，位置环必须靠"攒出 R/kp 的误差"才能命令出 R —— 那就是
             * 跟踪滞后，误差一直存在、积分一直往外顶。
             * 放在减速收窄【之后】，所以前馈不会被收窄砍掉（与台架同序）。
             */
            target_speed[m] += ramp_rate[m];

            speed_output[m] = pid_calculate(
                &self->pid[LIFT_CONTROL_LOOP_SPEED][m],
                (float)target_speed[m], self->speed_filtered[m],
                self->config.period_s);
            value = control_f32_to_i32_sat(speed_output[m]);
        }

        pid_effort[m] = value;
    }

    /* 交给模态限幅的"控制量"（前馈已含在内，与限幅比例一起算） */
    {
        lift_sync_effort_t combined;

        combined.height = pid_effort[MODE_HEIGHT];
        combined.roll = pid_effort[MODE_ROLL];
        combined.pitch = pid_effort[MODE_PITCH];
        limit_scale = control_limit_scale(
            &self->config, position, &combined, &feedforward);
    }

    /*
     * ---- ⑤ 模态限幅 + 逐级开放限幅 + 电流斜坡 ----
     *
     * 模态限幅只压 PID 那一部分（前馈是静态保持力，压掉车会掉）；
     * 逐级开放与硬上限压在【合成之后】的量上，所以保持电流也受它管；
     * 电流斜坡最后做，保证任何情况下进 CAN 的 di/dt 都是受控的。
     */
    {
        const float full_limit =
            self->config.pid[LIFT_CONTROL_LOOP_SPEED].output_limit;
        int32_t value;

        value = control_f32_to_i32_sat(
            (float)pid_effort[MODE_HEIGHT] * limit_scale +
            (float)feedforward.height);
        value = control_clamp_i32(value, limit_now[MODE_HEIGHT], full_limit);
        effort->height = control_ramp_current(
            self, MODE_HEIGHT, value, self->config.period_s);

        value = control_f32_to_i32_sat(
            (float)pid_effort[MODE_ROLL] * limit_scale);
        value = control_clamp_i32(value, limit_now[MODE_ROLL], full_limit);
        effort->roll = control_ramp_current(
            self, MODE_ROLL, value, self->config.period_s);

        value = control_f32_to_i32_sat(
            (float)pid_effort[MODE_PITCH] * limit_scale);
        value = control_clamp_i32(value, limit_now[MODE_PITCH], full_limit);
        effort->pitch = control_ramp_current(
            self, MODE_PITCH, value, self->config.period_s);
    }

    status->limit_scale = limit_scale;
    status->limit_active = (limit_scale < 0.999F);

    status->ramped_height_ecd = self->ramped_height_ecd;
    status->ramped_roll_ecd = self->ramped_roll_ecd;
    status->ramped_pitch_ecd = self->ramped_pitch_ecd;
    status->target_speed_height = target_speed[LIFT_CONTROL_MODE_HEIGHT];
    status->target_speed_roll = target_speed[LIFT_CONTROL_MODE_ROLL];
    status->target_speed_pitch = target_speed[LIFT_CONTROL_MODE_PITCH];
    status->error_height_ecd = command->target_height_ecd - modes.height;
    status->error_roll_ecd = command->target_roll_ecd - modes.roll;
    status->error_pitch_ecd = command->target_pitch_ecd - modes.pitch;
    for (m = 0U; m < (uint32_t)LIFT_CONTROL_MODE_COUNT; m++)
    {
        status->pid_output[LIFT_CONTROL_LOOP_POSITION][m] = position_output[m];
        status->pid_output[LIFT_CONTROL_LOOP_SPEED][m] = speed_output[m];
        status->hold_state[m] = self->hold_state[m];
        status->limit_open_current[m] = limit_now[m];
    }

    status->at_target =
        (control_abs_i32(status->error_height_ecd) <=
         self->config.settle_tolerance_ecd) &&
        (control_abs_i32(status->error_roll_ecd) <=
         self->config.settle_tolerance_ecd) &&
        (control_abs_i32(status->error_pitch_ecd) <=
         self->config.settle_tolerance_ecd);
}
