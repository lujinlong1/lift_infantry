#include "lift_control.h"

#include <stddef.h>
#ifdef LIFT_LIMIT_TRACE
#include <stdio.h>
#endif

/* speed_rpm -> ecd/s 与 rpm -> deg/s 共用的换算常数：360 / 60。 */
#define CONTROL_RPM_TO_DEG_S 6.0F

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
        /* 配置填的是"保持电流大小"，取绝对值；方向由模块内部固定为向下。 */
        self->config.gravity_feedforward_current =
            -self->config.gravity_feedforward_current;
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
        if (self->config.limit_roll_max_ecd <= self->config.limit_roll_min_ecd)
        {
            self->config.limit_roll_min_ecd = -wide;
            self->config.limit_roll_max_ecd = wide;
        }
        if (self->config.limit_pitch_max_ecd <= self->config.limit_pitch_min_ecd)
        {
            self->config.limit_pitch_min_ecd = -wide;
            self->config.limit_pitch_max_ecd = wide;
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
    uint32_t m;

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
    }
    status->target_speed_height = 0;
    status->target_speed_roll = 0;
    status->target_speed_pitch = 0;
    status->limit_scale = 1.0F;
    status->limit_active = false;
    status->at_target = false;

    /*
     * 重力前馈：丝杆不自锁，停在行程中间时托住车身的力【只能】由电机出。
     * 配置值填的是"保持电流大小"（正数），这里按【向下】施加。
     *
     * 前馈刻意只加负向（向下）：
     *   - 车在行程中间，它抵消重力，让位置环只纠残余误差；
     *   - 车压在下限时，重力本来由挡块承担，再给向上的力只会白烧电机。
     *
     * 若不给这份前馈，保持电流就只能靠位置环积分去凑，而"要出力就必须有
     * 误差"，于是位置永远收敛不到目标——仿真里表现为目标 20000、
     * 稳态停在 18974（差 5.1%），三个模态看起来却都"收敛了"。
     */
    feedforward.height = -(int32_t)self->config.gravity_feedforward_current;
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

    /* 首次进入使能：斜坡吸附到当前位置，PID 从零积分起步。 */
    if (!self->ramp_initialized)
    {
        self->ramped_height_ecd = modes.height;
        self->ramped_roll_ecd = modes.roll;
        self->ramped_pitch_ecd = modes.pitch;
        self->ramp_initialized = true;
        control_reset_all_pid(self);
    }

    self->ramped_height_ecd = control_ramp_toward(
        self->ramped_height_ecd, command->target_height_ecd,
        self->config.max_target_step_ecd);
    self->ramped_roll_ecd = control_ramp_toward(
        self->ramped_roll_ecd, command->target_roll_ecd,
        self->config.max_target_step_ecd);
    self->ramped_pitch_ecd = control_ramp_toward(
        self->ramped_pitch_ecd, command->target_pitch_ecd,
        self->config.max_target_step_ecd);

    /* 外环：模态位置误差 -> 目标模态速度。 */
    position_output[LIFT_CONTROL_MODE_HEIGHT] = pid_calculate(
        &self->pid[LIFT_CONTROL_LOOP_POSITION][LIFT_CONTROL_MODE_HEIGHT],
        (float)self->ramped_height_ecd, (float)modes.height,
        self->config.period_s);
    position_output[LIFT_CONTROL_MODE_ROLL] = pid_calculate(
        &self->pid[LIFT_CONTROL_LOOP_POSITION][LIFT_CONTROL_MODE_ROLL],
        (float)self->ramped_roll_ecd, (float)modes.roll,
        self->config.period_s);
    position_output[LIFT_CONTROL_MODE_PITCH] = pid_calculate(
        &self->pid[LIFT_CONTROL_LOOP_POSITION][LIFT_CONTROL_MODE_PITCH],
        (float)self->ramped_pitch_ecd, (float)modes.pitch,
        self->config.period_s);

    target_speed[LIFT_CONTROL_MODE_HEIGHT] =
        control_f32_to_i32_sat(position_output[LIFT_CONTROL_MODE_HEIGHT]);
    target_speed[LIFT_CONTROL_MODE_ROLL] =
        control_f32_to_i32_sat(position_output[LIFT_CONTROL_MODE_ROLL]);
    target_speed[LIFT_CONTROL_MODE_PITCH] =
        control_f32_to_i32_sat(position_output[LIFT_CONTROL_MODE_PITCH]);

    /* 内环：模态速度误差 -> 模态控制量。 */
    speed_output[LIFT_CONTROL_MODE_HEIGHT] = pid_calculate(
        &self->pid[LIFT_CONTROL_LOOP_SPEED][LIFT_CONTROL_MODE_HEIGHT],
        (float)target_speed[LIFT_CONTROL_MODE_HEIGHT],
        (float)modes_speed.height,
        self->config.period_s);
    speed_output[LIFT_CONTROL_MODE_ROLL] = pid_calculate(
        &self->pid[LIFT_CONTROL_LOOP_SPEED][LIFT_CONTROL_MODE_ROLL],
        (float)target_speed[LIFT_CONTROL_MODE_ROLL],
        (float)modes_speed.roll,
        self->config.period_s);
    speed_output[LIFT_CONTROL_MODE_PITCH] = pid_calculate(
        &self->pid[LIFT_CONTROL_LOOP_SPEED][LIFT_CONTROL_MODE_PITCH],
        (float)target_speed[LIFT_CONTROL_MODE_PITCH],
        (float)modes_speed.pitch,
        self->config.period_s);

    effort->height = control_f32_to_i32_sat(speed_output[LIFT_CONTROL_MODE_HEIGHT]);
    effort->roll = control_f32_to_i32_sat(speed_output[LIFT_CONTROL_MODE_ROLL]);
    effort->pitch = control_f32_to_i32_sat(speed_output[LIFT_CONTROL_MODE_PITCH]);

    /*
     * ---- 模态限幅 ----
     *
     * 把前馈并入之后再统一算比例 s，使得"控制量 + 前馈"这个实际下发的量
     * 不会让任何一根轴越过行程。前馈也参与限幅，否则它自己就能把轴推过界。
     *
     * 注意 s 只压【向上】的合成量；向下的前馈（托住车身）在到界时本来就
     * 不该再往上了，被压到 0 是正确行为。
     */
    {
        const float limit_scale = control_limit_scale(
            &self->config, position, effort, &feedforward);

        effort->height = control_f32_to_i32_sat(
            (float)effort->height * limit_scale + (float)feedforward.height);
        effort->roll = control_f32_to_i32_sat(
            (float)effort->roll * limit_scale + (float)feedforward.roll);
        effort->pitch = control_f32_to_i32_sat(
            (float)effort->pitch * limit_scale + (float)feedforward.pitch);

        status->limit_scale = limit_scale;
        status->limit_active = (limit_scale < 0.999F);
    }

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
    }

    status->at_target =
        (control_abs_i32(status->error_height_ecd) <=
         self->config.settle_tolerance_ecd) &&
        (control_abs_i32(status->error_roll_ecd) <=
         self->config.settle_tolerance_ecd) &&
        (control_abs_i32(status->error_pitch_ecd) <=
         self->config.settle_tolerance_ecd);
}
