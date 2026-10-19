#include "lift_guard.h"

#include <stddef.h>

/** 取 32 位绝对值，用于转速阈值比较（避免 int16 取负溢出）。 */
static int32_t guard_abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

/** 返回 -1 / 0 / +1。 */
static int guard_sign_i16(int16_t value)
{
    if (value > 0)
    {
        return 1;
    }
    if (value < 0)
    {
        return -1;
    }
    return 0;
}

/** 限幅到 ±limit；limit 为负按 0 处理。 */
static int16_t guard_clamp_i16(int16_t value, int16_t limit)
{
    if (limit < 0)
    {
        limit = 0;
    }
    if (value > limit)
    {
        return limit;
    }
    if (value < (int16_t)(-limit))
    {
        return (int16_t)(-limit);
    }
    return value;
}

/** 清除全部缓降堵转计时。 */
static void guard_clear_stall(lift_guard_t *self)
{
    uint32_t ch;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        self->stall_active[ch] = false;
        self->stall_start_ms[ch] = 0U;
    }
}

/** 进入故障态：锁存故障码并停止全部输出。 */
static void guard_set_fault(lift_guard_t *self, lift_guard_fault_t fault, int8_t axis)
{
    self->state = LIFT_GUARD_STATE_FAULT;
    self->fault = fault;
    self->fault_axis = axis;
    guard_clear_stall(self);
}

/** 计算某通道的移动方向：+1 使 position 增大，-1 减小，0 不动。 */
static int guard_move_sign(const lift_guard_config_t *config, uint32_t ch, int16_t current)
{
    return (int)config->direction[ch] * guard_sign_i16(current);
}

void lift_guard_init(lift_guard_t *self, const lift_guard_config_t *config)
{
    uint32_t ch;

    if ((self == NULL) || (config == NULL))
    {
        return;
    }

    *self = (lift_guard_t){0};
    self->config = *config;

    if (self->config.current_limit < 0)
    {
        self->config.current_limit = 0;
    }
    if (self->config.descent_stall_speed_rpm < 0)
    {
        self->config.descent_stall_speed_rpm =
            (int16_t)(-self->config.descent_stall_speed_rpm);
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        /* 方向只允许 ±1；未配置时默认 +1，避免限位判断失效。 */
        if (self->config.direction[ch] > 0)
        {
            self->config.direction[ch] = 1;
        }
        else if (self->config.direction[ch] < 0)
        {
            self->config.direction[ch] = -1;
        }
        else
        {
            self->config.direction[ch] = 1;
        }

        /* 缓降电流必须落在本模块硬限幅内。 */
        self->config.descent_current[ch] =
            guard_clamp_i16(self->config.descent_current[ch], self->config.current_limit);

        /* 停放点必须落在行程区间内，否则缓降会一直走不到目标。 */
        if (self->config.descent_target_ecd[ch] <
            self->config.position_min_ecd[ch])
        {
            self->config.descent_target_ecd[ch] =
                self->config.position_min_ecd[ch];
        }
        if (self->config.descent_target_ecd[ch] >
            self->config.position_max_ecd[ch])
        {
            self->config.descent_target_ecd[ch] =
                self->config.position_max_ecd[ch];
        }

        /* 限位上下界写反时自动纠正，避免把机构锁死。 */
        if (self->config.position_min_ecd[ch] > self->config.position_max_ecd[ch])
        {
            const int32_t swap = self->config.position_min_ecd[ch];
            self->config.position_min_ecd[ch] = self->config.position_max_ecd[ch];
            self->config.position_max_ecd[ch] = swap;
        }
    }

    self->state = LIFT_GUARD_STATE_DISARMED;
    self->fault = LIFT_GUARD_FAULT_NONE;
    self->fault_axis = -1;
}

void lift_guard_update(
    lift_guard_t *self,
    const lift_guard_input_t *in,
    lift_guard_output_t *out)
{
    uint32_t ch;
    bool reset_rising;

    if ((self == NULL) || (in == NULL) || (out == NULL))
    {
        return;
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        out->output_current[ch] = 0;
        out->limit_blocked[ch] = false;
        out->at_target[ch] = false;
    }
    out->descent_elapsed_ms = 0U;

    reset_rising = in->reset_request && !self->reset_last;
    self->reset_last = in->reset_request;

    if (reset_rising)
    {
        self->fault = LIFT_GUARD_FAULT_NONE;
        self->fault_axis = -1;
        self->state = in->homed
            ? LIFT_GUARD_STATE_READY
            : LIFT_GUARD_STATE_DISARMED;
        self->descent_start_ms = 0U;
        guard_clear_stall(self);
    }

    /* 故障锁存：只有 reset 上升沿能解除。 */
    if (self->state == LIFT_GUARD_STATE_FAULT)
    {
        goto emit;
    }

    /* 优先级 1：离线保护。软件兜不住断电，但兜得住反馈丢失。 */
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        if (!in->online[ch])
        {
            guard_set_fault(self, LIFT_GUARD_FAULT_OFFLINE, (int8_t)ch);
            goto emit;
        }
    }

    /* 优先级 2：找零门禁。没找零就没有位置基准，任何运动都不得通过。 */
    if (!in->homed)
    {
        self->state = LIFT_GUARD_STATE_DISARMED;
        goto emit;
    }

    /* 优先级 3：受控缓降。这是 ADR-006 的核心——超时不归零，主动往下走。 */
    if (!in->command_fresh)
    {
        bool all_at_target = true;

        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            /* 目标是"停放点"，不是行程下限：下限可能配在机械死点之下。 */
            const int32_t target = self->config.descent_target_ecd[ch];
            const int move =
                guard_move_sign(&self->config, ch, self->config.descent_current[ch]);
            bool reached;

            if (move > 0)
            {
                reached = in->position_ecd[ch] >= target;
            }
            else if (move < 0)
            {
                reached = in->position_ecd[ch] <= target;
            }
            else
            {
                /* 未配置缓降电流，视为已到位，不做无意义输出。 */
                reached = true;
            }

            out->at_target[ch] = reached;

            if (reached)
            {
                self->stall_active[ch] = false;
                continue;
            }

            all_at_target = false;
            out->output_current[ch] =
                guard_clamp_i16(self->config.descent_current[ch],
                                self->config.current_limit);

            /* 该下降却不动，说明卡死，必须报故障而不是继续硬顶。 */
            if (guard_abs_i32((int32_t)in->speed_rpm[ch]) <=
                (int32_t)self->config.descent_stall_speed_rpm)
            {
                if (!self->stall_active[ch])
                {
                    self->stall_active[ch] = true;
                    self->stall_start_ms[ch] = in->now_ms;
                }
                else if ((uint32_t)(in->now_ms - self->stall_start_ms[ch]) >=
                         self->config.descent_stall_ms)
                {
                    guard_set_fault(self, LIFT_GUARD_FAULT_DESCENT_STALL, (int8_t)ch);
                    goto emit;
                }
                else
                {
                    /* 计时中。 */
                }
            }
            else
            {
                self->stall_active[ch] = false;
            }
        }

        if (all_at_target)
        {
            self->state = LIFT_GUARD_STATE_PARKED;
            self->descent_start_ms = 0U;
            out->descent_elapsed_ms = 0U;
        }
        else
        {
            if (self->state != LIFT_GUARD_STATE_DESCENDING)
            {
                self->state = LIFT_GUARD_STATE_DESCENDING;
                self->descent_start_ms = in->now_ms;
            }
            out->descent_elapsed_ms =
                (uint32_t)(in->now_ms - self->descent_start_ms);

            if (out->descent_elapsed_ms >= self->config.descent_timeout_ms)
            {
                guard_set_fault(self, LIFT_GUARD_FAULT_DESCENT_TIMEOUT, -1);
                goto emit;
            }
        }

        goto emit;
    }

    /* 优先级 4：正常通路 + 行程软限位（兜底）。 */
    self->state = LIFT_GUARD_STATE_READY;
    self->descent_start_ms = 0U;
    guard_clear_stall(self);

    /*
     * ---- 行程软限位：分工说明（重要）--------------------------------
     *
     * 限位的【正常工作】不在本模块，而在 lift_control 的模态限幅里。
     *
     * 原因：四根丝杆顶的是同一块刚性车身，任意一根轴上的电流都是
     * 「高度 + 横滚 + 俯仰」三个分量之和，其中一部分是【在帮别的轴调平】。
     * 逐轴判决只看得见"这根轴想去哪"，看不见"它还在替谁出力"。若把逐轴
     * 清零当常规手段，一旦某轴触界就把它从协同里踢出去，车身再也拉不平，
     * 而且会反复触发（车一歪就再踢一次）。
     *
     * 所以正常路径是：lift_control 提前算出统一比例，把"会让某轴越界"的
     * 那部分模态出力压下去，而保留其余模态对它的贡献。
     *
     * 本模块保留下面这段，只是【兜底】：万一上游没接模态限幅、或参数配错，
     * 这里必须挡住。此时唯一的正确动作就是把该轴电流清零——到了界外还在
     * 往外推，任何"温和修正"都只会让电机堵转发热。
     *
     * 换句话说：这里的清零不是用来协作的，是用来止损的。
     * 正常运行时 limit_blocked 应当【始终为 false】；一旦它为 true，
     * 说明上游限幅没生效，应该去查 lift_control 的 limit_* 配置，
     * 而不是去调 guard。
     */
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        int16_t current =
            guard_clamp_i16(in->requested_current[ch], self->config.current_limit);
        const int move = guard_move_sign(&self->config, ch, current);

        if ((move > 0) && (in->position_ecd[ch] >= self->config.position_max_ecd[ch]))
        {
            current = 0;
            out->limit_blocked[ch] = true;
        }
        else if ((move < 0) &&
                 (in->position_ecd[ch] <= self->config.position_min_ecd[ch]))
        {
            current = 0;
            out->limit_blocked[ch] = true;
        }
        else
        {
            /* 未触界，放行。 */
        }

        out->output_current[ch] = current;
    }

emit:
    out->state = self->state;
    out->fault = self->fault;
    out->fault_axis = self->fault_axis;

    if (self->state == LIFT_GUARD_STATE_FAULT)
    {
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            out->output_current[ch] = 0;
            out->limit_blocked[ch] = false;
        }
        out->descent_elapsed_ms = 0U;
    }
}

bool lift_guard_has_fault(const lift_guard_t *self)
{
    if (self == NULL)
    {
        return false;
    }
    return self->state == LIFT_GUARD_STATE_FAULT;
}
