#include "lift_homing.h"

#include <stddef.h>

/** 取 32 位有符号数绝对值，供行程比较使用。 */
static int32_t lift_abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

/** 将电流限幅到 ±limit；limit 为负时按 0 处理。 */
static int16_t lift_clamp_i16(int16_t value, int16_t limit)
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

/** 进入故障态：记录故障码并停止输出。 */
static void lift_homing_set_fault(lift_homing_t *self, lift_homing_fault_t fault)
{
    self->state = LIFT_HOMING_STATE_FAULT;
    self->fault = fault;
    self->stall_active = false;
}

void lift_homing_init(lift_homing_t *self, const lift_homing_config_t *config)
{
    if ((self == NULL) || (config == NULL))
    {
        return;
    }

    *self = (lift_homing_t){0};
    self->config = *config;

    /* 配置自检：把三个工作电流收进硬限幅，防止配置写错变成大电流硬顶。 */
    self->config.seek_current =
        lift_clamp_i16(self->config.seek_current, self->config.current_limit);
    self->config.backoff_current =
        lift_clamp_i16(self->config.backoff_current, self->config.current_limit);
    self->config.preposition_current = lift_clamp_i16(
        self->config.preposition_current, self->config.current_limit);

    if (self->config.stall_speed_rpm < 0)
    {
        self->config.stall_speed_rpm = (int16_t)(-self->config.stall_speed_rpm);
    }
    if (self->config.seek_min_travel_ecd < 0)
    {
        self->config.seek_min_travel_ecd = -self->config.seek_min_travel_ecd;
    }
    if (self->config.backoff_ecd < 0)
    {
        self->config.backoff_ecd = -self->config.backoff_ecd;
    }
    if (self->config.preposition_ecd < 0)
    {
        self->config.preposition_ecd = -self->config.preposition_ecd;
    }

    self->state = LIFT_HOMING_STATE_IDLE;
    self->fault = LIFT_HOMING_FAULT_NONE;
}

void lift_homing_update(
    lift_homing_t *self,
    const lift_homing_input_t *in,
    lift_homing_output_t *out)
{
    bool start_rising;
    bool abort_rising;
    int32_t travel;

    if ((self == NULL) || (in == NULL) || (out == NULL))
    {
        return;
    }

    start_rising = in->start_request && !self->start_last;
    abort_rising = in->abort_request && !self->abort_last;
    self->start_last = in->start_request;
    self->abort_last = in->abort_request;

    /* 中止优先于启动：先处理 abort，再允许同周期的 start 重新开始。 */
    if (abort_rising &&
        ((self->state == LIFT_HOMING_STATE_PREPOSITION) ||
         (self->state == LIFT_HOMING_STATE_SEEK) ||
         (self->state == LIFT_HOMING_STATE_BACKOFF)))
    {
        lift_homing_set_fault(self, LIFT_HOMING_FAULT_ABORTED);
    }

    if (start_rising)
    {
        self->state_start_ms = in->now_ms;
        self->stall_active = false;
        self->zero_ecd = 0;
        self->fault = LIFT_HOMING_FAULT_NONE;

        if (!in->online)
        {
            lift_homing_set_fault(self, LIFT_HOMING_FAULT_OFFLINE);
        }
        else if (self->config.preposition_ecd > 0)
        {
            /* 先离开死点，保证 SEEK 的最小行程条件一定成立。 */
            self->state = LIFT_HOMING_STATE_PREPOSITION;
            self->start_position_ecd = in->position_ecd;
        }
        else
        {
            self->state = LIFT_HOMING_STATE_SEEK;
            self->start_position_ecd = in->position_ecd;
        }
    }

    switch (self->state)
    {
        case LIFT_HOMING_STATE_PREPOSITION:
            if (!in->online)
            {
                lift_homing_set_fault(self, LIFT_HOMING_FAULT_OFFLINE);
                break;
            }

            travel = in->position_ecd - self->start_position_ecd;
            if (lift_abs_i32(travel) >= self->config.preposition_ecd)
            {
                /*
                 * 已经离开死点足够远。立刻转入 SEEK，并以【这里】作为
                 * 行程基准——于是 SEEK 的最小行程条件天然成立。
                 */
                self->state = LIFT_HOMING_STATE_SEEK;
                self->start_position_ecd = in->position_ecd;
                self->state_start_ms = in->now_ms;
                self->stall_active = false;
            }
            else if ((uint32_t)(in->now_ms - self->state_start_ms) >=
                     self->config.preposition_timeout_ms)
            {
                /*
                 * 前置段走不到位：顶住了、卡住了，或者 preposition_current
                 * 太小推不动。这跟"找不到死点"是两回事，单独报码，
                 * 现场才分得清该查哪一边。
                 */
                lift_homing_set_fault(
                    self, LIFT_HOMING_FAULT_PREPOSITION_TIMEOUT);
            }
            else
            {
                /* 继续离开死点。 */
            }
            break;

        case LIFT_HOMING_STATE_SEEK:
            if (!in->online)
            {
                lift_homing_set_fault(self, LIFT_HOMING_FAULT_OFFLINE);
                break;
            }

            travel = in->position_ecd - self->start_position_ecd;

            /* 条件①：先走够最小行程，才允许进入堵转判定。 */
            if (lift_abs_i32(travel) >= self->config.seek_min_travel_ecd)
            {
                /* 条件②：速度足够小。 */
                if (lift_abs_i32((int32_t)in->speed_rpm) <=
                    (int32_t)self->config.stall_speed_rpm)
                {
                    if (!self->stall_active)
                    {
                        self->stall_active = true;
                        self->stall_start_ms = in->now_ms;
                    }
                    /* 条件③：连续保持够久，才确认到达死点。 */
                    else if ((uint32_t)(in->now_ms - self->stall_start_ms) >=
                             self->config.stall_hold_ms)
                    {
                        self->zero_ecd = in->position_ecd;
                        self->backoff_start_ecd = in->position_ecd;
                        self->state = LIFT_HOMING_STATE_BACKOFF;
                        self->state_start_ms = in->now_ms;
                        self->stall_active = false;
                    }
                    else
                    {
                        /* 还在累计中。 */
                    }
                }
                else
                {
                    self->stall_active = false;
                }
            }
            else
            {
                self->stall_active = false;
            }

            /* 只有仍在 SEEK 才判超时，避免刚转入 BACKOFF 就被误判。 */
            if ((self->state == LIFT_HOMING_STATE_SEEK) &&
                ((uint32_t)(in->now_ms - self->state_start_ms) >=
                 self->config.seek_timeout_ms))
            {
                lift_homing_set_fault(self, LIFT_HOMING_FAULT_SEEK_TIMEOUT);
            }
            break;

        case LIFT_HOMING_STATE_BACKOFF:
            if (!in->online)
            {
                lift_homing_set_fault(self, LIFT_HOMING_FAULT_OFFLINE);
                break;
            }

            travel = in->position_ecd - self->backoff_start_ecd;
            if (lift_abs_i32(travel) >= self->config.backoff_ecd)
            {
                self->state = LIFT_HOMING_STATE_DONE;
            }
            else if ((uint32_t)(in->now_ms - self->state_start_ms) >=
                     self->config.backoff_timeout_ms)
            {
                /* 碰到了死点却退不出来，说明顶死或卡死，必须报故障。 */
                lift_homing_set_fault(self, LIFT_HOMING_FAULT_BACKOFF_TIMEOUT);
            }
            else
            {
                /* 继续回退。 */
            }
            break;

        case LIFT_HOMING_STATE_IDLE:
        case LIFT_HOMING_STATE_DONE:
        case LIFT_HOMING_STATE_FAULT:
        default:
            break;
    }

    /* 组装输出。非工作状态一律输出 0 电流。 */
    out->state = self->state;
    out->fault = self->fault;
    out->zero_ecd = self->zero_ecd;
    out->stall_ms = self->stall_active
        ? (uint32_t)(in->now_ms - self->stall_start_ms)
        : 0U;

    switch (self->state)
    {
        case LIFT_HOMING_STATE_PREPOSITION:
            out->commanded_current = self->config.preposition_current;
            out->travel_ecd = in->position_ecd - self->start_position_ecd;
            out->elapsed_ms = (uint32_t)(in->now_ms - self->state_start_ms);
            break;

        case LIFT_HOMING_STATE_SEEK:
            out->commanded_current = self->config.seek_current;
            out->travel_ecd = in->position_ecd - self->start_position_ecd;
            out->elapsed_ms = (uint32_t)(in->now_ms - self->state_start_ms);
            break;

        case LIFT_HOMING_STATE_BACKOFF:
            out->commanded_current = self->config.backoff_current;
            out->travel_ecd = in->position_ecd - self->backoff_start_ecd;
            out->elapsed_ms = (uint32_t)(in->now_ms - self->state_start_ms);
            break;

        default:
            out->commanded_current = 0;
            out->travel_ecd = 0;
            out->elapsed_ms = 0U;
            break;
    }
}

void lift_homing_force_done(lift_homing_t *self, int32_t logical_position_ecd)
{
    if (self == NULL)
    {
        return;
    }

    self->state = LIFT_HOMING_STATE_DONE;
    self->fault = LIFT_HOMING_FAULT_NONE;
    self->stall_active = false;
    self->zero_ecd = logical_position_ecd;
    self->start_position_ecd = logical_position_ecd;
    self->backoff_start_ecd = logical_position_ecd;
}

bool lift_homing_is_homed(const lift_homing_t *self)
{
    if (self == NULL)
    {
        return false;
    }
    return self->state == LIFT_HOMING_STATE_DONE;
}
