#include "lift_seq.h"

#include <stddef.h>

/** 进入某一步：重置计时，并锁定当前姿态作为 WAIT 步骤的保持目标。 */
static void seq_enter_step(
    lift_seq_t *self,
    const lift_seq_input_t *in,
    uint32_t index)
{
    self->step_index = index;
    self->step_start_ms = in->now_ms;
    self->settle_active = false;
    self->settle_start_ms = 0U;
    self->hold_height_ecd = in->actual_height_ecd;
    self->hold_roll_ecd = in->actual_roll_ecd;
    self->hold_pitch_ecd = in->actual_pitch_ecd;
}

/** 取得某一步应当下发的三个模态目标。 */
static void seq_step_targets(
    const lift_seq_t *self,
    const lift_seq_step_t *step,
    int32_t *height,
    int32_t *roll,
    int32_t *pitch)
{
    if (step->kind == LIFT_SEQ_STEP_WAIT)
    {
        *height = self->hold_height_ecd;
        *roll = self->hold_roll_ecd;
        *pitch = self->hold_pitch_ecd;
    }
    else
    {
        *height = step->height_ecd;
        *roll = step->roll_ecd;
        *pitch = step->pitch_ecd;
    }
}

void lift_seq_init(lift_seq_t *self, const lift_seq_program_t *program)
{
    if ((self == NULL) || (program == NULL))
    {
        return;
    }

    *self = (lift_seq_t){0};
    self->program = *program;
    self->state = LIFT_SEQ_STATE_IDLE;
    self->fault = LIFT_SEQ_FAULT_NONE;
}

void lift_seq_reset(lift_seq_t *self)
{
    if (self == NULL)
    {
        return;
    }

    self->state = LIFT_SEQ_STATE_IDLE;
    self->fault = LIFT_SEQ_FAULT_NONE;
    self->step_index = 0U;
    self->settle_active = false;
    self->settle_start_ms = 0U;
}

bool lift_seq_is_active(const lift_seq_t *self)
{
    if (self == NULL)
    {
        return false;
    }
    return (self->state == LIFT_SEQ_STATE_RUNNING) ||
           (self->state == LIFT_SEQ_STATE_WAIT_CONFIRM);
}

void lift_seq_update(
    lift_seq_t *self,
    const lift_seq_input_t *in,
    lift_seq_status_t *status)
{
    bool start_rising;
    bool abort_rising;
    bool confirm_rising;
    bool advance = false;
    int32_t tgt_h = 0;
    int32_t tgt_r = 0;
    int32_t tgt_p = 0;
    const lift_seq_step_t *step;

    if ((self == NULL) || (in == NULL) || (status == NULL))
    {
        return;
    }

    status->command.enabled = false;
    status->command.target_height_ecd = 0;
    status->command.target_roll_ecd = 0;
    status->command.target_pitch_ecd = 0;
    status->command_valid = false;
    status->step_elapsed_ms = 0U;
    status->step_count = self->program.count;

    start_rising = in->start_request && !self->start_last;
    abort_rising = in->abort_request && !self->abort_last;
    confirm_rising = in->confirm_request && !self->confirm_last;
    self->start_last = in->start_request;
    self->abort_last = in->abort_request;
    self->confirm_last = in->confirm_request;

    /* 故障锁存：只有 lift_seq_reset() 能解除。 */
    if (self->state == LIFT_SEQ_STATE_FAULT)
    {
        goto emit;
    }

    if (abort_rising && lift_seq_is_active(self))
    {
        self->state = LIFT_SEQ_STATE_FAULT;
        self->fault = LIFT_SEQ_FAULT_ABORTED;
        goto emit;
    }

    if (start_rising)
    {
        if (!in->homed)
        {
            self->state = LIFT_SEQ_STATE_FAULT;
            self->fault = LIFT_SEQ_FAULT_NOT_HOMED;
            goto emit;
        }
        if ((self->program.steps == NULL) || (self->program.count == 0U))
        {
            self->state = LIFT_SEQ_STATE_DONE;
            goto emit;
        }

        seq_enter_step(self, in, 0U);
        self->state = LIFT_SEQ_STATE_RUNNING;
    }

    if (!lift_seq_is_active(self))
    {
        goto emit;
    }

    step = &self->program.steps[self->step_index];

    switch (step->kind)
    {
        case LIFT_SEQ_STEP_STOP:
            self->state = LIFT_SEQ_STATE_DONE;
            break;

        case LIFT_SEQ_STEP_WAIT:
            /* 进入时已锁定姿态，这里只是保持；只等确认。 */
            if (confirm_rising)
            {
                advance = true;
            }
            break;

        case LIFT_SEQ_STEP_MOVE:
        default:
            if (in->at_target)
            {
                if (!self->settle_active)
                {
                    self->settle_active = true;
                    self->settle_start_ms = in->now_ms;
                }
                else if ((uint32_t)(in->now_ms - self->settle_start_ms) >=
                         step->settle_ms)
                {
                    if (step->require_confirm)
                    {
                        self->state = LIFT_SEQ_STATE_WAIT_CONFIRM;
                    }
                    else
                    {
                        advance = true;
                    }
                }
                else
                {
                    /* 稳定计时中。 */
                }
            }
            else
            {
                self->settle_active = false;
            }
            break;
    }

    /* 已进入 WAIT_CONFIRM 的步骤，等确认沿推进。 */
    if ((self->state == LIFT_SEQ_STATE_WAIT_CONFIRM) && confirm_rising)
    {
        advance = true;
    }

    /* 超时。刚够到目标的周期不判超时（advance 优先）。 */
    if (!advance &&
        (self->state != LIFT_SEQ_STATE_DONE) &&
        (step->kind != LIFT_SEQ_STEP_STOP) &&
        (step->timeout_ms != 0U) &&
        ((uint32_t)(in->now_ms - self->step_start_ms) >= step->timeout_ms))
    {
        self->state = LIFT_SEQ_STATE_FAULT;
        self->fault = LIFT_SEQ_FAULT_STEP_TIMEOUT;
        goto emit;
    }

    if (advance && lift_seq_is_active(self))
    {
        const uint32_t next = self->step_index + 1U;

        if (next >= self->program.count)
        {
            self->state = LIFT_SEQ_STATE_DONE;
        }
        else
        {
            const lift_seq_step_t *next_step;

            seq_enter_step(self, in, next);
            self->state = LIFT_SEQ_STATE_RUNNING;
            next_step = &self->program.steps[next];

            if (next_step->kind == LIFT_SEQ_STEP_STOP)
            {
                self->state = LIFT_SEQ_STATE_DONE;
            }
            else
            {
                /* 立刻产出新步骤目标，避免输出一周期旧目标造成抖动。 */
                seq_step_targets(self, next_step, &tgt_h, &tgt_r, &tgt_p);
            }
        }
    }

    if (lift_seq_is_active(self))
    {
        if (!advance)
        {
            seq_step_targets(self, step, &tgt_h, &tgt_r, &tgt_p);
        }
        status->command.enabled = true;
        status->command.target_height_ecd = tgt_h;
        status->command.target_roll_ecd = tgt_r;
        status->command.target_pitch_ecd = tgt_p;
        status->command_valid = true;
        status->step_elapsed_ms = (uint32_t)(in->now_ms - self->step_start_ms);
    }

emit:
    status->state = self->state;
    status->fault = self->fault;
    status->step_index = self->step_index;
}
