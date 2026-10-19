#include "lift_seq.h"

#include <stddef.h>
#ifdef LIFT_SEQ_TRACE
#include <stdio.h>
#endif

/*
 * 上台阶流程状态机。设计理由、机械前提与能力缺口全部写在 lift_seq.h 的文件头，
 * 这里只留实现相关的注释。
 *
 * 判据设计一句话总结：
 *   每个 MOVE 阶段要求 "目标组都在容差内 且 都够慢 且 稳定够久"；
 *   每个 VERIFY_*_SUPPORT 阶段要求 "人工确认（或接触信号）+ 电气必要条件"；
 *   每个阶段都有独立超时与独立故障明细，现场能一眼看出卡在哪一步、为什么。
 */

/* 全部通道掩码。 */
#define SEQ_ALL_MASK ((uint8_t)((1U << LIFT_CHANNEL_COUNT) - 1U))

/*
 * 前置检查的稳定窗口（ms）。见 PRECHECK 分支里的说明：
 * 它保证"检查过"这件事不是"同一周期里看了一眼就动"。
 */
#define SEQ_PRECHECK_SETTLE_MS 100U

/*
 * 前部"找接触"的起步高度 = clearance − 这个量。
 *
 * ⚠️ 它必须由 `clearance_mm` 派生，不能写成一个固定毫米数：
 *    固定值在"目标高度改成 150 mm"时会掉到收回位以下，config 校验会直接
 *    判非法，而现场看到的是"配置莫名其妙不生效"。
 */
#define SEQ_CONTACT_BACKOFF_MM 20

/*
 * "姿态超限但已经在位"需要持续多久才判故障（ms）。
 *
 * 取 500 ms：它必须明显长于一次姿态收敛（正常收敛在 100~200 ms 量级），
 * 否则会把"正在回正"误判成"回不来了"。见 seq_tilt_unrecovered()。
 */
#define SEQ_TILT_SETTLE_MS 500U

/* ==================================================================== */
/*  小工具                                                              */
/* ==================================================================== */

static int32_t seq_abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static bool seq_pulse(bool now, bool *last)
{
    const bool rising = now && (!(*last));

    *last = now;
    return rising;
}

static bool seq_in_mask(uint8_t mask, uint32_t ch)
{
    return ((mask >> ch) & 0x01U) != 0U;
}

/** 所有轴统一到一个高度（mm）。 */
static void seq_targets_all(lift_seq_t *self, int32_t mm)
{
    uint32_t ch;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        self->axis_target_mm[ch] = mm;
    }
    self->targets_valid = true;
}

/** 一组动到 move_mm，另一组留在 hold_mm。 */
static void seq_targets_group(
    lift_seq_t *self,
    uint8_t mask,
    int32_t move_mm,
    int32_t hold_mm)
{
    uint32_t ch;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        self->axis_target_mm[ch] = seq_in_mask(mask, ch) ? move_mm : hold_mm;
    }
    self->targets_valid = true;
}

/** 进入某阶段：重置阶段计时、稳定计时、确认锁存与堵转计时。 */
static void seq_enter(lift_seq_t *self, lift_seq_state_t state, uint32_t now_ms)
{
    uint32_t ch;

    self->state = state;
    self->step_start_ms = now_ms;
    self->settle_active = false;
    self->settle_start_ms = 0U;
    /*
     * 把"本周期刚按下的确认"交给新阶段。
     *
     * 注意这里【没有】作废它：发起的阶段（例如 VERIFY_FRONT_SUPPORT）
     * 在调用本函数之前已经声明了"这次按键是给谁的"。没有声明的默认是
     * IDLE，也就等于作废 —— 见 lift_seq_t 里 pending_confirm_target 的说明。
     */
    self->confirm_for = self->pending_confirm_target;
    self->pending_confirm_target = LIFT_SEQ_STATE_IDLE;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        self->stall_active[ch] = false;
        self->stall_start_ms[ch] = 0U;
    }
}

/**
 * @brief 取出一个"属于本阶段"的人工确认。
 *
 * 优先用本拍的上升沿；本拍没有就消费【定向挂起】给本阶段的那一个。
 * 挂起是定向的：只有 confirm_for == 当前阶段时才认，避免一次按键
 * 被下一个毫不相干的阶段顺手消费掉。
 */
static bool seq_take_confirm(lift_seq_t *self, bool confirm_rising)
{
    if (confirm_rising)
    {
        return true;
    }
    if (self->confirm_for == self->state)
    {
        self->confirm_for = LIFT_SEQ_STATE_IDLE;
        return true;
    }
    return false;
}

/** 本阶段超时（ms）。0 = 不限时。 */
static uint32_t seq_stage_timeout(
    const lift_seq_config_t *cfg,
    lift_seq_state_t state)
{
    switch (state)
    {
        case LIFT_SEQ_STATE_PRECHECK:
            return cfg->precheck_timeout_ms;
        case LIFT_SEQ_STATE_LIFT_CLEARANCE:
        case LIFT_SEQ_STATE_TEST_LOWER:
            return cfg->lift_timeout_ms;
        case LIFT_SEQ_STATE_SEEK_FRONT_CONTACT:
        case LIFT_SEQ_STATE_RETRACT_FRONT:
        case LIFT_SEQ_STATE_RETRACT_REAR:
            return cfg->retract_timeout_ms;
        case LIFT_SEQ_STATE_APPROACH_STEP:
        case LIFT_SEQ_STATE_ADVANCE_MIDDLE:
        case LIFT_SEQ_STATE_FINAL_ADVANCE:
            return cfg->advance_timeout_ms;
        case LIFT_SEQ_STATE_VERIFY_FRONT_SUPPORT:
        case LIFT_SEQ_STATE_VERIFY_REAR_SUPPORT:
            /* 在等人按确认，用更长的 support_timeout。 */
            return cfg->support_timeout_ms;
        case LIFT_SEQ_STATE_VERIFY_CLEARANCE:
        case LIFT_SEQ_STATE_VERIFY_FRONT_RETRACT:
        case LIFT_SEQ_STATE_VERIFY_REAR_RETRACT:
            return cfg->verify_timeout_ms;
        default:
            return 0U;
    }
}

/* ==================================================================== */
/*  配置                                                                */
/* ==================================================================== */

bool lift_seq_config_is_valid(const lift_seq_config_t *config)
{
    if (config == NULL)
    {
        return false;
    }
    if ((config->front_mask == 0U) || (config->rear_mask == 0U))
    {
        return false;
    }
    if ((config->front_mask & config->rear_mask) != 0U)
    {
        return false;
    }
    if (((uint32_t)config->front_mask | (uint32_t)config->rear_mask) !=
        (uint32_t)SEQ_ALL_MASK)
    {
        return false;
    }

    if (config->clearance_mm <= 0)
    {
        return false;
    }
    if (config->retracted_mm < 0)
    {
        return false;
    }
    if (config->retracted_mm >= config->clearance_mm)
    {
        return false;
    }
    /* contact_mm = 0 表示不找接触；填了就必须落在 (retracted, clearance) 之间 */
    if ((config->contact_mm != 0) &&
        ((config->contact_mm <= config->retracted_mm) ||
         (config->contact_mm >= config->clearance_mm)))
    {
        return false;
    }
    if ((config->max_sync_error_mm <= 0) ||
        (config->max_deviation_mm <= 0) ||
        (config->position_tolerance_mm <= 0) ||
        (config->settled_speed_mm_s <= 0) ||
        (config->max_current <= 0))
    {
        return false;
    }
    if ((config->load_current_min < 0) ||
        (config->load_current_min > config->max_current))
    {
        return false;
    }
    if ((config->stall_speed_mm_s < 0) || (config->stall_ms == 0U))
    {
        return false;
    }
    /* 关键阶段必须有限时，否则会永久挂住。 */
    if ((config->precheck_timeout_ms == 0U) ||
        (config->lift_timeout_ms == 0U) ||
        (config->retract_timeout_ms == 0U))
    {
        return false;
    }
    return true;
}

void lift_seq_config_default(lift_seq_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    *config = (lift_seq_config_t){0};

    /*
     * 2026-10-13 用户确认 ch0/ch1 = 前辅助轮，ch2/ch3 = 后麦轮。
     * `layout_confirmed` 仍默认 false，更换接线后需再次核对。
     */
    config->front_mask = 0x03U;   /* ch0, ch1 */
    config->rear_mask = 0x0CU;    /* ch2, ch3 */

    /* 任务目标：整车竖直抬升约 200 mm。 */
    config->clearance_mm = 200;
    /*
     * 前部找接触的起点：从安全高度往下 20 mm。
     *
     * ⚠️ 这只是一个【起点】，不是标定值。真实的接触高度由
     *    「台阶高度 − 升降支撑点到轮心的距离」决定，只有把这两个量
     *    量出来才能定死。在拿到之前，操作手在 SEEK_FRONT_CONTACT
     *    阶段用眼睛确认"前轮压上了"，所以这个数偏一点不影响安全 ——
     *    它只决定"从哪开始找"，不决定"什么时候算找到"。
     */
    config->contact_mm = config->clearance_mm - SEQ_CONTACT_BACKOFF_MM;
    config->retracted_mm = 40;

    config->max_sync_error_mm = 10;
    config->max_deviation_mm = 6;
    config->position_tolerance_mm = 3;
    config->settled_speed_mm_s = 2;
    config->max_current = 16000;
    /*
     * 0 = 关闭"电流在带载区间"这条判据。理由见 lift_seq.h 文件头：
     * 它的正确取值必须由实测保持电流来定，随便填会让所有 SUPPORT 阶段
     * 直接判故障 —— 比不判更糟。
     */
    config->load_current_min = 0;

    config->stall_speed_mm_s = 1;
    config->stall_ms = 1500U;

    config->precheck_timeout_ms = 2000U;
    config->lift_timeout_ms = 60000U;
    config->verify_timeout_ms = 5000U;
    config->advance_timeout_ms = 60000U;
    config->retract_timeout_ms = 30000U;
    config->support_timeout_ms = 60000U;
}

void lift_seq_init(lift_seq_t *self, const lift_seq_config_t *config)
{
    if ((self == NULL) || (config == NULL))
    {
        return;
    }

    *self = (lift_seq_t){0};
    self->config = *config;
    self->state = LIFT_SEQ_STATE_IDLE;
    self->fault = LIFT_SAFETY_FAULT_NONE;
    self->stop_action = LIFT_STOP_HOLD;
    self->fault_axis = -1;
    self->test_stall_axis = -1;
}

void lift_seq_reset(lift_seq_t *self)
{
    uint32_t ch;

    if (self == NULL)
    {
        return;
    }

    self->state = LIFT_SEQ_STATE_IDLE;
    self->fault = LIFT_SAFETY_FAULT_NONE;
    self->stop_action = LIFT_STOP_HOLD;
    self->released = false;
    self->step_start_ms = 0U;
    self->run_start_ms = 0U;
    self->settle_active = false;
    self->settle_start_ms = 0U;
    self->targets_valid = false;
    self->confirm_for = LIFT_SEQ_STATE_IDLE;
    self->fault_detail = NULL;
    self->fault_axis = -1;
    self->test_stall_axis = -1;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        self->stall_active[ch] = false;
        self->stall_start_ms[ch] = 0U;
        self->axis_target_mm[ch] = 0;
    }
}

bool lift_seq_is_active(const lift_seq_t *self)
{
    if (self == NULL)
    {
        return false;
    }
    return (self->state != LIFT_SEQ_STATE_IDLE) &&
           (self->state != LIFT_SEQ_STATE_COMPLETE) &&
           (self->state != LIFT_SEQ_STATE_FAULT);
}

const char *lift_seq_state_name(lift_seq_state_t state)
{
    switch (state)
    {
        case LIFT_SEQ_STATE_IDLE:                 return "IDLE";
        case LIFT_SEQ_STATE_PRECHECK:             return "PRECHECK";
        case LIFT_SEQ_STATE_LIFT_CLEARANCE:       return "LIFT_CLEARANCE";
        case LIFT_SEQ_STATE_VERIFY_CLEARANCE:     return "VERIFY_CLEARANCE";
        case LIFT_SEQ_STATE_APPROACH_STEP:        return "APPROACH_STEP";
        case LIFT_SEQ_STATE_VERIFY_FRONT_SUPPORT: return "VERIFY_FRONT_SUPPORT";
        case LIFT_SEQ_STATE_SEEK_FRONT_CONTACT:   return "SEEK_FRONT_CONTACT";
        case LIFT_SEQ_STATE_RETRACT_FRONT:        return "RETRACT_FRONT";
        case LIFT_SEQ_STATE_VERIFY_FRONT_RETRACT: return "VERIFY_FRONT_RETRACT";
        case LIFT_SEQ_STATE_ADVANCE_MIDDLE:       return "ADVANCE_MIDDLE";
        case LIFT_SEQ_STATE_VERIFY_REAR_SUPPORT:  return "VERIFY_REAR_SUPPORT";
        case LIFT_SEQ_STATE_RETRACT_REAR:         return "RETRACT_REAR";
        case LIFT_SEQ_STATE_VERIFY_REAR_RETRACT:  return "VERIFY_REAR_RETRACT";
        case LIFT_SEQ_STATE_FINAL_ADVANCE:        return "FINAL_ADVANCE";
        case LIFT_SEQ_STATE_COMPLETE:             return "COMPLETE";
        case LIFT_SEQ_STATE_FAULT:                return "FAULT";
        case LIFT_SEQ_STATE_TEST_HOLD:            return "TEST_HOLD";
        case LIFT_SEQ_STATE_TEST_PAUSED:          return "TEST_PAUSED";
        case LIFT_SEQ_STATE_TEST_LOWER:           return "TEST_LOWER";
        case LIFT_SEQ_STATE_TEST_PARKED:          return "TEST_PARKED";
        default:                                  return "?";
    }
}

/* ==================================================================== */
/*  支撑前提与故障                                                      */
/* ==================================================================== */

/*
 * 当前阶段的支撑前提。
 *
 * 这是停机策略唯一需要的、而且【不是猜】的信息：状态机自己知道它让四根
 * 齿条各处于什么位置，于是知道车靠什么托着。
 *
 *   已经落回地面 / 已经收完            → GROUND      （断电无风险）
 *   操作手按了"人力解除"               → MECHANICAL  （断电无风险）
 *   其余（四根都撑在空中，或正在做一次还没确认的载荷转移） → AIRBORNE
 *
 * 刻意【没有】UNKNOWN 这一支：状态机总能说出自己在哪一步，所以不需要猜。
 * UNKNOWN 只留给"调用方不知道"的场景（见 lift_safety.h），本模块不用它。
 */
static lift_support_t seq_support_now(const lift_seq_t *self)
{
    if (self->released)
    {
        return LIFT_SUPPORT_MECHANICAL;
    }

    switch (self->state)
    {
        case LIFT_SEQ_STATE_IDLE:
        case LIFT_SEQ_STATE_PRECHECK:
        case LIFT_SEQ_STATE_COMPLETE:
        case LIFT_SEQ_STATE_RETRACT_REAR:
        case LIFT_SEQ_STATE_VERIFY_REAR_RETRACT:
        case LIFT_SEQ_STATE_FINAL_ADVANCE:
            /* 还没抬起来，或者后部已经收回（车已落到台阶/地面上）。 */
            return LIFT_SUPPORT_GROUND;

        default:
            /* 四根齿条都在支撑，或在做尚未确认的载荷转移 → 按悬空处理。 */
            return LIFT_SUPPORT_AIRBORNE;
    }
}

/** 进入故障：锁存、算停机策略、记录明细。 */
static void seq_fault(
    lift_seq_t *self,
    lift_safety_fault_t fault,
    const char *detail,
    int32_t axis,
    bool moving,
    uint32_t now_ms)
{
    lift_safety_context_t context;

    context.fault = fault;
    context.support = seq_support_now(self);
    /* Legacy request selection assumes a datum here, including PRECHECK faults.
     * This is not measured evidence; the top executor checks actual homed state
     * and the task gates unconfirmed travel. Do not treat this as support proof. */
    context.homed = true;
    context.calibrated = true;
    context.moving = moving;

    self->stop_action = lift_safety_policy_for(&context);
    self->fault = fault;
    self->fault_detail = detail;
    self->fault_axis = axis;
    self->state = LIFT_SEQ_STATE_FAULT;
    self->step_start_ms = now_ms;
    self->settle_active = false;
    self->targets_valid = false;
}

/* ==================================================================== */
/*  判据                                                                */
/* ==================================================================== */

/** 全部通道是否都到目标容差内且停住。 */
static bool seq_all_done(const lift_seq_t *self, const lift_seq_input_t *in)
{
    uint32_t ch;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        if (seq_abs_i32(in->position_mm[ch] - self->axis_target_mm[ch]) >
            self->config.position_tolerance_mm)
        {
            return false;
        }
        if (seq_abs_i32(in->speed_mm_s[ch]) >
            self->config.settled_speed_mm_s)
        {
            return false;
        }
    }
    return true;
}

/** 掩码内通道是否都到目标容差内且停住。 */
static bool seq_group_done(
    const lift_seq_t *self,
    const lift_seq_input_t *in,
    uint8_t mask)
{
    uint32_t ch;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        if (!seq_in_mask(mask, ch))
        {
            continue;
        }
        if (seq_abs_i32(in->position_mm[ch] - self->axis_target_mm[ch]) >
            self->config.position_tolerance_mm)
        {
            return false;
        }
        if (seq_abs_i32(in->speed_mm_s[ch]) >
            self->config.settled_speed_mm_s)
        {
            return false;
        }
    }
    return true;
}

/** 四轴是否都停住了。 */
static bool seq_all_settled(const lift_seq_t *self, const lift_seq_input_t *in)
{
    uint32_t ch;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        if (seq_abs_i32(in->speed_mm_s[ch]) >
            self->config.settled_speed_mm_s)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 掩码内通道彼此是否足够齐（组内极差 ≤ limit_mm）。
 *
 * 为什么需要"组内"这个概念：上台阶过程中【前组与后组本来就会差很多】
 * （前部收到 40 mm、后部还撑在 200 mm，差 160 mm 是设计状态）。
 * 拿"四轴极差"当判据会把正常流程判成故障；真正要防的是
 * "同一组里一根卡住、另一根照走"——那才会把机构扭坏。
 */
static bool seq_group_consistent(
    const lift_seq_input_t *in,
    uint8_t mask,
    int32_t limit_mm)
{
    int32_t lowest = 0;
    int32_t highest = 0;
    bool any = false;
    uint32_t ch;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        if (!seq_in_mask(mask, ch))
        {
            continue;
        }
        if (!any)
        {
            lowest = in->position_mm[ch];
            highest = in->position_mm[ch];
            any = true;
        }
        else
        {
            if (in->position_mm[ch] < lowest)
            {
                lowest = in->position_mm[ch];
            }
            if (in->position_mm[ch] > highest)
            {
                highest = in->position_mm[ch];
            }
        }
    }

    if (!any || (limit_mm <= 0))
    {
        return true;
    }
    return (highest - lowest) <= limit_mm;
}

/** "条件成立后再稳住 settle_ms"的通用计时。 */
static bool seq_settle_elapsed(
    lift_seq_t *self,
    const lift_seq_input_t *in,
    bool condition_ok,
    uint32_t settle_ms)
{
    if (!condition_ok)
    {
        self->settle_active = false;
        return false;
    }
    if (!self->settle_active)
    {
        self->settle_active = true;
        self->settle_start_ms = in->now_ms;
        return (settle_ms == 0U);
    }
    return (uint32_t)(in->now_ms - self->settle_start_ms) >= settle_ms;
}

/**
 * @brief "停住之后再看姿态有没有变"的防抖双拍判据。
 *
 * ★ 为什么需要它（这是本模块自己踩过的坑，值得写清楚）：
 *
 *   `deviation_max_mm` 是"某轴相对其余轴参考高度"的偏差。**在正常收回
 *   过程中，它必然会先变大** —— 前部两根在往下降、后部两根还撑在安全
 *   高度上，差值就是中间过程的真实姿态差（实测会到 20 mm）。
 *
 *   如果拿它当"载荷转移出问题了"的判据直接查，正常流程会被自己判成故障：
 *   实测现象是"一进找接触阶段立刻 SYNC 故障、明细写着前轮没压住"，
 *   而前轮其实压得好好的。
 *
 *   正确的判据是【停下来之后还没有收敛】：到位了就该重新变平。
 *
 * @return true 表示"已经到位并稳定，但姿态仍然超限" —— 这才是故障。
 */
static bool seq_tilt_unrecovered(
    lift_seq_t *self,
    const lift_seq_input_t *in,
    uint8_t mask,
    uint32_t settle_ms)
{
    if ((in->tracking_deviation_valid ? in->tracking_deviation_mm :
        in->deviation_max_mm) <= self->config.max_deviation_mm)
    {
        /* 姿态还在允许范围内：清掉计时，等下一次。 */
        self->settle_active = false;
        return false;
    }
    return seq_settle_elapsed(
        self, in, seq_group_done(self, in, mask), settle_ms);
}

/**
 * @brief 电气必要条件：本组至少有一根轴的电流达到 load_current_min。
 * @note load_current_min == 0 时该判据【关闭】，直接返回 true。
 */
static bool seq_load_evidence(
    const lift_seq_t *self,
    const lift_seq_input_t *in,
    uint8_t mask)
{
    uint32_t ch;

    if (self->config.load_current_min <= 0)
    {
        return true;
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        if (!seq_in_mask(mask, ch))
        {
            continue;
        }
        if (seq_abs_i32(in->current[ch]) >= self->config.load_current_min)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief 堵转看门狗：掩码内某轴"该动却长时间不动"。
 * @return 出问题的轴号；-1 表示没有。
 */
static int32_t seq_stall_axis(
    lift_seq_t *self,
    const lift_seq_input_t *in,
    uint8_t mask)
{
    uint32_t ch;
    int32_t found = -1;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        const bool expected_to_move =
            seq_in_mask(mask, ch) &&
            (seq_abs_i32(in->position_mm[ch] - self->axis_target_mm[ch]) >
             self->config.position_tolerance_mm);

        if (!expected_to_move)
        {
            self->stall_active[ch] = false;
            continue;
        }

        if (seq_abs_i32(in->speed_mm_s[ch]) <= self->config.stall_speed_mm_s)
        {
            if (!self->stall_active[ch])
            {
                self->stall_active[ch] = true;
                self->stall_start_ms[ch] = in->now_ms;
            }
            else if ((uint32_t)(in->now_ms - self->stall_start_ms[ch]) >=
                     self->config.stall_ms)
            {
                if (found < 0)
                {
                    found = (int32_t)ch;
                }
            }
            else
            {
                /* 还在计时。 */
            }
        }
        else
        {
            self->stall_active[ch] = false;
        }
    }
    return found;
}

/* ==================================================================== */
/*  主更新                                                              */
/* ==================================================================== */

void lift_seq_update(
    lift_seq_t *self,
    const lift_seq_input_t *in,
    lift_seq_status_t *status)
{
    bool start_rising;
    bool abort_rising;
    bool confirm_rising;
    bool release_rising;
    bool reset_rising;
    lift_seq_state_t state_at_entry;
    uint32_t timeout;
    uint32_t ch;

    if ((self == NULL) || (in == NULL) || (status == NULL))
    {
        return;
    }

    /* 每周期完整重建，避免分支残留上一拍的值。 */
    *status = (lift_seq_status_t){0};
    status->fault_axis = -1;
    status->operator_action = "";
    status->fault_detail = "";
    status->released = self->released;
    status->support = seq_support_now(self);

    start_rising = seq_pulse(in->start_request, &self->start_last);
    abort_rising = seq_pulse(in->abort_request, &self->abort_last);
    confirm_rising = seq_pulse(in->confirm_request, &self->confirm_last);
    release_rising = seq_pulse(in->release_request, &self->release_last);
    reset_rising = seq_pulse(in->reset_request, &self->reset_last);

    /* 本周期确认的默认归属：没人认领（= 作废）。 */
    self->pending_confirm_target = LIFT_SEQ_STATE_IDLE;

    if (confirm_rising)
    {
        self->confirm_count++;
    }

    /* Reject invalid states before pause/release handlers can transform them. */
    if ((unsigned)self->state >= (unsigned)LIFT_SEQ_STATE_COUNT)
    {
        seq_fault(self, LIFT_SAFETY_FAULT_UNSUPPORTED,
            "invalid sequence state", -1, false, in->now_ms);
        goto emit;
    }

    /* ---- 人工解除：任何时刻都生效，包括 FAULT ---- */
    if (release_rising)
    {
        self->released = true;
        status->released = true;
        status->support = LIFT_SUPPORT_MECHANICAL;
        if (self->state == LIFT_SEQ_STATE_FAULT)
        {
            lift_safety_context_t context;

            context.fault = self->fault;
            context.support = LIFT_SUPPORT_MECHANICAL;
            context.homed = true;
            context.calibrated = true;
            context.moving = false;
            /* 已确认机械支撑 → 策略降级成 CUT（立刻断电）。 */
            self->stop_action = lift_safety_policy_for(&context);
        }
    }

    /* ---- 故障复位：只在故障态下接受复位沿 ---- */
    if (reset_rising && (self->state == LIFT_SEQ_STATE_FAULT))
    {
        if (!in->recovery_confirmed) status->recovery_blocked |= LIFT_RECOVERY_SUPPORT;
        if (!in->motion_command_disabled || in->start_request || in->abort_request)
            status->recovery_blocked |= LIFT_RECOVERY_DISABLE;
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            if (!in->axis_online[ch]) status->recovery_blocked |= LIFT_RECOVERY_OFFLINE;
            if (in->speed_mm_s[ch] > self->config.settled_speed_mm_s ||
                in->speed_mm_s[ch] < -self->config.settled_speed_mm_s)
                status->recovery_blocked |= LIFT_RECOVERY_MOVING;
        }
        if (status->recovery_blocked != 0U) goto emit;
        lift_seq_reset(self);
        status->state = self->state;
        status->fault = self->fault;
        status->stop_action = self->stop_action;
        status->step_index = (uint32_t)self->state;
        return;
    }

    /* ---- 故障锁存：只有复位能解 ---- */
    if (self->state == LIFT_SEQ_STATE_FAULT)
    {
        goto emit;
    }

    /* ---- 中止 ---- */
    if (abort_rising && lift_seq_is_active(self))
    {
        seq_fault(self, LIFT_SAFETY_FAULT_ABORTED, "abort_request",
                  -1, true, in->now_ms);
        goto emit;
    }

    /* ---- 启动 ---- */
    if (start_rising && (self->state == LIFT_SEQ_STATE_IDLE))
    {
        self->released = false;
        seq_enter(self, LIFT_SEQ_STATE_PRECHECK, in->now_ms);
        self->run_start_ms = in->now_ms;
    }

    if (self->state == LIFT_SEQ_STATE_IDLE)
    {
        goto emit;
    }

    if (self->state != LIFT_SEQ_STATE_COMPLETE)
    {
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            if (!in->axis_online[ch])
            {
                seq_fault(self, LIFT_SAFETY_FAULT_OFFLINE, "axis offline", (int32_t)ch, false, in->now_ms);
                goto emit;
            }
        if (self->state != LIFT_SEQ_STATE_PRECHECK &&
            (!in->homed || !in->calibrated || !in->layout_confirmed))
        {
            seq_fault(self, LIFT_SAFETY_FAULT_UNSUPPORTED,
                "motion prerequisite lost", -1, false, in->now_ms);
            goto emit;
        }
    }

    /* ---- 上游安全闸报故：任何阶段都停 ---- */
    if (in->guard_fault)
    {
        seq_fault(self, LIFT_SAFETY_FAULT_OFFLINE, "guard_fault",
                  -1, true, in->now_ms);
        goto emit;
    }

    /*
     * ---- 限位监视线（任何阶段）----
     *
     * 限位被拦不是"某一个阶段的问题"，而是"这一步不该再往下走"的信号，
     * 所以它必须在【所有】活动阶段生效，而不是只在 PRECHECK 里查一次。
     * 触界了还继续走流程，等于让机构带着一个已经到头的自由度去做载荷转移。
     */
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        if (in->limit_blocked[ch])
        {
            seq_fault(self, LIFT_SAFETY_FAULT_LIMIT, "limit blocked",
                      (int32_t)ch, true, in->now_ms);
            goto emit;
        }
    }

    /*
     * ★ 记下进入本函数时的阶段。下面 switch 里各阶段可能在结束时
     *   seq_enter() 到下一阶段，于是出口处的 self->state 与"本拍展示的
     *   阶段字段"会对不上。用这个快照做比较，才能发现"本周期换阶段了"。
     */
    state_at_entry = self->state;

    if (self->config.stair_debug_run && !self->config.lift_test_only)
    {
        if (in->test_pause_request && self->state != LIFT_SEQ_STATE_TEST_PAUSED &&
            self->state != LIFT_SEQ_STATE_COMPLETE)
        {
            self->test_resume_state = self->state;
            self->stair_resume_elapsed_ms = in->now_ms - self->step_start_ms;
            self->test_stall_axis = -1;
            for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            {
                self->stair_resume_mm[ch] = self->axis_target_mm[ch];
                self->test_pause_ecd[ch] = in->position_ecd[ch];
                self->axis_target_mm[ch] = in->position_mm[ch];
            }
            self->targets_valid = true;
            seq_enter(self, LIFT_SEQ_STATE_TEST_PAUSED, in->now_ms);
        }
        else if (self->state == LIFT_SEQ_STATE_TEST_PAUSED && in->test_resume_request)
        {
            for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
                self->axis_target_mm[ch] = self->stair_resume_mm[ch];
            seq_enter(self, self->test_resume_state, in->now_ms);
            self->step_start_ms = in->now_ms - self->stair_resume_elapsed_ms;
            self->test_stall_axis = -1;
        }
    }
    else if (self->config.lift_test_only)
    {
        const bool test_controllable =
            self->state == LIFT_SEQ_STATE_LIFT_CLEARANCE ||
            self->state == LIFT_SEQ_STATE_VERIFY_CLEARANCE ||
            self->state == LIFT_SEQ_STATE_TEST_HOLD ||
            self->state == LIFT_SEQ_STATE_TEST_PAUSED ||
            self->state == LIFT_SEQ_STATE_TEST_LOWER;
        /* Lower has priority over pause/resume. Abort and safety faults above
         * always have priority over every normal test command. */
        if (test_controllable && in->test_lower_request)
        {
            seq_targets_all(self, self->config.retracted_mm);
            seq_enter(self, LIFT_SEQ_STATE_TEST_LOWER, in->now_ms);
        }
        else if (test_controllable && in->test_pause_request &&
                 self->state != LIFT_SEQ_STATE_TEST_PAUSED)
        {
            self->test_resume_state = self->state;
            self->test_stall_axis = -1;
            for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            {
                int32_t pose = in->position_ecd[ch];
                if (pose < in->limit_min_ecd[ch]) pose = in->limit_min_ecd[ch];
                if (pose > in->limit_max_ecd[ch]) pose = in->limit_max_ecd[ch];
                self->test_pause_ecd[ch] = pose;
                self->axis_target_mm[ch] = in->position_mm[ch];
            }
            self->targets_valid = true;
            seq_enter(self, LIFT_SEQ_STATE_TEST_PAUSED, in->now_ms);
        }
        else if (self->state == LIFT_SEQ_STATE_TEST_PAUSED &&
                 in->test_resume_request)
        {
            const bool lowering = self->test_resume_state == LIFT_SEQ_STATE_TEST_LOWER;
            self->test_stall_axis = -1;
            seq_targets_all(self, lowering ? self->config.retracted_mm :
                            self->config.clearance_mm);
            seq_enter(self, lowering ? LIFT_SEQ_STATE_TEST_LOWER :
                      LIFT_SEQ_STATE_LIFT_CLEARANCE, in->now_ms);
        }
    }

    switch (self->state)
    {
    /* ---------------------------------------------------------------- */
    case LIFT_SEQ_STATE_PRECHECK:
    {
        /*
         * ★ 前置检查必须【占用一个完整的稳定窗口】，不能在同一周期里
         *   "看到全部条件成立就立刻开始抬升"。
         *
         * 两个理由：
         *   1. 反馈是异步来的。这一拍读到的位置/在线状态可能正好是上一轮
         *      的残留（例如刚复位完、CAN 还没上来），一致地看满一段时间
         *      才叫"检查过"。
         *   2. 它把"请求启动"和"开始出力"分成两个可观察的时刻 ——
         *      现场能先看到 PRECHECK 的结果再看到动作，而不是按下去就动。
         */
        if (!seq_settle_elapsed(self, in, true, SEQ_PRECHECK_SETTLE_MS))
        {
            status->operator_action = "checking";
            break;
        }

        status->operator_action = "wait";
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            if (!in->axis_online[ch])
            {
                seq_fault(self, LIFT_SAFETY_FAULT_OFFLINE,
                          "axis offline in precheck", (int32_t)ch, false,
                          in->now_ms);
                goto emit;
            }
        }
        if (!in->homed)
        {
            seq_fault(self, LIFT_SAFETY_FAULT_UNSUPPORTED,
                      "not homed: no position datum", -1, false, in->now_ms);
            goto emit;
        }
        if (!in->layout_confirmed)
        {
            /*
             * 前后轮组布局没确认 —— 收错一组就是把还在承重的支撑收掉。
             * 这不是"警告"，是拒绝启动。
             */
            seq_fault(self, LIFT_SAFETY_FAULT_UNSUPPORTED,
                      "front/rear layout NOT confirmed (see lift_seq.h)",
                      -1, false, in->now_ms);
            goto emit;
        }
        if (!in->calibrated)
        {
            /* 没标定就要求 200 mm 绝对位置 —— 明确拒绝。 */
            seq_fault(self, LIFT_SAFETY_FAULT_NOT_CALIBRATED,
                      "kinematics not calibrated", -1, false, in->now_ms);
            goto emit;
        }
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            const int32_t recovery = in->limit_min_ecd[ch] == 0 &&
                in->contact_zero_recovery_ecd[ch] > 0 ?
                in->contact_zero_recovery_ecd[ch] : 0;
            if (in->limit_blocked[ch])
            {
                seq_fault(self, LIFT_SAFETY_FAULT_LIMIT, "limit blocked",
                          (int32_t)ch, false, in->now_ms);
                goto emit;
            }
            if ((in->limit_min_ecd[ch] < in->limit_max_ecd[ch]) &&
                (((int64_t)in->position_ecd[ch] + recovery <
                  (int64_t)in->limit_min_ecd[ch]) ||
                 (in->position_ecd[ch] > in->limit_max_ecd[ch])))
            {
                seq_fault(self, LIFT_SAFETY_FAULT_LIMIT,
                          "axis already outside soft limits", (int32_t)ch,
                          false, in->now_ms);
                goto emit;
            }
        }
        if (in->sync_error_mm > self->config.max_sync_error_mm)
        {
            seq_fault(self, LIFT_SAFETY_FAULT_SYNC,
                      "sync error already over limit at start", -1, false,
                      in->now_ms);
            goto emit;
        }
        if (in->deviation_max_mm > self->config.max_deviation_mm)
        {
            seq_fault(self, LIFT_SAFETY_FAULT_SYNC,
                      "attitude already tilted at start", -1, false,
                      in->now_ms);
            goto emit;
        }
        /* 全部通过 → 开始抬升。 */
        seq_enter(self, LIFT_SEQ_STATE_LIFT_CLEARANCE, in->now_ms);
        seq_targets_all(self, self->config.clearance_mm);
        break;
    }

    /* ---------------------------------------------------------------- */
    case LIFT_SEQ_STATE_LIFT_CLEARANCE:
        status->operator_action = "hold still - lifting";
        if (seq_all_done(self, in))
        {
            /* 到位后再稳 300 ms 才算数（斜坡末端的速度指令还没走完）。 */
            if (seq_settle_elapsed(self, in, true, 300U))
            {
                seq_enter(self, LIFT_SEQ_STATE_VERIFY_CLEARANCE, in->now_ms);
            }
        }
        else
        {
            (void)seq_settle_elapsed(self, in, false, 0U);
        }
        break;

    /* ---------------------------------------------------------------- */
    case LIFT_SEQ_STATE_VERIFY_CLEARANCE:
        status->operator_action = "wait";
        if (!seq_all_done(self, in))
        {
            seq_fault(self, LIFT_SAFETY_FAULT_SYNC,
                      "clearance height lost during verify", -1, true,
                      in->now_ms);
            goto emit;
        }
        if (in->sync_error_mm > self->config.max_sync_error_mm)
        {
            seq_fault(self, LIFT_SAFETY_FAULT_SYNC,
                      "sync error after lift", -1, true, in->now_ms);
            goto emit;
        }
        if (in->deviation_max_mm > self->config.max_deviation_mm)
        {
            seq_fault(self, LIFT_SAFETY_FAULT_SYNC,
                      "attitude error after lift", -1, true, in->now_ms);
            goto emit;
        }
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            if (seq_abs_i32(in->current[ch]) > self->config.max_current)
            {
                seq_fault(self, LIFT_SAFETY_FAULT_OVERCURRENT,
                          "current over limit after lift", (int32_t)ch, true,
                          in->now_ms);
                goto emit;
            }
        }
        seq_enter(self, self->config.lift_test_only ? LIFT_SEQ_STATE_TEST_HOLD :
                  LIFT_SEQ_STATE_APPROACH_STEP, in->now_ms);
        /*
         * 靠近台阶期间四轴【保持】在 clearance_mm：抬升机构一直伸着，
         * 底盘靠自己的轮子走，直到前轮组压上台阶。
         */
        seq_targets_all(self, self->config.clearance_mm);
        break;

    /* ---------------------------------------------------------------- */
    case LIFT_SEQ_STATE_APPROACH_STEP:
        /*
         * 底盘前进由【人】或【底盘模块】执行：抬升模块没有权限动底盘，
         * 也不应该动。这里只声明"现在该往前走了"，并等 drive_done / 人工确认。
         */
        status->drive_request = true;
        status->operator_action =
            "drive forward slowly until the FRONT wheels touch the step, then confirm";
        if (in->drive_done || confirm_rising)
        {
            seq_enter(self, LIFT_SEQ_STATE_VERIFY_FRONT_SUPPORT, in->now_ms);
        }
        break;

    /* ---------------------------------------------------------------- */
    case LIFT_SEQ_STATE_VERIFY_FRONT_SUPPORT:
        status->waiting_confirm = true;
        status->operator_action =
            "confirm the FRONT wheels are resting ON the step";
        if (!seq_load_evidence(self, in, self->config.front_mask))
        {
            seq_fault(self, LIFT_SAFETY_FAULT_UNSUPPORTED,
                      "no load current on the front group", -1, false,
                      in->now_ms);
            goto emit;
        }
        if ((seq_take_confirm(self, confirm_rising) || in->contact_front) &&
            seq_all_settled(self, in))
        {
            if (self->config.contact_mm != 0)
            {
                /*
                 * 先降到"前轮刚好压上台阶"的接触点。这一小段不需要再按
                 * 一次确认：操作手刚刚说的就是"前轮已经压在台阶上了"，
                 * 接着往下走是同一件事的延续。
                 */
                seq_enter(self, LIFT_SEQ_STATE_SEEK_FRONT_CONTACT, in->now_ms);
                seq_targets_group(self, self->config.front_mask,
                                  self->config.contact_mm,
                                  self->config.clearance_mm);
            }
            else
            {
                /* 不找接触（台架）：确认之后直接一路收到收回位。 */
                seq_enter(self, LIFT_SEQ_STATE_RETRACT_FRONT, in->now_ms);
                seq_targets_group(self, self->config.front_mask,
                                  self->config.retracted_mm,
                                  self->config.clearance_mm);
            }
        }
        break;

    /* ---------------------------------------------------------------- */
    case LIFT_SEQ_STATE_SEEK_FRONT_CONTACT:
        /*
         * 载荷转移段：前部两根降到 contact_mm（前轮预计刚好压上台阶的高度）。
         *
         * ★ 这一小段【不需要人工确认】。
         *   操作手在上一阶段刚说过"前轮已经压在台阶上了"，接着往下走是
         *   同一件事的延续；再要一次按键只会让人以为"卡住了"。
         *   （本模块早期版本在这里要了三次确认，实测表现为"进这一阶段就
         *     永远停住"，很难判断是等确认还是坏了。）
         *
         * ★ 这一段也【不判姿态】。
         *   前部本来就会比后部低最多 (clearance − contact) mm，那正是这一步
         *   的目的，不是异常。载荷转移成不成功的判据在下一段 RETRACT_FRONT：
         *   那时前部要【从接触点继续往下收】，若前轮没承重，姿态回不来。
         */
        status->operator_action = "watch - seeking front contact";
        if (seq_group_done(self, in, self->config.front_mask))
        {
            seq_enter(self, LIFT_SEQ_STATE_RETRACT_FRONT, in->now_ms);
            seq_targets_group(self, self->config.front_mask,
                              self->config.retracted_mm,
                              self->config.clearance_mm);
        }
        break;

    /* ---------------------------------------------------------------- */
    case LIFT_SEQ_STATE_RETRACT_FRONT:
        /*
         * 前部从接触点继续收到 retracted_mm，后两根继续撑在 clearance_mm。
         *
         * 这一步是载荷转移的【验证】：接触点已经确认过前轮压住了台阶，
         * 所以前轮现在可以承担前部重量 —— 前部两根继续收，车不会前倾。
         * 若前轮其实没压住（`front_unsupported` 那种情形），整车会前倾，
         * 于是 deviation_max_mm 超限 → SYNC 故障、原地停下。
         */
        status->operator_action = "watch - retracting FRONT";
        if (seq_all_done(self, in))
        {
            if (seq_settle_elapsed(self, in, true, 500U))
            {
                seq_enter(self, LIFT_SEQ_STATE_VERIFY_FRONT_RETRACT,
                          in->now_ms);
            }
        }
        else if (seq_tilt_unrecovered(self, in, self->config.front_mask,
                                      SEQ_TILT_SETTLE_MS))
        {
            /*
             * ★ 载荷转移失败的唯一判据。
             *
             * 前部两根已经收到底（seq_group_done）、也稳住了 500 ms，
             * 姿态却仍然回不来 —— 说明前轮没有真正承重。
             *
             * 注意用 seq_group_done 而不是 seq_all_done 当"到位"条件：
             * 这一段的姿态差本来就来自"前低后高"，用四轴全到位会把
             * "前部到位但整车还没收敛"错判成"到位了却还歪"。
             */
            seq_fault(self, LIFT_SAFETY_FAULT_SYNC,
                      "tilt not recovered after retracting FRONT "
                      "(front wheel probably NOT on the step)",
                      -1, true, in->now_ms);
            goto emit;
        }
        else
        {
            /* 正常下降过程：姿态差是过程量，不判故障。 */
        }
        break;

    /* ---------------------------------------------------------------- */
    case LIFT_SEQ_STATE_VERIFY_FRONT_RETRACT:
        /*
         * ★ 这里只查【本组】，不查整车姿态 / 四轴极差。
         *
         * 原因：前部在 40 mm、后部在 200 mm 是这一步的【设计状态】，
         * 极差 160 mm 完全正常。拿"四轴极差"或"相对参考轴的偏差"当判据，
         * 正常流程会被判成故障（实测：一进这个阶段就 sync error 报故）。
         *
         * 载荷转移成不成功已经在 RETRACT_FRONT 里查过了（那时四轴应当
         * 回到同高）；这里要查的是"前部两根自己有没有都到位" ——
         * 一根卡住、另一根到位，就是机构要扭的前兆。
         */
        status->operator_action = "wait";
        if (!seq_group_done(self, in, self->config.front_mask))
        {
            seq_fault(self, LIFT_SAFETY_FAULT_SYNC,
                      "front group not retracted", -1, true, in->now_ms);
            goto emit;
        }
        if (!seq_group_consistent(in, self->config.front_mask,
                                  self->config.max_sync_error_mm))
        {
            seq_fault(self, LIFT_SAFETY_FAULT_SYNC,
                      "front group axes diverged after retract", -1, true,
                      in->now_ms);
            goto emit;
        }
        seq_enter(self, LIFT_SEQ_STATE_ADVANCE_MIDDLE, in->now_ms);
        /* 后组留在 clearance_mm 撑住，前组留在 retracted_mm。 */
        seq_targets_group(self, self->config.front_mask,
                          self->config.retracted_mm,
                          self->config.clearance_mm);
        break;

    /* ---------------------------------------------------------------- */
    case LIFT_SEQ_STATE_ADVANCE_MIDDLE:
        status->drive_request = true;
        status->operator_action =
            "drive forward until the REAR wheels are on the step, then confirm";
        if (in->drive_done || confirm_rising)
        {
            seq_enter(self, LIFT_SEQ_STATE_VERIFY_REAR_SUPPORT, in->now_ms);
        }
        break;

    /* ---------------------------------------------------------------- */
    case LIFT_SEQ_STATE_VERIFY_REAR_SUPPORT:
        status->waiting_confirm = true;
        status->operator_action =
            "confirm the REAR wheels are resting ON the step";
        if (!seq_load_evidence(self, in, self->config.rear_mask))
        {
            seq_fault(self, LIFT_SAFETY_FAULT_UNSUPPORTED,
                      "no load current on the rear group", -1, false,
                      in->now_ms);
            goto emit;
        }
        if ((seq_take_confirm(self, confirm_rising) || in->contact_rear) &&
            seq_all_settled(self, in))
        {
            seq_enter(self, LIFT_SEQ_STATE_RETRACT_REAR, in->now_ms);
            seq_targets_all(self, self->config.retracted_mm);
        }
        break;

    /* ---------------------------------------------------------------- */
    case LIFT_SEQ_STATE_RETRACT_REAR:
        status->operator_action = "watch - retracting REAR";
        if (seq_all_done(self, in))
        {
            if (seq_settle_elapsed(self, in, true, 500U))
            {
                seq_enter(self, LIFT_SEQ_STATE_VERIFY_REAR_RETRACT, in->now_ms);
            }
        }
        else if (seq_tilt_unrecovered(self, in, self->config.rear_mask,
                                      SEQ_TILT_SETTLE_MS))
        {
            seq_fault(self, LIFT_SAFETY_FAULT_SYNC,
                      "tilt not recovered after retracting REAR", -1, true,
                      in->now_ms);
            goto emit;
        }
        else
        {
            /* 正常下降过程：姿态差是过程量，不判故障。 */
        }
        break;

    /* ---------------------------------------------------------------- */
    case LIFT_SEQ_STATE_VERIFY_REAR_RETRACT:
        /*
         * 同 VERIFY_FRONT_RETRACT：只查本组 + 本组内一致性，
         * 不拿"四轴极差 / 相对参考轴的偏差"当判据（那会把设计状态判成故障）。
         */
        status->operator_action = "wait";
        if (!seq_all_done(self, in))
        {
            seq_fault(self, LIFT_SAFETY_FAULT_SYNC,
                      "rear group not retracted", -1, true, in->now_ms);
            goto emit;
        }
        if (!seq_group_consistent(in, SEQ_ALL_MASK,
                                  self->config.max_sync_error_mm))
        {
            seq_fault(self, LIFT_SAFETY_FAULT_SYNC,
                      "axes diverged after rear retract", -1, true,
                      in->now_ms);
            goto emit;
        }
        seq_enter(self, LIFT_SEQ_STATE_FINAL_ADVANCE, in->now_ms);
        seq_targets_all(self, self->config.retracted_mm);
        break;

    /* ---------------------------------------------------------------- */
    case LIFT_SEQ_STATE_FINAL_ADVANCE:
        status->drive_request = true;
        status->operator_action =
            "drive fully onto the step, then confirm";
        if (in->drive_done || confirm_rising)
        {
            seq_enter(self, LIFT_SEQ_STATE_COMPLETE, in->now_ms);
        }
        break;

    /* ---------------------------------------------------------------- */
    case LIFT_SEQ_STATE_TEST_LOWER:
        status->operator_action = "controlled descent - low window cuts current";
        /* All axes must reach the low window before removing support current.
         * Once parked, do not re-enable output if gravity changes feedback. */
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            if (in->position_mm[ch] > self->config.retracted_mm + 1 ||
                in->position_mm[ch] < -1) break;
        if (ch == LIFT_CHANNEL_COUNT)
            seq_enter(self, LIFT_SEQ_STATE_TEST_PARKED, in->now_ms);
        break;

    case LIFT_SEQ_STATE_TEST_HOLD:
    case LIFT_SEQ_STATE_TEST_PAUSED:
    case LIFT_SEQ_STATE_TEST_PARKED:
        if (self->state == LIFT_SEQ_STATE_TEST_PARKED)
        {
            status->operator_action = "low window reached - output latched OFF";
            break;
        }
        status->operator_action = self->state == LIFT_SEQ_STATE_TEST_PAUSED ?
            (self->test_stall_axis >= 0 ? "motion stalled - powered hold; inspect before resume" :
             "paused - powered position hold") :
            self->state == LIFT_SEQ_STATE_TEST_PARKED ?
            "low position - still powered, verify mechanical support" :
            "lift test holding - pause/resume/lower only";
        if (self->config.stair_debug_run ?
            (in->tracking_deviation_valid && in->tracking_deviation_mm > self->config.max_deviation_mm) :
            (in->sync_error_mm > self->config.max_sync_error_mm ||
             in->deviation_max_mm > self->config.max_deviation_mm))
        {
            if (seq_settle_elapsed(self, in, true, SEQ_TILT_SETTLE_MS))
            {
                seq_fault(self, LIFT_SAFETY_FAULT_SYNC, "test hold attitude error",
                          -1, true, in->now_ms);
                goto emit;
            }
        }
        else (void)seq_settle_elapsed(self, in, false, 0U);
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            if (seq_abs_i32(in->current[ch]) > self->config.max_current)
            {
                seq_fault(self, LIFT_SAFETY_FAULT_OVERCURRENT,
                          "test hold current over limit", (int32_t)ch, true, in->now_ms);
                goto emit;
            }
        break;

    case LIFT_SEQ_STATE_COMPLETE:
        status->operator_action = "sequence complete - manual control restored";
        self->targets_valid = false;
        goto emit;

    /* ---------------------------------------------------------------- */
    default:
        seq_fault(self, LIFT_SAFETY_FAULT_UNSUPPORTED, "invalid sequence state", -1, false, in->now_ms);
        goto emit;
    }

    /* ---- 堵转看门狗（只在"该动"的阶段生效）---- */
    if ((self->state == LIFT_SEQ_STATE_LIFT_CLEARANCE) ||
        (self->state == LIFT_SEQ_STATE_TEST_LOWER) ||
        (self->state == LIFT_SEQ_STATE_SEEK_FRONT_CONTACT) ||
        (self->state == LIFT_SEQ_STATE_RETRACT_FRONT) ||
        (self->state == LIFT_SEQ_STATE_RETRACT_REAR))
    {
        const uint8_t mask =
            ((self->state == LIFT_SEQ_STATE_SEEK_FRONT_CONTACT) ||
             (self->state == LIFT_SEQ_STATE_RETRACT_FRONT))
                ? self->config.front_mask
                : ((self->state == LIFT_SEQ_STATE_RETRACT_REAR)
                       ? self->config.rear_mask
                       : SEQ_ALL_MASK);
        const int32_t stalled = seq_stall_axis(self, in, mask);

        if (stalled >= 0)
        {
            if (self->config.lift_test_only || self->config.stair_debug_run)
            {
                self->test_resume_state = self->state;
                self->test_stall_axis = stalled;
                self->stair_resume_elapsed_ms = in->now_ms - self->step_start_ms;
                for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
                {
                    int32_t pose = in->position_ecd[ch];
                    if (pose < in->limit_min_ecd[ch]) pose = in->limit_min_ecd[ch];
                    if (pose > in->limit_max_ecd[ch]) pose = in->limit_max_ecd[ch];
                    self->test_pause_ecd[ch] = pose;
                    self->stair_resume_mm[ch] = self->axis_target_mm[ch];
                    self->axis_target_mm[ch] = in->position_mm[ch];
                }
                self->targets_valid = true;
                seq_enter(self, LIFT_SEQ_STATE_TEST_PAUSED, in->now_ms);
                goto emit;
            }
            seq_fault(self, LIFT_SAFETY_FAULT_OVERCURRENT,
                      "axis stalled while commanded to move", stalled, true,
                      in->now_ms);
            goto emit;
        }
    }

    /* ---- 阶段超时 ---- */
    timeout = seq_stage_timeout(&self->config, self->state);
    if ((timeout != 0U) &&
        ((uint32_t)(in->now_ms - self->step_start_ms) >= timeout))
    {
        seq_fault(self, LIFT_SAFETY_FAULT_TIMEOUT, "stage timeout",
                  -1, true, in->now_ms);
        goto emit;
    }

emit:
    /*
     * ---- 状态在本周期内变过的话，把新阶段的输出字段补上 ----
     *
     * ★ 这是一个【契约】问题，不是显示问题。
     *
     * 大多数阶段在结束时会 seq_enter() 到下一阶段并 break，于是 switch 里
     * 为【旧】阶段写的 drive_request / waiting_confirm / operator_action
     * 一直留到函数出口，而 status->state 已经是【新】阶段了。
     * 调用方拿到的是自相矛盾的一拍：
     *     state = APPROACH_STEP, drive_request = 0, action = "wait"
     * —— 而 APPROACH_STEP 的全部意义就是"声明该往前走了"。
     *
     * 现场表现更糟：如果下一阶段恰好只存在一拍（APPROACH_STEP 收到
     * drive_done 就推进），它可能【从来没有被观察到过一次】，
     * 操作手也就永远看不到那句"现在该往前开了"。
     *
     * 修法：这里按【最终状态】重算一遍阶段相关的展示字段。
     * 只重算"这一拍该告诉人什么"，不改任何决策量（那些已经在 switch 里
     * 算完了）。加上这一段之后，`st` 永远自洽。
     */
    if ((self->state != state_at_entry) && (self->targets_valid))
    {
        status->drive_request = false;
        status->waiting_confirm = false;
        status->operator_action = "";
        switch (self->state)
        {
        case LIFT_SEQ_STATE_PRECHECK:
            status->operator_action = "checking";
            break;
        case LIFT_SEQ_STATE_LIFT_CLEARANCE:
            status->operator_action = "hold still - lifting";
            break;
        case LIFT_SEQ_STATE_VERIFY_CLEARANCE:
            status->operator_action = "wait";
            break;
        case LIFT_SEQ_STATE_APPROACH_STEP:
            status->drive_request = true;
            status->operator_action =
                "drive forward slowly until the FRONT wheels touch the step, then confirm";
            break;
        case LIFT_SEQ_STATE_TEST_HOLD:
            status->operator_action = "lift test holding - pause/resume/lower only";
            break;
        case LIFT_SEQ_STATE_TEST_PAUSED:
            status->operator_action = self->test_stall_axis >= 0 ?
                "motion stalled - powered hold; inspect before resume" :
                "paused - powered position hold";
            break;
        case LIFT_SEQ_STATE_TEST_LOWER:
            status->operator_action = "controlled descent to 2 mm - keep support ready";
            break;
        case LIFT_SEQ_STATE_TEST_PARKED:
            status->operator_action = "low window reached - output latched OFF";
            break;
        case LIFT_SEQ_STATE_VERIFY_FRONT_SUPPORT:
            status->waiting_confirm = true;
            status->operator_action =
                "confirm the FRONT wheels are resting ON the step";
            break;
        case LIFT_SEQ_STATE_SEEK_FRONT_CONTACT:
            status->operator_action = "watch - seeking front contact";
            break;
        case LIFT_SEQ_STATE_RETRACT_FRONT:
            status->operator_action = "watch - retracting FRONT";
            break;
        case LIFT_SEQ_STATE_VERIFY_FRONT_RETRACT:
            status->operator_action = "wait";
            break;
        case LIFT_SEQ_STATE_ADVANCE_MIDDLE:
            status->drive_request = true;
            status->operator_action =
                "drive forward until the REAR wheels are on the step, then confirm";
            break;
        case LIFT_SEQ_STATE_VERIFY_REAR_SUPPORT:
            status->waiting_confirm = true;
            status->operator_action =
                "confirm the REAR wheels are resting ON the step";
            break;
        case LIFT_SEQ_STATE_RETRACT_REAR:
            status->operator_action = "watch - retracting REAR";
            break;
        case LIFT_SEQ_STATE_VERIFY_REAR_RETRACT:
            status->operator_action = "wait";
            break;
        case LIFT_SEQ_STATE_FINAL_ADVANCE:
            status->drive_request = true;
            status->operator_action = "drive fully onto the step, then confirm";
            break;
        case LIFT_SEQ_STATE_COMPLETE:
            status->operator_action =
                "sequence complete - manual control restored";
            break;
        default:
            /* FAULT / IDLE：不需要给操作手动作提示。 */
            break;
        }
    }

    /* ---- 组装输出 ---- */
    status->state = self->state;
    status->fault = self->fault;
    status->stop_action = self->stop_action;
    status->released = self->released;
    status->support = seq_support_now(self);
    status->step_index = (uint32_t)self->state;
    status->step_elapsed_ms = (uint32_t)(in->now_ms - self->step_start_ms);
    status->elapsed_ms = (self->run_start_ms != 0U)
                             ? (uint32_t)(in->now_ms - self->run_start_ms)
                             : 0U;
    status->fault_detail = (self->fault_detail != NULL) ? self->fault_detail
                                                        : "";
    status->fault_axis = self->fault_axis;
    status->confirm_count = self->confirm_count;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        status->axis_target_mm[ch] = self->axis_target_mm[ch];
        status->axis_target_ecd[ch] = self->test_pause_ecd[ch];
    }
    status->command_ecd_valid = self->state == LIFT_SEQ_STATE_TEST_PAUSED;
    status->command_zero_output = self->state == LIFT_SEQ_STATE_TEST_PARKED;

    /* hold_mm = 四轴平均高度：FAULT+HOLD 时控制链要把车停在"当前姿态"。 */
    {
        int64_t sum = 0;

        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            sum += (int64_t)in->position_mm[ch];
        }
        status->hold_mm = (int32_t)(sum / (int64_t)LIFT_CHANNEL_COUNT);
    }

    /*
     * 只在"正在控制"的阶段持有命令。
     * IDLE / COMPLETE / FAULT 一律不持有 —— 交还手动控制，
     * 否则安全闸会把"没有新命令"解释成超时并走受控缓降，
     * 而这正是我们想要的：流程结束时车应该稳稳停住，不是继续被流程摆布。
     */
    if (!lift_seq_is_active(self) || !self->targets_valid)
    {
        status->command_valid = false;
        status->command_enabled = false;
        return;
    }

    status->command_valid = true;
    status->command_enabled = true;
}
