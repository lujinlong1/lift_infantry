#include "lift_safety.h"

#include <stddef.h>

/* ==================================================================== */
/*  命令通道所有权                                                      */
/* ==================================================================== */

/*
 * 当前所有者与认领时刻。读写都在临界区外也不要紧的前提是：
 *   - 只有 32 位对齐量的单次读写，Cortex-M4 上是原子的；
 *   - 竞争窗口最坏导致"两边都以为自己有所有权"持续一个周期，
 *     而看门狗（默认 50 ms）会在下一周期把不刷新的那一方踢掉。
 * 真正要防的是【长期】双写，那个由刷新机制保证。
 */
static volatile lift_owner_t owner_state = LIFT_OWNER_NONE;
static volatile uint32_t owner_tick_ms = 0U;
static volatile uint32_t owner_timeout_ms = 50U;

bool lift_owner_claim(lift_owner_t owner, uint32_t now_ms)
{
    if (owner == LIFT_OWNER_NONE)
    {
        return false;
    }

    if ((owner_state == LIFT_OWNER_NONE) || (owner_state == owner))
    {
        owner_state = owner;
        owner_tick_ms = now_ms;
        return true;
    }

    return false;
}

void lift_owner_release(lift_owner_t owner)
{
    if ((owner != LIFT_OWNER_NONE) && (owner_state == owner))
    {
        owner_state = LIFT_OWNER_NONE;
    }
}

lift_owner_t lift_owner_current(void)
{
    return owner_state;
}

bool lift_owner_may_write(lift_owner_t want)
{
    const lift_owner_t current = owner_state;

    if (want == LIFT_OWNER_NONE)
    {
        return false;
    }
    return (current == LIFT_OWNER_NONE) || (current == want);
}

void lift_owner_set_timeout_ms(uint32_t timeout_ms)
{
    owner_timeout_ms = timeout_ms;
}

uint32_t lift_owner_timeout_ms(void)
{
    return owner_timeout_ms;
}

void lift_owner_tick(uint32_t now_ms)
{
    const uint32_t timeout = owner_timeout_ms;

    if ((owner_state == LIFT_OWNER_NONE) || (timeout == 0U))
    {
        return;
    }
    if ((uint32_t)(now_ms - owner_tick_ms) > timeout)
    {
        /*
         * 认领方超过 timeout 没刷新：它要么死了，要么忘了释放。
         * 自动失效，把通道让出来——否则抬升会永久没有写入者。
         */
        owner_state = LIFT_OWNER_NONE;
    }
}

void lift_owner_reset(void)
{
    owner_state = LIFT_OWNER_NONE;
    owner_tick_ms = 0U;
}

/* ==================================================================== */
/*  故障停机策略                                                        */
/* ==================================================================== */

lift_stop_action_t lift_safety_policy_for(
    const lift_safety_context_t *context)
{
    if (context == NULL)
    {
        /* 连上下文都没有：最保守 —— 既不知道支撑也不知道零点，只能不出力。 */
        return LIFT_STOP_CUT;
    }

    /*
     * 规则 1：已经确认有机械支撑，或车本来就落在地面上。
     * 这两种情况下"断电"不会导致任何跌落，是【最快、最干净】的停机。
     */
    if ((context->support == LIFT_SUPPORT_MECHANICAL) ||
        (context->support == LIFT_SUPPORT_GROUND))
    {
        return LIFT_STOP_CUT;
    }

    /*
     * 规则 2：没有可信零点时不存在"下降到停放点"这个动作 ——
     * LOWER 需要一个位置基准，没有基准就只能不出力。
     */
    if (!context->homed)
    {
        return LIFT_STOP_CUT;
    }

    /*
     * 规则 3：悬空或支撑未知。区分"正在动作"和"停着"：
     *   - 正在动作 → LOWER：受控降到停放点，比停在半空更可预测，
     *                        也让人能安全靠近处理
     *   - 停着     → HOLD：原地托住，不引入新的运动
     *
     * ⚠️ LIFT_SUPPORT_UNKNOWN 走的就是这一支，按"悬空"处理。
     *    宁可多花保持电流，也不要赌"它应该撑住了"。
     */
    return context->moving ? LIFT_STOP_LOWER : LIFT_STOP_HOLD;
}

bool lift_stop_can_hold(const lift_stop_config_t *config)
{
    if (config == NULL)
    {
        return false;
    }
    return (config->hold_current > 0) && (config->current_limit > 0);
}

void lift_stop_apply(
    lift_stop_action_t action,
    const lift_stop_config_t *config,
    const lift_stop_state_t *state,
    int32_t out_current[LIFT_CHANNEL_COUNT],
    bool *out_active)
{
    uint32_t ch;
    int32_t magnitude;
    bool active = false;

    if ((config == NULL) || (state == NULL) || (out_current == NULL))
    {
        if (out_active != NULL)
        {
            *out_active = false;
        }
        return;
    }

    /* 出力 = min(标定保持电流, 上限)；<= 0 时 HOLD/LOWER 退化成 CUT。 */
    magnitude = config->hold_current;
    if ((config->current_limit > 0) && (magnitude > config->current_limit))
    {
        magnitude = config->current_limit;
    }
    if (magnitude < 0)
    {
        magnitude = 0;
    }

    switch (action)
    {
        case LIFT_STOP_HOLD:
            if (magnitude <= 0)
            {
                /* 没标定保持电流 → 唯一已知安全的动作是断电。 */
                for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
                {
                    out_current[ch] = 0;
                }
                break;
            }
            /*
             * 四轴同向给【抬升方向】的电流把车托在原地。
             *
             * 为什么不在四轴之间按位置差分配：HOLD 的前提是"已经出故障"，
             * 此时最不该做的就是引入新的、未经验证的横向载荷转移。
             * 四轴同向、同幅值是这里唯一站得住的策略；真正的调平是正常
             * 控制链的职责，故障态不抢它的活。
             */
            for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            {
                out_current[ch] = magnitude;
            }
            active = true;
            break;

        case LIFT_STOP_LOWER:
            if (magnitude <= 0)
            {
                /* 没标定保持电流 → 无法受控下降，退化成断电。 */
                for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
                {
                    out_current[ch] = 0;
                }
                break;
            }
            /*
             * ★ 全体一起降，不许「各停各的」。
             *
             * 老的真实缺陷就是逐轴到位逐轴停：一根停住、其余还在降，
             * 车是在【倾斜中】被放下的，而这一步恰恰发生在系统已经出故障之后。
             *
             * 所以判据是【四轴都到停放点】才整体停止出力；只要还有一根没到，
             * 四根就都保持同一个下降电流 —— 牺牲一点停放精度，换取下放过程
             * 全程保持姿态。反正到了停放点就没有下一段动作了。
             */
            {
                bool any_not_parked = false;

                for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
                {
                    if (state->position_ecd[ch] > state->park_ecd[ch])
                    {
                        any_not_parked = true;
                    }
                }

                if (any_not_parked)
                {
                    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
                    {
                        out_current[ch] = -magnitude;
                    }
                    active = true;
                }
                else
                {
                    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
                    {
                        out_current[ch] = 0;
                    }
                }
            }
            break;

        case LIFT_STOP_CUT:
        default:
            /* 断电：不动 out_current 的内容由调用方清零，这里显式写 0。 */
            for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            {
                out_current[ch] = 0;
            }
            active = false;
            break;
    }

    if (out_active != NULL)
    {
        *out_active = active;
    }
}

const char *lift_safety_fault_name(lift_safety_fault_t fault)
{
    switch (fault)
    {
        case LIFT_SAFETY_FAULT_NONE:          return "NONE";
        case LIFT_SAFETY_FAULT_OFFLINE:       return "OFFLINE";
        case LIFT_SAFETY_FAULT_LIMIT:         return "LIMIT";
        case LIFT_SAFETY_FAULT_SYNC:          return "SYNC";
        case LIFT_SAFETY_FAULT_OVERCURRENT:   return "OVERCURRENT";
        case LIFT_SAFETY_FAULT_TIMEOUT:       return "TIMEOUT";
        case LIFT_SAFETY_FAULT_UNSUPPORTED:   return "UNSUPPORTED";
        case LIFT_SAFETY_FAULT_ABORTED:       return "ABORTED";
        case LIFT_SAFETY_FAULT_NOT_CALIBRATED:return "NOT_CALIBRATED";
        default:                              return "?";
    }
}

const char *lift_stop_action_name(lift_stop_action_t action)
{
    switch (action)
    {
        case LIFT_STOP_HOLD:  return "HOLD";
        case LIFT_STOP_LOWER: return "LOWER";
        case LIFT_STOP_CUT:   return "CUT";
        default:              return "?";
    }
}

const char *lift_support_name(lift_support_t support)
{
    switch (support)
    {
        case LIFT_SUPPORT_UNKNOWN:    return "UNKNOWN";
        case LIFT_SUPPORT_GROUND:     return "GROUND";
        case LIFT_SUPPORT_MECHANICAL: return "MECHANICAL";
        case LIFT_SUPPORT_AIRBORNE:   return "AIRBORNE";
        default:                      return "?";
    }
}
