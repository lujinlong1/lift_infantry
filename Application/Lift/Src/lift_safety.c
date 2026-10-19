#include "lift_safety.h"
#include "lift_lock.h"
#include <stddef.h>
#if !defined(__arm__) && !defined(__thumb__)
#include <stdatomic.h>
static atomic_flag owner_lock = ATOMIC_FLAG_INIT;
static _Thread_local uint32_t owner_lock_depth;
uint32_t lift_lock_enter(void)
{
    uint32_t key = owner_lock_depth++;
    if (key == 0U)
        while (atomic_flag_test_and_set_explicit(&owner_lock, memory_order_acquire)) { }
    return key;
}
void lift_lock_exit(uint32_t key)
{
    owner_lock_depth = key;
    if (key == 0U) atomic_flag_clear_explicit(&owner_lock, memory_order_release);
}
#endif

/* All state and lease timestamps use the same lock. Task APIs, not mutexes;
 * the lock also protects the final claim/check + cache write at call sites. */
static lift_owner_t owner_state = LIFT_OWNER_NONE;
static uint32_t owner_tick_ms;
static bool tests_inhibited;
static uint32_t owner_timeout_ms = 50U;
static bool owner_valid(lift_owner_t owner)
{
    return owner >= LIFT_OWNER_CONTROL && owner <= LIFT_OWNER_CAN_TEST;
}
bool lift_owner_claim(lift_owner_t owner, uint32_t now_ms)
{
    uint32_t key = lift_lock_enter();
    bool accepted = owner_valid(owner) && (!tests_inhibited || owner == LIFT_OWNER_CONTROL) &&
        (owner_state == LIFT_OWNER_NONE || owner_state == owner);
    if (accepted) { owner_state = owner; owner_tick_ms = now_ms; }
    lift_lock_exit(key);
    return accepted;
}
bool lift_owner_claim_override(lift_owner_t owner, uint32_t now_ms,
                                lift_owner_t *previous)
{
    uint32_t key = lift_lock_enter();
    bool accepted = !tests_inhibited && (owner == LIFT_OWNER_JOG || owner == LIFT_OWNER_CAN_TEST) &&
        (owner_state == LIFT_OWNER_NONE || owner_state == owner ||
         owner_state == LIFT_OWNER_CONTROL ||
         (owner == LIFT_OWNER_CAN_TEST && owner_state == LIFT_OWNER_JOG));
    if (previous != NULL) *previous = owner_state;
    if (accepted) { owner_state = owner; owner_tick_ms = now_ms; }
    lift_lock_exit(key);
    return accepted;
}
void lift_owner_release(lift_owner_t owner)
{
    uint32_t key = lift_lock_enter();
    if (owner_valid(owner) && owner_state == owner) owner_state = LIFT_OWNER_NONE;
    lift_lock_exit(key);
}
lift_owner_t lift_owner_current(void)
{
    uint32_t key = lift_lock_enter();
    lift_owner_t result = owner_state;
    lift_lock_exit(key);
    return result;
}
bool lift_owner_may_write(lift_owner_t want)
{
    uint32_t key = lift_lock_enter();
    bool result = owner_valid(want) && (!tests_inhibited || want == LIFT_OWNER_CONTROL) &&
        (owner_state == LIFT_OWNER_NONE || owner_state == want);
    lift_lock_exit(key);
    return result; /* advisory only: never use as check-then-write authorization */
}
void lift_owner_set_timeout_ms(uint32_t timeout_ms)
{
    uint32_t key = lift_lock_enter();
    owner_timeout_ms = timeout_ms;
    lift_lock_exit(key);
}
uint32_t lift_owner_timeout_ms(void)
{
    uint32_t key = lift_lock_enter();
    uint32_t result = owner_timeout_ms;
    lift_lock_exit(key);
    return result;
}
void lift_owner_tick(uint32_t now_ms)
{
    uint32_t key = lift_lock_enter();
    uint32_t age = now_ms - owner_tick_ms;
    /* A caller sampled now before being preempted by a newer lease refresh.
     * Do not interpret that stale timestamp as an almost-2^32-ms timeout. */
    if (owner_state != LIFT_OWNER_NONE && owner_timeout_ms != 0U &&
        age < 0x80000000UL && age > owner_timeout_ms)
        owner_state = LIFT_OWNER_NONE;
    lift_lock_exit(key);
}
void lift_owner_inhibit_tests(bool inhibited)
{
    const uint32_t key = lift_lock_enter();
    tests_inhibited = inhibited;
    if (inhibited && (owner_state == LIFT_OWNER_JOG || owner_state == LIFT_OWNER_CAN_TEST))
        owner_state = LIFT_OWNER_NONE;
    lift_lock_exit(key);
}
bool lift_owner_tests_inhibited(void)
{
    const uint32_t key = lift_lock_enter();
    const bool result = tests_inhibited;
    lift_lock_exit(key);
    return result;
}
void lift_owner_reset(void)
{
    uint32_t key = lift_lock_enter();
    owner_state = LIFT_OWNER_NONE;
    owner_tick_ms = 0U;
    tests_inhibited = false;
    lift_lock_exit(key);
}

/* Stop policy below is unchanged. */
lift_stop_action_t lift_safety_policy_for(
    const lift_safety_context_t *context)
{
    if (context == NULL)
    {
        /* 连上下文都没有：最保守 —— 既不知道支撑也不知道零点，只能不出力。 */
        return LIFT_STOP_CUT;
    }

    /* Legacy declared support selects zero output; physical support is unverified. */
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

    /* Preserve legacy moving/stationary request selection, not a safety proof. */
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
        if (out_active != NULL) *out_active = false;
        if (out_current != NULL)
            for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++) out_current[ch] = 0;
        return;
    }

    /* 出力 = min(标定保持电流, 上限)；<= 0 时 HOLD/LOWER 退化成 CUT。 */
    magnitude = config->current_limit > 0 ? config->hold_current : 0;
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
                /* 没标定保持电流 → 输出零电流；不代表机构具备安全支撑。 */
                for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
                {
                    out_current[ch] = 0;
                }
                break;
            }
            /* Legacy equal positive currents. No position or support regulation here. */
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
            /* Legacy group park predicate/equal negative currents, retained unchanged.
             * Equal currents do not prove equal travel, stable posture or slow descent.
             * The top executor also enforces each axis' existing soft boundaries. */
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
