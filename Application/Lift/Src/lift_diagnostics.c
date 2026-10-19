#include "lift_diagnostics.h"
#include "lift_lock.h"
#include <stddef.h>
#include <string.h>

/* Atomics make overlapping readers/writers defined, not merely detectable.
 * Aligned word copies compile to LDR/STR, with no masked frame-copy interval.
 * Sequence barriers reject changes during a bounded read. Single publisher. */
typedef uint32_t diag_word_t __attribute__((may_alias));
#define LOAD(x) __atomic_load_n(&(x), __ATOMIC_RELAXED)
#define STORE(x, v) __atomic_store_n(&(x), (v), __ATOMIC_RELAXED)
#define SEQ(x) __atomic_load_n(&(x), __ATOMIC_SEQ_CST)
#define ADVANCE(x) ((void)__atomic_add_fetch(&(x), 1U, __ATOMIC_SEQ_CST))
#define DIAG_BARRIER() __atomic_thread_fence(__ATOMIC_SEQ_CST)
_Static_assert(sizeof(lift_diag_frame_t) % 4U == 0U, "diagnostic frame alignment");
_Static_assert(sizeof(lift_diag_transport_t) % 4U == 0U, "transport alignment");
static void diag_copy(volatile void *destination, const volatile void *source, size_t size)
{
    volatile diag_word_t *dst = destination;
    const volatile diag_word_t *src = source;
    size_t i;
    for (i = 0U; i < size / sizeof(diag_word_t); ++i)
        __atomic_store_n(&dst[i], __atomic_load_n(&src[i], __ATOMIC_RELAXED), __ATOMIC_RELAXED);
}
volatile lift_diag_watch_t g_lift_diag;
volatile lift_diag_blackbox_t g_lift_blackbox;
volatile lift_diag_transport_t g_lift_diag_transport;
volatile uint32_t g_lift_diag_transport_sequence;
volatile bool g_lift_diag_rearm;
static uint32_t previous_reasons, sample_ms;
static bool sampled;
static void increment(volatile uint32_t *counter)
{
    uint32_t value = LOAD(*counter);
    if (value != UINT32_MAX) STORE(*counter, value + 1U);
}
void lift_diag_init(void)
{
    /* Startup only, before readers/writers exist. Never a runtime reset API. */
    memset((void *)&g_lift_diag, 0, sizeof(g_lift_diag));
    memset((void *)&g_lift_blackbox, 0, sizeof(g_lift_blackbox));
    memset((void *)&g_lift_diag_transport, 0, sizeof(g_lift_diag_transport));
    g_lift_diag_transport.send_result = 255U;
    g_lift_diag_transport_sequence = 0U;
    g_lift_diag_rearm = false;
    previous_reasons = sample_ms = 0U;
    sampled = false;
}
void lift_diag_request_rearm(void)
{
    __atomic_store_n(&g_lift_diag_rearm, true, __ATOMIC_RELEASE);
}
bool lift_diag_get_snapshot(lift_diag_frame_t *frame)
{
    uint32_t attempt;
    if (!frame) return false;
    for (attempt = 0; attempt < 3U; ++attempt) {
        uint32_t seq = SEQ(g_lift_diag.sequence);
        if (seq & 1U) continue;
        DIAG_BARRIER();
        diag_copy(frame, &g_lift_diag.frame, sizeof(*frame));
        DIAG_BARRIER();
        if (seq == SEQ(g_lift_diag.sequence)) return frame->cycle_id != 0U;
    }
    return false;
}
bool lift_diag_get_transport(lift_diag_transport_t *transport)
{
    uint32_t attempt;
    if (!transport) return false;
    for (attempt = 0; attempt < 3U; ++attempt) {
        uint32_t seq = SEQ(g_lift_diag_transport_sequence);
        if (seq & 1U) continue;
        DIAG_BARRIER();
        diag_copy(transport, &g_lift_diag_transport, sizeof(*transport));
        DIAG_BARRIER();
        if (seq == SEQ(g_lift_diag_transport_sequence)) return true;
    }
    return false;
}
bool lift_diag_get_record(lift_diag_read_t kind, uint16_t index, lift_diag_frame_t *frame)
{
    uint32_t attempt;
    if (!frame) return false;
    for (attempt = 0; attempt < 3U; ++attempt) {
        uint32_t seq = SEQ(g_lift_blackbox.sequence);
        uint16_t head, count;
        if (seq & 1U) continue;
        DIAG_BARRIER();
        head = LOAD(g_lift_blackbox.head);
        count = LOAD(g_lift_blackbox.count);
        if (kind == LIFT_DIAG_READ_HISTORY) {
            if (index >= count || head >= LIFT_DIAG_HISTORY || count > LIFT_DIAG_HISTORY) return false;
            diag_copy(frame, &g_lift_blackbox.history[(head + LIFT_DIAG_HISTORY - count + index) % LIFT_DIAG_HISTORY], sizeof(*frame));
        } else if (kind == LIFT_DIAG_READ_TRIGGER) {
            if (!LOAD(g_lift_blackbox.frozen)) return false;
            diag_copy(frame, &g_lift_blackbox.trigger, sizeof(*frame));
        } else if (kind == LIFT_DIAG_READ_POST) {
            if (!LOAD(g_lift_blackbox.post_valid)) return false;
            diag_copy(frame, &g_lift_blackbox.post, sizeof(*frame));
        } else if (kind == LIFT_DIAG_READ_BEFORE) {
            if (!LOAD(g_lift_blackbox.before_valid)) return false;
            diag_copy(frame, &g_lift_blackbox.before, sizeof(*frame));
        } else return false;
        DIAG_BARRIER();
        if (seq == SEQ(g_lift_blackbox.sequence)) return true;
    }
    return false;
}
void lift_diag_cache(const int16_t raw[4], uint32_t cycle, uint32_t tick_ms,
                     uint8_t source)
{
    uint32_t ch;
    const uint32_t key = lift_lock_enter();
    ADVANCE(g_lift_diag_transport_sequence);
    STORE(g_lift_diag_transport.cache_generation, LOAD(g_lift_diag_transport.cache_generation) + 1U);
    STORE(g_lift_diag_transport.cache_cycle_id, cycle);
    STORE(g_lift_diag_transport.cache_tick_ms, tick_ms);
    STORE(g_lift_diag_transport.cache_source, source);
    for (ch = 0; ch < 4U; ++ch) STORE(g_lift_diag_transport.cache_raw[ch], raw[ch]);
    ADVANCE(g_lift_diag_transport_sequence);
    lift_lock_exit(key);
}
void lift_diag_tx(const int16_t packed[4], uint32_t generation, uint32_t cycle,
                  uint32_t tick_ms, uint8_t result, bool timeout, bool override)
{
    uint32_t ch;
    const uint32_t key = lift_lock_enter();
    ADVANCE(g_lift_diag_transport_sequence);
    increment(&g_lift_diag_transport.tx_count);
    STORE(g_lift_diag_transport.tx_cache_generation, generation);
    STORE(g_lift_diag_transport.tx_cycle_id, cycle);
    STORE(g_lift_diag_transport.tx_tick_ms, tick_ms);
    STORE(g_lift_diag_transport.send_result, result);
    STORE(g_lift_diag_transport.timed_out, timeout);
    STORE(g_lift_diag_transport.test_override, override);
    for (ch = 0; ch < 4U; ++ch) STORE(g_lift_diag_transport.packed_raw[ch], packed[ch]);
    if (result == 0U) {
        increment(&g_lift_diag_transport.accepted_count);
        STORE(g_lift_diag_transport.consecutive_failures, 0U);
        STORE(g_lift_diag_transport.last_success_ms, tick_ms);
        STORE(g_lift_diag_transport.ever_accepted, true);
    } else {
        STORE(g_lift_diag_transport.last_failure_ms, tick_ms);
        STORE(g_lift_diag_transport.last_failure_cycle, cycle);
        STORE(g_lift_diag_transport.last_failure_generation, generation);
        STORE(g_lift_diag_transport.last_failure_result, result);
        for (ch = 0; ch < 4U; ++ch) STORE(g_lift_diag_transport.last_failure_raw[ch], packed[ch]);
        increment(&g_lift_diag_transport.failed_count);
        increment(&g_lift_diag_transport.consecutive_failures);
    }
    ADVANCE(g_lift_diag_transport_sequence);
    lift_lock_exit(key);
}
void lift_diag_publish(const lift_diag_frame_t *frame)
{
    uint32_t reason, rising;
    bool rearm;
    if (!frame) return;
    /* Atomic consume: a task request arriving after the exchange stays pending. */
    rearm = __atomic_exchange_n(&g_lift_diag_rearm, false, __ATOMIC_ACQ_REL);
    rising = frame->reasons & ~previous_reasons;
    ADVANCE(g_lift_diag.sequence);
    ADVANCE(g_lift_blackbox.sequence);
    if ((!LOAD(g_lift_blackbox.frozen) || rearm) && frame->reasons != 0U) {
        STORE(g_lift_blackbox.before_valid, LOAD(g_lift_diag.frame.cycle_id) != 0U);
        diag_copy(&g_lift_blackbox.before, &g_lift_diag.frame, sizeof(*frame));
    }
    diag_copy(&g_lift_diag.frame, frame, sizeof(*frame));
    for (reason = 0; reason < LIFT_DIAG_REASON_COUNT; ++reason)
        if (rising & LIFT_DIAG_BIT(reason)) increment(&g_lift_diag.event_count[reason]);
    ADVANCE(g_lift_diag.sequence);
    previous_reasons = frame->reasons;
    if (rearm) {
        STORE(g_lift_blackbox.frozen, false);
        STORE(g_lift_blackbox.post_valid, false);
        STORE(g_lift_blackbox.head, 0U);
        STORE(g_lift_blackbox.count, 0U);
        if (frame->reasons == 0U) STORE(g_lift_blackbox.before_valid, false);
        sampled = false;
    }
    if (LOAD(g_lift_blackbox.frozen)) {
        if (!LOAD(g_lift_blackbox.post_valid) && frame->cycle_id != LOAD(g_lift_blackbox.trigger_cycle_id)) {
            diag_copy(&g_lift_blackbox.post, frame, sizeof(*frame));
            STORE(g_lift_blackbox.post_valid, true);
        }
    } else if (frame->reasons != 0U) {
        diag_copy(&g_lift_blackbox.trigger, frame, sizeof(*frame));
        STORE(g_lift_blackbox.trigger_reasons, frame->reasons);
        STORE(g_lift_blackbox.trigger_cycle_id, frame->cycle_id);
        increment(&g_lift_blackbox.latch_count);
        STORE(g_lift_blackbox.frozen, true);
    } else if (!sampled || (uint32_t)(frame->tick_ms - sample_ms) >= LIFT_DIAG_SAMPLE_MS) {
        uint16_t head = LOAD(g_lift_blackbox.head);
        diag_copy(&g_lift_blackbox.history[head], frame, sizeof(*frame));
        STORE(g_lift_blackbox.head, (uint16_t)((head + 1U) % LIFT_DIAG_HISTORY));
        if (LOAD(g_lift_blackbox.count) < LIFT_DIAG_HISTORY)
            STORE(g_lift_blackbox.count, (uint16_t)(LOAD(g_lift_blackbox.count) + 1U));
        sample_ms = frame->tick_ms;
        sampled = true;
    }
    ADVANCE(g_lift_blackbox.sequence);
}
