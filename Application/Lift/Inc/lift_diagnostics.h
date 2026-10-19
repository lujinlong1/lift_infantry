#ifndef LIFT_DIAGNOSTICS_H
#define LIFT_DIAGNOSTICS_H

/* Observation only. All axis quantities are logical positive extension unless
 * suffixed _raw. Encoder counts and rotor ecd/s; current is C620 command units.
 * No allocation, logging, clock reads or control decisions in this module. */
#include <stdbool.h>
#include <stdint.h>
#define LIFT_DIAG_AXES 4U
#define LIFT_DIAG_HISTORY 101U /* inclusive endpoints: 2000 ms at 20 ms */
#define LIFT_DIAG_SAMPLE_MS 20U
#define LIFT_DIAG_NO_AXIS (-1)

typedef enum {
    LIFT_DIAG_SOURCE_WATCH = 0, LIFT_DIAG_SOURCE_API,
    LIFT_DIAG_SOURCE_SEQUENCE, LIFT_DIAG_SOURCE_HOMING,
    LIFT_DIAG_SOURCE_JOG, LIFT_DIAG_SOURCE_CAN_TEST,
    LIFT_DIAG_SOURCE_REMOTE, LIFT_DIAG_SOURCE_OTHER, LIFT_DIAG_SOURCE_CONTROL
} lift_diag_source_t;
typedef enum {
    LIFT_DIAG_HOMING = 0, LIFT_DIAG_STALL, LIFT_DIAG_SEQUENCE,
    LIFT_DIAG_TOP, LIFT_DIAG_GUARD, LIFT_DIAG_GATE,
    LIFT_DIAG_OWNER, LIFT_DIAG_CAN, LIFT_DIAG_FEEDBACK,
    LIFT_DIAG_LIMIT, LIFT_DIAG_COMMAND_TIMEOUT, LIFT_DIAG_REQUEST_REJECT, LIFT_DIAG_REASON_COUNT
} lift_diag_reason_t;
#define LIFT_DIAG_BIT(reason) (1UL << (reason))
typedef enum {
    LIFT_DIAG_CAN_SEND_FAILURE = 1U,
    LIFT_DIAG_CAN_CACHE_EXPIRED = 2U,
    LIFT_DIAG_CAN_TX_SILENT = 4U
} lift_diag_can_fault_t;

/* Cache and TX have their own generations. A CAN acceptance is NOT delivery
 * or acknowledgement by a motor. DJI feedback has no command sequence ACK. */
typedef struct {
    uint32_t cache_generation, cache_cycle_id, cache_tick_ms;
    int16_t cache_raw[4];
    uint8_t cache_source;
    uint32_t tx_count, tx_cache_generation, tx_cycle_id, tx_tick_ms;
    int16_t packed_raw[4];
    uint32_t accepted_count, failed_count, consecutive_failures;
    uint32_t last_success_ms;
    uint32_t last_failure_ms, last_failure_cycle, last_failure_generation;
    int16_t last_failure_raw[4];
    uint8_t last_failure_result;
    uint8_t send_result; /* can_tx_service_status_t; 255 = never attempted */
    bool ever_accepted, timed_out, test_override;
} lift_diag_transport_t;
typedef struct {
    int32_t target_ecd, position_ecd, position_raw, zero_raw;
    float reference_ecd, target_speed_ecd_s, actual_speed_ecd_s;
    float pid_current, feedforward_current, demand_current, limit_current;
    float current_limit;
    int16_t guard_current, protected_current, submitted_raw, feedback_raw;
    int32_t feedback_current; /* logical, int32 handles -32768 inversion */
    int16_t speed_raw_rpm;
    int8_t direction;
    uint8_t homing_state, homing_fault;
    bool online, feedback_seen, limit_blocked;
    uint32_t feedback_tick_ms, feedback_age_ms, feedback_count;
} lift_diag_axis_t;
typedef enum {
    LIFT_REJECT_NONE = 0, LIFT_REJECT_FAULT, LIFT_REJECT_BUSY,
    LIFT_REJECT_OFFLINE, LIFT_REJECT_MOVING, LIFT_REJECT_DATUM,
    LIFT_REJECT_LAYOUT, LIFT_REJECT_TRAVEL, LIFT_REJECT_RECOVERY
} lift_request_reject_t;
typedef enum {
    LIFT_STOP_REASON_NONE = 0, LIFT_STOP_REASON_DISABLED,
    LIFT_STOP_REASON_TIMEOUT, LIFT_STOP_REASON_OFFLINE,
    LIFT_STOP_REASON_HOMING, LIFT_STOP_REASON_GUARD,
    LIFT_STOP_REASON_SEQUENCE, LIFT_STOP_REASON_ABORT,
    LIFT_STOP_REASON_STALL, LIFT_STOP_REASON_BOUNDARY,
    LIFT_STOP_REASON_DATUM, LIFT_STOP_REASON_STATE
} lift_stop_reason_t;
typedef struct {
    uint32_t reject_cycle_id, recovery_blocked;
    uint8_t reject, reason, requested_action, datum_kind[4];
    int8_t reject_axis;
    bool output_configured, physically_verified, output_zero, travel_confirmed;
    bool stop_inhibited; /* do not resume a fault-stop current after feedback returns */
    bool tests_inhibited; /* bench motion revoked by main fault */
    /* output_zero describes submitted intent after gates. Cache/packet evidence
     * stays in transport; zero current never means proven physical standstill. */
} lift_diag_safety_t;
typedef struct {
    uint32_t cycle_id, tick_ms, parameter_version;
    uint32_t feedback_observed_ms, transport_observed_ms;
    uint32_t command_tick_ms, command_age_ms, decision_feedback_cycle_id;
    uint32_t jog_owner_lost_count, jog_owner_event_ms, owner_event_ms, owner_event_cycle_id;
    uint8_t owner_requester; /* lift_owner_t; 0: no event */
    float dt_s;
    int32_t command_axis_target[4], command_modal_target[3];
    int32_t sequence_target_mm_before[4];
    bool command_axis_valid;
    uint32_t reasons;
    int8_t reason_axis[LIFT_DIAG_REASON_COUNT]; /* -1: group/unknown */
    uint8_t source, state, top_fault, stage_before, stage, sequence_fault;
    uint8_t guard_state, guard_fault, gate, stop_action;
    uint8_t can_fault_bits; /* bit0 send failure, bit1 cache expired, bit2 TX silent */
    int8_t guard_fault_axis, sequence_fault_axis;
    const char *sequence_detail; /* static string from existing state machine */
    bool command_enabled, command_fresh, controller_ran, guard_ran;
    bool cache_written, owner_conflict, safety_limited;
    bool transport_valid;
    uint8_t retracted_mask;
    lift_diag_axis_t axis[4];
    lift_diag_transport_t transport;
    lift_diag_safety_t safety;
} lift_diag_frame_t;
typedef struct {
    volatile uint32_t sequence; /* odd: writing; even: stable, use snapshot API */
    lift_diag_frame_t frame;
    uint32_t event_count[LIFT_DIAG_REASON_COUNT]; /* rising edges, saturating */
} lift_diag_watch_t;
typedef struct {
    volatile uint32_t sequence;
    bool frozen, post_valid, before_valid;
    uint16_t head, count; /* next write; oldest=(head+101-count)%101 */
    uint32_t trigger_reasons, trigger_cycle_id, latch_count;
    lift_diag_frame_t before; /* previous completed control cycle, full rate */
    lift_diag_frame_t trigger; /* same cycle, includes pre/post protection */
    lift_diag_frame_t post;    /* next completed control cycle */
    lift_diag_frame_t history[LIFT_DIAG_HISTORY];
} lift_diag_blackbox_t;
extern volatile lift_diag_watch_t g_lift_diag;
extern volatile lift_diag_blackbox_t g_lift_blackbox;
extern volatile lift_diag_transport_t g_lift_diag_transport;
extern volatile uint32_t g_lift_diag_transport_sequence;
extern volatile bool g_lift_diag_rearm; /* consume in control task; no control reset */

typedef enum {
    LIFT_DIAG_READ_HISTORY = 0, LIFT_DIAG_READ_TRIGGER, LIFT_DIAG_READ_POST, LIFT_DIAG_READ_BEFORE
} lift_diag_read_t;
bool lift_diag_get_record(lift_diag_read_t kind, uint16_t chronological_index,
                          lift_diag_frame_t *frame);

void lift_diag_init(void); /* startup only, before scheduler */
void lift_diag_request_rearm(void); /* task callers use atomic request; Watch flag retained */
void lift_diag_publish(const lift_diag_frame_t *frame); /* control task only */
/* Shared Watch structures are read-only at runtime. Task readers use these
 * bounded APIs; startup init must not run concurrently with any task reader. */
bool lift_diag_get_snapshot(lift_diag_frame_t *frame); /* <=3 attempts, no spin */
bool lift_diag_get_transport(lift_diag_transport_t *transport);
/* Hooks serialize their small shared transport update internally. */
void lift_diag_cache(const int16_t raw[4], uint32_t cycle, uint32_t tick_ms,
                     uint8_t source);
void lift_diag_tx(const int16_t packed[4], uint32_t generation, uint32_t cycle,
                  uint32_t tick_ms, uint8_t result, bool timeout, bool override);
#endif
