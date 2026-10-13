#ifndef LIFT_CONTROL_TASK_H
#define LIFT_CONTROL_TASK_H

#include "lift_seq.h"
#include "lift_top.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 抬升控制任务：固件里唯一驱动抬升模块链的地方。
 *
 * 分层（见 讨论池/抬升模块架构.md）：
 *
 *   遥控（remote_lift） ┐
 *                       ├─ 写 g_lift_task_command
 *   lift_seq（上台阶）  ┘
 *        │
 *        ▼
 *   本任务（5 ms）
 *        │  快照反馈 → lift_top_update() → robot_lift_set_currents()
 *        ▼
 *   lift_top ─► 四轴 lift_control ─► lift_guard ─► CAN1 0x200
 *        └─► lift_kinematics（毫米 ↔ 每轴计数，每轴独立零点与系数）
 *
 * 对外只暴露这几样东西：
 *   1. g_lift_task_command：命令面（其它任务/调试器写，本任务读）
 *   2. g_lift_control_debug：标定参数（改完置 apply=1 才生效）
 *   3. g_lift_control_status：只读状态，给 Keil Watch 与遥测用
 *   4. g_lift_sequence / g_lift_seq_status：上台阶流程的状态与人工接口
 *   5. g_remote_lift_debug（在 remote_lift.h）：上台阶流程的启动/中止/确认
 */

/*
 * 运行时命令面。所有 request 位都是【单周期脉冲】：本任务消费后清零，
 * 调用方不需要（也不应该）自己清。
 *
 * 注意 enabled 的语义：它必须被【周期刷新】。设备层对抬升组也有
 * "命令超时归零"（100 ms），但那是兜底；抬升的正确超时行为是
 * lift_guard 的**受控缓降**（ADR-006），不是断力——本机构不自锁。
 */
typedef struct
{
    volatile bool enabled;
    volatile int32_t target_height_ecd;
    volatile int32_t target_roll_ecd;
    volatile int32_t target_pitch_ecd;
    volatile bool homing_request;
    volatile bool clear_homing_request;
    volatile bool reset_request;
    /* 每次命令 API 调用都会刷新；用于受控超时缓降判定。 */
    volatile uint32_t command_update_tick;
    /* ch2/ch3 已在机械最低点。先置 1，再发 homing_request；
     * 随找零请求一起消费清零，仅确认后轴，不影响前轴运动找零。 */
    volatile bool rear_at_minimum_confirmed;
    /* 前轴实际接触承重点后写 bit0=ch0、bit1=ch1（两根都接触写 3）。
     * 单周期请求，消费后清零。不得仅因停转而确认接触。 */
    volatile uint8_t front_contact_confirmed_mask;
} lift_task_command_t;

/*
 * 标定/调试参数。只有 apply 被置 1 时才拷贝进纯逻辑控制器。
 * 位置与电流都是【逻辑坐标】：单轴的原始极性由 axis_direction 处理。
 *
 * 这些值全部来自"待实机标定项"（LIFT-005/007/009/010），默认值是
 * 占位值 + 保守电流，不是调好的参数。
 */
typedef struct
{
    /* 完整生效：重建整条链，并【作废旧零点】（必须重新找零）。 */
    volatile bool apply;
    /*
     * 只重载增益：把 position_kp 与 speed_kp 这一组装进运行中的链，
     * 不动零点、不动 homed —— 调 PID 专用，改完立刻生效、不用重新找零。
     */
    volatile uint8_t apply_gains;
    /*
     * 软零点：把四轴当前位置当作零点、直接进入 READY。
     * 台架调 PID 时跳过真实找零用。⚠️ 无机械基准，限位/堵转判定意义打折，
     * 只能架空调试用；正式跑之前必须重新真实找零。
     */
    volatile uint8_t set_zero_now;
    /*
     * 手动增益开关：
     *   0 = 使用明确的初始 P/PI 参数（需实机验证，不由静态 K 推导）
     *   1 = 用手填的 position_kp/ki/kd 与 speed_kp/ki/kd（调试用）
     * 置 1 后 lift_select_pid_preset() 不再覆盖这些字段。
     */
    volatile bool pid_manual_override;
    /* Shared PID preset unless manually overridden. Per-axis override allows
     * front/rear load tuning; outer loop supports P only. */
    volatile bool axis_pid_override;
    volatile pid_config_t axis_position_pid[4];
    volatile pid_config_t axis_speed_pid[4];
    volatile float axis_counts_per_mm[4];
    volatile bool axis_feedforward_override;
    volatile float axis_load_current[4];
    volatile float target_rate_mm_s;
    volatile float target_accel_mm_s2;
    volatile float sync_slow_mm;
    volatile float sync_stop_mm;
    volatile int8_t axis_direction[4];

    volatile int16_t homing_seek_current;
    volatile int16_t homing_contact_current; /* 前轴接触找零：正=伸出 */
    volatile int16_t homing_backoff_current;
    volatile int16_t homing_preposition_current;
    volatile int16_t homing_current_limit;
    volatile int16_t homing_stall_speed_rpm;
    volatile int32_t homing_preposition_ecd;
    volatile uint32_t homing_preposition_timeout_ms;
    volatile int32_t homing_seek_min_travel_ecd;
    volatile uint32_t homing_stall_hold_ms;
    volatile int32_t homing_backoff_ecd;
    volatile uint32_t homing_seek_timeout_ms;
    volatile uint32_t homing_backoff_timeout_ms;

    volatile int32_t position_min_ecd[4];
    volatile int32_t position_max_ecd[4];
    volatile int16_t descent_current[4];
    /* 缓降停止位置（单轴计数，已找零坐标）。见 lift_guard.h 里的说明。 */
    volatile int32_t descent_target_ecd[4];
    volatile int16_t guard_current_limit;
    volatile int16_t guard_stall_speed_rpm;
    volatile uint32_t guard_stall_ms;
    volatile uint32_t guard_timeout_ms;

    volatile float position_kp;
    volatile float position_ki;
    volatile float position_kd;
    volatile float position_integral_limit;
    volatile float position_output_limit;
    volatile float speed_kp;
    volatile float speed_ki;
    volatile float speed_kd;
    volatile float speed_integral_limit;
    volatile float speed_output_limit;
    volatile int32_t ecd_per_rev;
    volatile float gear_ratio;
    volatile bool speed_is_rotor;
    volatile int32_t max_target_step_ecd;
    volatile float period_s;
    volatile int32_t settle_tolerance_ecd;
    volatile int32_t limit_margin_ecd;
    /*
     * 重力/保持前馈（单轴电流量纲，填正数 = 抬升方向）。
     * 逐轴高过 feedforward_min_position_ecd 时施加；axis_feedforward_override=1 用四轴独立值。
     */
    volatile float gravity_feedforward_current;
    volatile int32_t feedforward_min_position_ecd;
    volatile float actuator_gain_ecd_s_per_current;

    /*
     * ---- 平滑链（2026-10-11 从台架模块移植，量纲都是【模态量】）--------
     * max_target_step / approach_* 为旧模态兼容字段，主链不使用。
     * 新主链速度/加速度用 target_rate_mm_s / target_accel_mm_s2。
     */
    volatile int32_t approach_zone_ecd;         /* 减速区起点（模态计数） */
    volatile int32_t approach_min_speed_ecd_s;  /* 减速区最低速度（模态 ecd/s） */
    volatile int32_t hold_speed_ecd_s;          /* 判"停住"（新链为单轴 ecd/s） */
    volatile int32_t current_ramp_per_s;        /* 电流 di/dt（新链逐轴电流/秒） */
    volatile int32_t current_limit_start;       /* 逐级开放起点（电流） */
    volatile int32_t current_limit_ramp_per_s;  /* 逐级开放速度（电流/秒） */
    volatile int32_t speed_filter_tau_ms;       /* 速度反馈低通（ms，0 = 关） */

    /* ---- 分阶段上台阶流程（见 lift_seq.h）------------------------------
     *
     * 全部在这里，是因为流程的人机接口只有"几个开关 + 一句提示"：
     * 放进 Watch 就能操作与判读，不需要另开一套协议。
     */
    /*
     * ⚠️ **前后轮组布局确认**。默认 false。
     *
     * 2026-10-13 用户确认：ch0/ch1 为前辅助轮，ch2/ch3 为后麦轮。
     * 更换接线或机械布局后仍需重新核对；本开关保留为流程门禁。
     * 收错一组 = 把还在承担重量的支撑收掉 = 翻车。
     * 所以流程在 PRECHECK 会因为这个开关是 false 而**拒绝启动**。
     *
     * 确认方法（2 分钟，架空或垫住车做）：单轴给一点抬升力，
     * 看是车头还是车尾在动。确认后把它置 1。
     */
    volatile bool sequence_layout_confirmed;
    /* 底盘/操作手声明"这一段前进已经到位"。 */
    volatile bool sequence_drive_done;
    /* 接触开关输入（现在没有传感器，恒 false；接线后直接接上）。 */
    volatile bool sequence_contact_front;
    volatile bool sequence_contact_rear;
    /* 单周期请求脉冲（本任务消费后清零）。 */
    volatile bool sequence_start;
    volatile bool sequence_abort;
    volatile bool sequence_confirm;
    volatile bool sequence_release;   /* 人力解除：已确认机械支撑 */
    volatile bool sequence_reset;
    /* One-shot lift-only test. Default height is 50 mm;
     * test_height_mm is adjustable in Watch (9..200 mm). Sets height/8/2 mm,
     * confirmed front/rear layout, and clears stale advance requests.
     * Only the startup "already outside soft limits" fault is auto-reset.
     * Result: 0 idle, 1 accepted, 2 not homed, 3 busy, 4 fault,
     * 5 bench owns motors, 6 invalid test height, 7 datum moving/offline.
     * Normal sequence prechecks and controller travel margins still apply.
     * Reaches TEST_HOLD (16), not chassis APPROACH_STEP. */
    volatile bool test_start;
    volatile uint8_t test_start_result;
    volatile int32_t test_height_mm;
    volatile bool test_pause;  /* pulse: freeze measured pose, keep powered */
    volatile bool test_resume; /* pulse: resume lift or descent */
    volatile bool test_lower;  /* pulse: descend; all axes near 2 mm -> latched zero current */
    volatile bool test_auto_zero; /* Default true: use stationary start pose if unhomed. */
    volatile bool test_extend_travel; /* provisional 200mm displacement compensation */
} lift_control_debug_t;

/* 平铺的 volatile 状态，专供 Keil Watch 与调试记录使用。 */

/*
 * 本周期是谁把真实下发压成 0 的。顺序即优先级（与 lift_control_task.c 一致）。
 *
 * 为什么需要它：output_current 是"模块链算出来的电流"，
 * applied_current 才是"真的进了 CAN1 0x200 的电流"。两者不一致时，
 * 没有这个字段就只能靠猜——而"显示有电流、实际没出力"在调试期最容易误导人。
 */
typedef enum
{
    LIFT_OUTPUT_GATE_NONE = 0,   /* 没有门禁，applied = output */
    LIFT_OUTPUT_GATE_NO_CONFIG,  /* 标定参数没生效 */
    LIFT_OUTPUT_GATE_OFFLINE,    /* 有抬升轴掉线（四轴必须全在线） */
    LIFT_OUTPUT_GATE_JOG,        /* 诊断点动模块在占用 */
    LIFT_OUTPUT_GATE_CAN_TEST    /* CAN 点动测试在占用 */
} lift_output_gate_t;

typedef struct
{
    volatile bool running;
    volatile bool configuration_applied;
    volatile bool homed;
    volatile bool at_target;
    volatile bool safety_limited;         /* 目标被压回安全起点（未找零） */
    volatile bool output_gated;           /* 本周期真实下发被门禁压成 0 */
    /*
     * output_gate 取值（见 lift_output_gate_t）：
     *   0 无门禁 / 1 参数未生效 / 2 抬升轴掉线 / 3 诊断点动占用 / 4 CAN 点动测试占用
     * 有它才能一眼分清"控制器算出来是 0"和"算出来有值但被闸掉了"。
     */
    volatile uint8_t output_gate;
    volatile uint8_t state;
    volatile uint8_t fault;
    volatile uint8_t homing_state[4];
    volatile uint8_t homing_fault[4];
    volatile uint8_t guard_state;
    volatile uint8_t guard_fault;
    volatile bool online[4];
    volatile bool command_fresh;
    volatile bool configuration_rejected;
    volatile bool limit_blocked[4];
    /*
     * 故障停机动作的真实状态（见 lift_safety.h）：
     *   stop_action   0 = HOLD / 1 = LOWER / 2 = CUT
     *   stop_active   动作还没做完
     *   stop_capable  **保持电流已标定**，HOLD/LOWER 不会退化成断电
     *
     * ⚠️ stop_capable == 0 意味着：现在任何故障都会导致四轴断电。
     *    本机构不自锁，那就是掉车。**这是能力缺口，不是可以忽略的提示。**
     */
    volatile uint8_t stop_action;
    volatile bool stop_active;
    volatile bool stop_capable;
    /*
     * 运动学标定是否完整（系数 + 每轴行程）。见 lift_kinematics.h。
     * false 时"毫米"这个单位在固件里没有意义，绝对值动作应当被拒绝。
     */
    volatile bool kinematics_ready;
    volatile int32_t counts_per_mm_x1000[4];   /* 每轴系数 × 1000 */
    volatile int32_t travel_max_mm[4];         /* 每轴有效行程（mm） */
    /*
     * 四轴一致性判据（**毫米**，由 lift_kinematics 每轴独立换算后算出）。
     *   sync_error_mm    四轴高度极差（最高那根减最低那根）
     *   deviation_max_mm 单轴相对"其余轴参考高度"的最大偏差
     *
     * ⚠️ 这两个量在【上台阶过程中本来就会很大】：前组收到 40 mm、
     *    后组还撑在 200 mm，极差 160 mm 是设计状态。
     *    所以它们只能用来判"收敛后的姿态"，不能拿来判过程中的动作 ——
     *    lift_seq 里就是按这个原则分的（见 lift_seq.c 的 seq_tilt_unrecovered）。
     */
    volatile int32_t sync_error_mm;
    volatile int32_t deviation_max_mm;
    volatile int32_t raw_position_ecd[4];
    volatile int32_t zero_ecd[4];
    volatile int32_t homed_position_ecd[4];
    volatile int16_t speed_rpm[4];
    volatile uint8_t temperature[4];
    /*
     * 模块链算出的电流（命令层）。想确认"到底发出去多少"要看
     * applied_current[]，两者只有在 output_gated = 0 时才相同。
     */
    volatile int16_t output_current[4];
    /* 真实交给设备层、最终进 CAN1 0x200 的电流，带符号。 */
    volatile int16_t applied_current[4];
    volatile int32_t actual_height;
    volatile int32_t actual_roll;
    volatile int32_t actual_pitch;
    volatile int32_t actual_warp;
    volatile int32_t actual_speed_height;
    volatile int32_t actual_speed_roll;
    volatile int32_t actual_speed_pitch;
    volatile int32_t actual_speed_warp;
    volatile int32_t target_height_ecd;
    volatile int32_t target_roll_ecd;
    volatile int32_t target_pitch_ecd;
    /*
     * 行程软限位的实际区间（已找零坐标，含余量）。发布出来是给命令来源
     * （遥控映射）用的：来源必须知道"能推到哪"，而不是自己再猜一套数字。
     *
     * ⚠️ 单位说明（2026-10-11 修正）：
     *   limit_height_*_ecd[ch]      是【单轴】行程，模态限幅内部用；
     *   command_height_*_ecd        是【模态高度】= 四轴之和 = 4 × 单轴，
     *                               命令面的 target_height_ecd 与
     *                               actual_height 都是这个量纲。
     *   两者以前混用：命令面（遥控/安全夹取）拿单轴区间去夹模态目标，
     *   结果可达高度只有配置行程的 1/4。命令来源一律用 command_height_*。
     */
    volatile int32_t limit_height_min_ecd[4];
    volatile int32_t limit_height_max_ecd[4];
    volatile int32_t command_height_min_ecd;
    volatile int32_t command_height_max_ecd;
    /* 旧模态兼容读数；新链看 axis_sync_scale / axis_* 数组。 */
    volatile float limit_scale;
    volatile bool limit_active;
    /* 变换后的参考/速度仅供诊断；control_effort_height 在轴链为 0。 */
    volatile int32_t ramped_height_ecd;
    volatile int32_t target_speed_height_ecd_s;
    volatile int32_t control_effort_height;
    volatile int32_t error_height_ecd;
    /*
     * 平滑链的实时状态（按模态：0 = 高度 / 1 = 横滚 / 2 = 俯仰）：
     *   mode_hold_state          0 = APPROACH / 1 = BRAKE / 2 = HOLD
     *   mode_limit_open_current  本周期逐级开放后的电流上限
     * "到位了为什么还在动/还有电流"先看这两个。
     */
    volatile uint8_t mode_hold_state[3];
    volatile int32_t mode_limit_open_current[3];
    volatile uint32_t command_age_ms;
    volatile uint32_t update_count;
    volatile uint32_t parameter_apply_count;
    volatile uint32_t safety_refresh_count;   /* 目标被强制压回的周期数 */
    /* Shared effective preset/manual gains. speed_loop_kp_max is deprecated
     * and zero: the static actuator K is not a physical stability bound.
     * Per-axis override gains are read from lift_top.config.control.axis_pid. */
    volatile float speed_loop_kp;
    volatile float speed_loop_kp_max;
    volatile float position_loop_kp;
    volatile bool axis_control_active;
    volatile float cycle_dt_s;
    volatile int32_t axis_target_ecd[4];
    volatile float axis_reference_ecd[4];
    volatile float axis_position_error[4];
    volatile float axis_target_speed_ecd_s[4];
    volatile float axis_speed_ecd_s[4];
    volatile float axis_filtered_speed_ecd_s[4];
    volatile float axis_speed_error[4];
    volatile float axis_position_pid_output[4];
    volatile float axis_speed_pid_output[4];
    volatile float axis_feedforward_current[4];
    volatile float axis_final_current[4]; /* logical applied, not requested */
    volatile int16_t axis_feedback_current[4]; /* logical C620 measured */
    volatile float axis_speed_integral[4];
    volatile float axis_limit_now[4];
    volatile float axis_current_lower[4];
    volatile float axis_current_upper[4];
    volatile float axis_current_demand[4];
    volatile float axis_sync_error_mm[4];
    volatile uint8_t axis_hold_state[4];
    volatile float axis_sync_scale;
    volatile float axis_sync_spread_mm;
} lift_task_status_t;

/*
 * 分阶段上台阶流程的状态（见 lift_seq.h）。挑出操作手与调试最需要的几项
 * 平铺发布，方便 Keil Watch 直接看，不用展开嵌套结构。
 */
typedef struct
{
    volatile uint8_t seq_state;          /* lift_seq_state_t */
    volatile uint8_t seq_fault;          /* lift_safety_fault_t */
    volatile uint8_t seq_stop_action;    /* lift_stop_action_t */
    volatile uint8_t seq_support;        /* lift_support_t */
    volatile bool    seq_active;         /* 流程正在控制抬升 */
    volatile bool    seq_waiting_confirm;
    volatile bool    seq_drive_request;  /* 该往前开了（人/底盘执行） */
    volatile bool    seq_released;       /* 已人工解除（确认有机械支撑） */
    volatile uint32_t seq_confirm_count;
    volatile uint32_t seq_elapsed_ms;
    volatile int32_t  seq_axis_target_mm[4];
    volatile int32_t  seq_hold_mm;
    /*
     * 需要人做什么 / 故障明细。**静态字符串**（编译期常量），
     * 不做动态分配、也不指向栈上变量。
     */
    volatile const char *seq_operator_action;
    volatile const char *seq_fault_detail;
    volatile int32_t     seq_fault_axis;   /* -1 = 与单轴无关 */
    volatile int32_t test_stall_axis; /* stalled test motion -> powered pause */
} lift_seq_pub_t;

/* Latest sequence fault, captured before stop handling changes the outputs.
 * Survives sequence/top resets; overwritten only by a new fault or task init.
 * valid is published last. Currents/target_ecd are the preceding control cycle;
 * position, raw feedback current and speed come from this fault cycle. */
typedef struct
{
    volatile bool valid;
    volatile uint32_t count;
    volatile uint32_t tick_ms;
    volatile uint32_t stage_elapsed_ms;
    volatile uint8_t stage;
    volatile const char *stage_name;
    volatile uint8_t fault;
    volatile const char *detail;
    volatile int32_t fault_axis;
    volatile uint8_t guard_state;
    volatile uint8_t guard_fault;
    volatile uint8_t top_fault;
    volatile bool command_fresh;
    volatile int32_t sync_error_mm;
    volatile int32_t deviation_max_mm;
    volatile int32_t position_ecd[4];
    volatile int32_t target_ecd[4];
    volatile int32_t target_mm[4];
    volatile float applied_current[4]; /* logical direction */
    volatile int16_t feedback_current[4]; /* logical direction */
    volatile int16_t speed_rpm[4]; /* raw motor direction */
    volatile bool online[4];
} lift_fault_snapshot_t;

extern lift_task_command_t g_lift_task_command;
extern lift_control_debug_t g_lift_control_debug;
extern lift_task_status_t g_lift_control_status;
extern lift_seq_t g_lift_sequence;
extern lift_seq_pub_t g_lift_seq_status;
extern lift_fault_snapshot_t g_lift_fault_snapshot;

/* 初始化命令面与标定参数的默认值（不产生任何电流）。 */
void lift_control_task_init(void);

/* 用静态内存创建 5 ms 抬升控制任务。 */
bool lift_control_task_create(void);

/* ---- 命令 API：任何来源（遥控/时序/调试器）都走这几个入口 ---- */

void lift_control_enable(bool enable);
void lift_control_set_target(
    int32_t height_ecd,
    int32_t roll_ecd,
    int32_t pitch_ecd);
void lift_control_request_homing(void);
void lift_control_request_clear_homing(void);
void lift_control_request_reset(void);

#endif
