#ifndef LIFT_KINEMATICS_H
#define LIFT_KINEMATICS_H

/*
 * 抬升运动学换算（对应议题 LIFT-007 / LIFT-009）。
 *
 * ── 存在的理由 ─────────────────────────────────────────────────────────
 * 在这之前，全项目【没有任何一处】把编码器计数换成毫米：
 *   - `lift_control_task.c` 里只有一个宏 `LIFT_COUNTS_PER_MM_AXIS`，
 *     而且它只用来把"mm/s"折成模态速度参数，从来没有用来换算位置；
 *   - `lift_geometry.c` 是【丝杆导程】模型（`lead_um` + `drive_num/den`），
 *     与真实机构（齿轮齿条）不是一回事，而且零调用。
 * 结果是"抬升 200 mm"这句话在固件里根本无法表达——只能手填一串计数。
 *
 * ── 机构与公式 ─────────────────────────────────────────────────────────
 *   M3508（行星减速箱）→ 输出轴驱动齿轮 → 齿条
 *
 *   齿轮转一圈，齿条走一个分度圆周：
 *       circumference_um = π × d_mm × 1000
 *
 *   所以：
 *                          d_mm × gear_ratio × counts_per_rev × 2
 *       counts_per_um = ──────────────────────────────────────────
 *                                  circumference_um
 *   （用 2d 而不是 πd 是为了不把 π 写死成常数——分度圆周长与直径本来就
 *     只差一个 π，标定表里填的是直径，模块自己不去近似 π。）
 *
 * ── ★ 实测参数（2026-10-11，取代此前全部图纸推算）★ ────────────────────
 *
 *   用户手测（三条独立测量，互相印证）：
 *     缩回死点 → 伸出【硬限位】= 150 000 计数 = 200 mm
 *     齿轮转一圈            = 110 000 计数 = 146.7 mm 齿条位移
 *     110000 ÷ 8192         = 13.43（减速比）
 *     146.7 ÷ π             = 46.7 mm（分度圆直径）
 *
 *   ⇒ counts_per_mm = 750.0（150000/200 = 750；110000/146.7 = 749.8，一致）
 *
 *   ⚠️ 图纸参数（46 齿 × 模数 1.5 = 69 mm 分度圆 + 19.2:1）推出 725.7
 *      计数/mm，比实测小 3.2%，**并且与"齿轮一圈 146.7 mm"直接冲突**
 *      （69 mm 分度圆一圈该走 216.8 mm）。按用户要求：以实测为准。
 *
 * ── 三条必须遵守的约定 ─────────────────────────────────────────────────
 *  1. 【逻辑坐标】本模块输入输出全是逻辑量：正 mm = 抬升（齿条伸出把车顶起）。
 *     原始域的极性由 `direction`（与 lift_top 的 axis_direction 同一个数）折掉。
 *  2. 【零点是逻辑零】`zero_ecd` 存的是【原始域】的死点计数，
 *     与 `lift_top.zero_ecd` 同域、同约定，可以直接互相赋值。
 *     逻辑位置 = direction × (原始计数 − 原始零点)。
 *  3. 【每轴独立】系数、零点、方向都是四轴各一份。绝不允许用"四轴一样"
 *     的假设去算目标——那样四根齿条的实际高度会跟着安装误差散开。
 *
 * ── 剩余待标定 ─────────────────────────────────────────────────────────
 *    - `direction[4]`：手推，看计数往哪走（P2）
 *    - `zero_ecd[4]`：找零得到（或软零点，仅台架）
 *    - `position_max_mm[4]`：由 LIFT_TRAVEL_MAX_ECD = 148500 换算
 *      （硬限位 150000 留 1500 余量；硬限位是铁碰铁，不能配成软限位）
 *
 * 设计约束：纯逻辑，无时间依赖、无 RTOS/HAL 依赖、无动态分配、无浮点。
 *           中间运算用 int64，避免 150000 量级的乘法溢出。
 */

#include <stdbool.h>
#include <stdint.h>

#ifndef LIFT_CHANNEL_COUNT
#define LIFT_CHANNEL_COUNT 4U
#endif

/*
 * 标称计数/毫米（转子侧）。
 *
 * ★ 2026-10-11 实测定案（用户手测，取代图纸推算）：
 *     缩回死点 → 伸出硬限位 = 150 000 计数 = 200 mm
 *     ⇒ 750.0 计数/mm
 *   两条独立测量互相印证：150000/200 = 750，而"齿轮一圈 110000 计数
 *   = 146.7 mm"给出 110000/146.7 = 749.8，一致。
 *
 *   ⚠️ 这个值**不是**由 π×d 推出来的。图纸参数（69 mm 分度圆 + 19.2:1）
 *      推出 725.7，比实测小 3.2%，并且与"一圈 146.7 mm"直接冲突。
 *      按用户 2026-10-11 的要求：**以实测为准，不再参考图纸。**
 *
 * 只用于"配置与标称值差多少"的提醒，不参与任何实际换算。
 * 实际换算的系数由 `lift_kin_config_t` 的机构参数派生，见下面那一节。
 */
#define LIFT_KIN_NOMINAL_COUNTS_PER_MM   750.0F

/* 齿条有效行程的合法上限（mm）。超过它一定是配置写错了，直接判非法。 */
#define LIFT_KIN_MAX_TRAVEL_MM           1000

typedef struct
{
    /* 电机转子侧每圈编码器计数。M3508/C620 = 8192。 */
    int32_t counts_per_rev;
    /*
     * 电机到驱动齿轮的减速比。
     *
     * 2026-10-11 实测定案：**13.43**，不是图纸的 19.2。
     *   依据：齿轮转一圈 = 110 000 计数；110000 ÷ 8192 = 13.43。
     *   （图纸的 3591/187 ≈ 19.2 与"一圈 146.7 mm"冲突，按实测取 13.43。）
     *
     * 它参与 counts_per_mm 的推导，也是速度换算里"转子 rpm → 输出轴"的除数。
     * 填错会让**位置和速度一起错**，而且速度错得更多（它是线性的）。
     */
    float   gear_ratio;
    /*
     * 驱动齿轮【分度圆直径】，mm。
     *
     * 2026-10-11 实测定案：**46.693**，不是图纸的 69。
     *   依据：齿轮一圈齿条走 146.7 mm ⇒ 分度圆周 146.7 ⇒ d = 146.7/π。
     *
     * ⚠️ 这个数是"反推出来的"，唯一的作用是让公式结构保持不变。
     *    换算的**权威是实测的 750.0 计数/mm**（见 lift_control_task.c 里
     *    LIFT_TRAVEL_MAX_ECD 上方那段）。如果以后有更准的实测，
     *    改的是本字段，不是公式。
     */
    float   pinion_diameter_mm;
    /*
     * ★★★ 实测系数覆盖（counts/mm，转子侧，单轴）。0 = 不覆盖，用派生值。
     *
     * 为什么要这个字段：机构参数（直径 + 减速比）是**反推**出来的中间量，
     * 而且直径在内部按 µm 取整，于是派生出来的 counts/mm 只能是
     * 749.99 或 750.01 这种离散值，**永远落不到整 750.0**。
     *
     * 但实测的权威是那个比例本身：
     *     150000 计数 = 200 mm   ⇒  750.0 计数/mm
     * 所以标定时**直接填这一项**，让换算严格等于实测比例；
     * 直径/减速比只用于速度换算和显示。
     *
     * ⚠️ 填了它，`pinion_diameter_mm` 就不再影响位置换算（但**仍然影响
     *    速度换算**）。两者要一起标定，别只改一个。
     */
    float   counts_per_mm;
    /*
     * 原始域 → 逻辑域的方向，只允许 +1 / -1。
     * +1 = 原始计数增大即抬升；-1 = 原始计数增大即下降。
     * 必须与 `lift_top_config_t.axis_direction[ch]` 是同一个数，
     * 否则位置读数与毫米换算会一个正一个反。
     */
    int8_t  direction;
} lift_kin_axis_config_t;

typedef struct
{
    lift_kin_axis_config_t axis[LIFT_CHANNEL_COUNT];
    /*
     * 每轴【原始域】死点计数（与 lift_top.zero_ecd 同域）。
     * 找零完成后由调用方写入；0 表示还没建立零点。
     */
    int32_t zero_ecd[LIFT_CHANNEL_COUNT];
    /*
     * 每轴【已找零】软限位，单位 mm（逻辑坐标，正 = 抬升）。
     * min >= max 表示该轴不启用限位（交给上游兜底）。
     *
     * ⚠️ 真实有效行程没量过。默认值故意配成"必然拒绝"，
     *    见 lift_control_task.c 的 LIFT_TRAVEL_*_MM。
     */
    int32_t position_min_mm[LIFT_CHANNEL_COUNT];
    int32_t position_max_mm[LIFT_CHANNEL_COUNT];
} lift_kin_config_t;

/* 每轴的派生量与标定就绪标志。 */
typedef struct
{
    /* counts_per_output_rev = counts_per_rev × gear_ratio —— 以分数精确保存 */
    int64_t counts_per_rev_num;
    int64_t counts_per_rev_den;
    /*
     * counts_per_mm 以分数保存：
     *     counts_per_mm = counts_per_mm_num / counts_per_mm_den
     * ⚠️ 注意这里的分母【不是】分度圆周长，而是「周长(µm) × 1000」，
     *    因为 1 mm = 1000 µm 已经在分母里折掉了。要算"输出轴一圈走多少毫米"
     *    请用下面单独保存的 circumference_um，不要再从这对分数反推 ——
     *    那正是本模块早期版本把速度算大 1000 倍的原因。
     */
    int64_t counts_per_mm_num;
    int64_t counts_per_mm_den;
    /* 分度圆周长（µm）：输出轴转一圈齿条走过的长度。速度换算用。 */
    int64_t circumference_um;
    /* 本轴配置是否合法（系数、方向） */
    bool    valid;
    /*
     * 本轴是否【可以接受绝对位置命令】：
     *   valid && direction != 0 && 行程区间已配置（min < max）
     * 标定没做完的轴一律 false —— 没经过标定的轴不得执行 200 mm 绝对位置命令。
     */
    bool    ready;
} lift_kin_axis_info_t;

/* 一次配置的完整校验结果。 */
typedef struct
{
    bool all_valid;      /* 四轴配置自洽（能算换算） */
    bool all_ready;      /* 四轴都能接受绝对位置命令 */
    bool axes_valid[LIFT_CHANNEL_COUNT];
    bool axes_ready[LIFT_CHANNEL_COUNT];
    /*
     * 计算出来的 counts/mm（×100 定点，便于无浮点比较），invalid 轴为 0。
     * 供 Watch / 自检脚本检查"配置是不是真的按 69 mm 算出来的"。
     */
    int32_t counts_per_mm_x100[LIFT_CHANNEL_COUNT];
    /* 与标称 750.0 的偏差是否超过 tol_permille（‰）。 */
    bool    deviates_from_nominal[LIFT_CHANNEL_COUNT];
} lift_kin_calibration_t;

/* 把一个配置整体初始化成"未标定"状态（所有轴 valid=false）。 */
void lift_kin_config_init_uncalibrated(lift_kin_config_t *config);

/*
 * 填一份【由图纸推出的标称】配置：8192 计数/圈、19.2:1、69 mm 分度圆。
 * 方向与零点仍需按实测填。行程区间不动（保持调用方已填的值）。
 *
 * @param direction 四轴方向（与 lift_top.axis_direction 同值），可为 NULL
 */
void lift_kin_config_nominal_rack_and_pinion(
    lift_kin_config_t *config,
    const int8_t direction[LIFT_CHANNEL_COUNT]);

/* 单轴配置是否自洽。config 为 NULL 或 axis 越界返回 false。 */
bool lift_kin_axis_config_is_valid(const lift_kin_axis_config_t *axis);

/* 四轴配置是否自洽（逐轴检查 + 轴号范围）。 */
bool lift_kin_config_is_valid(const lift_kin_config_t *config);

/*
 * 校验并派生。config 为 NULL 时返回 false 且不写 out。
 * out 为 NULL 时只返回 all_valid。
 */
bool lift_kin_validate(
    const lift_kin_config_t *config,
    lift_kin_calibration_t *out,
    float tolerance_permille);

/* 本轴是否已具备接受绝对位置命令的条件。 */
bool lift_kin_axis_is_ready(const lift_kin_config_t *config, uint32_t axis);

/* 四轴是否都已具备接受绝对位置命令的条件（整体 200 mm 命令的前置条件）。 */
bool lift_kin_all_ready(const lift_kin_config_t *config);

/* ---- 系数（每轴独立）---- */

/* counts/mm × 1000（定点，方便 Watch 与无浮点比较）。非法时返回 0。 */
int32_t lift_kin_counts_per_mm_x1000(
    const lift_kin_config_t *config,
    uint32_t axis);

/* counts/mm 的浮点值（仅用于显示/日志）。非法时返回 0.0F。 */
float lift_kin_counts_per_mm(const lift_kin_config_t *config, uint32_t axis);

/* ---- 位置换算（逻辑 mm ↔ 逻辑 ecd，每轴独立零点）---- */

/*
 * @brief 逻辑位置(mm) → 逻辑计数(ecd)。
 * @return false 表示参数非法（轴号越界 / 配置非法 / 输入超范围），*out 不被修改。
 * @note 输入是【逻辑毫米】：正 = 抬升。
 */
bool lift_kin_mm_to_ecd(
    const lift_kin_config_t *config,
    uint32_t axis,
    int32_t mm,
    int32_t *out);

/*
 * @brief 逻辑计数(ecd) → 逻辑位置(mm)。截断向零，往返误差 < 1 mm。
 * @return false 表示参数非法，*out 不被修改。
 */
bool lift_kin_ecd_to_mm(
    const lift_kin_config_t *config,
    uint32_t axis,
    int32_t ecd,
    int32_t *out);

/*
 * @brief 由【原始编码器位置】算【已找零逻辑位置】(ecd)。
 *        = direction × raw − direction × zero_ecd
 * @return false 表示参数非法，*out 不被修改。axis 为 NULL 时按未提供处理
 *         （等价于所有轴 direction = +1，仅调试用）。
 */
bool lift_kin_position_from_raw(
    const lift_kin_config_t *config,
    uint32_t axis,
    int32_t raw_ecd,
    int32_t *out);

/* ---- 速度换算 ---- */

/*
 * @brief 原始 speed_rpm → 逻辑 mm/s。
 *        rpm 是【转子侧】（与 counts_per_rev 同域），先除减速比回到输出轴。
 * @return false 表示参数非法，*out 不被修改。
 */
bool lift_kin_speed_to_mm_s(
    const lift_kin_config_t *config,
    uint32_t axis,
    int16_t speed_rpm,
    float *out);

/* @brief 逻辑 mm/s → 原始 speed_rpm（反向换算，用于自检）。 */
bool lift_kin_mm_s_to_speed(
    const lift_kin_config_t *config,
    uint32_t axis,
    float mm_s,
    int16_t *out);

/* ---- 行程与限位 ---- */

/*
 * @brief 该轴的软限位区间（逻辑 mm）。未配置（min >= max）时返回 false。
 */
bool lift_kin_limits_mm(
    const lift_kin_config_t *config,
    uint32_t axis,
    int32_t *min_mm,
    int32_t *max_mm);

/*
 * @brief 把逻辑位置夹到该轴的软限位内。
 * @return true 表示被夹过（发生了限位），false 表示在区间内或未配置限位。
 */
bool lift_kin_clamp_mm(
    const lift_kin_config_t *config,
    uint32_t axis,
    int32_t *mm);

/*
 * @brief 逻辑位置是否在软限位外（用于流程前置检查）。
 * @return false 表示未配置限位（无法判定"在外"）。
 */
bool lift_kin_out_of_limits(
    const lift_kin_config_t *config,
    uint32_t axis,
    int32_t mm);

/* ---- 编码器回绕 ---- */

/*
 * @brief 单圈编码器（0..counts_per_rev-1）的【最小位移】展开。
 *
 * 与 ODrive / 本工程 `dji_motor.c` 用的是同一个算法：
 *     delta = new − last;
 *     if (delta < −cpr/2) delta += cpr;
 *     else if (delta > cpr/2) delta −= cpr;
 *
 * ⚠️ 前提是"一帧内位移不超过半圈"。转速再快 1 ms 也只走十几个计数；
 *    真正的风险是【丢帧】——帧间隔超过约 260 ms 才会猜错方向。
 *    所以这个函数不能替代"采样够快"，只能作为换算层的保护。
 *
 * @return false 表示参数非法或值域越界，*delta 不被修改。
 */
bool lift_kin_wrap_delta(
    int32_t counts_per_rev,
    int32_t last_ecd,
    int32_t new_ecd,
    int32_t *delta);

/* ---- 四轴一致性 ---- */

/*
 * @brief 四轴逻辑位置的极差（mm），即"最高那根减最低那根"。
 *
 * 这是"同步误差"的原始定义：和力矩无关，也和四点是否均匀受力无关。
 * 只靠"四轴电流相同"判断同步是错的——受力不同时同样的电流对应不同的位移。
 *
 * @return false 表示参数非法，*out 不被修改。
 */
bool lift_kin_position_spread_mm(
    const lift_kin_config_t *config,
    const int32_t logical_ecd[LIFT_CHANNEL_COUNT],
    int32_t *out);

/*
 * @brief 某轴与"其余轴参考高度"的偏差（mm，带符号：正 = 比其余轴高）。
 *
 * 参考量 = 去掉【一个】最高与【一个】最低之后的平均。
 *   · 四轴同高        → 参考 = 该高度，偏差 0
 *   · 一根高出 10 mm  → 参考 = 其余三根的高度，偏差 **+10**（正是想要的读数）
 *   · 两根偏高一根偏低 → 参考 = 剩下两根的均值
 *
 * **不是**四轴平均。理由：平均会被跑偏的那一根拖走 —— 一根高出 10 mm 时
 * 平均只低 2.5 mm，判据读到 7.5 而不是 10，偏差越大反而越钝。
 * 这里要回答的是"这一根比其它几根高多少"，参考量就不该包含这一根。
 *
 * ⚠️ 精度：输入是"已找零逻辑计数"，输出是"毫米"，中间有一次 ÷counts_per_mm
 *    的取整，所以单轴读数有 ±1 mm 的量化误差，四轴极差最坏可差 ±2 mm。
 *    判据阈值（例如 6 mm）必须显著大于它 —— 默认配置满足这一点。
 */
bool lift_kin_axis_deviation_mm(
    const lift_kin_config_t *config,
    const int32_t logical_ecd[LIFT_CHANNEL_COUNT],
    uint32_t axis,
    int32_t *out);

#endif
