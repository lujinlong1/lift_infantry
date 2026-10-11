#ifndef LIFT_CONTROL_H
#define LIFT_CONTROL_H

/*
 * 抬升模态控制器（对应议题 LIFT-008 的上半部分、LIFT-011 的时序基础）。
 *
 * 架构位置：
 *      lift_control（本模块：目标 -> 模态位置环 -> 模态速度环 -> 模态控制量）
 *            ↓  lift_sync_effort_t
 *      lift_sync（模态 -> 4 轴电流，翘曲恒 0）
 *            ↓  int32_t current[4]
 *      lift_guard（限位 / 缓降 / 门禁 / 离线）
 *            ↓
 *      robot_lift_set_currents()
 *
 * 本模块做四件事：
 *   1. 把 4 轴位置正变换成 4 个模态（复用 lift_sync_forward）
 *   2. 把 4 轴转速正变换成 4 个模态速度（复用 lift_sync_forward_speed）
 *   3. 对【高度 / 横滚 / 俯仰】各跑一条【串级】PID：
 *        位置环  目标位置 - 实际位置      -> 目标模态速度
 *        速度环  目标模态速度 - 实际速度  -> 模态控制量
 *   4. 对目标模态做斜坡限速，禁止目标突变
 *
 * ── 为什么是串级，而不是"位置环直接出电流" ───────────────────────────
 * 丝杆【不自锁】，而且负载是整车重力（ADR-006）。位置环单环在目标附近
 * 靠误差反向来刹车，落地时等效一根只有刚度、没有阻尼的弹簧：超调、
 * 来回抖，而抖动的代价是减速箱和 C620。
 * 速度环提供的正是那一点阻尼。参考实现（旧车 chassis_task.c 的 RisePosition）
 * 也是 位置 → 速度 → 电流 的三层结构，Yaw 轴同样是串级。
 *
 * ── 量纲（这里最容易搞错）───────────────────────────────────────────
 *   位置        ecd（编码器计数，单轴）
 *   模态位置    4 × ecd（转置伴随的约定，不除 4）
 *   速度        ecd/s（单轴），由 speed_rpm 换算
 *   模态速度    4 × ecd/s
 *   模态控制量  电流量纲；因为反变换不除 4，单模态指令下它就等于四轴各自的电流值
 *
 * 所以：
 *   - pid[POSITION].output_limit 必须填【最大模态速度】(ecd/s)
 *   - pid[SPEED].output_limit    必须填【最大轴电流】(C620 量纲)
 * 外环的输出是内环的目标，这两个上限量纲不同，串级最容易在这里出错。
 *
 * ── 转速语义开关 ────────────────────────────────────────────────────
 * C620 反馈的 speed_rpm 是【转子侧】还是【输出侧】，目前没有实机确认
 * （LIFT-007）。两种解释相差约 19 倍。与其在注释里写"待确认"，不如做成
 * 显式开关 speed_is_rotor：实机标定时翻一下就行，不用改公式。
 * 注意这个开关【不影响位置环】——位置的绝对量纲由零点与行程决定，
 * 只影响速度环的增益标定。
 *
 * 刻意不做的事：
 *   - 不碰翘曲模态。模态控制量里根本没有它的位置，见 lift_sync.h。
 *   - 不做电流限幅。那是 lift_guard 的职责，本模块输出的是未限幅的控制量。
 *   - 不判断"到位后停留多久"。本模块只给出瞬时到位标志，时间迟滞由上层时序负责。
 *
 * 设计约束：纯逻辑，无时间依赖（dt 与周期由配置给出），无 RTOS/HAL 依赖。
 */

#include "lift_sync.h"
#include "pid.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LIFT_CONTROL_MODE_HEIGHT = 0,   /* 整车高度 */
    LIFT_CONTROL_MODE_ROLL,         /* 横滚：左减右 */
    LIFT_CONTROL_MODE_PITCH,        /* 俯仰：前减后 */
    LIFT_CONTROL_MODE_COUNT
} lift_control_mode_t;

typedef enum
{
    LIFT_CONTROL_LOOP_POSITION = 0, /* 外环：位置误差 -> 目标模态速度 */
    LIFT_CONTROL_LOOP_SPEED,        /* 内环：速度误差 -> 模态控制量 */
    LIFT_CONTROL_LOOP_COUNT
} lift_control_loop_t;

/*
 * 每个模态的三态（2026-10-11 从台架模块移植）：
 *   APPROACH  正常串级，目标速度按距离收窄
 *   BRAKE     进死区但还在动：目标速度强制 0，让速度环主动拉停（不是断力）
 *   HOLD      进死区且停住：两条环退出，只留保持前馈把车托住
 */
typedef enum
{
    LIFT_CONTROL_HOLD_APPROACH = 0,
    LIFT_CONTROL_HOLD_BRAKE,
    LIFT_CONTROL_HOLD_HOLD
} lift_control_hold_state_t;

typedef struct
{
    /*
     * 串级两条环的 PID，索引见 lift_control_loop_t。
     * 两条环各自有独立的 output_limit，且【量纲不同】，见文件头说明。
     */
    pid_config_t pid[LIFT_CONTROL_LOOP_COUNT];
    /* 坐标变换权重，通常用 lift_sync_config_rect() 填好 */
    lift_sync_config_t sync;
    /* 编码器每圈计数（GM6020/M3508 均为 8192） */
    int32_t ecd_per_rev;
    /* 电机到负载的减速比 */
    float gear_ratio;
    /* true = speed_rpm 是转子侧转速，需除以减速比还原到输出侧 */
    bool speed_is_rotor;
    /* 目标斜坡：每个控制周期目标模态最多变化多少计数；<=0 表示不限制 */
    int32_t max_target_step_ecd;
    /* 控制周期（秒），作为两条环的 PID dt */
    float period_s;
    /* 到位判定容差（计数） */
    int32_t settle_tolerance_ecd;

    /*
     * ---- 模态限幅（对应 LIFT-009 行程软限位）------------------------
     *
     * 为什么限位必须在【模态空间】而不是逐轴做：
     *
     *   四根丝杆顶的是同一块刚性车身。任意一根轴上的电流都是
     *   「高度分量 + 横滚分量 + 俯仰分量」之和，其中一部分是【在帮别的轴
     *   调平】。逐轴判决只看得见"这根轴想去哪"，看不见"它还在替谁出力"，
     *   于是一旦它触界就把整轴清零，等于把它从协同里踢出去——
     *   车身再也拉不平，而且会反复触发（车一歪就再踢一次）。
     *
     * 本模块的做法：对三个受控模态各设一个位置区间，再算一个统一比例 s
     * （0~1），把三个模态控制量同时乘以 s，使得【没有任何一根轴会越界】。
     * 于是：
     *   - 某一根轴接近上限时，压制的是"让它继续往上"的那部分模态出力；
     *   - 其余模态对它的贡献被保留下来，它仍然能参与调平；
     *   - 到行程端点时 s=0，出力自然归零，不会硬顶机械挡块。
     *
     * 区间按【已找零坐标】给出。min >= max 表示该模态不启用限幅。
     */
    int32_t limit_height_min_ecd[LIFT_CHANNEL_COUNT];
    int32_t limit_height_max_ecd[LIFT_CHANNEL_COUNT];
    /*
     * 曾经还有 limit_roll_* / limit_pitch_* 四个字段，已删除（2026-10-11）：
     * 它们被认真填了值、也被认真归一化过，但限幅计算里【一行都没读】——
     * 属于"看起来配了、其实不起作用"的死配置，比没有更危险。
     * 姿态边界目前只在命令面（remote_lift / lift_apply_safety）夹取；
     * 若以后要在模态空间限制横滚/俯仰，必须在这里新增字段【并且】
     * 在 control_limit_scale() 里真的参与计算，不要再留只填不读的字段。
     */
    /*
     * 限幅生效的滞回深度（计数）：只要某轴离界不足这么多，就开始按比例压制。
     * 取 0 表示"到界才削"，容易在界上抖；给一个几百计数的软区更稳。
     */
    int32_t limit_margin_ecd;

    /*
     * ---- 重力/负载前馈（抬升机构特有，不能省）----------------------
     *
     * 本机构【不自锁】（ADR-006）：停在行程中间时，托住车身的力【只能】由
     * 电机提供，是一份持续的保持电流。
     *
     * 若不给前馈，这份电流只能靠位置环的积分去凑——那么"要出力就必须有
     * 误差"，位置永远收敛不到目标，稳态会固定偏小几个百分点。
     * 仿真里正是这个现象：目标 20000，稳态停在 18974（差 5.1%），
     * 而三个受控模态看起来都"收敛了"。这不是 PID 没调好，
     * 是缺了前馈这个自由度。
     *
     * ⚠️ 方向：填【正数】，模块按【抬升方向（逻辑正）】施加。
     *    实测确认的机构关系是"正电流 = 齿条伸出 = 把车顶起来"，所以托住车
     *    必须给正向电流。
     *    （2026-10-11 修正：此前代码取负号按"向下"施加，方向是反的——
     *      那是按"四丝杆"旧机构推的；而且台架模块 hold_feedforward_current
     *      一直是正向施加，同一固件里两处符号相反。填正数、按抬升施加。）
     *
     * 单位是"模态控制量"。因为正/反变换是转置伴随约定（不除 4），
     * 单模态指令下模态控制量的数值就等于四轴各自的电流值，所以：
     *      模态前馈 = 单轴保持电流（不是 ×4 后的值）
     *     （旧注释说"再乘 4"，那是把模态量当成"四轴之和"了；实际反变换
     *       current[ch] = w_height[ch] × effort.height = effort.height，
     *       单模态下发时模态控制量与单轴电流 1:1。）
     *
     * 标定方法：车停在行程中段、四轴已找零，读出让车身静止所需的【单轴】
     * 电流，直接填入。
     */
    float gravity_feedforward_current;

    /*
     * 前馈的施加门槛（单轴计数）：四轴里【最低】的那根高过它才施加前馈。
     *
     * 车落在最低端（零点）时，重量由地面/机械挡块承担，此时再给保持电流
     * 只会白烧电机（而且推不动），所以门槛以下前馈强制为 0。
     * <= 0 表示不设门槛（任何位置都施加）。
     */
    int32_t feedforward_min_position_ecd;

    /*
     * ---- 平滑链（2026-10-11 从台架模块移植，五个环节）------------------
     *
     * 台架模块（Diagnostics/MotorTest/lift_jog_task.c）在实机上调出来的那套
     * "怎么让 M3508 丝滑到位"，本来只有台架有；主链只有"位置环+速度环+目标斜坡"，
     * 所以同样的机构在主链上会冲、会欠阻尼。这里把缺的五个搬进来：
     *
     *   ① 跟踪前馈     目标以速率 R 走时，速度指令直接加 R（否则要靠误差去攒，
     *                  斜坡期间一直有跟踪滞后，位置环积分一直往外顶）
     *   ② 接近减速     误差进入 approach_zone 后把【目标速度】线性收窄，
     *                  但只收到 approach_min_speed_ecd_s（收到 0 会卡在死区外）
     *   ③ 三态机       APPROACH / BRAKE / HOLD：到位后不是靠 PID 微振，
     *                  而是刹停→只给保持前馈
     *   ④ 逐级开放     使能后电流上限从 current_limit_start 慢慢放开
     *   ⑤ 电流斜坡     di/dt 限幅，防"一瞬间到满电流"（护齿轮/护 C620）
     *   另加：速度反馈一阶低通（C620 的 rpm 是整数，1 rpm 台阶会被内环放大）
     *
     * 全部按【模态】给（三个模态共用同一组数），量纲是模态计数/模态 ecd/s。
     * 想换算到单轴：÷ LIFT_CHANNEL_COUNT（= 4）。
     */
    int32_t approach_zone_ecd;          /* 减速区起点；<=0 = 不减速 */
    int32_t approach_min_speed_ecd_s;   /* 减速区最低目标速度；0 会卡在死区外，别填 0 */
    int32_t hold_speed_ecd_s;           /* 判定"停住了"的模态速度阈值；<=0 = 不进 HOLD */
    int32_t current_ramp_per_s;         /* 模态控制量的 di/dt；0 = 不限制 */
    int32_t current_limit_start;        /* 逐级开放起点；0 = 从头就是满限幅 */
    int32_t current_limit_ramp_per_s;   /* 逐级开放速度；0 = 不逐级开放 */
    int32_t speed_filter_tau_ms;        /* 速度反馈一阶低通时间常数；0 = 关 */

    /*
     * ---- 执行器增益（模态限幅预测位移时必需）------------------------
     *
     * 单位：(单轴 ecd/s) 每 电流。也就是"给 1 个电流单位，丝杆最终跑多快"，
     * 与仿真台架 / 真机标定用的是同一个量（K = v_max / i_max）。
     *
     * 为什么限幅需要它：限幅要判断"本周期这个出力会让轴走多远"，
     * 而 pid 输出是【电流】。换算链是
     *     每轴电流 × K = 每轴 ecd/s，再经反变换（每轴取 1/4）得模态速度。
     * 少了这一步，预测位移会大出一个 K/4 的倍数，限幅会立刻误判成"要越界"
     * 而把出力全压掉——现象是车原地不动，却怎么查都查不出控制逻辑的错。
     *
     * <= 0 表示未知，此时【不做限幅预测】，只保留 guard 的逐轴兜底。
     */
    float actuator_gain_ecd_s_per_current;
} lift_control_config_t;

typedef struct
{
    bool enabled;
    int32_t target_height_ecd;
    int32_t target_roll_ecd;
    int32_t target_pitch_ecd;
} lift_control_command_t;

typedef struct
{
    lift_sync_modes_t actual;          /* 正变换得到的实际模态位置 */
    lift_sync_modes_t actual_speed;    /* 正变换得到的实际模态速度（ecd/s） */
    int32_t ramped_height_ecd;         /* 斜坡后的目标，供观察突变量 */
    int32_t ramped_roll_ecd;
    int32_t ramped_pitch_ecd;
    int32_t error_height_ecd;          /* 目标 - 实际（用未斜坡的原始目标） */
    int32_t error_roll_ecd;
    int32_t error_pitch_ecd;
    int32_t target_speed_height;       /* 外环输出：目标模态速度（ecd/s） */
    int32_t target_speed_roll;
    int32_t target_speed_pitch;
    float pid_output[LIFT_CONTROL_LOOP_COUNT][LIFT_CONTROL_MODE_COUNT];
    /*
     * 模态限幅比例 0..1：=1 表示没有限幅；<1 表示有轴接近行程界，
     * 三个模态控制量按此比例统一压制。用于在 Watch 里看到"限位在起作用"。
     */
    float limit_scale;
    bool  limit_active;
    bool  at_target;                    /* 三个模态误差都在容差内 */
    /* 每个模态的三态（见 lift_control_hold_state_t）：调"到位后为什么还在动"看它 */
    uint8_t hold_state[LIFT_CONTROL_MODE_COUNT];
    /* 本周期实际生效的逐级开放电流上限（模态量纲） */
    int32_t limit_open_current[LIFT_CONTROL_MODE_COUNT];
} lift_control_status_t;

typedef struct
{
    lift_control_config_t config;
    pid_controller_t pid[LIFT_CONTROL_LOOP_COUNT][LIFT_CONTROL_MODE_COUNT];
    int32_t ramped_height_ecd;
    int32_t ramped_roll_ecd;
    int32_t ramped_pitch_ecd;
    bool ramp_initialized;
    /* 平滑链的状态（按模态） */
    float speed_filtered[LIFT_CONTROL_MODE_COUNT];   /* 速度低通 */
    float current_ramped[LIFT_CONTROL_MODE_COUNT];   /* 电流斜坡缓存 */
    int32_t limit_open[LIFT_CONTROL_MODE_COUNT];     /* 逐级开放已放到多少 */
    uint8_t hold_state[LIFT_CONTROL_MODE_COUNT];     /* 三态机 */
} lift_control_t;

/* 初始化：填入默认 PID 参数与矩形布局，清空斜坡状态。config 为 NULL 时不修改。 */
void lift_control_init(lift_control_t *self, const lift_control_config_t *config);

/*
 * 把一份 speed_rpm 按配置换算成 ecd/s，即内环实际使用的被控量。
 *
 * 这个函数对外公开，是为了让【台架模型】和控制器用同一个换算因子：
 * 测试里的电机模型若要产生 ecd/s 量纲的速度反馈，就必须按这个式子走，
 * 否则控制器与模型各用一套比例，内环增益会被隐式放大几千倍，
 * 表现为"一上电就冲、然后在目标两侧来回撞限幅"。
 * 真机上这个因子由实测的 ecd_per_rev / gear_ratio / speed_is_rotor 决定。
 */
float lift_control_speed_to_ecd_s(
    const lift_control_config_t *config,
    int16_t speed_rpm);

/* 清空两条环的 PID 与斜坡状态（用于退出使能后重新进入）。 */
void lift_control_reset(lift_control_t *self);

/*
 * 推进一个控制周期。
 * position 为 4 轴【已找零位置】(ecd)，speed_rpm 为 4 轴原始转速反馈。
 * command 为本周期指令。
 * 任一指针为 NULL 时直接返回，不修改状态。
 * 未使能时输出零控制量并复位 PID，斜坡重新初始化到下个目标。
 *
 * 模态限幅与重力前馈都在本函数内部完成，输出 effort 已经是"可以下发的量"；
 * 调用方不必（也不应）再做逐轴限位——那正是本模块要取代的做法，理由见
 * lift_control_config_t 里模态限幅那一段注释。
 */
void lift_control_update(
    lift_control_t *self,
    const int32_t position[LIFT_CHANNEL_COUNT],
    const int16_t speed_rpm[LIFT_CHANNEL_COUNT],
    const lift_control_command_t *command,
    lift_sync_effort_t *effort,
    lift_control_status_t *status);

#endif
