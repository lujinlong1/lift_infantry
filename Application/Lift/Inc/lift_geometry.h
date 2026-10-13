#ifndef LIFT_GEOMETRY_H
#define LIFT_GEOMETRY_H

/*
 * 抬升位置/速度换算 ——【已被 lift_kinematics 取代，仅作兼容保留】
 *
 * ── 为什么它不再是权威 ────────────────────────────────────────────────
 *
 * 本模块建的是【丝杆导程】模型：
 *      位移_um = ecd × 导程_um × 传动比_分母 / (每圈计数 × 传动比_分子)
 *
 * 而实车的机构是 **齿轮齿条**：M3508（自带 19.2:1 减速箱）→ 输出轴上
 * 46 齿 × 模数 1.5 的驱动齿轮（分度圆 69 mm）→ 齿条。没有"导程"这个量。
 *
 * 更关键的是：本模块**从来没有被任何地方调用过**（只有主机测试在调它）。
 * 也就是说，"毫米"这件事在固件里一直是靠注释和手算维持的，直到
 * `lift_kinematics` 出现。
 *
 * ── 现在谁在做什么 ────────────────────────────────────────────────────
 *
 *   lift_kinematics.c   ← 权威：每轴独立的零点/系数/行程，齿轮齿条模型，
 *                          已接入 lift_control_task 的实际控制链
 *   lift_geometry.c     ← 本文件：保留 API 与实现不改（还有 144 项主机测试
 *                          钉着它），但**不要在固件里新调用它**
 *
 * 之所以不直接删掉：它是一条**独立的验算途径**。同一个 total_ecd 用两套
 * 完全不同的模型换算，如果两边算出来的毫米一致，说明至少有一边的算术
 * 没写错 —— 这在标定阶段是有用的交叉检查，不是死代码。
 *
 * ⚠️ 但要注意：`lift_geometry_config_3508_direct()` 的默认值
 *    （speed_is_rotor = true、gear 3591/187）与 lift_kinematics 的口径
 *    **不同**，直接互相比较之前先把两边配置对齐。
 *
 * 设计约束：纯逻辑，不依赖 RTOS/HAL，不做动态分配。中间运算用 int64，避免溢出。
 */

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /* 电机输出轴每圈编码器计数。M3508 / M2006 均为 8192。 */
    int32_t counts_per_rev;
    /* 丝杆导程，单位微米。T8 导程 2mm 即 2000。 */
    int32_t lead_um;
    /* 电机到丝杆的传动比分子 / 分母。直连填 1 / 1。 */
    int32_t drive_num;
    int32_t drive_den;
    /* 反馈转速是否为转子侧。M3508 若为转子侧需除减速比 3591/187。 */
    bool    speed_is_rotor;
    int32_t gear_num;   /* 减速比分子，M3508 = 3591 */
    int32_t gear_den;   /* 减速比分母，M3508 = 187  */
} lift_geometry_t;

/* 填一份 M3508 直连的常用默认值（转速按转子侧处理）。 */
void lift_geometry_config_3508_direct(lift_geometry_t *config, int32_t lead_um);

/* 配置是否可用。任何除数非正都判为不可用。 */
bool lift_geometry_validate(const lift_geometry_t *config);

/* 编码器计数 -> 位移微米。配置非法或指针为空返回 0。 */
int32_t lift_geometry_ecd_to_um(const lift_geometry_t *config, int32_t ecd);

/* 位移微米 -> 编码器计数。 */
int32_t lift_geometry_um_to_ecd(const lift_geometry_t *config, int32_t um);

/* 反馈转速 -> 位移速度（微米/秒）。 */
int32_t lift_geometry_speed_to_um_per_s(
    const lift_geometry_t *config,
    int16_t speed_rpm);

#endif
