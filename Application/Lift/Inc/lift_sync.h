#ifndef LIFT_SYNC_H
#define LIFT_SYNC_H

/*
 * 抬升四轴协同 —— 过约束保护（对应议题 LIFT-008）。
 *
 * 问题：四个刚性丝杆顶在刚性车身上，是【超静定】结构。
 *   每个丝杆各管各的位置，地面稍有不平就会变成巨大的内力：
 *   电流飙升、抖动、顶坏联轴器。架空测试永远看不出来，落地才炸。
 *
 * 关键认识：刚性车身只有 3 个高度自由度（高度 z、横滚 roll、俯仰 pitch），
 *   而四个丝杆提供了 4 个。多出来的第 4 个叫【翘曲 warp】——
 *   它的正确控制量是【零】：让机械变形去吸收它，控制器绝不出力。
 *
 * 因此本模块把控制搬到【模态空间】：
 *
 *      位置:  4 个通道  --正变换-->  高度/横滚/俯仰/翘曲
 *      出力:  4 个电流  <--反变换--  高度/横滚/俯仰  (翘曲恒为 0，无输入口)
 *
 * 反变换刻意【不提供】翘曲输入口，从接口上保证控制器永远无法与过约束对抗。
 *
 * 权重约定（矩形布局，通道顺序与 CAN 0x201~0x204 一致）：
 *   0 = 右前(FR)   1 = 左前(FL)   2 = 左后(RL)   3 = 右后(RR)
 *   车体 +x 向前、+y 向左
 *
 *             height  roll  pitch  warp
 *   FR(0)       +1     -1     +1     +1
 *   FL(1)       +1     +1     +1     -1
 *   RL(2)       +1     +1     -1     +1
 *   RR(3)       +1     -1     -1     -1
 *
 * 四行两两正交且每行平方和为 4(=通道数)，故权重矩阵满足 W·Wᵀ = 4·I。
 *
 * 本模块采用【转置伴随】约定，而不是除以 4：
 *      正变换   M = W · x
 *      反变换   c = Wᵀ · U
 * 于是 forward(distribute(U)) = 4·U，比例常数 4 由调用方的模态 PID 增益吸收。
 *
 * 之所以不除以 4，是为了【整数精确】：反变换里没有除法，翘曲分量严格恒为 0，
 * 不会因整除截断而漏出残余内力——那正是本模块要消灭的东西（见单测 [4]）。
 *
 * 设计约束：纯逻辑，无时间依赖、无 RTOS/HAL 依赖、无动态分配。
 */

#include <stdbool.h>
#include <stdint.h>

#ifndef LIFT_CHANNEL_COUNT
#define LIFT_CHANNEL_COUNT 4U
#endif

typedef struct
{
    int8_t w_height[LIFT_CHANNEL_COUNT];
    int8_t w_roll[LIFT_CHANNEL_COUNT];
    int8_t w_pitch[LIFT_CHANNEL_COUNT];
    int8_t w_warp[LIFT_CHANNEL_COUNT];
} lift_sync_config_t;

typedef struct
{
    int32_t height;   /* 四轴平均高度（编码器计数，未除 4） */
    int32_t roll;     /* 左减右 */
    int32_t pitch;    /* 前减后 */
    int32_t warp;     /* 对角差；仅供诊断，绝不参与控制 */
} lift_sync_modes_t;

typedef struct
{
    int32_t height;   /* 高度模态的控制量 */
    int32_t roll;     /* 横滚模态的控制量 */
    int32_t pitch;    /* 俯仰模态的控制量 */
    /* 刻意没有 warp：翘曲模态不受控，这是本模块存在的理由 */
} lift_sync_effort_t;

/* 填入标准矩形布局权重。若实车接线顺序不同，改这里的表或自行填写 config。 */
void lift_sync_config_rect(lift_sync_config_t *config);

/* 正变换：4 轴位置 -> 4 个模态。config/pos/modes 任一为 NULL 时直接返回。 */
void lift_sync_forward(
    const lift_sync_config_t *config,
    const int32_t position[LIFT_CHANNEL_COUNT],
    lift_sync_modes_t *modes);

/*
 * 速度的正变换：4 轴转速 -> 4 个模态速度，用的是【同一张权重表】。
 *
 * 为什么速度也要过模态：串级控制的内环需要"模态速度"作为被控量，
 * 而速度不是位置，不能靠差分位置得到——差分会把编码器量化噪声放大成尖峰。
 * 权重只描述几何，与量纲无关，所以直接复用，不必再写一张表。
 *
 * speed 的单位由调用方统一（本工程用 ecd/s），权重与量纲无关。
 */
void lift_sync_forward_speed(
    const lift_sync_config_t *config,
    const int32_t speed[LIFT_CHANNEL_COUNT],
    lift_sync_modes_t *modes);

/*
 * 反变换：模态控制量 -> 4 轴电流。
 *
 * 不含除法，输出为未经限幅的原始 int32（限幅交给 lift_guard）。
 * 结果按构造满足「翘曲分量严格为 0」，可用 lift_sync_warp_component 断言。
 * 注意本变换是转置伴随：forward(distribute(U)) = 通道数 × U。
 */
void lift_sync_distribute(
    const lift_sync_config_t *config,
    const lift_sync_effort_t *effort,
    int32_t current[LIFT_CHANNEL_COUNT]);

/* 计算一组电流的翘曲分量 Σ w_warp[i] * current[i]。正常应恒为 0。 */
int32_t lift_sync_warp_component(
    const lift_sync_config_t *config,
    const int32_t current[LIFT_CHANNEL_COUNT]);

/* 自检：四行权重是否两两正交且每行平方和等于通道数。 */
bool lift_sync_config_is_orthogonal(const lift_sync_config_t *config);

#endif
