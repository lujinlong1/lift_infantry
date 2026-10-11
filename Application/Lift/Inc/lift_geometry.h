#ifndef LIFT_GEOMETRY_H
#define LIFT_GEOMETRY_H

/*
 * 抬升丝杆位置/速度换算（对应议题 LIFT-007）。
 *
 * 存在的理由：`total_ecd -> 毫米` 这条换算如果不集中到一处，
 * 就会在各处冒出来一堆 8192 和"导程"的魔法数字，而且没人说得清哪个是对的。
 *
 * 另一个更重要的理由：C620 反馈的 `speed_rpm` 到底是【转子侧】还是【输出侧】，
 * 目前**没有实机确认**。与其在注释里写一句"待确认"，不如把它做成一个显式开关：
 * 实机标定时翻一下 flag 就完事，不用改公式。
 *
 * 全部用整数微米（um）表示长度，避免浮点：
 *   位移_um = ecd * 导程_um * 传动比_分母 / (每圈计数 * 传动比_分子)
 *
 * 传动比定义：丝杆转 1 圈需要电机转 `drive_num / drive_den` 圈。
 *   直连 = 1/1；同步带 2:1 减速（电机转 2 圈丝杆转 1 圈）= 2/1。
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
