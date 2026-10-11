#include "app_tasks.h"

#include "can_rx_service.h"
#include "chassis_control.h"
#include "friction_control.h"
#include "feed_control.h"
#include "lift_control_task.h"
#include "lift_jog_task.h"
#include "robot_motor_commands.h"
#include "shooter_control.h"
#include "yaw_control.h"
#include "pitch_control.h"
#include "remote_control_task.h"
#include "telemetry_task.h"
#include "auto_aim_link.h"
#include "runtime_monitor.h"

/**
 * @brief 创建全部业务任务。
 * @return 全部创建成功返回 true，任一任务创建失败立即返回 false。
 * @note 电机发送任务是 CAN 电机命令的唯一发送者，应先于各控制任务创建。
 */
bool app_tasks_create(void)
{
    if (!auto_aim_link_task_create())
        return false;
    if (!can_rx_service_task_create())
        return false;
    if (!robot_motor_tx_task_create())
        return false;
    if (!chassis_control_task_create())
        return false;
    if (!friction_control_task_create())
        return false;
    if (!feed_control_task_create())
        return false;
    if (!shooter_control_task_create())
        return false;
    if (!yaw_control_task_create())
        return false;
    if (!pitch_control_task_create())
        return false;
    /* 抬升控制任务：必须有，否则抬升命令没有执行者，机构不会被驱动。 */
    if (!lift_control_task_create())
        return false;
    /*
     * 抬升诊断点动（P1/P2/P3 标定用）。它默认不可动：axis 未指定、enabled
     * 为 false，只有调试器里显式写好参数并置 enabled 才会有电流。
     */
    if (!lift_jog_task_create())
        return false;
    if (!remote_mapping_task_create())
        return false;
    if (!telemetry_task_create())
        return false;
    if (!runtime_monitor_task_create())
        return false;
    return true;
}
