#include "system_init.h"

#include "bsp_can.h"
#include "can_tx_service.h"
#include "chassis_control.h"
#include "friction_control.h"
#include "feed_control.h"
#include "lift_control_task.h"
#include "robot_motor_commands.h"
#include "robot_motors.h"
#include "shooter_control.h"
#include "yaw_control.h"
#include "pitch_control.h"
#include "remote_control_task.h"
#include "dbus_remote.h"
#include "telemetry_task.h"
#include "usb_device.h"
#include "auto_aim_link.h"
#include "runtime_monitor.h"

/**
 * @brief 初始化业务模块、CAN 外设和遥控器接收。
 * @return 所有必要模块初始化成功返回 true。
 * @note 该函数只完成初始化，不创建任务；任务由 app_tasks_create 统一创建。
 */
bool system_init(void)
{
    auto_aim_link_init();
    robot_motors_init();
    robot_motor_commands_init();
    chassis_control_init();
    friction_control_init();
    feed_control_init();
    shooter_control_init();
    yaw_control_init();
    pitch_control_init();
    /*
     * 抬升：先建纯逻辑链（标定参数 + 派生增益 + 空配置），再建遥控映射。
     * 这里【只初始化，不使能】：没有遥控命令时不会有任何电流。
     */
    lift_control_task_init();
    remote_mapping_init();
    telemetry_init();
    runtime_monitor_init();
    can_tx_service_init();

    if (bsp_can_init() != BSP_CAN_OK)
        return false;
    if (!dbus_remote_init())
        return false;
    MX_USB_DEVICE_Init();
    return true;
}
