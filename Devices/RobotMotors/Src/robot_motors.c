#include "robot_motors.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "robot_motor_config.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

robot_motor_state_t g_robot_motors;

/** 将通用 CAN 帧参数转交 DJI 电机反馈解码器。 */
static bool robot_motors_decode_dji(
    dji_motor_feedback_t *motor,
    const can_rx_frame_t *frame,
    uint32_t current_tick)
{
    return dji_motor_decode_feedback(
        motor, frame->data, frame->dlc, current_tick);
}

/** 原子清空整车所有电机反馈状态。 */
void robot_motors_init(void)
{
    taskENTER_CRITICAL();
    (void)memset(&g_robot_motors, 0, sizeof(g_robot_motors));
    taskEXIT_CRITICAL();
}

/**
 * @brief 按 CAN 总线和标准 ID 将反馈帧路由到对应的实际电机对象。
 * @return 识别且成功解码返回 true，未知 ID 或非法帧返回 false。
 */
bool robot_motors_process_frame(const can_rx_frame_t *frame)
{
    uint32_t current_tick;

    if (frame == NULL)
    {
        return false;
    }

    current_tick = osKernelGetTickCount();

    if (frame->bus == CAN_BUS_1)
    {
        switch (frame->std_id)
        {
            case ROBOT_LIFT_MOTOR_1_ID:
            case ROBOT_LIFT_MOTOR_2_ID:
            case ROBOT_LIFT_MOTOR_3_ID:
            case ROBOT_LIFT_MOTOR_4_ID:
                return robot_motors_decode_dji(
                    &g_robot_motors.lift.motor[
                        frame->std_id - ROBOT_LIFT_MOTOR_1_ID],
                    frame,
                    current_tick);

            case ROBOT_PITCH_MOTOR_ID:
                return robot_motors_decode_dji(
                    &g_robot_motors.gimbal.pitch,
                    frame,
                    current_tick);

            case ROBOT_FRICTION_MOTOR_1_ID:
                return robot_motors_decode_dji(
                    &g_robot_motors.shooter.friction[0],
                    frame,
                    current_tick);

            case ROBOT_FRICTION_MOTOR_2_ID:
                return robot_motors_decode_dji(
                    &g_robot_motors.shooter.friction[1],
                    frame,
                    current_tick);

            default:
                return false;
        }
    }

    if (frame->bus == CAN_BUS_2)
    {
        switch (frame->std_id)
        {
            case ROBOT_CHASSIS_WHEEL_1_ID:
            case ROBOT_CHASSIS_WHEEL_2_ID:
            case ROBOT_CHASSIS_WHEEL_3_ID:
            case ROBOT_CHASSIS_WHEEL_4_ID:
                return robot_motors_decode_dji(
                    &g_robot_motors.chassis.wheel[
                        frame->std_id - ROBOT_CHASSIS_WHEEL_1_ID],
                    frame,
                    current_tick);

            case ROBOT_TRIGGER_MOTOR_ID:
                return robot_motors_decode_dji(
                    &g_robot_motors.shooter.trigger,
                    frame,
                    current_tick);

            case ROBOT_YAW_MOTOR_ID:
                return robot_motors_decode_dji(
                    &g_robot_motors.gimbal.yaw,
                    frame,
                    current_tick);

            default:
                return false;
        }
    }

    return false;
}

/** 在临界区内取得整车电机反馈快照，供控制任务一致地读取。 */
void robot_motors_get_snapshot(robot_motor_state_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = g_robot_motors;
    taskEXIT_CRITICAL();
}
