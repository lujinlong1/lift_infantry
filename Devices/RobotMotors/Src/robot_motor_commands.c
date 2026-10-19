#include "robot_motor_commands.h"

#include "can_tx_service.h"
#include "lift_diagnostics.h"
#include "lift_safety.h"
#include "cmsis_os2.h"
#include "dji_motor.h"
#include "FreeRTOS.h"
#include "robot_motor_config.h"
#include "robot_motors.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

#define ROBOT_MOTOR_TX_TASK_STACK_WORDS 512U
#define DJI_CAN_TEST_CURRENT_LIMIT       1000
#define DJI_CAN_TEST_DEFAULT_DURATION_MS 2000U
#define DJI_CAN_TEST_MAX_DURATION_MS     5000U

static robot_motor_commands_t command_cache;
/* Only the unique TX task accesses this source snapshot. */
static lift_motor_commands_t lift_tx_snapshot;
static bool lift_tx_override, lift_tx_timeout;
static osThreadId_t motor_tx_task_handle;
static StaticTask_t motor_tx_task_control;
static StackType_t motor_tx_task_stack[ROBOT_MOTOR_TX_TASK_STACK_WORDS];

dji_can_test_command_t g_dji_can_test_command = {
    .enabled = false,
    .can_bus = 1U,
    .feedback_id = 0x204U,
    .test_current = 300,
    .duration_ms = DJI_CAN_TEST_DEFAULT_DURATION_MS
};

dji_can_test_status_t g_dji_can_test_status;

/** 将电流命令限制在对应电机型号的安全范围内。 */
static int16_t clamp_current(int16_t current, int32_t limit)
{
    if ((int32_t)current > limit)
    {
        return (int16_t)limit;
    }
    if ((int32_t)current < -limit)
    {
        return (int16_t)(-limit);
    }
    return current;
}

/** 判断控制命令是否在允许的刷新超时时间内。 */
static bool command_is_fresh(
    uint32_t current_tick,
    uint32_t update_tick)
{
    return (uint32_t)(current_tick - update_tick) <=
           pdMS_TO_TICKS(ROBOT_MOTOR_COMMAND_TIMEOUT_MS);
}

/** 封装并发送一组最多四路 DJI 电机电流命令。 */
static can_tx_service_status_t send_dji_group(
    can_bus_t bus,
    uint16_t std_id,
    int16_t motor1,
    int16_t motor2,
    int16_t motor3,
    int16_t motor4)
{
    uint8_t data[8];
    /* Never replay a bench cache/override through a latched main-chain fault.
     * A CONTROL fault-stop command remains subject to its existing executor. */
    if (bus == CAN_BUS_1 && std_id == ROBOT_DJI_COMMAND_ID_1_TO_4 &&
        lift_owner_tests_inhibited() &&
        (lift_tx_override || lift_tx_snapshot.diagnostic_cycle_id == 0U))
        motor1 = motor2 = motor3 = motor4 = 0;

    dji_motor_pack_group_command(
        data, motor1, motor2, motor3, motor4);
    {
        const can_tx_service_status_t result = can_tx_service_send(bus, std_id, data, 8U);
        if (bus == CAN_BUS_1 && std_id == ROBOT_DJI_COMMAND_ID_1_TO_4)
        {
            int16_t packed[4];
            uint32_t ch;
            const uint32_t tick = osKernelGetTickCount();
            for (ch = 0U; ch < 4U; ++ch)
                packed[ch] = (int16_t)(((uint16_t)data[ch * 2U] << 8) |
                                       data[ch * 2U + 1U]);
            taskENTER_CRITICAL();
            lift_diag_tx(packed, lift_tx_snapshot.diagnostic_generation,
                lift_tx_override ? 0U : lift_tx_snapshot.diagnostic_cycle_id, tick * portTICK_PERIOD_MS,
                (uint8_t)result, lift_tx_timeout,
                lift_tx_override);
            taskEXIT_CRITICAL();
        }
        return result;
    }
}

/** 根据被测反馈 ID 反推出组命令 ID 和帧内电机槽位。 */
static bool dji_can_test_resolve(
    uint8_t bus_value,
    uint16_t feedback_id,
    can_bus_t *bus,
    uint16_t *command_id,
    uint8_t *motor_slot)
{
    if ((bus == NULL) || (command_id == NULL) || (motor_slot == NULL))
    {
        return false;
    }

    if (bus_value == 1U)
    {
        *bus = CAN_BUS_1;
    }
    else if (bus_value == 2U)
    {
        *bus = CAN_BUS_2;
    }
    else
    {
        return false;
    }

    if ((feedback_id >= 0x201U) && (feedback_id <= 0x204U))
    {
        *command_id = 0x200U;
        *motor_slot = (uint8_t)(feedback_id - 0x201U);
        return true;
    }

    if ((feedback_id >= 0x205U) && (feedback_id <= 0x208U))
    {
        *command_id = 0x1FFU;
        *motor_slot = (uint8_t)(feedback_id - 0x205U);
        return true;
    }

    if ((feedback_id >= 0x209U) && (feedback_id <= 0x20BU))
    {
        *command_id = 0x2FFU;
        *motor_slot = (uint8_t)(feedback_id - 0x209U);
        return true;
    }

    return false;
}

/** 将通用 DJI 点动测试时长限制在默认值和最大值范围内。 */
static uint32_t dji_can_test_limit_duration(uint32_t duration_ms)
{
    if (duration_ms == 0U)
    {
        return DJI_CAN_TEST_DEFAULT_DURATION_MS;
    }
    if (duration_ms > DJI_CAN_TEST_MAX_DURATION_MS)
    {
        return DJI_CAN_TEST_MAX_DURATION_MS;
    }
    return duration_ms;
}

/**
 * @brief 维护 DJI 单电机点动测试状态并生成覆盖电流。
 * @note 测试结束后继续保持一段时间零电流，确保旧命令不会立即重新接管。
 */
static bool dji_can_test_prepare(
    uint32_t current_tick,
    can_bus_t *bus,
    uint16_t *command_id,
    int16_t currents[4])
{
    static bool active = false;
    static bool owns_lift = false;
    static bool zero_hold_active = false;
    static uint32_t zero_hold_start_tick;
    static uint32_t start_tick;
    static uint32_t duration_ticks;
    static can_bus_t selected_bus;
    static uint16_t selected_command_id;
    static uint8_t selected_slot;
    static int16_t selected_current;
    uint32_t elapsed_ticks;

    if (g_dji_can_test_command.enabled && !active)
    {
        if (!dji_can_test_resolve(
                g_dji_can_test_command.can_bus,
                g_dji_can_test_command.feedback_id,
                &selected_bus,
                &selected_command_id,
                &selected_slot))
        {
            g_dji_can_test_command.enabled = false;
            g_dji_can_test_status.valid = false;
            return false;
        }

        if (owns_lift && (selected_bus != CAN_BUS_1 || selected_command_id != ROBOT_DJI_COMMAND_ID_1_TO_4))
        { lift_owner_release(LIFT_OWNER_CAN_TEST); owns_lift = false; }
        selected_current = clamp_current(
            g_dji_can_test_command.test_current,
            DJI_CAN_TEST_CURRENT_LIMIT);
        start_tick = current_tick;
        duration_ticks = pdMS_TO_TICKS(dji_can_test_limit_duration(
            g_dji_can_test_command.duration_ms));
        active = true;
        zero_hold_active = false;
        g_dji_can_test_status.zero_holding = false;
        g_dji_can_test_status.valid = true;
        g_dji_can_test_status.running = true;
        g_dji_can_test_status.resolved_bus =
            (uint8_t)selected_bus + 1U;
        g_dji_can_test_status.command_id = selected_command_id;
        g_dji_can_test_status.motor_slot = selected_slot;
    }

    if (active)
    {
        elapsed_ticks = (uint32_t)(current_tick - start_tick);
        g_dji_can_test_status.elapsed_ms =
            elapsed_ticks * portTICK_PERIOD_MS;

        if (!g_dji_can_test_command.enabled ||
            (elapsed_ticks >= duration_ticks))
        {
            active = false;
            zero_hold_active = true;
            zero_hold_start_tick = current_tick;
            g_dji_can_test_command.enabled = false;
            g_dji_can_test_status.running = false;
            g_dji_can_test_status.zero_holding = true;
            selected_current = 0;
        }
    }

    if (zero_hold_active &&
        ((uint32_t)(current_tick - zero_hold_start_tick) >=
         pdMS_TO_TICKS(ROBOT_DJI_TEST_ZERO_HOLD_MS)))
    {
        zero_hold_active = false;
        g_dji_can_test_status.zero_holding = false;
    }

    if (!active && !zero_hold_active)
    {
        if (owns_lift) { lift_owner_release(LIFT_OWNER_CAN_TEST); owns_lift = false; }
        return false;
    }

    if (selected_bus == CAN_BUS_1 && selected_command_id == ROBOT_DJI_COMMAND_ID_1_TO_4)
    {
        if (lift_owner_tests_inhibited())
        {
            active = zero_hold_active = false;
            g_dji_can_test_command.enabled = false;
            g_dji_can_test_status.running = false;
            g_dji_can_test_status.zero_holding = false;
            g_dji_can_test_status.applied_current = 0;
            lift_owner_release(LIFT_OWNER_CAN_TEST);
            owns_lift = false;
            return false;
        }
        owns_lift = lift_owner_claim_override(LIFT_OWNER_CAN_TEST,
            current_tick * portTICK_PERIOD_MS, NULL);
        if (!owns_lift) return false;
    }
    currents[0] = 0;
    currents[1] = 0;
    currents[2] = 0;
    currents[3] = 0;
    currents[selected_slot] = active ? selected_current : 0;
    *bus = selected_bus;
    *command_id = selected_command_id;
    g_dji_can_test_status.applied_current = currents[selected_slot];

    return true;
}

/** 发送正常组命令；若当前组正在点动测试，则仅发送测试槽位电流。 */
static void send_dji_group_with_test_override(
    can_bus_t bus,
    uint16_t command_id,
    int16_t motor1,
    int16_t motor2,
    int16_t motor3,
    int16_t motor4,
    bool test_ready,
    can_bus_t test_bus,
    uint16_t test_command_id,
    const int16_t test_currents[4])
{
    can_tx_service_status_t status;

    if (test_ready && (bus == test_bus) &&
        (command_id == test_command_id))
    {
        status = send_dji_group(
            bus,
            command_id,
            test_currents[0],
            test_currents[1],
            test_currents[2],
            test_currents[3]);
        g_dji_can_test_status.last_send_status = (uint8_t)status;
        return;
    }

    (void)send_dji_group(
        bus, command_id, motor1, motor2, motor3, motor4);
}

/**
 * @brief 全车电机 CAN 命令的唯一发送任务。
 * @note 统一发送可避免多个控制任务抢占同一个 DJI 组帧；所有过期命令先归零。
 */
static void robot_motor_tx_task(void *argument)
{
    robot_motor_commands_t commands;
    TickType_t last_wake_tick;
    uint32_t current_tick;
    bool test_ready;
    can_bus_t test_bus = CAN_BUS_1;
    uint16_t test_command_id = 0U;
    int16_t test_currents[4] = {0, 0, 0, 0};

    (void)argument;
    last_wake_tick = xTaskGetTickCount();

    for (;;)
    {
        robot_motor_commands_get_snapshot(&commands);
        current_tick = osKernelGetTickCount();
        lift_tx_snapshot = commands.lift;
        lift_tx_timeout = !command_is_fresh(current_tick, commands.lift.update_tick);

        /* 各子系统独立超时归零，某一控制任务卡死不会保留最后一次电流。 */
        if (!command_is_fresh(current_tick, commands.chassis.update_tick))
        {
            (void)memset(commands.chassis.wheel, 0,
                         sizeof(commands.chassis.wheel));
        }

        if (!command_is_fresh(current_tick, commands.lift.update_tick))
        {
            (void)memset(commands.lift.motor, 0,
                         sizeof(commands.lift.motor));
        }

        if (!command_is_fresh(
                current_tick, commands.shooter.friction_update_tick))
        {
            (void)memset(commands.shooter.friction, 0,
                         sizeof(commands.shooter.friction));
        }
        if (!command_is_fresh(
                current_tick, commands.shooter.trigger_update_tick))
        {
            commands.shooter.trigger_current = 0;
        }

        if (!command_is_fresh(current_tick, commands.gimbal.yaw_update_tick))
        {
            commands.gimbal.yaw_current = 0;
        }

        /*
         * Pitch 是 GM6020，与 Yaw 同属"控制任务每周期必须刷新"的一组。
         * 超时归零对云台是安全的（松开会停住），这里沿用同样的策略。
         */
        if (!command_is_fresh(current_tick, commands.gimbal.pitch_update_tick))
        {
            commands.gimbal.pitch_current = 0;
        }

        test_ready = dji_can_test_prepare(
            current_tick,
            &test_bus,
            &test_command_id,
            test_currents);

        send_dji_group_with_test_override(
            CAN_BUS_2,
            ROBOT_DJI_COMMAND_ID_1_TO_4,
            commands.chassis.wheel[0],
            commands.chassis.wheel[1],
            commands.chassis.wheel[2],
            commands.chassis.wheel[3],
            test_ready,
            test_bus,
            test_command_id,
            test_currents);

        lift_tx_override = test_ready && test_bus == CAN_BUS_1 &&
            test_command_id == ROBOT_DJI_COMMAND_ID_1_TO_4;
        send_dji_group_with_test_override(
            CAN_BUS_1,
            ROBOT_DJI_COMMAND_ID_1_TO_4,
            commands.lift.motor[0],
            commands.lift.motor[1],
            commands.lift.motor[2],
            commands.lift.motor[3],
            test_ready,
            test_bus,
            test_command_id,
            test_currents);

        send_dji_group_with_test_override(
            CAN_BUS_2,
            ROBOT_DJI_COMMAND_ID_5_TO_8,
            0,
            0,
            commands.shooter.trigger_current,
            commands.gimbal.yaw_current,
            test_ready,
            test_bus,
            test_command_id,
            test_currents);

        /* CAN1 0x1FF = { Pitch(GM6020, 电压), 摩擦轮1, 摩擦轮2, 预留 }。 */
        send_dji_group_with_test_override(
            CAN_BUS_1,
            ROBOT_DJI_COMMAND_ID_5_TO_8,
            commands.gimbal.pitch_current,
            commands.shooter.friction[0],
            commands.shooter.friction[1],
            0,
            test_ready,
            test_bus,
            test_command_id,
            test_currents);

        if (test_ready && (test_command_id == 0x2FFU))
        {
            g_dji_can_test_status.last_send_status =
                (uint8_t)send_dji_group(
                    test_bus,
                    test_command_id,
                    test_currents[0],
                    test_currents[1],
                    test_currents[2],
                    test_currents[3]);
        }

        vTaskDelayUntil(
            &last_wake_tick,
            pdMS_TO_TICKS(ROBOT_MOTOR_TX_PERIOD_MS));
    }
}

/** 清空电机命令缓存和测试状态。 */
void robot_motor_commands_init(void)
{
    taskENTER_CRITICAL();
    (void)memset(&command_cache, 0, sizeof(command_cache));
    (void)memset(&g_dji_can_test_status, 0, sizeof(g_dji_can_test_status));
    g_dji_can_test_command.enabled = false;
    taskEXIT_CRITICAL();
}

/** 创建高优先级、静态内存形式的统一电机发送任务。 */
bool robot_motor_tx_task_create(void)
{
    const osThreadAttr_t attributes = {
        .name = "motor_tx",
        .cb_mem = &motor_tx_task_control,
        .cb_size = sizeof(motor_tx_task_control),
        .stack_mem = motor_tx_task_stack,
        .stack_size = sizeof(motor_tx_task_stack),
        .priority = osPriorityHigh
    };

    if (motor_tx_task_handle != NULL)
    {
        return true;
    }

    motor_tx_task_handle = osThreadNew(
        robot_motor_tx_task, NULL, &attributes);
    return motor_tx_task_handle != NULL;
}

/** 限幅并原子更新四个底盘轮电流及其时间戳。 */
void robot_chassis_set_currents(
    int16_t wheel1,
    int16_t wheel2,
    int16_t wheel3,
    int16_t wheel4)
{
    const int16_t currents[4] = {wheel1, wheel2, wheel3, wheel4};
    uint32_t index;
    const uint32_t tick = osKernelGetTickCount();

    taskENTER_CRITICAL();
    for (index = 0U; index < 4U; index++)
    {
        command_cache.chassis.wheel[index] =
            clamp_current(currents[index], ROBOT_3508_CURRENT_LIMIT);
    }
    command_cache.chassis.update_tick = tick;
    taskEXIT_CRITICAL();
}

/** 限幅并原子更新四个抬升电机电流及其时间戳。 */
static void robot_lift_cache_currents(
    int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4,
    uint32_t cycle_id, uint8_t source)
{
    const int16_t currents[ROBOT_LIFT_MOTOR_COUNT] = {
        motor1, motor2, motor3, motor4};
    uint32_t index;
    const uint32_t tick = osKernelGetTickCount();

    taskENTER_CRITICAL();
    for (index = 0U; index < ROBOT_LIFT_MOTOR_COUNT; index++)
    {
        command_cache.lift.motor[index] =
            clamp_current(currents[index], ROBOT_3508_CURRENT_LIMIT);
    }
    command_cache.lift.update_tick = tick;
    command_cache.lift.diagnostic_cycle_id = cycle_id;
    ++command_cache.lift.diagnostic_generation;
    lift_diag_cache(command_cache.lift.motor, cycle_id,
                    tick * portTICK_PERIOD_MS, source);
    taskEXIT_CRITICAL();
}

void robot_lift_set_currents(int16_t m0, int16_t m1, int16_t m2, int16_t m3)
{
    robot_lift_cache_currents(m0, m1, m2, m3, 0U,
        lift_owner_current() == LIFT_OWNER_JOG ? LIFT_DIAG_SOURCE_JOG : LIFT_DIAG_SOURCE_OTHER);
}
void robot_lift_set_currents_diagnostic(int16_t m0, int16_t m1,
    int16_t m2, int16_t m3, uint32_t cycle_id)
{
    robot_lift_cache_currents(m0, m1, m2, m3, cycle_id, LIFT_DIAG_SOURCE_CONTROL);
}

/** 限幅并原子更新 Yaw 电机电流及其时间戳。 */
void robot_gimbal_set_yaw_current(int16_t current)
{
    const uint32_t tick = osKernelGetTickCount();

    taskENTER_CRITICAL();
    command_cache.gimbal.yaw_current =
        clamp_current(current, ROBOT_6020_CURRENT_LIMIT);
    command_cache.gimbal.yaw_update_tick = tick;
    taskEXIT_CRITICAL();
}

/**
 * @brief 原子更新 Pitch 电机电流（GM6020 走 0x1FF，实际为电压量纲）。
 * @note 限幅用 ROBOT_6020_CURRENT_LIMIT(±30000)，与 Yaw 同量纲，
 *       不可误用 C620/C610 的 ±16384/±10000。
 */
void robot_gimbal_set_pitch_current(int16_t current)
{
    const uint32_t tick = osKernelGetTickCount();

    taskENTER_CRITICAL();
    command_cache.gimbal.pitch_current =
        clamp_current(current, ROBOT_6020_CURRENT_LIMIT);
    command_cache.gimbal.pitch_update_tick = tick;
    taskEXIT_CRITICAL();
}

/** 同时更新全部摩擦轮和拨弹电机电流，主要供独占诊断模式使用。 */
void robot_shooter_set_command(
    int16_t friction1,
    int16_t friction2,
    int16_t trigger_current)
{
    const int16_t friction[ROBOT_FRICTION_MOTOR_COUNT] = {
        friction1, friction2};
    uint32_t index;
    const uint32_t tick = osKernelGetTickCount();

    taskENTER_CRITICAL();
    for (index = 0U; index < ROBOT_FRICTION_MOTOR_COUNT; index++)
    {
        command_cache.shooter.friction[index] =
            clamp_current(friction[index], ROBOT_2006_CURRENT_LIMIT);
    }
    command_cache.shooter.trigger_current =
        clamp_current(trigger_current, ROBOT_TRIGGER_CURRENT_LIMIT);
    command_cache.shooter.friction_update_tick = tick;
    command_cache.shooter.trigger_update_tick = tick;
    taskEXIT_CRITICAL();
}

/** 独立更新四个摩擦轮电流，避免覆盖拨弹控制任务的命令。 */
void robot_shooter_set_friction_currents(
    int16_t friction1,
    int16_t friction2)
{
    const int16_t friction[ROBOT_FRICTION_MOTOR_COUNT] = {
        friction1, friction2};
    uint32_t index;
    const uint32_t tick = osKernelGetTickCount();

    taskENTER_CRITICAL();
    for (index = 0U; index < ROBOT_FRICTION_MOTOR_COUNT; index++)
    {
        command_cache.shooter.friction[index] =
            clamp_current(friction[index], ROBOT_2006_CURRENT_LIMIT);
    }
    command_cache.shooter.friction_update_tick = tick;
    taskEXIT_CRITICAL();
}

/** 独立更新供弹轮和拨盘电流，避免覆盖摩擦轮控制任务的命令。 */
void robot_shooter_set_feed_currents(int16_t trigger_current)
{
    const uint32_t tick = osKernelGetTickCount();

    taskENTER_CRITICAL();
    command_cache.shooter.trigger_current =
        clamp_current(trigger_current, ROBOT_TRIGGER_CURRENT_LIMIT);
    command_cache.shooter.trigger_update_tick = tick;
    taskEXIT_CRITICAL();
}

/** 在临界区内复制完整电机命令快照，供统一发送任务读取。 */
void robot_motor_commands_get_snapshot(robot_motor_commands_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = command_cache;
    taskEXIT_CRITICAL();
}
