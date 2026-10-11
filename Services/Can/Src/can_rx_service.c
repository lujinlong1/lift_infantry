#include "can_rx_service.h"

#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <string.h>

/*
 * 接收队列深度。
 *
 * 为什么从 32 提到 128：4 个抬升电机 + Pitch + 2 个摩擦轮都在发反馈，
 * 按每个 1 kHz 算，总帧率约 4~7 kHz；32 帧只够缓冲 8 ms。
 * 一旦接收任务被短暂抢占超过 8 ms，队列就满、帧被丢 —— 实测 21 万帧里
 * 丢了 35 帧（0.017%）。
 *
 * 丢帧本身不会致命（算过：即使 35 帧是一次突发，缺口内最快位移也只有
 * 几百计数，远小于"半圈假设"的 4096 边界），但它是**不该出现的隐患**：
 * 一旦某次突发更长，"多圈展开"就可能猜错一整圈，而位置会永久偏掉且不自愈。
 * 128 帧 ≈ 32 ms 缓冲，成本只有约 2 KB RAM。
 */
#define CAN_RX_QUEUE_LENGTH      128U
#define CAN_RX_TASK_STACK_WORDS  512U

static osMessageQueueId_t can_rx_queue;
static osThreadId_t can_rx_task_handle;

static StaticQueue_t can_rx_queue_control;
static uint8_t can_rx_queue_storage[CAN_RX_QUEUE_LENGTH * sizeof(can_rx_frame_t)];

static StaticTask_t can_rx_task_control;
static StackType_t can_rx_task_stack[CAN_RX_TASK_STACK_WORDS];

static volatile uint32_t received_count;
static volatile uint32_t dropped_count;

can_rx_debug_t g_can_rx_debug;

static void can_rx_task(void *argument);

/** 保存最近一帧及常用报文 ID 的计数，便于在线排查 CAN 通信。 */
static void can_rx_debug_record(const can_rx_frame_t *frame)
{
    uint32_t bus_index;
    uint32_t data_index;

    if ((frame == NULL) ||
        ((frame->bus != CAN_BUS_1) && (frame->bus != CAN_BUS_2)))
    {
        return;
    }

    bus_index = (uint32_t)frame->bus;
    g_can_rx_debug.total_count[bus_index]++;
    g_can_rx_debug.last_std_id[bus_index] = frame->std_id;
    g_can_rx_debug.last_dlc[bus_index] = frame->dlc;

    for (data_index = 0U; data_index < 8U; data_index++)
    {
        g_can_rx_debug.last_data[bus_index][data_index] =
            frame->data[data_index];
    }

    if (frame->std_id == 0x001U)
    {
        g_can_rx_debug.id_001_count[bus_index]++;
    }
    else if (frame->std_id == 0x011U)
    {
        g_can_rx_debug.id_011_count[bus_index]++;
    }
    else if ((frame->std_id >= 0x201U) &&
             (frame->std_id <= 0x208U))
    {
        g_can_rx_debug.dji_id_count[bus_index]
                                       [frame->std_id - 0x201U]++;
    }
    else
    {
    }
}

/** 创建静态 CAN 接收消息队列并清空调试统计。 */
bool can_rx_service_init(void)
{
    const osMessageQueueAttr_t queue_attributes = {
        .name = "can_rx_queue",
        .cb_mem = &can_rx_queue_control,
        .cb_size = sizeof(can_rx_queue_control),
        .mq_mem = can_rx_queue_storage,
        .mq_size = sizeof(can_rx_queue_storage)
    };

    (void)memset(&g_can_rx_debug, 0, sizeof(g_can_rx_debug));

    if (can_rx_queue != NULL)
    {
        return true;
    }

    can_rx_queue = osMessageQueueNew(
        CAN_RX_QUEUE_LENGTH,
        sizeof(can_rx_frame_t),
        &queue_attributes);

    return can_rx_queue != NULL;
}

/** 创建负责出队和协议分发的高优先级 CAN 接收任务。 */
bool can_rx_service_task_create(void)
{
    const osThreadAttr_t task_attributes = {
        .name = "can_rx",
        .cb_mem = &can_rx_task_control,
        .cb_size = sizeof(can_rx_task_control),
        .stack_mem = can_rx_task_stack,
        .stack_size = sizeof(can_rx_task_stack),
        .priority = osPriorityHigh
    };

    if (can_rx_task_handle != NULL)
    {
        return true;
    }

    can_rx_task_handle = osThreadNew(
        can_rx_task,
        NULL,
        &task_attributes);

    return can_rx_task_handle != NULL;
}

/**
 * @brief 从 CAN 接收中断向静态队列压入一帧。
 * @return 入队成功返回 true；队列未初始化或已满时计入丢帧。
 */
bool can_rx_service_push_from_isr(const can_rx_frame_t *frame)
{
    if ((frame == NULL) || (can_rx_queue == NULL))
    {
        dropped_count++;
        g_can_rx_debug.dropped_count = dropped_count;
        return false;
    }

    if (osMessageQueuePut(
            can_rx_queue,
            frame,
            0U,
            0U) != osOK)
    {
        dropped_count++;
        g_can_rx_debug.dropped_count = dropped_count;
        return false;
    }

    received_count++;
    g_can_rx_debug.received_count = received_count;
    return true;
}

/** 获取自启动以来成功进入接收队列的帧数。 */
uint32_t can_rx_service_get_received_count(void)
{
    return received_count;
}

/** 获取因队列未就绪或队列已满而丢弃的帧数。 */
uint32_t can_rx_service_get_dropped_count(void)
{
    return dropped_count;
}

/** 阻塞等待 CAN 帧，记录调试数据后分发到具体协议解析器。 */
static void can_rx_task(void *argument)
{
    can_rx_frame_t frame;

    (void)argument;

    for (;;)
    {
        if (osMessageQueueGet(
                can_rx_queue,
                &frame,
                NULL,
                osWaitForever) == osOK)
        {
            can_rx_debug_record(&frame);
            can_rx_service_dispatch(&frame);
        }
    }
}

/**
 * @brief 默认的弱符号分发钩子。
 * @note 正常工程由 can_dispatch.c 中的同名强符号覆盖，弱实现便于裁剪模块。
 */
__weak void can_rx_service_dispatch(const can_rx_frame_t *frame)
{
    /*
     * 当前弱实现不解析报文。
     * 后续在 can_dispatch.c 中编写同名强实现。
     */
    (void)frame;
}
