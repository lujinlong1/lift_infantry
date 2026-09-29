#include "dbus_remote.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#include "usart.h"

#include <stddef.h>
#include <string.h>

#define DBUS_FRAME_SIZE      18U
#define DBUS_DMA_BUFFER_SIZE 36U
#define DBUS_CHANNEL_OFFSET  1024
#define DBUS_CHANNEL_LIMIT   700

volatile dbus_remote_state_t g_dbus_remote;

/*
 * STM32F407 的 DMA 无法访问 0x10000000 开始的 CCM RAM。
 * 单独命名段可阻止 ArmClang 将该小数组合并进 CCM 中的
 * .L_MergedGlobals；链接后必须确认地址位于 0x20000000 SRAM。
 */
static uint8_t dbus_dma_buffer[DBUS_DMA_BUFFER_SIZE]
    __attribute__((section(".bss.dbus_dma_buffer")));
static uint8_t dbus_frame_buffer[DBUS_FRAME_SIZE];
static uint16_t dbus_dma_position;
static uint8_t dbus_frame_length;

/** 返回遥控通道值的绝对值，用于帧合法性检查。 */
static int16_t dbus_abs(int16_t value)
{
    return value < 0 ? (int16_t)-value : value;
}

/**
 * @brief 解包一帧 18 字节 DJI DBUS 数据并发布最新遥控状态。
 * @return 通道与拨杆值合法时返回 true，否则累计无效帧数并返回 false。
 */
static bool dbus_parse(const uint8_t *data)
{
    dbus_remote_state_t state;

    if (data == NULL)
    {
        return false;
    }

    state.channel[0] = (int16_t)(
        (((uint16_t)data[0] | ((uint16_t)data[1] << 8U)) & 0x07FFU) -
        DBUS_CHANNEL_OFFSET);
    state.channel[1] = (int16_t)(
        ((((uint16_t)data[1] >> 3U) | ((uint16_t)data[2] << 5U)) &
          0x07FFU) - DBUS_CHANNEL_OFFSET);
    state.channel[2] = (int16_t)(
        ((((uint16_t)data[2] >> 6U) | ((uint16_t)data[3] << 2U) |
          ((uint16_t)data[4] << 10U)) & 0x07FFU) - DBUS_CHANNEL_OFFSET);
    state.channel[3] = (int16_t)(
        ((((uint16_t)data[4] >> 1U) | ((uint16_t)data[5] << 7U)) &
          0x07FFU) - DBUS_CHANNEL_OFFSET);
    /*
     * 该遥控器的物理左拨杆位于 bit[7:6]，物理右拨杆位于 bit[5:4]。
     * 在协议层完成物理名称校正，业务层便可始终按“左云台、右底盘”理解。
     */
    state.switch_left = (uint8_t)((data[5] >> 6U) & 0x03U);
    state.switch_right = (uint8_t)((data[5] >> 4U) & 0x03U);

    /* 11 位通道跨字节紧凑排列；越界值或 0 档拨杆通常表示帧没有对齐。 */
    if ((dbus_abs(state.channel[0]) > DBUS_CHANNEL_LIMIT) ||
        (dbus_abs(state.channel[1]) > DBUS_CHANNEL_LIMIT) ||
        (dbus_abs(state.channel[2]) > DBUS_CHANNEL_LIMIT) ||
        (dbus_abs(state.channel[3]) > DBUS_CHANNEL_LIMIT) ||
        (state.switch_right == 0U) || (state.switch_left == 0U))
    {
        g_dbus_remote.invalid_count++;
        return false;
    }

    state.mouse_x = (int16_t)((uint16_t)data[6] |
                              ((uint16_t)data[7] << 8U));
    state.mouse_y = (int16_t)((uint16_t)data[8] |
                              ((uint16_t)data[9] << 8U));
    state.mouse_z = (int16_t)((uint16_t)data[10] |
                              ((uint16_t)data[11] << 8U));
    state.mouse_left = data[12] != 0U;
    state.mouse_right = data[13] != 0U;
    state.keyboard = (uint16_t)data[14] | ((uint16_t)data[15] << 8U);
    state.channel[4] = (int16_t)(
        ((uint16_t)data[16] | ((uint16_t)data[17] << 8U)) -
        DBUS_CHANNEL_OFFSET);
    state.last_update_tick = osKernelGetTickCount();
    state.frame_count = g_dbus_remote.frame_count + 1U;
    state.invalid_count = g_dbus_remote.invalid_count;

    g_dbus_remote = state;
    return true;
}

/** 逐字节组装 DBUS 帧；解析失败时滑动一个字节继续寻找帧头。 */
static void dbus_consume_byte(uint8_t value)
{
    dbus_frame_buffer[dbus_frame_length++] = value;
    if (dbus_frame_length < DBUS_FRAME_SIZE)
    {
        return;
    }

    if (dbus_parse(dbus_frame_buffer))
    {
        dbus_frame_length = 0U;
    }
    else
    {
        /* 热插拔可能丢失字节，保留后 17 字节并滑窗寻找下一处正确对齐。 */
        (void)memmove(
            dbus_frame_buffer,
            &dbus_frame_buffer[1],
            DBUS_FRAME_SIZE - 1U);
        dbus_frame_length = DBUS_FRAME_SIZE - 1U;
    }
}

/** 将 DMA 环形缓冲区指定区间的字节依次送入帧组装器。 */
static void dbus_consume_dma_region(uint16_t start, uint16_t end)
{
    uint16_t index;

    for (index = start; index < end; index++)
    {
        dbus_consume_byte(dbus_dma_buffer[index]);
    }
}

/** 清空接收位置并启动 USART3 空闲线 DMA 接收。 */
static bool dbus_start_receive(void)
{
    dbus_dma_position = 0U;
    dbus_frame_length = 0U;

    if (HAL_UARTEx_ReceiveToIdle_DMA(
            &huart3,
            dbus_dma_buffer,
            DBUS_DMA_BUFFER_SIZE) != HAL_OK)
    {
        return false;
    }

    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    return true;
}

/** 初始化遥控状态和缓冲区，然后启动 DBUS DMA 接收。 */
bool dbus_remote_init(void)
{
    (void)memset((void *)&g_dbus_remote, 0, sizeof(g_dbus_remote));
    (void)memset(dbus_dma_buffer, 0, sizeof(dbus_dma_buffer));
    (void)memset(dbus_frame_buffer, 0, sizeof(dbus_frame_buffer));
    return dbus_start_receive();
}

/** 根据最后有效帧时间判断遥控器是否在线。 */
bool dbus_remote_is_online(uint32_t timeout_ms)
{
    return (g_dbus_remote.frame_count > 0U) &&
           ((uint32_t)(osKernelGetTickCount() -
                       g_dbus_remote.last_update_tick) <=
            pdMS_TO_TICKS(timeout_ms));
}

/** 在临界区中复制遥控状态快照，避免任务读到一半更新的数据。 */
void dbus_remote_get_snapshot(dbus_remote_state_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    (void)memcpy(snapshot, (const void *)&g_dbus_remote, sizeof(*snapshot));
    taskEXIT_CRITICAL();
}

/**
 * @brief USART 空闲事件回调，消费 DMA 自上次位置以来新增的数据。
 * @note size 回绕时分成缓冲区尾部和头部两段处理。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if ((huart == NULL) || (huart->Instance != USART3) ||
        (size > DBUS_DMA_BUFFER_SIZE))
    {
        return;
    }

    if (size >= dbus_dma_position)
    {
        dbus_consume_dma_region(dbus_dma_position, size);
    }
    else
    {
        dbus_consume_dma_region(dbus_dma_position, DBUS_DMA_BUFFER_SIZE);
        dbus_consume_dma_region(0U, size);
    }

    dbus_dma_position =
        size == DBUS_DMA_BUFFER_SIZE ? 0U : size;
}

/** USART3 接收错误回调：中止当前接收并重新启动 DBUS DMA。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART3))
    {
        return;
    }

    (void)HAL_UART_AbortReceive(huart);
    (void)dbus_start_receive();
}
