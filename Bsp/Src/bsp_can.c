#include "bsp_can.h"

#include "can_rx_service.h"

#include <stddef.h>
#include <string.h>

#define CAN2_START_FILTER_BANK 14U

#define CAN_RX_NOTIFICATIONS       \
    (CAN_IT_RX_FIFO0_MSG_PENDING | \
     CAN_IT_ERROR |                \
     CAN_IT_BUSOFF |               \
     CAN_IT_LAST_ERROR_CODE)

bsp_can_error_stats_t g_bsp_can_error_stats;

/** 配置一个接收所有标准帧的 32 位掩码过滤器。 */
static HAL_StatusTypeDef can_config_accept_all_filter(
    CAN_HandleTypeDef *hcan,
    uint32_t filter_bank)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = filter_bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = CAN2_START_FILTER_BANK;

    return HAL_CAN_ConfigFilter(hcan, &filter);
}

/** 配置过滤器、启动指定 CAN 外设并打开接收及错误中断。 */
static bsp_can_status_t can_start(
    CAN_HandleTypeDef *hcan,
    uint32_t filter_bank)
{
    if (hcan == NULL)
    {
        return BSP_CAN_INVALID_ARGUMENT;
    }

    if (can_config_accept_all_filter(hcan, filter_bank) != HAL_OK)
    {
        return BSP_CAN_FILTER_ERROR;
    }

    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        return BSP_CAN_START_ERROR;
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_RX_NOTIFICATIONS) != HAL_OK)
    {
        (void)HAL_CAN_Stop(hcan);
        return BSP_CAN_NOTIFICATION_ERROR;
    }

    return BSP_CAN_OK;
}

/** 初始化 CAN1、CAN2，并清空总线错误统计。 */
bsp_can_status_t bsp_can_init(void)
{
    bsp_can_status_t status;

    (void)memset(
        &g_bsp_can_error_stats, 0, sizeof(g_bsp_can_error_stats));

    status = can_start(&hcan1, 0U);
    if (status != BSP_CAN_OK)
    {
        return status;
    }

    status = can_start(&hcan2, CAN2_START_FILTER_BANK);
    if (status != BSP_CAN_OK)
    {
        (void)HAL_CAN_Stop(&hcan1);
        return status;
    }

    return BSP_CAN_OK;
}

/** HAL CAN 错误回调：按总线记录错误码、错误次数和 Bus-Off 次数。 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t bus_index;
    uint32_t error;

    if (hcan == NULL)
    {
        return;
    }
    if (hcan->Instance == CAN1)
    {
        bus_index = 0U;
    }
    else if (hcan->Instance == CAN2)
    {
        bus_index = 1U;
    }
    else
    {
        return;
    }

    error = HAL_CAN_GetError(hcan);
    g_bsp_can_error_stats.error_count[bus_index]++;
    g_bsp_can_error_stats.last_error[bus_index] = error;
    if ((error & HAL_CAN_ERROR_BOF) != 0U)
    {
        g_bsp_can_error_stats.bus_off_count[bus_index]++;
    }
}

/**
 * @brief 发送一帧标准 CAN 数据帧。
 * @return 区分参数错误、邮箱忙和 HAL 发送失败，供上层决定是否重试。
 */
bsp_can_tx_status_t bsp_can_send(
    CAN_HandleTypeDef *hcan,
    uint16_t std_id,
    const uint8_t *data,
    uint8_t dlc)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    if ((hcan == NULL) || (data == NULL) ||
        (std_id > 0x7FFU) || (dlc > 8U))
    {
        return BSP_CAN_TX_INVALID_ARGUMENT;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return BSP_CAN_TX_BUSY;
    }

    header.StdId = std_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = dlc;
    header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(hcan, &header, (uint8_t *)data, &mailbox) != HAL_OK)
    {
        return BSP_CAN_TX_HAL_ERROR;
    }

    return BSP_CAN_TX_OK;
}

/**
 * @brief CAN FIFO0 接收中断回调。
 * @note 中断中只做取帧和入队，协议解析留给接收任务完成，以缩短中断时间。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    can_rx_frame_t frame;

    if (hcan == NULL)
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, frame.data) != HAL_OK)
        {
            break;
        }

        if ((header.IDE != CAN_ID_STD) ||
            (header.RTR != CAN_RTR_DATA) ||
            (header.DLC > 8U))
        {
            continue;
        }

        if (hcan->Instance == CAN1)
        {
            frame.bus = CAN_BUS_1;
        }
        else if (hcan->Instance == CAN2)
        {
            frame.bus = CAN_BUS_2;
        }
        else
        {
            continue;
        }

        frame.std_id = (uint16_t)header.StdId;
        frame.dlc = (uint8_t)header.DLC;
        (void)can_rx_service_push_from_isr(&frame);
    }
}
