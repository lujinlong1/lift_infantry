#include "can_tx_service.h"

#include "bsp_can.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

static can_tx_service_stats_t tx_stats;

/** 在线程临界区内清空 CAN 发送统计。 */
void can_tx_service_init(void)
{
    taskENTER_CRITICAL();
    (void)memset(&tx_stats, 0, sizeof(tx_stats));
    taskEXIT_CRITICAL();
}

/**
 * @brief 将逻辑总线号映射为 HAL 句柄并发送标准帧。
 * @return 返回可区分的发送结果，并累计各类发送次数。
 */
can_tx_service_status_t can_tx_service_send(
    can_bus_t bus,
    uint16_t std_id,
    const uint8_t *data,
    uint8_t dlc)
{
    CAN_HandleTypeDef *hcan;
    bsp_can_tx_status_t status;

    if (bus == CAN_BUS_1)
    {
        hcan = &hcan1;
    }
    else if (bus == CAN_BUS_2)
    {
        hcan = &hcan2;
    }
    else
    {
        tx_stats.invalid_count++;
        return CAN_TX_SERVICE_INVALID_ARGUMENT;
    }

    status = bsp_can_send(hcan, std_id, data, dlc);

    switch (status)
    {
        case BSP_CAN_TX_OK:
            tx_stats.sent_count++;
            return CAN_TX_SERVICE_OK;

        case BSP_CAN_TX_BUSY:
            tx_stats.busy_count++;
            return CAN_TX_SERVICE_BUSY;

        case BSP_CAN_TX_INVALID_ARGUMENT:
            tx_stats.invalid_count++;
            return CAN_TX_SERVICE_INVALID_ARGUMENT;

        default:
            tx_stats.error_count++;
            return CAN_TX_SERVICE_HAL_ERROR;
    }
}

/** 原子复制 CAN 发送统计，避免读取过程中被其他任务更新。 */
void can_tx_service_get_stats(can_tx_service_stats_t *stats)
{
    if (stats == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *stats = tx_stats;
    taskEXIT_CRITICAL();
}
