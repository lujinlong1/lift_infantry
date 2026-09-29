#include "can_rx_service.h"

#include "robot_motors.h"

#include <stddef.h>

/** 将接收服务取出的 CAN 帧交给机器人电机协议层识别和解码。 */
void can_rx_service_dispatch(const can_rx_frame_t *frame)
{
    if (frame == NULL)
    {
        return;
    }

    (void)robot_motors_process_frame(frame);
}
