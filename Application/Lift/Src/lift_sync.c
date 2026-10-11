#include "lift_sync.h"

#include <stddef.h>

#if LIFT_CHANNEL_COUNT != 4
#error "lift_sync_config_rect() 只定义了 4 通道矩形布局，请自行填写 lift_sync_config_t。"
#endif

void lift_sync_config_rect(lift_sync_config_t *config)
{
    /* 通道顺序：0=右前(FR) 1=左前(FL) 2=左后(RL) 3=右后(RR) */
    static const int8_t cfg_height[4] = { 1,  1,  1,  1};
    static const int8_t cfg_roll[4]   = {-1,  1,  1, -1};
    static const int8_t cfg_pitch[4]  = { 1,  1, -1, -1};
    static const int8_t cfg_warp[4]   = { 1, -1,  1, -1};
    uint32_t ch;

    if (config == NULL)
    {
        return;
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        config->w_height[ch] = cfg_height[ch];
        config->w_roll[ch] = cfg_roll[ch];
        config->w_pitch[ch] = cfg_pitch[ch];
        config->w_warp[ch] = cfg_warp[ch];
    }
}

/**
 * 正变换的内部实现。位置和速度【共用同一张权重表】——
 * 权重只描述几何，与量纲无关，所以没有理由写两份。
 */
static void sync_forward_i32(
    const lift_sync_config_t *config,
    const int32_t channel[LIFT_CHANNEL_COUNT],
    lift_sync_modes_t *modes)
{
    uint32_t ch;

    modes->height = 0;
    modes->roll = 0;
    modes->pitch = 0;
    modes->warp = 0;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        modes->height += (int32_t)config->w_height[ch] * channel[ch];
        modes->roll += (int32_t)config->w_roll[ch] * channel[ch];
        modes->pitch += (int32_t)config->w_pitch[ch] * channel[ch];
        modes->warp += (int32_t)config->w_warp[ch] * channel[ch];
    }
}

void lift_sync_forward(
    const lift_sync_config_t *config,
    const int32_t position[LIFT_CHANNEL_COUNT],
    lift_sync_modes_t *modes)
{
    if ((config == NULL) || (position == NULL) || (modes == NULL))
    {
        return;
    }

    sync_forward_i32(config, position, modes);
}

void lift_sync_forward_speed(
    const lift_sync_config_t *config,
    const int32_t speed[LIFT_CHANNEL_COUNT],
    lift_sync_modes_t *modes)
{
    if ((config == NULL) || (speed == NULL) || (modes == NULL))
    {
        return;
    }

    sync_forward_i32(config, speed, modes);
}

void lift_sync_distribute(
    const lift_sync_config_t *config,
    const lift_sync_effort_t *effort,
    int32_t current[LIFT_CHANNEL_COUNT])
{
    uint32_t ch;

    if ((config == NULL) || (effort == NULL) || (current == NULL))
    {
        return;
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        /*
         * 只叠加三个受控模态。翘曲模态【没有输入口】，
         * 所以它的分量在构造上就是 0，不依赖任何运行时判断。
         * 这里刻意不做除法，保证翘曲分量在整数意义下严格为 0。
         */
        current[ch] =
            (int32_t)config->w_height[ch] * effort->height +
            (int32_t)config->w_roll[ch] * effort->roll +
            (int32_t)config->w_pitch[ch] * effort->pitch;
    }
}

int32_t lift_sync_warp_component(
    const lift_sync_config_t *config,
    const int32_t current[LIFT_CHANNEL_COUNT])
{
    int32_t warp = 0;
    uint32_t ch;

    if ((config == NULL) || (current == NULL))
    {
        return 0;
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        warp += (int32_t)config->w_warp[ch] * current[ch];
    }
    return warp;
}

bool lift_sync_config_is_orthogonal(const lift_sync_config_t *config)
{
    const int8_t *rows[4];
    uint32_t i;
    uint32_t j;
    uint32_t ch;
    int32_t dot;
    int32_t norm;

    if (config == NULL)
    {
        return false;
    }

    rows[0] = config->w_height;
    rows[1] = config->w_roll;
    rows[2] = config->w_pitch;
    rows[3] = config->w_warp;

    for (i = 0U; i < 4U; i++)
    {
        norm = 0;
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            norm += (int32_t)rows[i][ch] * rows[i][ch];
        }
        if (norm != (int32_t)LIFT_CHANNEL_COUNT)
        {
            return false;
        }

        for (j = i + 1U; j < 4U; j++)
        {
            dot = 0;
            for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            {
                dot += (int32_t)rows[i][ch] * rows[j][ch];
            }
            if (dot != 0)
            {
                return false;
            }
        }
    }

    return true;
}
