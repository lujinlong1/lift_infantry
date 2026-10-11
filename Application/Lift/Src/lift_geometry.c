#include "lift_geometry.h"

#include <stddef.h>

/** int64 饱和到 int32。 */
static int32_t geometry_sat_i32(int64_t value)
{
    if (value > 2147483647LL)
    {
        return 2147483647;
    }
    if (value < (-2147483647LL - 1LL))
    {
        return (-2147483647 - 1);
    }
    return (int32_t)value;
}

void lift_geometry_config_3508_direct(lift_geometry_t *config, int32_t lead_um)
{
    if (config == NULL)
    {
        return;
    }

    config->counts_per_rev = 8192;
    config->lead_um = lead_um;
    config->drive_num = 1;
    config->drive_den = 1;
    /*
     * 默认按【转子侧】处理。这一项必须实机标定确认：
     * 手转丝杆整一圈，看 total_ecd 是否恰好变化 8192；
     * 再看给定转速下 speed_rpm 的数量级是否符合转子侧。
     */
    config->speed_is_rotor = true;
    config->gear_num = 3591;
    config->gear_den = 187;
}

bool lift_geometry_validate(const lift_geometry_t *config)
{
    if (config == NULL)
    {
        return false;
    }
    if (config->counts_per_rev <= 0)
    {
        return false;
    }
    if (config->lead_um <= 0)
    {
        return false;
    }
    if (config->drive_num <= 0)
    {
        return false;
    }
    if (config->drive_den <= 0)
    {
        return false;
    }
    if (config->speed_is_rotor)
    {
        if (config->gear_num <= 0)
        {
            return false;
        }
        if (config->gear_den <= 0)
        {
            return false;
        }
    }
    return true;
}

int32_t lift_geometry_ecd_to_um(const lift_geometry_t *config, int32_t ecd)
{
    int64_t numerator;

    if (!lift_geometry_validate(config))
    {
        return 0;
    }

    numerator = (int64_t)ecd * (int64_t)config->lead_um *
                (int64_t)config->drive_den;

    return geometry_sat_i32(
        numerator /
        ((int64_t)config->counts_per_rev * (int64_t)config->drive_num));
}

int32_t lift_geometry_um_to_ecd(const lift_geometry_t *config, int32_t um)
{
    int64_t numerator;

    if (!lift_geometry_validate(config))
    {
        return 0;
    }

    numerator = (int64_t)um * (int64_t)config->counts_per_rev *
                (int64_t)config->drive_num;

    return geometry_sat_i32(
        numerator /
        ((int64_t)config->lead_um * (int64_t)config->drive_den));
}

int32_t lift_geometry_speed_to_um_per_s(
    const lift_geometry_t *config,
    int16_t speed_rpm)
{
    int64_t numerator;
    int64_t denominator;

    if (!lift_geometry_validate(config))
    {
        return 0;
    }

    numerator = (int64_t)speed_rpm * (int64_t)config->drive_den *
                (int64_t)config->lead_um;
    denominator = (int64_t)config->drive_num * 60LL;

    if (config->speed_is_rotor)
    {
        /* 转子 RPM 先换算到输出轴 RPM，再换算到位移速度。 */
        numerator *= (int64_t)config->gear_den;
        denominator *= (int64_t)config->gear_num;
    }

    return geometry_sat_i32(numerator / denominator);
}
