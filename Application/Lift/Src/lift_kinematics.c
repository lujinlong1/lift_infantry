#include "lift_kinematics.h"

#include <stddef.h>

/* 微米与毫米之间的换算。全部整数，避免浮点在换算链里引入不可复现的误差。 */
#define KIN_UM_PER_MM            1000
/*
 * counts/mm 的定点标尺：counts_per_mm_x1000 用 1e6 表示 1000.000。
 * 选 1e6 是因为它同时能容纳 0.001 计数/mm 的分辨率与
 * "模数 10 的齿轮 + 19.2 减速比"这种 192000 量级的分子，不会溢出。
 */
#define KIN_PPM_SCALE            1000000

/*
 * 把减速比按 ×1e6 定点化。gear_ratio 已经在上游限幅到 [0.001, 100000]，
 * 所以 (int64)(gear_ratio × 1e6 + 0.5) 最大 1e11，安全。
 */
#define KIN_GEAR_SCALE           1000000

/** int64 饱和到 int32（换算结果超出编码器量程时宁可饱和也不要 UB）。 */
static int32_t kin_sat_i32(int64_t value)
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

/**
 * @brief 四舍五入的整数除法（round half away from zero）。
 *
 * 为什么不能直接截断：725 计数/mm 的尺度上，截断会白送将近 1 个计数，
 * 看起来无所谓；但 ecd→mm 那一步的方向是"除以 725"，截断一次就是
 * 整整 1 mm 的常量偏置 —— 而且对正负不对称，会让"正反换算一致"这条
 * 最基本的自检通不过。往返换算必须是对称的，所以两边都用同一个舍入。
 */
static int64_t kin_div_round(int64_t numerator, int64_t denominator)
{
    int64_t quotient;

    if (denominator == 0)
    {
        return 0;
    }
    if (denominator < 0)
    {
        numerator = -numerator;
        denominator = -denominator;
    }

    if (numerator >= 0)
    {
        quotient = (numerator + (denominator / 2)) / denominator;
    }
    else
    {
        quotient = -((-numerator + (denominator / 2)) / denominator);
    }
    return quotient;
}

/** float 是否有限且量级合理（KIN 层不接受 NaN / inf / 天文数字）。 */
static bool kin_float_valid(float value)
{
    return (value == value) && (value > -1.0e9F) && (value < 1.0e9F);
}

/**
 * @brief 派生单轴系数。全部用 int64 分数保存，不做浮点近似。
 * @return true 表示该轴配置自洽，info 已填好；false 表示非法，info 只保证被清零。
 */
static bool kin_derive_axis(
    const lift_kin_axis_config_t *axis,
    lift_kin_axis_info_t *info)
{
    int64_t gear_scaled;
    int64_t diameter_um;
    int64_t circumference_um;
    int64_t numerator;

    if ((axis == NULL) || (info == NULL))
    {
        return false;
    }

    *info = (lift_kin_axis_info_t){0};

    if (axis->counts_per_rev <= 0)
    {
        return false;
    }
    if (!kin_float_valid(axis->gear_ratio) || (axis->gear_ratio <= 0.0F))
    {
        return false;
    }
    if (!kin_float_valid(axis->pinion_diameter_mm) ||
        (axis->pinion_diameter_mm <= 0.0F))
    {
        return false;
    }
    if ((axis->direction != 1) && (axis->direction != -1))
    {
        return false;
    }

    /* 减速比定点化：gear_ratio × 1e6。 */
    gear_scaled = (int64_t)((axis->gear_ratio * (float)KIN_GEAR_SCALE) + 0.5F);
    if (gear_scaled <= 0)
    {
        return false;
    }

    diameter_um = (int64_t)((axis->pinion_diameter_mm *
                             (float)KIN_UM_PER_MM) + 0.5F);
    if (diameter_um <= 0)
    {
        return false;
    }

    /*
     * 分度圆周长。这里【不引入 π】：
     *   实际标定表里填的是"输出轴转一圈齿条走多少 µm"（也就是分度圆周长），
     *   而我们在配置里只暴露【直径】。两者只差一个 π，用整数 3141593/1000000
     *   近似 π 到 1e-6，对 69 mm 而言误差约 0.07 µm/圈，远小于机械误差。
     */
    circumference_um = (diameter_um * 3141593LL) / 1000000LL;
    if (circumference_um <= 0)
    {
        return false;
    }

    /*
     * counts_per_mm
     *
     *   输出轴一圈齿条走   circumference_um / 1000 毫米
     *   输出轴一圈需要     counts_per_rev × gear_ratio 个【转子侧】计数
     *                      = counts_per_rev × gear_scaled / 1e6
     *
     *                        cpr × gear_scaled / 1e6
     *   counts_per_mm = ─────────────────────────────────
     *                          circumference_um / 1e6
     *
     *                       cpr × gear_scaled          分母再乘 1000：
     *                     = ───────────────────   ← 因为 1 mm = 1000 µm
     *                       circumference_um × 1000
     *
     * 用一个分数而不是一个整数比例，是为了让减速比保持精确——
     * 写成整数比例会丢精度，没有理由白送。这条路径是**派生值**，
     * 通常会被下面的 counts_per_mm 覆盖（见那里）。
     */
    numerator = (int64_t)axis->counts_per_rev * gear_scaled;

    info->counts_per_rev_num = numerator;
    info->counts_per_rev_den = KIN_GEAR_SCALE;
    info->counts_per_mm_num = numerator;
    info->counts_per_mm_den = circumference_um * KIN_UM_PER_MM;
    info->circumference_um = circumference_um;

    /*
     * ★ 实测系数覆盖。
     *
     * 直径在内部按 µm 取整，于是派生出来的 counts/mm 只能落在离散点上
     * （实测例子：46.693 mm → 750.0072，46.694 mm → 749.9919，
     *   **永远落不到整 750.0**）。而标定得到的权威恰恰是那个比例本身：
     *   150000 计数 = 200 mm ⇒ 750.0 计数/mm。
     *
     * 所以填了就覆盖，让换算严格等于实测比例。
     */
    if (kin_float_valid(axis->counts_per_mm) && (axis->counts_per_mm > 0.0F))
    {
        /*
         * ⚠️ 分母必须与派生路径【同一套约定】：
         *      counts_per_mm_num / counts_per_mm_den = counts_per_mm / 1000
         *    （派生路径的 den = circumference_um × 1000 也是这个含义）
         *    分子所以是 counts_per_mm × 1000（精确到 0.001 计数/mm）。
         */
        info->counts_per_mm_num =
            (int64_t)((axis->counts_per_mm * 1000.0F) + 0.5F);
        info->counts_per_mm_den = KIN_UM_PER_MM;
        if (info->counts_per_mm_num <= 0)
        {
            return false;
        }
    }

    info->valid = true;
    /* ready 还要看行程区间，由 lift_kin_axis_is_ready() 决定。 */
    info->ready = false;
    return true;
}

void lift_kin_config_init_uncalibrated(lift_kin_config_t *config)
{
    uint32_t ch;

    if (config == NULL)
    {
        return;
    }

    *config = (lift_kin_config_t){0};

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        /* 系数留 0 → 非法；方向留 0 → 非法。整个配置明确是"没标定"。 */
        config->axis[ch].counts_per_rev = 0;
        config->axis[ch].gear_ratio = 0.0F;
        config->axis[ch].pinion_diameter_mm = 0.0F;
        config->axis[ch].direction = 0;
        config->zero_ecd[ch] = 0;
        /* min >= max 表示不启用限位。 */
        config->position_min_mm[ch] = 0;
        config->position_max_mm[ch] = 0;
    }
}

void lift_kin_config_nominal_rack_and_pinion(
    lift_kin_config_t *config,
    const int8_t direction[LIFT_CHANNEL_COUNT])
{
    uint32_t ch;

    if (config == NULL)
    {
        return;
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        /*
         * ★ 2026-10-11 实测定案（取代图纸推算，见 lift_kinematics.h）：
         *     150000 计数 / 200 mm        → 750.0 计数/mm
         *     110000 计数 / 齿轮一圈      → 13.43:1，分度圆 46.693 mm
         *   两条测量互相印证：110000 ÷ (π × 46.693) = 749.9 ≈ 750。
         *
         * 直径取 46.693（而不是 46.7）是为了让派生出来的 counts/mm 落回
         * 整 750.0 —— 实测的权威是"150000 计数 = 200 mm"这个比例，
         * 直径只是让公式结构不变的一个中间量。
         */
        config->axis[ch].counts_per_rev = 8192;
        config->axis[ch].gear_ratio = 13.43F;
        config->axis[ch].pinion_diameter_mm = 46.693F;
        /*
         * 实测比例是**权威**：150000 计数 = 200 mm ⇒ 750.0 计数/mm。
         * 直接锚定它，不让它从几何参数里派生（派生值落不到整数）。
         * 见 lift_kin_axis_config_t 里 counts_per_mm 的说明。
         */
        config->axis[ch].counts_per_mm = 750.0F;
        if (direction != NULL)
        {
            config->axis[ch].direction = direction[ch];
        }
        /* 行程区间保持调用方已填的值，本函数不碰。 */
    }
}

bool lift_kin_axis_config_is_valid(const lift_kin_axis_config_t *axis)
{
    lift_kin_axis_info_t info;

    return kin_derive_axis(axis, &info);
}

bool lift_kin_config_is_valid(const lift_kin_config_t *config)
{
    uint32_t ch;

    if (config == NULL)
    {
        return false;
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        if (!lift_kin_axis_config_is_valid(&config->axis[ch]))
        {
            return false;
        }
    }
    return true;
}

int32_t lift_kin_counts_per_mm_x1000(
    const lift_kin_config_t *config,
    uint32_t axis)
{
    lift_kin_axis_info_t info;
    int64_t value;

    if ((config == NULL) || (axis >= (uint32_t)LIFT_CHANNEL_COUNT))
    {
        return 0;
    }
    if (!kin_derive_axis(&config->axis[axis], &info))
    {
        return 0;
    }

    value = (info.counts_per_mm_num * 1000LL) / info.counts_per_mm_den;
    return kin_sat_i32(value);
}

float lift_kin_counts_per_mm(const lift_kin_config_t *config, uint32_t axis)
{
    lift_kin_axis_info_t info;

    if ((config == NULL) || (axis >= (uint32_t)LIFT_CHANNEL_COUNT))
    {
        return 0.0F;
    }
    if (!kin_derive_axis(&config->axis[axis], &info))
    {
        return 0.0F;
    }
    return (float)info.counts_per_mm_num / (float)info.counts_per_mm_den;
}

bool lift_kin_axis_is_ready(const lift_kin_config_t *config, uint32_t axis)
{
    if ((config == NULL) || (axis >= (uint32_t)LIFT_CHANNEL_COUNT))
    {
        return false;
    }
    if (!lift_kin_axis_config_is_valid(&config->axis[axis]))
    {
        return false;
    }
    /*
     * 行程区间必须【显式配置】才认为可接受绝对位置命令。
     * min >= max 表示"没配"，而不是"区间为空" —— 与 lift_control 里
     * limit_height_* 的约定一致，全项目只用这一套语义。
     */
    if (config->position_min_mm[axis] >= config->position_max_mm[axis])
    {
        return false;
    }
    if (config->position_max_mm[axis] > LIFT_KIN_MAX_TRAVEL_MM)
    {
        return false;
    }
    if (config->position_min_mm[axis] < -LIFT_KIN_MAX_TRAVEL_MM)
    {
        return false;
    }
    return true;
}

bool lift_kin_all_ready(const lift_kin_config_t *config)
{
    uint32_t ch;

    if (config == NULL)
    {
        return false;
    }
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        if (!lift_kin_axis_is_ready(config, ch))
        {
            return false;
        }
    }
    return true;
}

bool lift_kin_validate(
    const lift_kin_config_t *config,
    lift_kin_calibration_t *out,
    float tolerance_permille)
{
    uint32_t ch;
    bool all_valid = true;
    bool all_ready = true;
    float tol;

    if (config == NULL)
    {
        return false;
    }

    tol = kin_float_valid(tolerance_permille) ? tolerance_permille : 0.0F;

    if (out != NULL)
    {
        *out = (lift_kin_calibration_t){0};
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        const bool valid = lift_kin_axis_config_is_valid(&config->axis[ch]);
        const bool ready = lift_kin_axis_is_ready(config, ch);
        int32_t ppm1000 = 0;

        if (!valid)
        {
            all_valid = false;
        }
        if (!ready)
        {
            all_ready = false;
        }

        if (valid)
        {
            ppm1000 = lift_kin_counts_per_mm_x1000(config, ch);
        }

        if (out != NULL)
        {
            lift_kin_axis_info_t info;

            (void)kin_derive_axis(&config->axis[ch], &info);
            out->axes_valid[ch] = valid;
            out->axes_ready[ch] = ready;
            out->counts_per_mm_x100[ch] =
                (int32_t)(((int64_t)ppm1000 * 100LL) / 1000LL);

            if (valid && (tol > 0.0F))
            {
                const float deviation =
                    (((float)ppm1000 / 1000.0F) -
                     LIFT_KIN_NOMINAL_COUNTS_PER_MM) /
                    LIFT_KIN_NOMINAL_COUNTS_PER_MM;
                /* 偏差按千分比比较，避免在 Watch 里看小数。 */
                out->deviates_from_nominal[ch] =
                    (deviation > (tol / 1000.0F)) ||
                    (deviation < (-tol / 1000.0F));
            }
        }
        else if (out != NULL)
        {
            out->axes_valid[ch] = false;
            out->axes_ready[ch] = false;
            out->counts_per_mm_x100[ch] = 0;
            out->deviates_from_nominal[ch] = false;
        }
        else
        {
            /* 不需要输出，什么都不做。 */
        }
    }

    if (out != NULL)
    {
        out->all_valid = all_valid;
        out->all_ready = all_ready;
    }
    return all_valid;
}

/* ------------------------------------------------------------------ */
/*  位置换算                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief 取某轴的 counts_per_mm 分子/分母（已派生）。内部用。
 * @return false 表示配置非法。
 */
static bool kin_ppm_fraction(
    const lift_kin_config_t *config,
    uint32_t axis,
    int64_t *num,
    int64_t *den)
{
    lift_kin_axis_info_t info;

    if ((config == NULL) || (axis >= (uint32_t)LIFT_CHANNEL_COUNT))
    {
        return false;
    }
    if (!kin_derive_axis(&config->axis[axis], &info))
    {
        return false;
    }
    if ((num != NULL) && (den != NULL))
    {
        *num = info.counts_per_mm_num;
        *den = info.counts_per_mm_den;
    }
    return true;
}

bool lift_kin_mm_to_ecd(
    const lift_kin_config_t *config,
    uint32_t axis,
    int32_t mm,
    int32_t *out)
{
    int64_t num;
    int64_t den;
    int64_t delta;

    if ((out == NULL) || (config == NULL) ||
        (axis >= (uint32_t)LIFT_CHANNEL_COUNT))
    {
        return false;
    }
    if (mm > LIFT_KIN_MAX_TRAVEL_MM || mm < -LIFT_KIN_MAX_TRAVEL_MM)
    {
        return false;
    }
    if (!kin_ppm_fraction(config, axis, &num, &den))
    {
        return false;
    }

    /*
     * 逻辑计数 = mm × counts_per_mm。
     * 先乘后除，int64 中间量：mm ≤ 1000、num ≤ 8192×1e11、den ≥ 1e5。
     * |mm × num| ≤ 8.2e17 —— 在 int64（9.2e18）以内，安全。
     */
    delta = kin_div_round((int64_t)mm * num, den);
    *out = kin_sat_i32(delta);
    return true;
}

bool lift_kin_ecd_to_mm(
    const lift_kin_config_t *config,
    uint32_t axis,
    int32_t ecd,
    int32_t *out)
{
    int64_t num;
    int64_t den;
    int32_t ppm1000;

    if ((out == NULL) || (config == NULL) ||
        (axis >= (uint32_t)LIFT_CHANNEL_COUNT))
    {
        return false;
    }

    ppm1000 = lift_kin_counts_per_mm_x1000(config, axis);
    if (ppm1000 <= 0)
    {
        return false;
    }
    if (!kin_ppm_fraction(config, axis, &num, &den))
    {
        return false;
    }

    /* mm = ecd / (num/den) = ecd × den / num。 */
    *out = kin_sat_i32(kin_div_round((int64_t)ecd * den, num));
    return true;
}

bool lift_kin_position_from_raw(
    const lift_kin_config_t *config,
    uint32_t axis,
    int32_t raw_ecd,
    int32_t *out)
{
    int8_t direction;
    int32_t logical_raw;
    int32_t logical_zero;

    if ((out == NULL) || (config == NULL) ||
        (axis >= (uint32_t)LIFT_CHANNEL_COUNT))
    {
        return false;
    }

    direction = config->axis[axis].direction;
    if ((direction != 1) && (direction != -1))
    {
        return false;
    }

    /*
     * 逻辑量 = direction × 原始量。零点也在原始域，所以要【分别】折算再相减，
     * 不能先减再乘（那等价，但分别折算能让"零点存原始域"这条约定自解释）。
     */
    logical_raw = (direction < 0) ? -raw_ecd : raw_ecd;
    logical_zero = (direction < 0) ? -config->zero_ecd[axis]
                                   : config->zero_ecd[axis];

    *out = kin_sat_i32((int64_t)logical_raw - (int64_t)logical_zero);
    return true;
}

/* ------------------------------------------------------------------ */
/*  速度换算                                                            */
/* ------------------------------------------------------------------ */

bool lift_kin_speed_to_mm_s(
    const lift_kin_config_t *config,
    uint32_t axis,
    int16_t speed_rpm,
    float *out)
{
    lift_kin_axis_info_t info;
    float counts_per_output_rev;
    float mm_per_output_rev;
    float rpm_output;

    if ((out == NULL) || (config == NULL) ||
        (axis >= (uint32_t)LIFT_CHANNEL_COUNT))
    {
        return false;
    }
    if (!kin_derive_axis(&config->axis[axis], &info))
    {
        return false;
    }

    counts_per_output_rev = (float)info.counts_per_rev_num /
                            (float)info.counts_per_rev_den;
    if (counts_per_output_rev <= 0.0F)
    {
        return false;
    }

    /*
     * 输出轴一圈齿条走的毫米数 = 分度圆周长(µm) / 1000。
     * 直接用保存好的 circumference_um，【不要】从 counts_per_mm 那对分数反推——
     * 那对分数的分母里已经折掉了 mm→µm 的 1000，反推会把速度算大 1000 倍
     * （本模块早期版本正是这么错的，被 [9] 速度往返用例抓出来）。
     */
    mm_per_output_rev = (float)info.circumference_um / (float)KIN_UM_PER_MM;
    if (mm_per_output_rev <= 0.0F)
    {
        return false;
    }

    /* C620 的 speed_rpm 是【转子侧】，先除减速比回到输出轴。 */
    rpm_output = (float)speed_rpm /
                 (float)config->axis[axis].gear_ratio;

    *out = (rpm_output / 60.0F) * mm_per_output_rev *
           (float)config->axis[axis].direction;
    return true;
}

bool lift_kin_mm_s_to_speed(
    const lift_kin_config_t *config,
    uint32_t axis,
    float mm_s,
    int16_t *out)
{
    lift_kin_axis_info_t info;
    float counts_per_output_rev;
    float mm_per_output_rev;
    float rpm_output;
    float rpm_rotor;

    if ((out == NULL) || (config == NULL) ||
        (axis >= (uint32_t)LIFT_CHANNEL_COUNT))
    {
        return false;
    }
    if (!kin_float_valid(mm_s))
    {
        return false;
    }
    if (!kin_derive_axis(&config->axis[axis], &info))
    {
        return false;
    }

    counts_per_output_rev = (float)info.counts_per_rev_num /
                            (float)info.counts_per_rev_den;
    if (counts_per_output_rev <= 0.0F)
    {
        return false;
    }
    mm_per_output_rev = (float)info.circumference_um / (float)KIN_UM_PER_MM;
    if (mm_per_output_rev <= 0.0F)
    {
        return false;
    }

    rpm_output = (mm_s * (float)config->axis[axis].direction) /
                 mm_per_output_rev * 60.0F;
    rpm_rotor = rpm_output * config->axis[axis].gear_ratio;

    if (rpm_rotor > 32767.0F)
    {
        rpm_rotor = 32767.0F;
    }
    if (rpm_rotor < -32768.0F)
    {
        rpm_rotor = -32768.0F;
    }
    *out = (int16_t)(rpm_rotor + ((rpm_rotor >= 0.0F) ? 0.5F : -0.5F));
    return true;
}

/* ------------------------------------------------------------------ */
/*  行程与限位                                                          */
/* ------------------------------------------------------------------ */

bool lift_kin_limits_mm(
    const lift_kin_config_t *config,
    uint32_t axis,
    int32_t *min_mm,
    int32_t *max_mm)
{
    if ((config == NULL) || (axis >= (uint32_t)LIFT_CHANNEL_COUNT))
    {
        return false;
    }
    if (config->position_min_mm[axis] >= config->position_max_mm[axis])
    {
        return false;
    }
    if (min_mm != NULL)
    {
        *min_mm = config->position_min_mm[axis];
    }
    if (max_mm != NULL)
    {
        *max_mm = config->position_max_mm[axis];
    }
    return true;
}

bool lift_kin_clamp_mm(
    const lift_kin_config_t *config,
    uint32_t axis,
    int32_t *mm)
{
    int32_t minimum;
    int32_t maximum;

    if ((mm == NULL) || (config == NULL) ||
        (axis >= (uint32_t)LIFT_CHANNEL_COUNT))
    {
        return false;
    }
    if (!lift_kin_limits_mm(config, axis, &minimum, &maximum))
    {
        return false;
    }

    if (*mm > maximum)
    {
        *mm = maximum;
        return true;
    }
    if (*mm < minimum)
    {
        *mm = minimum;
        return true;
    }
    return false;
}

bool lift_kin_out_of_limits(
    const lift_kin_config_t *config,
    uint32_t axis,
    int32_t mm)
{
    int32_t minimum;
    int32_t maximum;

    if (!lift_kin_limits_mm(config, axis, &minimum, &maximum))
    {
        return false;
    }
    return (mm > maximum) || (mm < minimum);
}

/* ------------------------------------------------------------------ */
/*  编码器回绕                                                          */
/* ------------------------------------------------------------------ */

bool lift_kin_wrap_delta(
    int32_t counts_per_rev,
    int32_t last_ecd,
    int32_t new_ecd,
    int32_t *delta)
{
    int32_t half;
    int32_t raw;

    if (delta == NULL)
    {
        return false;
    }
    if (counts_per_rev <= 1)
    {
        return false;
    }
    if ((last_ecd < 0) || (last_ecd >= counts_per_rev) ||
        (new_ecd < 0) || (new_ecd >= counts_per_rev))
    {
        return false;
    }

    half = counts_per_rev / 2;
    raw = new_ecd - last_ecd;

    if (raw < -half)
    {
        raw += counts_per_rev;
    }
    else if (raw > half)
    {
        raw -= counts_per_rev;
    }
    else
    {
        /* 同圈，直接用原始差。 */
    }

    *delta = raw;
    return true;
}

/* ------------------------------------------------------------------ */
/*  四轴一致性                                                          */
/* ------------------------------------------------------------------ */

bool lift_kin_position_spread_mm(
    const lift_kin_config_t *config,
    const int32_t logical_ecd[LIFT_CHANNEL_COUNT],
    int32_t *out)
{
    int32_t lowest = 0;
    int32_t highest = 0;
    uint32_t ch;

    if ((config == NULL) || (logical_ecd == NULL) || (out == NULL))
    {
        return false;
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        int32_t mm;

        if (!lift_kin_ecd_to_mm(config, ch, logical_ecd[ch], &mm))
        {
            return false;
        }
        if (ch == 0U)
        {
            lowest = mm;
            highest = mm;
        }
        else
        {
            if (mm < lowest)
            {
                lowest = mm;
            }
            if (mm > highest)
            {
                highest = mm;
            }
        }
    }

    *out = kin_sat_i32((int64_t)highest - (int64_t)lowest);
    return true;
}

bool lift_kin_axis_deviation_mm(
    const lift_kin_config_t *config,
    const int32_t logical_ecd[LIFT_CHANNEL_COUNT],
    uint32_t axis,
    int32_t *out)
{
    int32_t values[LIFT_CHANNEL_COUNT];
    int32_t sampled[LIFT_CHANNEL_COUNT];
    int32_t reference;
    int64_t sum = 0;
    uint32_t sampled_count;
    uint32_t ch;
    uint32_t i;

    if ((config == NULL) || (logical_ecd == NULL) || (out == NULL) ||
        (axis >= (uint32_t)LIFT_CHANNEL_COUNT))
    {
        return false;
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        if (!lift_kin_ecd_to_mm(config, ch, logical_ecd[ch], &values[ch]))
        {
            return false;
        }
    }

    /*
     * 参考量 = 【去掉一个最高和一个最低之后的平均】。
     *
     * 四轴时就是中间两根的均值，所以"一根高出 10 mm"读出来正好是 +10。
     *
     * ⚠️ 刻意【不用四轴平均】：平均会被跑偏的那一根拖走 —— 一根高出 10 mm
     *    时平均只低 2.5 mm，判据读到 7.5 而不是 10，偏差越大反而越钝。
     *    这里要回答的是"这一根比其它几根高多少"，参考量就不该包含这一根。
     *
     * ⚠️ 也刻意【不按"值等于极值"剔除】：四轴里三根同高时，"等于最低值"
     *    会一次命中三根，参考量只剩跑偏的那一根，判据彻底反号。
     *    必须按【名次】各剔一个，而不是按值。
     *
     * 也不取严格中位数：四轴是偶数个，"中位数"本身就没有单一定义。
     */
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        sampled[ch] = values[ch];
    }
    sampled_count = (uint32_t)LIFT_CHANNEL_COUNT;

    if (sampled_count > 2U)
    {
        uint32_t drop_at = 0U;

        /* 剔除一个最大值（按名次，不按值） */
        for (i = 1U; i < sampled_count; i++)
        {
            if (sampled[i] > sampled[drop_at])
            {
                drop_at = i;
            }
        }
        for (i = drop_at; (i + 1U) < sampled_count; i++)
        {
            sampled[i] = sampled[i + 1U];
        }
        sampled_count--;

        /* 剔除一个最小值。索引必须【重新找】—— 上一步的拷贝会挪动下标。 */
        drop_at = 0U;
        for (i = 1U; i < sampled_count; i++)
        {
            if (sampled[i] < sampled[drop_at])
            {
                drop_at = i;
            }
        }
        for (i = drop_at; (i + 1U) < sampled_count; i++)
        {
            sampled[i] = sampled[i + 1U];
        }
        sampled_count--;
    }

    if (sampled_count == 0U)
    {
        /* 通道数 <= 2 的退化情形：用极值中点。 */
        int32_t lowest = values[0];
        int32_t highest = values[0];

        for (ch = 1U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            if (values[ch] < lowest)
            {
                lowest = values[ch];
            }
            if (values[ch] > highest)
            {
                highest = values[ch];
            }
        }
        reference = (int32_t)(((int64_t)lowest + (int64_t)highest) / 2);
    }
    else
    {
        for (i = 0U; i < sampled_count; i++)
        {
            sum += (int64_t)sampled[i];
        }
        reference = (int32_t)(sum / (int64_t)sampled_count);
    }

    *out = kin_sat_i32((int64_t)values[axis] - (int64_t)reference);
    return true;
}
