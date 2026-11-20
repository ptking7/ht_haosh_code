#include "eangle_drv.h"                           

/**
 * 函数功能: 电角度发生器（开环模式）
 * 输入参数: p - 电角度结构体指针，包含设定速度、采样时间等参数
 * 返回参数: 无
 * 说    明: 根据设定的电角速度，通过积分计算生成连续递增（或递减）的电角度，
 *           用于无传感器开环启动或虚拟角度生成。电角度以标幺值（0~1）表示，一周对应1.0。
 */
void electrical_angle_generator(e_angle_struct *p)
{
    // ------------------------------------------------------------------------
    // 1. 电角度积分更新（位置 = 位置 + 速度 × 时间）
    // ------------------------------------------------------------------------
    // p->electrical_angle_spd_set : 设定电角速度（单位：转/秒？通常为Hz或rps）
    // p->ts                       : 控制周期（秒）
    // 0.01666f = 1.0f / 60.0f，此处用于单位转换：
    //   假定 electrical_angle_spd_set 单位是 RPM（转/分钟），乘以 ts 后需除以60转换为转/秒，
    //   再积分得到转数（小数部分即为电角度标幺值）。
    // 最终累加到 electrical_angle_set_pu（标幺电角度，范围0~1）。
    p->electrical_angle_set_pu += (p->ts * p->electrical_angle_spd_set * 0.01666f);

    // ------------------------------------------------------------------------
    // 2. 边界处理：将电角度限制在 [0, 1) 范围内（循环累加）
    // ------------------------------------------------------------------------
    // 当电角度超过上限1.0时，减去1.0，相当于角度绕回0点
    if(p->electrical_angle_set_pu >= 1)
    {
        p->electrical_angle_set_pu = p->electrical_angle_set_pu - 1;
    }
    // 当电角度低于0时，加上1.0（通常发生在设定速度为负时）
    if(p->electrical_angle_set_pu < 0)
    {
        p->electrical_angle_set_pu = p->electrical_angle_set_pu + 1;
    }
}


/**
 * 函数功能: 计算外部编码器数据（有感模式）
 * 输入参数: p - 电角度结构体指针，包含编码器原始值、校准偏移、极对数等参数
 * 返回参数: 无
 * 说    明: 根据外部增量式编码器或绝对式编码器的原始计数值，经过方向处理、零点校准、
 *           极对数映射，计算出当前转子的电角度（标幺值，范围0~1）。
 */
void calculate_encoder_data(e_angle_struct *p)
{
    s32 encoder_max = p->encoder_val_max;
    s32 normalized_encoder;
    s32 electrical_val;
    s32 relative;

    if(encoder_max <= 0)
    {
        p->electrical_angle_pu = 0.0f;
        return;
    }

    /*
     * Work on a local copy so encoder_val always remains the raw sensor value.
     * Direction reversal and offset subtraction are normalized before scaling.
     */
    normalized_encoder = p->encoder_val;
    if(p->dir == 1)
    {
        normalized_encoder = encoder_max - normalized_encoder;
    }

    normalized_encoder %= encoder_max;
    if(normalized_encoder < 0)
    {
        normalized_encoder += encoder_max;
    }

    relative = normalized_encoder - p->calib_offset;
    relative %= encoder_max;
    if(relative < 0)
    {
        relative += encoder_max;
    }

    electrical_val = (relative * p->pole_pairs) % encoder_max;
    if(electrical_val < 0)
    {
        electrical_val += encoder_max;
    }

    p->electrical_angle_pu =
        (float)electrical_val / (float)encoder_max;
}




