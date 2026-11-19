#ifndef __EANGLE_DRV_H
#define __EANGLE_DRV_H

#include "main.h"

typedef struct
{
    u8    dir;                        // 编码器方向
    u8    pole_pairs;                 // 转子极对数
    s32   encoder_val;                // 编码器原始数据
    s32   encoder_val_max;            // 编码器最大原始值
    s32   encoder_val_change;         // 编码器值变化量
    u16   calib_flag;                 // 校准完成标志
    s32   calib_offset;               // 转子零位偏差
    float ts;
    float electrical_angle_pu;        // 编码器电角度标幺值
    float electrical_angle_spd_set;   // 给定的电角速度
    float electrical_angle_set_pu;    // 给定电角度标幺值
} e_angle_struct;

void electrical_angle_generator(e_angle_struct *p);
void calculate_encoder_data(e_angle_struct *p);


#endif 


