#ifndef __SAMPLE_DRV_H
#define __SAMPLE_DRV_H

#include "main.h"

typedef struct
{    
    s8    current_dir;                // 电流采样方向
    u8    calib_end_flag;             // 校准完成标志
    u16   offset_cnt;                 // 基准值计算次数
    s32   bus_offset;                 // 母线电压基准值
    s32   iu_offset;                  // U相电流偏置值
    s32   iv_offset;                  // V相电流偏置值
    s32   iw_offset;                  // W相电流偏置值
    s32   bus_raw;                    // 母线电压原始值
    s32   iu_raw;                     // U相电流原始值
    s32   iv_raw;                     // V相电流原始值
    s32   iw_raw;                     // W相电流原始值
    float iu_real;                    // U相电流真实值
    float iv_real;                    // V相电流真实值
    float iw_real;                    // W相电流真实值
    float bus_real;                   // 母线电压真实值(动态)
    float bus_calib_real;             // 母线电压真实值(校准值，静态)
    float bus_change;                 // 母线电压变化值
    float bus_factor;                 // 母线电压计算系数
    float current_factor;             // 相电流计算系数
    u16   adc_buff[3];                // 用于ADC规则通道接收数据
} sample_struct;


void calculate_adc_offset(sample_struct *p);
void calculate_phase_current(sample_struct *p);
void calculate_bus_voltage(sample_struct *p);





#endif 


