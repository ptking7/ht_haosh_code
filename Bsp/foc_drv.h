#ifndef __FOC_DRV_H
#define __FOC_DRV_H

#include "main.h"

typedef struct
{        
    float iu;            // U相电流  
    float iv;            // V相电流 
    float iw;            // W相电流    
    float ialpha;        // alpha轴电流 
    float ibeta;         // beta轴电流 
    
    float sin_val;       // 正弦值
    float cos_val;       // 余弦值
    float id;            // d轴电流 
    float iq;            // q轴电流 
    
    float id_lpf;        // d轴电流滤波值
    float iq_lpf;        // q轴电流滤波值
    float id_lpf_factor; // d轴电流滤波系数
    float iq_lpf_factor; // q轴电流滤波系数
    
    float ud;            // d轴电压 
    float uq;            // q轴电压     
    float ualpha;        // alpha轴电压
    float ubeta;         // beta轴电压        
    float ubus;          // 母线电压    
    
    u16   pwm_cycle;     // PWM周期
    u16   pwm_limit;     // 最大占空比
    u16   duty_cycle_a;  // A相占空比
    u16   duty_cycle_b;  // B相占空比
    u16   duty_cycle_c;  // C相占空比
} foc_struct;

void clark_transform(foc_struct *p);
void park_transform(foc_struct *p); 
void inverse_park_transform(foc_struct *p);
void calculate_svpwm(foc_struct *p);


#endif 


