#ifndef __MATH_DRV_H
#define __MATH_DRV_H

#include "main.h"

#define SIN_RAD     0X0300
#define U0_90       0x0000 
#define U90_180     0x0100
#define U180_270    0x0200
#define U270_360    0x0300

#define MOVE_WINDOW_SIZE 2
typedef struct
{
    u16   data_num;                   //数据位置
    float in_val;                     //输入数据
    float out_val;                    //输出平均值
    float data_buf[MOVE_WINDOW_SIZE]; //采样数据区
} move_average_filter_struct;
   
float my_atan2(float x, float y);
void calculate_sin_cos(float angle, float *sin_val, float *cos_val);
void amplitude_limit(float *input, float min, float max);
void move_average_filter(move_average_filter_struct *p);

#endif 


