#ifndef __PID_DRV_H
#define __PID_DRV_H

#include "main.h"

typedef struct 
{
    float kp;          //比例系数
    float ki;          //积分系数
    float kd;          //微分系数
    float ref;         //目标值
    float fbk;         //反馈值
    float out;         //输出值
    float err;         //本次误差
    float err_last;    //上次误差
    float allow_err;   //允许误差
    float integrate;   //积分项
    float out_max;     //输出上限
    float out_min;     //输出下限
    float kp_max;      //比例系数上限（适用于分段式或模糊PID）
    float kp_min;      //比例系数下限（适用于分段式或模糊PID）
} pid_struct;

void pid_clear(pid_struct *p);
void pid_control(pid_struct *p);

#endif 


