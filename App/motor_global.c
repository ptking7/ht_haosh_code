
/**
  ******************************************************************************
  * 文件名程: 
  * 作    者: 浩然
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/

#include "motor_global.h"
#include "adc.h"
#include "tim.h"
#include "motor_publicdata.h"
#include "usart_task.h"

static u16 led_task_tim = 0;
static volatile u8 vofa_send_flag = 0;

/**
  * 函数功能: 全局初始化
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void global_init(void)
{
    HAL_Delay(100);                                         //延时等待电源稳定
    HAL_TIM_Base_Start_IT(&htim2);                          //LED任务定时器
    HAL_TIM_Base_Start_IT(&htim6);                          //VOFA发送定时器
}

/**
  * 函数功能: 主循环
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void global_loop(void)
{
    if(vofa_send_flag)
    {
        vofa_send_flag = 0;
        usart_task();
    }
}

/**
  * 函数功能: 定时器中断回调函数
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == htim1.Instance)                        //20KHZ   50US
    {
        HAL_ADCEx_InjectedStart_IT(&hadc2);                     //开启ADC注入通道中断
    }
    
    if(htim->Instance == htim2.Instance)                        //10KHZ   100US
    {
        led_task_tim++;
        if(led_task_tim >= 500)
        {
            led_task_tim = 0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
        }
    }

    if(htim->Instance == htim6.Instance)                       //100HZ   10MS
    {
        vofa_send_flag = 1;
    }
}

/**
  * 函数功能: 编码器原始数据读取
  * 输入参数: 
  * 返回参数: 编码器原始位置信息
  * 说    明: 
  */
u16 encoder_data_get(void)
{
    /*注释部分为KTH7823绝对值模式，使用需要修改引脚配置（CUBEMX）以及更改编码器板接线*/
//    u8 angle_reg[2] = {0x00,0x00};
//    u8 reg_org_data[2] = {0};
//    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
//    HAL_SPI_TransmitReceive(&hspi1, (u8 *)angle_reg, (u8 *)&reg_org_data[0], 2, 0xFFFF);
//    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
//    return (reg_org_data[0] << 8 | reg_org_data[1]) >> 2;
    return TIM3->CNT;
}

void target_set(void)
{
    switch(mc.motor.run_mode)
    {
        case CURRENT_CLOSE_LOOP:                                           //电流闭环
        {                        
            mc.iq_pid.ref = mc.sample.adc_buff[1] * 0.002f;                    //使用波轮电位器给电机目标电流（电流闭环模式下）
        }break;    
        
        case SPEED_CURRENT_LOOP:                                           //速度闭环
        {        
            mc.speed.mechanical_speed_set  =  speed_set_dir * mc.sample.adc_buff[1] * 0.5f;            //使用波轮电位器给电机目标转速（速度闭环模式下）
            if(mc.speed.mechanical_speed_set <= 5 && mc.speed.mechanical_speed_set >= -5)
            {
                mc.speed.mechanical_speed_set = 0;                               //消除电位器在0位附近采样值抖动引起电机蠕动
            }                
        }break;    
        
        case POS_SPEED_CURRENT_LOOP:                                       //位置闭环
        {
            mc.position.mechanical_pos_set = -mc.sample.adc_buff[1];            //使用波轮电位器给电机目标位置（位置闭环模式下）
        }break;    
        case STRONG_DRAG_SMO_SPEED_CURRENT_LOOP:
        {    
            mc.speed.mechanical_speed_set  =  speed_set_dir * mc.sample.adc_buff[1] * 0.5f;             //使用波轮电位器给电机目标转速（速度闭环模式下）
        }break;                
        
        case HFI_SMO_SPEED_CURRENT_CLOSE:                                      
        {       
            mc.speed.mechanical_speed_set  =  speed_set_dir * mc.sample.adc_buff[1] * 0.5f;            //使用波轮电位器给电机目标转速（速度闭环模式下）
            if(mc.speed.mechanical_speed_set <= 5 && mc.speed.mechanical_speed_set >= -5)
            {
                mc.speed.mechanical_speed_set = 0;                               //消除电位器在0位附近采样值抖动引起电机蠕动
            }                
        }break;    
    }
}

/**
  * 函数功能: ADC注入中断回调函数
  * 输入参数:
  * 返回参数:
  * 说    明: 20KHZ频率即50US执行一次
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{        
    mc.sample.iu_raw = ADC2->JDR1;              //获取相电流
    mc.sample.iw_raw = ADC2->JDR2;              //获取相电流
    mc.sample.bus_raw = mc.sample.adc_buff[0];   //获取母线电压
    mc.e_angle.encoder_val = encoder_data_get(); //获取编码器值
    target_set();                              //电位器给定目标值

    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,mc.foc.duty_cycle_a);     //更新PWM比较值
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,mc.foc.duty_cycle_b);     //更新PWM比较值
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,mc.foc.duty_cycle_c);     //更新PWM比较值
}





