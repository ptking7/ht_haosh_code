#ifndef __MOTOR_PUBLICDATA_H
#define __MOTOR_PUBLICDATA_H

#include "main.h"
#include "foc_drv.h"
#include "pid_drv.h"
#include "math_drv.h"
#include "speed_drv.h"
#include "sample_drv.h"
#include "eangle_drv.h"
#include "position_drv.h"
#include "observer_drv.h"
#include "motor_pwm_config.h"

#define LOW_RESITOR        10.0f    //母线电压检测下端电阻(KΩ)
#define HIGH_RESITOR       169.0f   //母线电压检测上端电阻(KΩ)
#define SAMPLING_RESITOR   0.001f   //相电流采样电阻(Ω)
#define MAGNIFICATION      (100.0f / 4.99f) //采样放大倍数
#define ADC_RESOLUTION     4096.0f  //ADC分辨率
#define ADC_VREF           3.3f     //ADC基准电压（V）
#define VBUS_FACTOR            ((ADC_VREF / ADC_RESOLUTION) / (LOW_RESITOR / (LOW_RESITOR + HIGH_RESITOR))) // 母线电压计算系数
#define PHASE_CURRENT_FACTOR   ((ADC_VREF / ADC_RESOLUTION) / MAGNIFICATION / SAMPLING_RESITOR)             // 相电流计算系数

#define INV_SQRT3          0.57735f //1/√3
#define OVER_CURRENT       12.0f    //最大相电流值(A)  超过这个值报错停机
#define BUS_VOLTAGE_MIN    10.0f    //最小供电电压(V)  小于这个值报错停机
#define BUS_VOLTAGE_MAX    40.0f    //最大供电电压(V)  大于这个值报错停机

#define B_VALUE            3434.0f  //B值 (25℃/50℃)3380K  (25℃/85℃)3434K  (25℃/100℃)3455K
#define TEMP_REF           298.15f  //参考温度值  25℃ + 273.15
#define RESISTOR_REF       10000.0f //参考温度下的阻值
#define RESISTOR_OTHER     10000.0f //分压的阻值




/****************参数辨识状态*******************/  
typedef enum{
    RESISTANCE_IDENTIFICATION = 0,                            // 相电阻识别
    INDUCTANCE_IDENTIFICATION,                                // 相电感识别
    ENCODER_ROTOR_ALIGN                                        // 编码器对齐
}identify_state;

/******************运行状态*********************/
typedef enum{
    ADC_CALIB = 0,                                                            // ADC校准
    MOTOR_STOP,                                                                    // 停机
    MOTOR_ERROR,                                                                // 故障报错
    MOTOR_IDENTIFY,                                                            // 参数辨识
    MOTOR_SENSORUSE,                                                        // 有感控制
    MOTOR_SENSORLESS                                                        // 无感控制
}motor_run_state;

/****************有感运行模式*******************/
#define ENCODER_CALIB                       0X00  // 编码器校准
#define CURRENT_OPEN_LOOP                    0X01  // 电流开环
#define CURRENT_CLOSE_LOOP                    0X02  // 电流闭环
#define SPEED_CURRENT_LOOP                  0X03  // 速度闭环
#define POS_SPEED_CURRENT_LOOP                0X04  // 位置闭环

/****************无感运行模式*******************/
#define STRONG_DRAG_CURRENT_OPEN            0X05  // 电流开环强拖
#define STRONG_DRAG_CURRENT_CLOSE            0X06  // 电流闭环强拖
#define STRONG_DRAG_SMO_SPEED_CURRENT_LOOP  0X07  // 强拖切滑膜速度电流闭环
#define HFI_CURRENT_CLOSE                   0X08  // 电流闭环高频注入（测试HFI角度收敛效果）
#define HFI_SPEED_CURRENT_CLOSE             0X09  // 高频注入速度电流闭环
#define HFI_POS_SPEED_CURRENT_CLOSE         0X0A  // 高频注入位置速度电流闭环
#define HFI_SMO_SPEED_CURRENT_CLOSE         0X0B
/******************无感通用运行模式*********************/
typedef enum {
    OPEN_LOOP = 0,                                  //开环
    CLOSE_LOOP                                      //闭环
}general_mode;
/******************无感观测器类型*********************/
typedef enum{
    HFI = 0,                                        //高频注入
    FLUX,                                           //磁链
    SMO                                             //滑模
}observer_mode;


/******************错误码*********************/
typedef enum{
    NONE_ERR = 0,                                                                //无错误
    ADC_CALIB_ERR,                                                        //ADC校准错误
    ENCODER_ERR,                                                                //编码器错误
    POWER_VOLT_ERR,                                                            //欠压或过压
    OVER_CURRENT_ERR,                                                        //过流
    TEMPERATURE_ERR                                                          //温度过高
}motor_error_code;



#define HALF_PI  1.5707963f
#define ONE_PI   3.1415926f
#define TWO_PI   6.2831853f

#define CW  0                        //编码器顺时针方向
#define CCW 1                        //编码器逆时针方向
                       
#define POLEPAIRS   14               //默认极对数
#define OUTPUT_GEAR_RATIO 9.0f       //输出轴/电机轴减速比
#define ACCELERATION 4800            //默认加速度(rpm/s)

#define ENCODER_COUNTS_PER_REV 65536U       //编码器单圈计数模值
#define PUL_MAX (ENCODER_COUNTS_PER_REV - 1U) //编码器原始值最大值
#define PUL_MAX_HALF (ENCODER_COUNTS_PER_REV / 2U)
#define PUL_ANGLE_FACTOR (360.0f / ENCODER_COUNTS_PER_REV) //角度系数，单位：度/计数



#define SPEED_DIVISION_FACTOR  2     //速度环分频系数
#define POS_DIVISION_FACTOR    4     //位置环分频系数

typedef struct
{    
    motor_run_state    run_state;          // 运行状态
    motor_error_code    error_code;
    u8    run_mode;           // 运行模式
}motor_struct;

typedef struct
{    
    identify_state    state;
    u8    flag;
    u8    end_flag;           // 辨识完成标志    
    u16   count;             // 计数
    u16   wait_tim;           // 等待时间
    u32   ls_time_sum;        // 电感测量的总计时代数
    float rs;                // 相电阻
    float ls;                // 相电感
    float lq;                // q轴电感
    float ld;                // d轴电感    
    float flux;              // 磁链
    float ls_sum;             // 电感累计值
    float cur_max;            // 最大识别电流
    float cur_sum;            // 电流累计值
    float cur_average[2];     // 电流平均值
    float voltage_set[2];     // 电压给定值
    float verify_angle_deg;   // 编码器电角度验证值
    float verify_id;          // 编码器角度下的d轴电流
    float verify_iq;          // 编码器角度下的q轴电流
}identify_struct;

typedef struct{
    general_mode general_mode;
    general_mode last_general_mode;
    motion_state motion_state;
    u16 close_run_time;
    float open_curr;
    float open_curr_last;
    float open_curr_max;
    s16 check_cnt;
    float theta_ref;
    float theta_obs;
    float theta_err;
    float open_speed;
    float close_speed;
    float ele_open_speed_abs;
    float ele_close_speed_abs;
    float speed_err;
    float speed_err_flt;
    u8 err_flag;
    u16 err_cnt;
    u16 err_times;
    float last_target_speed;
    float open_to_close_switch_speed;
    float close_to_open_switch_speed;
    float close_min_speed;
    float curr_change_rate;
    float obs_mag;
}strong_drag_to_observer;

typedef struct{
    float speed_min;
    float speed_mid;
    float speed_max;
    float speed_ref;
    float speed_ref_abs;
    float hfi_ele_speed;
    float hfi_ele_speed_abs;
    float hfi_theta;
    float obs_ele_speed;
    float obs_ele_speed_abs;
    float obs_theta;
    float theta_err;
    s16 check_cnt;
    float ele_speed_out;
    float theta_out;
    float theta_offset;
    observer_mode obs_mode;
    observer_mode last_obs_mode;
    float speed_change_rate;
    float speed_last;
    float hfi_inject_mag_max;
    float hfi_inject_mag_min;
}hfi_to_observer;

typedef struct
{
    motor_struct                         motor;            
  identify_struct              identify;         
    e_angle_struct                      e_angle;
    sample_struct                       sample;
    foc_struct                          foc;
    pid_struct                          iq_pid;
    pid_struct                          id_pid;
    pid_struct                          spd_pid;
    pid_struct                          pos_pid;
    speed_struct                        speed; 
    t_shaped_acc_dec_struct      t_acc_dec;        
    position_struct                      position;              
    smo_struct smo;
    hfi_struct hfi;
    pll_struct                  spll;             
    pll_struct                  hpll;
    strong_drag_to_observer strong_drag_to_obs;
    hfi_to_observer hfi_to_obs;
}motor_control_struct;

extern motor_control_struct mc;

extern s8 speed_set_dir;

void motor_struct_init(void);




#endif 


