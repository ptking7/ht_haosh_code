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

#include "motor_publicdata.h"

//电位器给定速度转向
s8 speed_set_dir = 1;

motor_control_struct mc;                         //实例化总结构体

/**
 * 函数功能: 电机控制结构体全局初始化
 * 输入参数: 无（直接操作全局结构体 mc）
 * 返回参数: 无
 * 说    明: 在系统启动时调用，配置电机控制所需的所有参数，包括：
 *           运行状态、采样系数、编码器参数、滤波器系数、PID参数、
 *           各种观测器参数（滑模、锁相环、高频注入、强拖切换等）。
 *           所有参数均根据实际硬件和电机特性预先设定。
 */
void motor_struct_init(void)
{
    // ============================================================================
    // 1. 电机基本运行状态与模式
    // ============================================================================
    // run_state: 电机运行状态机状态，ADC_CALIB表示初始状态为ADC校准（采样偏移校准）
    mc.motor.run_state = ADC_CALIB;
    // error_code: 故障代码，初始无错误
    mc.motor.error_code = NONE_ERR;
    // run_mode: 电机运行时的控制模式，HFI_SMO_SPEED_CURRENT_CLOSE 表示混合模式：
    //          低速时使用高频注入(HFI)，中高速切换为滑模观测器(SMO)，并启用速度电流闭环
    mc.motor.run_mode = SPEED_CURRENT_LOOP;

    // ============================================================================
    // 2. 采样参数（电流、电压）
    // ============================================================================
    // current_dir: 电流采样方向修正（1表示硬件采样方向与定义相反时取反）
    mc.sample.current_dir = 1;
    // current_factor: 相电流计算系数，将ADC原始值转换为实际安培数
    //                由采样电阻、运放放大倍数、ADC参考电压及分辨率决定
    mc.sample.current_factor = PHASE_CURRENT_FACTOR;
    // bus_factor: 母线电压计算系数，将ADC原始值转换为实际伏特数
    //            由分压电阻分压比和ADC分辨率决定
    mc.sample.bus_factor = VBUS_FACTOR;

    // ============================================================================
    // 3. 编码器与电角度参数
    // ============================================================================
    // dir: 编码器旋转方向，CCW（逆时针）表示角度递增方向为正
    mc.e_angle.dir = CCW;
    // pole_pairs: 电机极对数（永磁体磁极对数 = 磁钢数量/2）
    mc.e_angle.pole_pairs = POLEPAIRS;
    // encoder_val_max: 编码器单圈脉冲数（增量式编码器线数 × 4倍频后的值，或绝对式编码器最大码值）
    mc.e_angle.encoder_val_max = ENCODER_COUNTS_PER_REV;
    // ts: 控制周期（秒），用于角度积分和观测器时间基准
    mc.e_angle.ts = TS;

    // ============================================================================
    // 4. FOC（磁场定向控制）参数
    // ============================================================================
    // id_lpf_factor / iq_lpf_factor: d/q轴电流低通滤波系数（一阶滤波α值，0~1）
    //                             0.1表示滤波效果较强，响应稍慢
    mc.foc.id_lpf_factor = 0.05f;
    mc.foc.iq_lpf_factor = 0.05f;
    // pwm_cycle: PWM周期（通常为定时器的自动重装载值），用于计算占空比
    mc.foc.pwm_cycle = PWM_CYCLE;
    // pwm_limit: PWM输出限幅值，防止占空比过大导致上下桥臂直通或过调制
    mc.foc.pwm_limit = PWM_LIMLT;

    // ============================================================================
    // 5. 位置环参数
    // ============================================================================
    // electrical_val_max: 电角度单圈脉冲最大值（与编码器单圈最大值相同）
    mc.position.electrical_val_max = PUL_MAX;

    // ============================================================================
    // 6. T型加减速参数（速度斜坡）
    // ============================================================================
    mc.t_acc_dec.pole_pairs = POLEPAIRS;   // 极对数，用于机械速度与电速度转换
    mc.t_acc_dec.ts = TS;                 // 控制周期
    mc.t_acc_dec.acc_speed = ACCELERATION; // 加速度设定值（单位：rpm/s 或 转/秒2）

    // ============================================================================
    // 7. 速度环参数
    // ============================================================================
    mc.speed.pole_pairs = POLEPAIRS;
    // speed_divsion_factor: 速度环分频因子（速度环执行周期 = 电流环周期 × 该因子）
    mc.speed.speed_divsion_factor = SPEED_DIVISION_FACTOR;
    // electrical_val_max: 电角度单圈最大脉冲数
    mc.speed.electrical_val_max = PUL_MAX;
    // electrical_speed_lpf_factor: 速度反馈的低通滤波系数（一阶滤波）
    mc.speed.electrical_speed_lpf_factor = 0.02f;
    // electrical_speed_factor: 速度计算系数，将电角度差转换为实际转速（rpm）
    //                        公式： (1 / (ts * speed_divsion_factor)) * 60
    mc.speed.electrical_speed_factor = (1.0f / (TS * SPEED_DIVISION_FACTOR)) * 60.0f;

    // ============================================================================
    // 8. 电机参数识别相关（电阻电感识别）
    // ============================================================================
    // cur_max: 识别过程中允许的最大电流（防止过流损坏电机）
    mc.identify.cur_max = 0.6f;

    // ============================================================================
    // 9. 滑模观测器（SMO）参数（用于无传感器中高速段）
    // ============================================================================
    // gain: 滑模增益，越大观测收敛越快但可能引起抖振
    mc.smo.gain = 14.0f;
    mc.smo.ts = TS;                         // 观测器运行周期
    // eab_fore_lpf_factor: 反电动势（Ealpha, Ebeta）低通滤波系数
    mc.smo.eab_fore_lpf_factor = 0.1f;

    // ============================================================================
    // 10. 锁相环（SPLL）参数（用于从反电动势提取转速和角度）
    // ============================================================================
    mc.spll.ts = TS;
    mc.spll.kp = 1200.0f;                  // 比例系数
    mc.spll.ki = 100.0f;                   // 积分系数
    mc.spll.we_fore_lpf_factor = 0.01f;       // 观测电角速度低通滤波系数

    // ============================================================================
    // 11. 高频注入（HFI）及锁相环（SPLL/HPLL）参数（用于低速无传感器）
    // ============================================================================
    mc.hfi.enable = 1;                     // 使能高频注入
    mc.hfi.uin = 1.4f;                     // 高频注入电压幅值（伏特）
    mc.hpll.dir = 1;                       // 锁相环输入方向（1为正方向）
    mc.hpll.kp = 900.0f;                   // 比例系数
    mc.hpll.ki = 20.0f;                    // 积分系数
    mc.hpll.ts = TS;                       // 运行周期
    mc.hpll.we_fore_lpf_factor = 0.01f;       // 观测电角速度低通滤波系数

    // ============================================================================
    // 12. 强拖（强制拖动）到观测器切换参数
    //     用于启动时从开环强拖平滑切换至闭环观测器
    // ============================================================================
    mc.strong_drag_to_obs.general_mode = OPEN_LOOP;              // 当前模式：开环强拖
    mc.strong_drag_to_obs.last_general_mode = OPEN_LOOP;          // 上一次模式
    mc.strong_drag_to_obs.close_run_time = 0;                     // 闭环运行计时
    mc.strong_drag_to_obs.open_curr = 0.05f;                     // 开环强拖电流（安培）
    mc.strong_drag_to_obs.open_curr_last = 0;                     // 上次开环电流
    mc.strong_drag_to_obs.open_curr_max = 3.0f;                   // 开环最大电流限制
    mc.strong_drag_to_obs.check_cnt = 0;                         // 切换检测计数器
    mc.strong_drag_to_obs.theta_ref = 0;                         // 参考角度（开环给定）
    mc.strong_drag_to_obs.theta_obs = 0;                         // 观测器角度
    mc.strong_drag_to_obs.theta_err = 0;                         // 角度误差
    mc.strong_drag_to_obs.open_speed = 0;                        // 开环目标速度
    mc.strong_drag_to_obs.close_speed = 0;                       // 闭环反馈速度
    mc.strong_drag_to_obs.speed_err = 0;                         // 速度误差
    mc.strong_drag_to_obs.speed_err_flt = 0;                      // 滤波后速度误差
    mc.strong_drag_to_obs.err_flag = 0;                          // 错误标志
    mc.strong_drag_to_obs.err_cnt = 0;                           // 错误计数
    mc.strong_drag_to_obs.err_times = 0;                         // 错误次数
    mc.strong_drag_to_obs.open_to_close_switch_speed = 4200.0f;     // 开环->闭环切换速度阈值（rpm）
    mc.strong_drag_to_obs.close_to_open_switch_speed = 3000.0f;     // 闭环->开环切换速度阈值（rpm）
    mc.strong_drag_to_obs.close_min_speed = 2000.0f;              // 允许闭环的最低速度
    mc.strong_drag_to_obs.obs_mag = 0;                           // 观测器幅值

    // ============================================================================
    // 13. 高频注入（HFI）到滑模观测器（SMO）切换参数
    //     实现低速HFI到中高速SMO的无缝过渡
    // ============================================================================
    mc.hfi_to_obs.speed_min = 3000.0f;          // HFI工作最低速度（低于此值仅用HFI）
    mc.hfi_to_obs.speed_mid = 2000.0f;          // 中间速度（用于切换缓冲区）
    mc.hfi_to_obs.speed_max = 4200.0f;          // SMO完全接管速度（高于此值仅用SMO）
    mc.hfi_to_obs.hfi_ele_speed = 0;             // HFI观测的电角速度
    mc.hfi_to_obs.hfi_ele_speed_abs = 0;          // HFI电角速度绝对值
    mc.hfi_to_obs.hfi_theta = 0;                // HFI观测的角度
    mc.hfi_to_obs.obs_ele_speed = 0;             // SMO观测的电角速度
    mc.hfi_to_obs.obs_ele_speed_abs = 0;          // SMO电角速度绝对值
    mc.hfi_to_obs.obs_theta = 0;                // SMO观测的角度
    mc.hfi_to_obs.theta_err = 0;                // 两种观测器的角度误差
    mc.hfi_to_obs.check_cnt = 0;                // 切换检测计数器
    mc.hfi_to_obs.ele_speed_out = 0;             // 最终输出的电角速度
    mc.hfi_to_obs.theta_out = 0;                // 最终输出的电角度
    mc.hfi_to_obs.theta_offset = 0;             // 角度偏移补偿
    mc.hfi_to_obs.obs_mode = HFI;               // 当前观测器模式（初始为HFI）
    mc.hfi_to_obs.hfi_inject_mag_max = 5.0f;      // 高频注入最大电压幅值
    mc.hfi_to_obs.hfi_inject_mag_min = 1.4f;      // 高频注入最小电压幅值

    // ============================================================================
    // 14. 电流环PID参数（Iq、Id环）
    // ============================================================================
    mc.iq_pid.kp = 0.08f;                    // 零速电流环调试增益
    mc.iq_pid.ki = 0.006f * CURRENT_KI_SCALE;
    mc.iq_pid.out_max = 0.5f;
    mc.iq_pid.out_min = -0.5f;

    mc.id_pid.kp = 0.08f;
    mc.id_pid.ki = 0.006f * CURRENT_KI_SCALE;
    mc.id_pid.out_max = 0.5f;
    mc.id_pid.out_min = -0.5f;

    // ============================================================================
    // 15. 速度环PID参数（带分段限制）
    // ============================================================================
    mc.spd_pid.kp = 0.05f;                   // 外轴rpm误差到Iq电流
    mc.spd_pid.kp_max = 0.005f;               // 比例系数最大值（用于变速调参）
    mc.spd_pid.kp_min = 0.001f;               // 比例系数最小值
    mc.spd_pid.ki = 0.002f;                  // 每20ms速度环周期积分系数
    mc.spd_pid.out_max = 0.8f;                // 速度环Iq输出限幅
    mc.spd_pid.out_min = -0.8f;

    // ============================================================================
    // 16. 位置环PID参数（用于位置伺服控制）
    // ============================================================================
    mc.pos_pid.kp = 0.5f;                    // 比例系数
    mc.pos_pid.ki = 0;                       // 积分系数（纯比例控制）
    mc.pos_pid.kd = 0;                       // 微分系数（未使用）
    mc.pos_pid.out_max = 14000;               // 输出上限（对应速度参考值，rpm）
    mc.pos_pid.out_min = -14000;              // 输出下限
}                                                  

