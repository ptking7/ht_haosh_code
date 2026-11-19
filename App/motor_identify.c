
#include "motor_identify.h"

/**
 * 函数功能: 电机参数辨识（电阻、电感）及编码器转子对齐
 * 输入参数: 无（所有参数通过全局结构体 mc.identify、mc.foc、mc.sample、mc.e_angle 传递）
 * 返回参数: 无（结果存入 mc.identify.rs、mc.identify.ls 等，最终设置 mc.identify.end_flag）
 * 说    明: 
 *         1. 该函数通过三个子状态自动完成电机参数辨识和编码器校准：
 *            - RESISTANCE_IDENTIFICATION：直流伏安法测量定子电阻 rs
 *            - INDUCTANCE_IDENTIFICATION：直流阶跃法测量定子电感 ls（假设 ld = lq）
 *            - ENCODER_ROTOR_ALIGN：两次定位（90°和0°）校准编码器零位，检测旋转方向
 *         2. 该函数应在 PWM 中断中周期性调用，每次执行一步状态机，直到 end_flag 置1。
 *         3. 辨识过程中电机轴会轻微转动，需确保负载脱离。
 *         4. 电阻辨识原理：向 d 轴注入直流电流（电角度=0），测量不同电压下的稳定电流，
 *            根据欧姆定律 rs = ΔU / ΔI。
 *         5. 电感辨识原理：施加阶跃电压，测量电流从0上升到稳定值95%的时间，
 *            对于一阶 RL 电路，L = R * t / ln(20)。
 *         6. 编码器对齐：先拉转子到90°电角度，记录编码器值；再拉到0°，计算两次差值
 *            判断方向，并得到零点偏移。
 */
void motor_identify(void)
{
    /* 根据当前辨识阶段执行不同操作 */
    switch (mc.identify.state)
    {
        /* ================= 阶段1：定子电阻辨识 ================= */
        case RESISTANCE_IDENTIFICATION:
        {
            /* ----- 子状态 flag = 0：清空参数，准备第一次电流设定 ----- */
            if (mc.identify.flag == 0)
            {
                mc.foc.uq = 0;                 // 交轴电压清零（只施加直轴电压）
                mc.foc.ud = 0;                 // 直轴电压从0开始递增
                mc.identify.count = 0;          // 计数清零（本阶段未使用）
                mc.identify.wait_tim = 0;        // 等待计时器清零
                mc.identify.flag = 1;           // 进入下一个子状态
            }

            /* ----- 子状态 flag = 1：逐步增加 ud 直到电流达到第一个目标值（0.6倍最大电流）----- */
            if (mc.identify.flag == 1)
            {
                // 计算当前实际电流值：iu * ud * 1.5 / ubus
                // 推导：ud施加于d轴，实际d轴电流 id ≈ iu（因为电角度=0，iu与id同相）
                // 1.5 为三相电流标幺化系数（取决于Clark变换实现）
                float current = (mc.sample.iu_real * mc.foc.ud * 1.5f) / mc.sample.bus_real;
                if (current >= 0.6f * mc.identify.cur_max)   // 达到目标电流的60%
                {
                    mc.identify.flag = 2;       // 进入等待稳定和记录阶段
                }
                else
                {
                    mc.foc.ud += 0.0001f;       // 每次调用增加0.0001V（约0.1mV），逐步升压
                    mc.identify.voltage_set[0] = mc.foc.ud;   // 记录第一次电压点
                }
            }

            /* ----- 子状态 flag = 2：等待电流稳定，采样100次求平均（第一组电流）----- */
            if (mc.identify.flag == 2)
            {
                mc.identify.wait_tim++;          // 计数器累加（每次函数调用约50~100us，需根据实际周期调整）
                if (mc.identify.wait_tim > 4000) // 等待约0.2秒（假设周期50us，4000次=0.2s）
                {
                    mc.identify.cur_sum += mc.sample.iu_real;   // 累加相电流
                }
                if (mc.identify.wait_tim >= 4100) // 再采集100次（4100-4000=100次）
                {
                    mc.identify.cur_average[0] = mc.identify.cur_sum * 0.01f; // 平均电流 = 累加和 / 100
                    mc.identify.wait_tim = 0;     // 计时器复位
                    mc.identify.cur_sum = 0;       // 累加和清零
                    mc.identify.flag = 3;         // 进入第二组电压/电流测量
                }
            }

            /* ----- 子状态 flag = 3：继续升压，直到电流达到最大电流 cur_max（第二组）----- */
            if (mc.identify.flag == 3)
            {
                float current = (mc.sample.iu_real * mc.foc.ud * 1.5f) / mc.sample.bus_real;
                if (current >= mc.identify.cur_max)   // 达到最大电流
                {
                    mc.identify.flag = 4;       // 进入等待稳定和记录阶段
                }
                else
                {
                    mc.foc.ud += 0.0001f;       // 继续升压
                    mc.identify.voltage_set[1] = mc.foc.ud;   // 记录第二次电压点
                }
            }

            /* ----- 子状态 flag = 4：等待稳定，采样100次求平均（第二组电流）----- */
            if (mc.identify.flag == 4)
            {
                mc.identify.wait_tim++;
                if (mc.identify.wait_tim > 4000)
                {
                    mc.identify.cur_sum += mc.sample.iu_real;
                }
                if (mc.identify.wait_tim >= 4100)
                {
                    mc.identify.cur_average[1] = mc.identify.cur_sum * 0.01f; // 平均电流
                    mc.identify.wait_tim = 0;
                    mc.identify.cur_sum = 0;
                    mc.identify.flag = 5;       // 进入计算电阻阶段
                }
            }

            /* ----- 子状态 flag = 5：计算定子电阻 rs = ΔU / ΔI ----- */
            if (mc.identify.flag == 5)
            {
                // 电阻 = (第二组电压 - 第一组电压) / (第二组电流 - 第一组电流)
                mc.identify.rs = (mc.identify.voltage_set[1] - mc.identify.voltage_set[0])
                                 / (mc.identify.cur_average[1] - mc.identify.cur_average[0]);
                mc.foc.ud = 0;                  // 关断电压
                mc.identify.flag = 0;           // 重置标志
                mc.identify.state = INDUCTANCE_IDENTIFICATION;  // 切换到电感辨识阶段
            }

            /* 电阻辨识过程中，始终保持电角度为0（d轴对齐） */
            mc.foc.sin_val = 0;                 // sin(0) = 0
            mc.foc.cos_val = 1;                 // cos(0) = 1
            inverse_park_transform(&mc.foc);    // 反Park变换，将ud/uq转换为ualpha/ubeta
        }
        break;

        /* ================= 阶段2：定子电感辨识 ================= */
        case INDUCTANCE_IDENTIFICATION:
        {
            /* ----- 子状态 flag = 0：等待电流归零，确保初始条件 ----- */
            if (mc.identify.flag == 0)
            {
                mc.foc.uq = 0;
                mc.foc.ud = 0;
                // 检查相电流是否接近0（±0.05A以内）
                if (mc.sample.iu_real >= -0.05f && mc.sample.iu_real <= 0.05f)
                {
                    mc.identify.flag = 1;       // 电流归零，开始测试
                }
            }

            /* ----- 子状态 flag = 1：施加固定电压 ud，测量电流上升时间，计算电感 ----- */
            if (mc.identify.flag == 1)
            {
                mc.foc.ud = mc.identify.voltage_set[1];   // 使用电阻辨识时第二次的电压值
                mc.identify.wait_tim++;                    // 计时器递增（单位：函数调用周期）

                // 当电流上升到平均电流的95%时，记录时间
                if (mc.sample.iu_real >= mc.identify.cur_average[1] * 0.95f)
                {
                    // 电感计算公式：L = R * t / ln(1/(1-0.95)) = R * t / ln(20)
                    // ln(20) ≈ 2.9957，系数 0.334 = 1/2.9957 ≈ 0.334
                    // 0.00005 为函数调用周期（假设周期50us），WaitTim为周期数，故 t = wait_tim * 0.00005
                    mc.identify.ls_sum += mc.identify.rs * 0.334f * 0.00005f * mc.identify.wait_tim;
                    mc.identify.wait_tim = 0;              // 计时器复位
                    mc.identify.count++;                  // 完成一次测量，计数器+1
                    mc.identify.flag = 0;                 // 返回等待电流归零状态，进行下一次测量
                    mc.foc.ud = 0;                        // 关断电压，让电流下降
                    if (mc.identify.count >= 100)         // 重复测量100次，取平均值
                    {
                        mc.identify.flag = 2;             // 进入计算最终电感
                    }
                }
            }

            /* ----- 子状态 flag = 2：计算平均电感，切换到编码器对齐阶段 ----- */
            if (mc.identify.flag == 2)
            {
                mc.identify.ls = mc.identify.ls_sum * 0.01f;   // 平均值 = 累加和 / 100
                mc.identify.ld = mc.identify.ls;             // 假设表贴式电机，ld = lq = ls
                mc.identify.lq = mc.identify.ls;
                mc.identify.flag = 0;
                mc.identify.ls_sum = 0;
                mc.identify.wait_tim = 0;
                mc.identify.state = ENCODER_ROTOR_ALIGN;    // 进入编码器校准阶段
            }

            /* 电感辨识过程中，同样保持电角度为0 */
            mc.foc.sin_val = 0;
            mc.foc.cos_val = 1;
            inverse_park_transform(&mc.foc);
        }
        break;

        /* ================= 阶段3：编码器转子对齐与方向检测 ================= */
        case ENCODER_ROTOR_ALIGN:
        {
            /* ----- 子状态 calib_flag = 0：第一次定位，将转子拉至90°电角度位置 ----- */
            if (mc.e_angle.calib_flag == 0)
            {
                mc.foc.ud += 0.0001f;               // 逐步增加ud（d轴电压）
                mc.foc.uq = 0;                      // q轴电压为0
                mc.foc.sin_val = 1;                 // 设定电角度为90°：sin90°=1
                mc.foc.cos_val = 0;                 // cos90°=0
                // 当ud达到电阻辨识时的最大电压（即电压点voltage_set[1]），表示电流已足够大
                if (mc.foc.ud >= mc.identify.voltage_set[1])
                {
                    mc.foc.ud = 0;                  // 关断电压
                    mc.e_angle.calib_flag = 1;       // 第一次定位完成
                    mc.e_angle.encoder_val_change = mc.e_angle.encoder_val; // 记录此时的编码器原始值
                }
            }

            /* ----- 子状态 calib_flag = 1：第二次定位，将转子拉至0°电角度位置 ----- */
            if (mc.e_angle.calib_flag == 1)
            {
                mc.foc.ud += 0.0001f;
                mc.foc.uq = 0;
                mc.foc.sin_val = 0;                 // 电角度0°：sin0°=0
                mc.foc.cos_val = 1;                 // cos0°=1
                if (mc.foc.ud >= mc.identify.voltage_set[1])
                {
                    // 计算两次定位的编码器差值，判断电机旋转方向
                    mc.e_angle.encoder_val_change = mc.e_angle.encoder_val - mc.e_angle.encoder_val_change;
                    // 处理编码器溢出（单圈最大值PUL_MAX，半圈为PUL_MAX_HALF）
                    if (mc.e_angle.encoder_val_change < -PUL_MAX_HALF)
                    {
                        mc.e_angle.encoder_val_change += PUL_MAX;
                    }
                    else if (mc.e_angle.encoder_val_change > PUL_MAX_HALF)
                    {
                        mc.e_angle.encoder_val_change -= PUL_MAX;
                    }
                    // 方向判断：正差值为正向，负差值为反向
                    if (mc.e_angle.encoder_val_change > 0)
                    {
                        mc.e_angle.dir = 1;         // 假设1表示正向
                    }
                    else if (mc.e_angle.encoder_val_change < 0)
                    {
                        mc.e_angle.dir = 0;         // 0表示反向
                    }
                    else
                    {
                        // 如果差值为0，可能是编码器故障或安装问题，可置错误标志
                        // mc.motor.run_state = MOTOR_ERROR;
                        // mc.motor.error_code = ENCODER_ERR;
                    }
                    
                    // 先对当前编码器值应用方向处理，再存为偏移量
                    if (mc.e_angle.dir == 1)
                    {
                        mc.e_angle.calib_offset = mc.e_angle.encoder_val_max - mc.e_angle.encoder_val;
                    }else{
                        mc.e_angle.calib_offset = mc.e_angle.encoder_val; // 更新偏移（可作为零点偏移）
                    }

                    mc.e_angle.calib_flag = 0;       // 校准完成
                    mc.foc.ud = 0;                  // 关断电压
                    mc.identify.end_flag = 1;        // 标识整个辨识流程结束
                }
            }
            inverse_park_transform(&mc.foc);        // 反Park变换
        }
        break;
    }

    /* 所有阶段共用的SVPWM输出（根据当前foc结构体中的ualpha/ubeta和ubus生成PWM） */
    mc.foc.ubus = mc.sample.bus_real;               // 更新实时母线电压
    calculate_svpwm(&mc.foc);                       // SVPWM调制输出
}




