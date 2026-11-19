#include "foc_drv.h"
                                                            
/**
 * 函数功能: Clark变换（恒幅值变换）
 * 输入参数: p - foc_struct结构体指针，包含三相电流iu、iv，输出ialpha、ibeta
 * 返回参数: 无（结果直接存入结构体成员p->ialpha, p->ibeta）
 * 说    明: 
 *         将三相静止坐标系(a,b,c)的电流变换到两相静止坐标系(α,β)。
 *         采用恒幅值变换，变换矩阵系数保证变换前后电流幅值不变。
 *         由于三相电流满足 iu+iv+iw=0，可消去iw，简化公式为：
 *           iα = iu
 *           iβ = (iu + 2iv) / √3
 *         代码中系数：1/√3 ≈ 0.57735027，2/√3 ≈ 1.1547004。
 *         该变换是矢量控制的基础步骤，为后续Park变换提供正交分量。
 */
void clark_transform(foc_struct *p)
{
    p->ialpha = p->iu;                                       // α轴电流等于U相电流
    p->ibeta  = (p->iu * 0.57735027f) + (p->iv * 1.1547004f); // β轴电流 = (iu + 2iv) / √3
}

/**
 * 函数功能: Park变换
 * 输入参数: p - foc_struct结构体指针，包含ialpha、ibeta、sin_val、cos_val，输出id、iq
 * 返回参数: 无（结果存入p->id, p->iq）
 * 说    明: 
 *         将两相静止坐标系(α,β)的电流变换到旋转坐标系(d,q)。
 *         变换公式：
 *           id =  iα * cosθ + iβ * sinθ
 *           iq = -iα * sinθ + iβ * cosθ
 *         其中θ为转子电角度，sin_val = sinθ，cos_val = cosθ。
 *         d轴电流id与磁场方向对齐，q轴电流iq与力矩方向对齐。
 *         经过Park变换后，交流量变为直流量，便于使用PI控制器实现无静差调节。
 *         该变换需要准确的转子位置信息（有感控制来自编码器，无感控制来自观测器）。
 */
void park_transform(foc_struct *p)
{
    p->id = (p->ialpha * p->cos_val) + (p->ibeta * p->sin_val);   // d轴电流
    p->iq = (-p->ialpha * p->sin_val) + (p->ibeta * p->cos_val);  // q轴电流
}

/**
 * 函数功能: 反Park变换
 * 输入参数: p - foc_struct结构体指针，包含ud、uq、sin_val、cos_val，输出ualpha、ubeta
 * 返回参数: 无（结果存入p->ualpha, p->ubeta）
 * 说    明: 
 *         将旋转坐标系(d,q)的电压指令变换到静止坐标系(α,β)。
 *         变换公式：
 *           uα = ud * cosθ - uq * sinθ
 *           uβ = uq * cosθ + ud * sinθ
 *         这是Park变换的逆过程，用于将PI控制器输出的ud、uq转换为SVPWM可处理的uα、uβ。
 *         变换后得到的uα、uβ仍然是正弦量，其幅值和相位决定了电机定子磁链的旋转。
 *         通常ud用于励磁控制（id闭环），uq用于力矩控制（iq闭环）。
 */
void inverse_park_transform(foc_struct *p)
{
    p->ualpha = p->ud * p->cos_val - p->uq * p->sin_val;          // α轴电压
    p->ubeta  = p->uq * p->cos_val + p->ud * p->sin_val;          // β轴电压
}

/**
 * 函数功能: 空间矢量脉宽调制（SVPWM）
 * 输入参数: p - foc_struct结构体指针，包含ualpha、ubeta、ubus、pwm_cycle、pwm_limit
 * 返回参数: 无（结果存入p->duty_cycle_a, p->duty_cycle_b, p->duty_cycle_c）
 * 说    明: 
 *         根据αβ轴电压指令ualpha、ubeta和直流母线电压ubus，计算三相逆变器的占空比。
 *         算法步骤：
 *         1. 扇区判断：计算u1=uβ, u2=(√3/2)uα - 0.5uβ, u3=-(√3/2)uα - 0.5uβ，
 *            通过u1,u2,u3的符号组合得到扇区编号n（1~6）。
 *         2. 计算基本矢量作用时间：定义x,y,z为相邻矢量作用时间（与PWM周期相关），
 *            根据扇区分配t1、t2（两个有效矢量的作用时间）。
 *         3. 过调制处理：若t1+t2超过pwm_limit，按比例缩小t1、t2，确保输出电压在线性区内。
 *         4. 生成三相比较值：ta = (pwm_cycle - t1 - t2)/4（零矢量起始点），
 *            tb = ta + t1/2，tc = tb + t2/2。
 *         5. 扇区映射：根据扇区将ta,tb,tc分配到a、b、c三相占空比寄存器。
 *         采用中心对齐PWM模式，可有效降低谐波含量，提高直流母线电压利用率。
 *         相比SPWM，SVPWM的电压利用率提高约15%。
 *         注意：pwm_cycle为PWM周期对应的定时器计数值（如1000），
 *               pwm_limit通常设为略小于pwm_cycle的值（如pwm_cycle*0.95），防止上下桥臂直通。
 */
void calculate_svpwm(foc_struct *p)
{
    float u1, u2, u3 = 0;
    float x, y, z = 0;
    float t1, t2, t1_temp, t2_temp = 0;
    u8 a, b, c, n = 0;
    u16 ta, tb, tc = 0;

    // ========== 1. 计算扇区判别变量 ==========
    // 将 αβ 电压转换为三个等效的相电压值（基于 60° 坐标系）
    u1 = p->ubeta;                                         // u1 = uβ
    u2 = (0.866f * p->ualpha) - (0.5f * p->ubeta);         // u2 = √3/2 * uα - 1/2 * uβ
    u3 = (-0.866f * p->ualpha) - (0.5f * p->ubeta);        // u3 = -√3/2 * uα - 1/2 * uβ

    // 根据正负确定三位二进制码（a,b,c）
    if (u1 > 0) { a = 1; } else { a = 0; }
    if (u2 > 0) { b = 1; } else { b = 0; }
    if (u3 > 0) { c = 1; } else { c = 0; }
    n = 4 * c + 2 * b + a;                                  // 扇区编号 1~6

    // ========== 2. 计算基本矢量作用时间（未归一化） ==========
    // x, y, z 对应不同扇区下两个相邻矢量的作用时间（单位：PWM计数值）
    x = (1.732f * p->pwm_cycle * p->ubeta) / p->ubus;      // x = √3 * t_pwm * uβ / ubus
    y = (1.5f * p->ualpha * p->pwm_cycle + 0.866f * p->ubeta * p->pwm_cycle) / p->ubus;
    z = (-1.5f * p->ualpha * p->pwm_cycle + 0.866f * p->ubeta * p->pwm_cycle) / p->ubus;

    // ========== 3. 根据扇区分配 t1, t2 ==========
    switch (n)
    {
        case 3: { t1 = -z; t2 =  x; } break;
        case 1: { t1 =  z; t2 =  y; } break;
        case 5: { t1 =  x; t2 = -y; } break;
        case 4: { t1 = -x; t2 =  z; } break;
        case 6: { t1 = -y; t2 = -z; } break;
        case 2: { t1 =  y; t2 = -x; } break;
        default: { t1 = 0; t2 = 0; } break;
    }

    // ========== 4. 过调制处理（限制总作用时间不超过 PWM 周期） ==========
    t1_temp = t1;
    t2_temp = t2;
    if (t1 + t2 > p->pwm_limit)                             // 超出最大允许占空比
    {
        t1 = p->pwm_limit * t1_temp / (t1_temp + t2_temp);  // 等比例缩小
        t2 = p->pwm_limit * t2_temp / (t1_temp + t2_temp);
    }

    // ========== 5. 计算三相比较值（中心对齐 PWM） ==========
    ta = (p->pwm_cycle - t1 - t2) * 0.25f;                 // 零矢量作用时间/2（起始点）
    tb = ta + t1 * 0.5f;                                   // 第一个有效矢量中点
    tc = tb + t2 * 0.5f;                                   // 第二个有效矢量中点

    // ========== 6. 根据扇区将 ta, tb, tc 映射到三相占空比 ==========
    switch (n)
    {
        case 3:
            p->duty_cycle_a = ta;
            p->duty_cycle_b = tb;
            p->duty_cycle_c = tc;
            break;
        case 1:
            p->duty_cycle_a = tb;
            p->duty_cycle_b = ta;
            p->duty_cycle_c = tc;
            break;
        case 5:
            p->duty_cycle_a = tc;
            p->duty_cycle_b = ta;
            p->duty_cycle_c = tb;
            break;
        case 4:
            p->duty_cycle_a = tc;
            p->duty_cycle_b = tb;
            p->duty_cycle_c = ta;
            break;
        case 6:
            p->duty_cycle_a = tb;
            p->duty_cycle_b = tc;
            p->duty_cycle_c = ta;
            break;
        case 2:
            p->duty_cycle_a = ta;
            p->duty_cycle_b = tc;
            p->duty_cycle_c = tb;
            break;
        default:
            p->duty_cycle_a = ta;
            p->duty_cycle_b = tb;
            p->duty_cycle_c = tc;
            break;
    }
}
