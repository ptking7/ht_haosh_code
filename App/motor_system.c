/**
  ******************************************************************************
  * @file    motor_system.c
  * @brief   Motor state-machine dispatcher.
  *
  *          Current bring-up scope:
  *          ADC calibration -> Rs identification -> Ls identification -> stop.
  *          Encoder alignment and all later control modes are intentionally
  *          disabled until the identification results are validated.
  ******************************************************************************
  */

#include "motor_system.h"
#include "motor_global.h"
#include "motor_tune.h"
#include "tim.h"

#define IDENTIFY_BUS_MIN_VOLTAGE       10.0f
#define IDENTIFY_BUS_MAX_VOLTAGE       55.0f
#define IDENTIFY_PHASE_OVERCURRENT     2.5f
#define IDENTIFY_OVERCURRENT_TICKS     5U
#define IDENTIFY_BUS_FAULT_TICKS       20U
#define IDENTIFY_MAX_UD_VOLTAGE        0.50f
#define IDENTIFY_TIMEOUT_TICKS         600000U
#define IDENTIFY_PWM_START_DELAY_TICKS 200000U
#define ENCODER_ALIGN_HOLD_TICKS       20000U
#define ENCODER_VERIFY_HOLD_TICKS      4000U
#define ENCODER_VERIFY_UD              0.30f
#define CURRENT_LOOP_SETTLE_TICKS      10000U
#define CURRENT_LOOP_TEST_ID_REF       0.30f
#define SPEED_LOOP_DIVIDER             400U
#define SPEED_FEEDBACK_LPF             0.15f
#define OUTPUT_GEAR_RATIO              8.0f
#define SPEED_IQ_RATE_LIMIT            0.02f
#define FIXED_UD_TEST_ENABLE           0
#define FIXED_UD_TEST_VOLTAGE          0.25f
#define FIXED_UD_TEST_TICKS            20000U

static u8 pwm_started = 0;
static u8 overcurrent_ticks = 0;
static u8 bus_fault_ticks = 0;
static u32 start_delay_ticks = 0;
static u32 identify_ticks = 0;
static u8 speed_mode_last = 0;
static u8 speed_estimator_initialized = 0;
static u16 speed_angle_count = 0;
static float speed_angle_accum = 0.0f;
static float speed_angle_last = 0.0f;
static float speed_feedback_rpm = 0.0f;
#if FIXED_UD_TEST_ENABLE
static u32 fixed_ud_test_ticks = 0;
#endif

static void speed_loop_step(void)
{
    float angle = mc.e_angle.electrical_angle_pu;
    float delta;
    float electrical_hz;
    float motor_rpm;
    float output_rpm;
    float iq_delta;
    float speed_target;
    float min_current;

    if(speed_estimator_initialized == 0U)
    {
        speed_angle_last = angle;
        speed_angle_accum = 0.0f;
        speed_angle_count = 0U;
        speed_feedback_rpm = 0.0f;
        speed_estimator_initialized = 1U;
    }

    delta = angle - speed_angle_last;
    if(delta > 0.5f)
    {
        delta -= 1.0f;
    }
    else if(delta < -0.5f)
    {
        delta += 1.0f;
    }
    speed_angle_last = angle;
    speed_angle_accum += delta;
    speed_angle_count++;

    if(speed_angle_count >= SPEED_LOOP_DIVIDER)
    {
        electrical_hz = speed_angle_accum /
                        ((float)speed_angle_count * TS);
        motor_rpm = electrical_hz * 60.0f /
                    (float)mc.e_angle.pole_pairs;
        output_rpm = motor_rpm / OUTPUT_GEAR_RATIO;

        speed_feedback_rpm +=
            SPEED_FEEDBACK_LPF * (output_rpm - speed_feedback_rpm);
        mc.speed.electrical_speed_lpf = electrical_hz;
        mc.speed.mechanical_speed = speed_feedback_rpm;

        mc.spd_pid.ref = motor_tune_get_speed_rpm();
        mc.spd_pid.fbk = speed_feedback_rpm;
        pid_control(&mc.spd_pid);

        speed_target = motor_tune_get_speed_rpm();
        min_current = motor_tune_get_speed_min_current();
        if((speed_target > 0.0f) &&
           (speed_feedback_rpm < speed_target) &&
           (mc.spd_pid.out < min_current))
        {
            mc.spd_pid.out = min_current;
        }
        else if((speed_target < 0.0f) &&
                (speed_feedback_rpm > speed_target) &&
                (mc.spd_pid.out > -min_current))
        {
            mc.spd_pid.out = -min_current;
        }

        iq_delta = mc.spd_pid.out - mc.iq_pid.ref;
        if(iq_delta > SPEED_IQ_RATE_LIMIT)
        {
            iq_delta = SPEED_IQ_RATE_LIMIT;
        }
        else if(iq_delta < -SPEED_IQ_RATE_LIMIT)
        {
            iq_delta = -SPEED_IQ_RATE_LIMIT;
        }
        mc.iq_pid.ref += iq_delta;

        speed_angle_accum = 0.0f;
        speed_angle_count = 0U;
    }
}

static void encoder_align_step(void)
{
    float voltage_target = mc.identify.voltage_set[1];
    s32 expected_delta;
    s32 delta;
    s32 half_turn;
    u16 ramp_half_ticks;
    float ref_min;
    float ref_max;
    float command_angle;

    mc.foc.uq = 0.0f;

    switch(mc.e_angle.calib_flag)
    {
        case 0:
            mc.foc.sin_val = 1.0f;
            mc.foc.cos_val = 0.0f;
            if(mc.foc.ud < voltage_target)
            {
                mc.foc.ud += 0.0001f;
            }
            else
            {
                mc.foc.ud = voltage_target;
                mc.identify.wait_tim = 0;
                mc.e_angle.calib_flag = 1;
            }
            break;

        case 1:
            mc.foc.sin_val = 1.0f;
            mc.foc.cos_val = 0.0f;
            mc.foc.ud = voltage_target;
            mc.identify.wait_tim++;
            if(mc.identify.wait_tim >= ENCODER_ALIGN_HOLD_TICKS)
            {
                mc.e_angle.encoder_val_change = mc.e_angle.encoder_val;
                mc.identify.wait_tim = 0;
                mc.foc.ud = 0.0f;
                mc.e_angle.calib_flag = 2;
            }
            break;

        case 2:
            mc.foc.sin_val = 0.0f;
            mc.foc.cos_val = 1.0f;
            if(mc.foc.ud < voltage_target)
            {
                mc.foc.ud += 0.0001f;
            }
            else
            {
                mc.foc.ud = voltage_target;
                mc.identify.wait_tim = 0;
                mc.e_angle.calib_flag = 3;
            }
            break;

        case 3:
            mc.foc.sin_val = 0.0f;
            mc.foc.cos_val = 1.0f;
            mc.foc.ud = voltage_target;
            mc.identify.wait_tim++;
            if(mc.identify.wait_tim >= ENCODER_ALIGN_HOLD_TICKS)
            {
                delta = mc.e_angle.encoder_val - mc.e_angle.encoder_val_change;
                half_turn = mc.e_angle.encoder_val_max / 2;

                if(delta < -half_turn)
                {
                    delta += mc.e_angle.encoder_val_max;
                }
                else if(delta > half_turn)
                {
                    delta -= mc.e_angle.encoder_val_max;
                }

                mc.e_angle.encoder_val_change = delta;
                expected_delta = mc.e_angle.encoder_val_max /
                                 (4 * mc.e_angle.pole_pairs);

                if((delta != 0) &&
                   ((delta > 0 ? delta : -delta) >= expected_delta / 2) &&
                   ((delta > 0 ? delta : -delta) <= expected_delta * 3 / 2))
                {
                    mc.e_angle.dir = (delta > 0) ? 1U : 0U;
                    if(mc.e_angle.dir == 1U)
                    {
                        mc.e_angle.calib_offset = mc.e_angle.encoder_val_max -
                                                  mc.e_angle.encoder_val;
                    }
                    else
                    {
                        mc.e_angle.calib_offset = mc.e_angle.encoder_val;
                    }
                    mc.e_angle.electrical_angle_pu = 0.0f;
                    mc.e_angle.calib_flag = 4;
                    mc.identify.wait_tim = 0;
                    mc.identify.verify_angle_deg = 0.0f;
                    mc.identify.verify_id = 0.0f;
                    mc.identify.verify_iq = 0.0f;
                }
                else
                {
                    mc.motor.error_code = ENCODER_ERR;
                    mc.motor.run_state = MOTOR_ERROR;
                    mc.identify.end_flag = 1;
                }

                mc.foc.ud = 0.0f;
                mc.identify.wait_tim = 0;
            }
            break;

        case 4:
            calculate_encoder_data(&mc.e_angle);
            calculate_sin_cos(mc.e_angle.electrical_angle_pu,
                              &mc.foc.sin_val,
                              &mc.foc.cos_val);

            mc.foc.iu = mc.sample.ia_real;
            mc.foc.iv = mc.sample.ib_real;
            clark_transform(&mc.foc);
            park_transform(&mc.foc);

            mc.identify.verify_angle_deg =
                mc.e_angle.electrical_angle_pu * 360.0f;
            mc.identify.verify_id = mc.foc.id;
            mc.identify.verify_iq = mc.foc.iq;

            mc.foc.uq = 0.0f;
            if(mc.foc.ud < ENCODER_VERIFY_UD)
            {
                mc.foc.ud += 0.0001f;
            }
            else
            {
                mc.foc.ud = ENCODER_VERIFY_UD;
                mc.identify.wait_tim++;
                if(mc.identify.wait_tim >= ENCODER_VERIFY_HOLD_TICKS)
                {
                    mc.identify.verify_angle_deg =
                        mc.e_angle.electrical_angle_pu * 360.0f;
                    mc.identify.verify_id = mc.foc.id;
                    mc.identify.verify_iq = mc.foc.iq;
                    pid_clear(&mc.id_pid);
                    pid_clear(&mc.iq_pid);
                    mc.id_pid.ref = CURRENT_LOOP_TEST_ID_REF;
                    mc.iq_pid.ref = 0.0f;
                    mc.foc.ud = 0.0f;
                    mc.foc.uq = 0.0f;
                    mc.identify.wait_tim = 0;
                    mc.e_angle.calib_flag = 5;
                }
            }
            break;

        case 5:
            calculate_encoder_data(&mc.e_angle);
            calculate_sin_cos(mc.e_angle.electrical_angle_pu,
                              &mc.foc.sin_val,
                              &mc.foc.cos_val);

            mc.foc.iu = mc.sample.ia_real;
            mc.foc.iv = mc.sample.ib_real;
            clark_transform(&mc.foc);
            park_transform(&mc.foc);

            mc.id_pid.fbk = mc.foc.id;
            pid_control(&mc.id_pid);

            mc.foc.ud = mc.id_pid.out;
            mc.foc.uq = 0.0f;
            mc.identify.verify_angle_deg =
                mc.e_angle.electrical_angle_pu * 360.0f;
            mc.identify.verify_id = mc.foc.id;
            mc.identify.verify_iq = mc.foc.iq;

            mc.identify.wait_tim++;
            if(mc.identify.wait_tim >= CURRENT_LOOP_SETTLE_TICKS)
            {
                mc.identify.verify_angle_deg =
                    mc.e_angle.electrical_angle_pu * 360.0f;
                mc.identify.verify_id = mc.foc.id;
                mc.identify.verify_iq = mc.foc.iq;
                mc.identify.wait_tim = 0;
                mc.e_angle.calib_flag = 6;
            }
            break;

        case 6:
            calculate_encoder_data(&mc.e_angle);
            if(motor_tune_get_speed_enable() != 0U)
            {
                if(speed_mode_last == 0U)
                {
                    pid_clear(&mc.spd_pid);
                    mc.spd_pid.integrate = motor_tune_get_speed_ff();
                    mc.iq_pid.ref = motor_tune_get_speed_ff();
                    speed_mode_last = 1U;
                }
                speed_loop_step();
                mc.id_pid.ref = 0.0f;
                mc.e_angle.electrical_angle_set_pu =
                    mc.e_angle.electrical_angle_pu;
            }
            else if(motor_tune_get_rotate_enable() != 0U)
            {
                speed_mode_last = 0U;
                speed_estimator_initialized = 0U;
                command_angle = mc.e_angle.electrical_angle_set_pu +
                    motor_tune_get_rotate_hz() * TS;
                if(command_angle >= 1.0f)
                {
                    command_angle -= 1.0f;
                }
                mc.e_angle.electrical_angle_set_pu = command_angle;
                mc.id_pid.ref = motor_tune_get_rotate_id();
                mc.iq_pid.ref = 0.0f;
            }
            else
            {
                speed_mode_last = 0U;
                speed_estimator_initialized = 0U;
                mc.e_angle.electrical_angle_set_pu =
                    mc.e_angle.electrical_angle_pu;
                ramp_half_ticks = motor_tune_get_ramp_half_ticks();
                ref_min = motor_tune_get_ref_min();
                ref_max = motor_tune_get_ref_max();

                if(mc.identify.wait_tim >= 2U * ramp_half_ticks)
                {
                    mc.identify.wait_tim = 0;
                }
                if(mc.identify.wait_tim < ramp_half_ticks)
                {
                    mc.id_pid.ref = ref_min +
                        (ref_max - ref_min) *
                        ((float)mc.identify.wait_tim /
                         (float)ramp_half_ticks);
                }
                else
                {
                    mc.id_pid.ref = ref_max -
                        (ref_max - ref_min) *
                        ((float)(mc.identify.wait_tim -
                                 ramp_half_ticks) /
                         (float)ramp_half_ticks);
                }
            }

            calculate_sin_cos(mc.e_angle.electrical_angle_set_pu,
                              &mc.foc.sin_val,
                              &mc.foc.cos_val);
            mc.foc.iu = mc.sample.ia_real;
            mc.foc.iv = mc.sample.ib_real;
            clark_transform(&mc.foc);
            park_transform(&mc.foc);

            mc.id_pid.fbk = mc.foc.id;
            mc.iq_pid.fbk = mc.foc.iq;
            pid_control(&mc.id_pid);
            pid_control(&mc.iq_pid);

            mc.foc.ud = mc.id_pid.out;
            mc.foc.uq = mc.iq_pid.out;
            mc.identify.verify_angle_deg =
                mc.e_angle.electrical_angle_pu * 360.0f;
            mc.identify.verify_id = mc.foc.id;
            mc.identify.verify_iq = mc.foc.iq;

            if(mc.identify.wait_tim < 60000U)
            {
                mc.identify.wait_tim++;
            }
            break;

        default:
            mc.e_angle.calib_flag = 0;
            break;
    }

    inverse_park_transform(&mc.foc);
    mc.foc.ubus = mc.sample.bus_real;
    calculate_svpwm(&mc.foc);
}

static void motor_pwm_start(void)
{
    u16 center_duty = (u16)(mc.foc.pwm_cycle * 0.25f);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, center_duty);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, center_duty);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, center_duty);

    if((HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) ||
       (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) ||
       (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) ||
       (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) ||
       (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) ||
       (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) ||
       (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK))
    {
        mc.motor.run_state = MOTOR_ERROR;
        mc.motor.error_code = OVER_CURRENT_ERR;
        return;
    }

    pwm_started = 1;
}

static void motor_pwm_stop(void)
{
    __HAL_TIM_MOE_DISABLE(&htim1);
    TIM1->CCER = 0;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_ENABLE(&htim1);
    mc.foc.ud = 0;
    mc.foc.uq = 0;
    pwm_started = 0;
}

static void motor_set_error(motor_error_code error_code)
{
    motor_pwm_stop();
    mc.motor.run_state = MOTOR_ERROR;
    mc.motor.error_code = error_code;
}

void Motor_System_Init(void)
{
    motor_struct_init();
    mc.motor.run_state = ADC_CALIB;
    mc.motor.error_code = NONE_ERR;
    mc.identify.state = RESISTANCE_IDENTIFICATION;
    mc.identify.flag = 0;
    mc.identify.end_flag = 0;
    mc.identify.rs = 0;
    mc.identify.ls = 0;
    mc.identify.ls_sum = 0;
    mc.identify.ls_time_sum = 0;
    mc.identify.verify_angle_deg = 0;
    mc.identify.verify_id = 0;
    mc.identify.verify_iq = 0;
    mc.identify.cur_max = 2.0f;

    pwm_started = 0;
    overcurrent_ticks = 0;
    bus_fault_ticks = 0;
    start_delay_ticks = 0;
    identify_ticks = 0;
#if FIXED_UD_TEST_ENABLE
    fixed_ud_test_ticks = 0;
#endif
}

void Motor_System_Run(void)
{
    calculate_bus_voltage(&mc.sample);
    calculate_phase_current(&mc.sample);
    mc.foc.ubus = mc.sample.bus_real;

    if(mc.identify.state == ENCODER_ROTOR_ALIGN)
    {
        mc.e_angle.encoder_val = encoder_data_get();
    }

    switch(mc.motor.run_state)
    {
        case ADC_CALIB:
            calculate_adc_offset(&mc.sample);
            if(mc.sample.calib_end_flag == 1)
            {
                mc.motor.run_state = MOTOR_IDENTIFY;
                start_delay_ticks = 0;
                identify_ticks = 0;
#if FIXED_UD_TEST_ENABLE
                fixed_ud_test_ticks = 0;
#endif
                overcurrent_ticks = 0;
                bus_fault_ticks = 0;
            }
            break;

        case MOTOR_IDENTIFY:
            if(pwm_started == 0)
            {
                if(start_delay_ticks < IDENTIFY_PWM_START_DELAY_TICKS)
                {
                    start_delay_ticks++;
                    break;
                }
                motor_pwm_start();
                if(mc.motor.run_state == MOTOR_ERROR)
                {
                    break;
                }
            }

            if((mc.sample.bus_real < IDENTIFY_BUS_MIN_VOLTAGE) ||
               (mc.sample.bus_real > IDENTIFY_BUS_MAX_VOLTAGE))
            {
                bus_fault_ticks++;
                if(bus_fault_ticks >= IDENTIFY_BUS_FAULT_TICKS)
                {
                    motor_set_error(POWER_VOLT_ERR);
                    break;
                }
            }
            else
            {
                bus_fault_ticks = 0;
            }

            if(mc.sample.current_mag > IDENTIFY_PHASE_OVERCURRENT)
            {
                overcurrent_ticks++;
                if(overcurrent_ticks >= IDENTIFY_OVERCURRENT_TICKS)
                {
                    motor_set_error(OVER_CURRENT_ERR);
                    break;
                }
            }
            else
            {
                overcurrent_ticks = 0;
            }

            if(mc.e_angle.calib_flag != 6U)
            {
                if(++identify_ticks > IDENTIFY_TIMEOUT_TICKS)
                {
                    motor_pwm_stop();
                    mc.motor.run_state = MOTOR_STOP;
                    break;
                }
            }
            else
            {
                identify_ticks = 0;
            }

#if FIXED_UD_TEST_ENABLE
            mc.foc.sin_val = 0;
            mc.foc.cos_val = 1;
            mc.foc.ud = FIXED_UD_TEST_VOLTAGE;
            mc.foc.uq = 0;
            inverse_park_transform(&mc.foc);
            mc.foc.ubus = mc.sample.bus_real;
            calculate_svpwm(&mc.foc);

            if(++fixed_ud_test_ticks > FIXED_UD_TEST_TICKS)
            {
                motor_pwm_stop();
                mc.motor.run_state = MOTOR_STOP;
                break;
            }
#else
            if(mc.identify.state == ENCODER_ROTOR_ALIGN)
            {
                encoder_align_step();
            }
            else
            {
                motor_identify();
            }

            if(mc.motor.run_state == MOTOR_ERROR)
            {
                break;
            }

            if(mc.identify.end_flag != 0)
            {
                motor_pwm_stop();
                mc.motor.run_state = MOTOR_STOP;
                break;
            }

            if(mc.foc.ud > IDENTIFY_MAX_UD_VOLTAGE)
            {
                mc.foc.ud = IDENTIFY_MAX_UD_VOLTAGE;
                inverse_park_transform(&mc.foc);
                calculate_svpwm(&mc.foc);
            }
#endif

            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, mc.foc.duty_cycle_a);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, mc.foc.duty_cycle_b);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, mc.foc.duty_cycle_c);
            break;

        /*
         * Not reached in the current bring-up stage.
         *
         * case MOTOR_SENSORUSE:
         *     calculate_encoder_data(&mc.e_angle);
         *     sensoruse_control();
         *     break;
         *
         * case MOTOR_SENSORLESS:
         *     calculate_encoder_data(&mc.e_angle);
         *     sensorless_control();
         *     break;
         */

        case MOTOR_ERROR:
        case MOTOR_STOP:
            motor_pwm_stop();
            break;

        default:
            break;
    }
}
