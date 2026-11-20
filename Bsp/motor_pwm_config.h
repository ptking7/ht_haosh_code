#ifndef __MOTOR_PWM_CONFIG_H
#define __MOTOR_PWM_CONFIG_H

/*
 * Keep the PWM timer, ADC trigger and current-loop timing on one definition.
 * TIM1 runs at 170 MHz in center-aligned mode, so one full PWM period is
 * two counter periods.
 */
#define PWM_TIMER_CLOCK_HZ             170000000.0f
#define PWM_SWITCHING_HZ               20000.0f
#define PWM_TIMER_ARR                  ((u16)((PWM_TIMER_CLOCK_HZ / (2.0f * PWM_SWITCHING_HZ)) + 0.5f))
#define PWM_CYCLE                      ((u16)(2U * (u16)PWM_TIMER_ARR))
#define TS                             (2.0f * (float)PWM_TIMER_ARR / PWM_TIMER_CLOCK_HZ)
#define CONTROL_TICKS_PER_SECOND       (1.0f / TS)
#define CONTROL_TICKS_PER_MS           (1.0f / (TS * 1000.0f))
#define CONTROL_VOLTAGE_STEP           (2.0f * TS)
#define CURRENT_KI_BASE_TS             0.00005f
#define CURRENT_KI_SCALE               (TS / CURRENT_KI_BASE_TS)
#define PWM_ADC_TRIGGER_LEAD_TICKS     100U
#define PWM_DEADTIME_TICKS             85U
#define PWM_LIMIT_RATIO                0.91764706f
#define PWM_LIMLT                      ((u16)((float)PWM_CYCLE * PWM_LIMIT_RATIO))

#endif
