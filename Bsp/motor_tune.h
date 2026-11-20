#ifndef __MOTOR_TUNE_H
#define __MOTOR_TUNE_H

#include "main.h"

void motor_tune_init(void);
void motor_tune_receive(const u8 *data, u32 length);
void motor_tune_task(void);

float motor_tune_get_display_alpha(void);
float motor_tune_get_ref_min(void);
float motor_tune_get_ref_max(void);
u16 motor_tune_get_ramp_half_ticks(void);
u8 motor_tune_get_rotate_enable(void);
float motor_tune_get_rotate_hz(void);
float motor_tune_get_rotate_id(void);
float motor_tune_get_track_angle_lpf(void);
float motor_tune_get_current_output_lpf(void);
float motor_tune_get_current_deadband(void);
u8 motor_tune_get_current_angle_hold(void);
u8 motor_tune_get_speed_enable(void);
u16 motor_tune_get_speed_window_ticks(void);
float motor_tune_get_speed_rpm(void);
float motor_tune_get_speed_ff(void);
float motor_tune_get_speed_min_current(void);
u8 motor_tune_get_open_enable(void);
u8 motor_tune_get_open_angle_hold(void);
float motor_tune_get_open_ud(void);
u8 motor_tune_get_open_sweep(void);
float motor_tune_get_open_ud_low(void);
float motor_tune_get_open_ud_high(void);
u32 motor_tune_get_open_sweep_ticks(void);
u32 motor_tune_get_open_ramp_ticks(void);
float motor_tune_get_open_dither_hz(void);
float motor_tune_get_open_dither_ud(void);

#endif
