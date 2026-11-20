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
u8 motor_tune_get_speed_enable(void);
float motor_tune_get_speed_rpm(void);
float motor_tune_get_speed_ff(void);
float motor_tune_get_speed_min_current(void);

#endif
