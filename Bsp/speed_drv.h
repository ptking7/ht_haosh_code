#ifndef __SPEED_DRV_H
#define __SPEED_DRV_H

#include "main.h"

typedef enum
{
    ACCELERATE = 0,
    UNIFORM,
    DECELERATE
} motion_state;

typedef struct
{
    u8    pole_pairs;
    s16   current_speed_dir;
    u16   speed_calculate_cnt;
    u16   speed_divsion_factor;
    u16   electrical_val_max;
    float electrical_pos_this;
    float electrical_pos_last;
    float electrical_pos_change;
    float electrical_speed_factor;
    float electrical_speed_raw;
    float electrical_speed_lpf;
    float electrical_speed_lpf_factor;
    float mechanical_speed;
    float mechanical_speed_set;
    float mechanical_speed_set_last;
} speed_struct;

typedef struct
{
    s8    speed_out_dir;
    u8    pole_pairs;
    float ts;
    float target_speed;
    float acc_speed;
    float speed_target_increment;
    float speed_increment;
    float speed_change_increment;
    float speed_out;
    float speed_iincrement_delta;
    float speed_increment_last;
    motion_state motion_state;
    u8    finish_flag;
} t_shaped_acc_dec_struct;

void calculate_speed(speed_struct *p);
void t_shaped_acc_dec(t_shaped_acc_dec_struct *p);

#endif
