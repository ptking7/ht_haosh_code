#ifndef __POSITION_DRV_H
#define __POSITION_DRV_H

#include "main.h"

typedef struct
{
    u16   pos_calculate_cnt;
    u16   electrical_val_max;
    s32   electrical_pos_this;
    s32   electrical_pos_last;
    s32   electrical_pos_change;
    s32   electrical_pos_sum;
    s32   mechanical_pos_raw;
    s32   mechanical_pos_set;
} position_struct;

void calculate_position(position_struct *p);

#endif
