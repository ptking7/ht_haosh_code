#ifndef __OBSERVER_DRV_H
#define __OBSERVER_DRV_H

#include "main.h"

typedef struct
{
    float ts;
    float rs;
    float ld;
    float gain;
    float ialpha;
    float ibeta;
    float ialpha_fore;
    float ibeta_fore;
    float ualpha;
    float ubeta;
    float ealpha_fore;
    float ealpha_fore_lpf;
    float ebeta_fore;
    float ebeta_fore_lpf;
    float eab_fore_lpf_factor;
    float emag;
} smo_struct;

typedef struct
{
    u8    enable;
    u8    nsd_out;
    u8    nsd_flag;
    u16   nsd_count;
    float nsd_sum1;
    float nsd_sum2;
    float id_ref;
    float id_high;
    u8    dir;
    float uin;
    float id;
    float id_last;
    float id_base;
    float iq;
    float iq_last;
    float iq_base;
    float ialpha;
    float ialpha_last;
    float ialpha_high;
    float ialpha_high_last;
    float ibeta;
    float ibeta_last;
    float ibeta_high;
    float ibeta_high_last;
    float ialpha_out;
    float ibeta_out;
} hfi_struct;

typedef struct
{
    s8    dir;
    float ts;
    float ain;
    float bin;
    float theta_err;
    float theta_fore;
    float theta_compensate;
    float e_theta;
    float e_theta_pu;
    float sin_val;
    float cos_val;
    float kp;
    float ki;
    float p_part;
    float i_part;
    float we_fore;
    float we_fore_lpf;
    float we_fore_lpf_factor;
} pll_struct;

void hfi_calculate(hfi_struct *p);
void pll_calculate(pll_struct *p);
void smo_calculate(smo_struct *p);

#endif
