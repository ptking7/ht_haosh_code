#include "usart_task.h"
#include "adc.h"
#include "motor_global.h"
#include "motor_publicdata.h"
#include "motor_tune.h"
#include "usbd_cdc_if.h"

usart_tx_data tx_data;
static float display_id = 0.0f;
static float display_iq = 0.0f;

/**
  * 函数功能: 定时发送 VOFA JustFloat 数据
  * 输入参数:
  * 返 回 值: 
  * 说    明:
  */
void usart_task(void)
{
    float alpha = motor_tune_get_display_alpha();
    display_id += alpha * (mc.foc.id - display_id);
    display_iq += alpha * (mc.foc.iq - display_iq);

    tx_data.fdata[0] = display_id;
    tx_data.fdata[1] = display_iq;
    tx_data.fdata[2] = mc.id_pid.ref;
    tx_data.fdata[3] = mc.iq_pid.ref;
    tx_data.fdata[4] = mc.foc.ud;
    tx_data.fdata[5] = mc.foc.uq;
    tx_data.fdata[6] = mc.sample.current_mag;
    tx_data.fdata[7] = mc.e_angle.electrical_angle_pu * 360.0f;
    tx_data.fdata[8] = (float)mc.e_angle.calib_flag;
    tx_data.fdata[9] = (float)mc.identify.state;
    tx_data.fdata[10] = (float)mc.motor.error_code;
    tx_data.fdata[11] = mc.foc.id;
    tx_data.fdata[12] = mc.foc.iq;
    tx_data.fdata[13] = mc.spd_pid.ref;
    tx_data.fdata[14] = mc.speed.mechanical_speed;
    tx_data.fdata[15] = mc.sample.bus_real;

    tx_data.tail[0] = 0x00;
    tx_data.tail[1] = 0x00;
    tx_data.tail[2] = 0x80;
    tx_data.tail[3] = 0x7f;

    (void)CDC_Transmit_FS((uint8_t *)&tx_data, sizeof(tx_data));
}

