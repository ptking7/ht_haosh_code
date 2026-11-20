#include "usart_task.h"
#include "adc.h"
#include "motor_global.h"
#include "motor_publicdata.h"
#include "motor_tune.h"
#include "usbd_cdc_if.h"

usart_tx_data tx_data;

/**
  * 函数功能: 定时发送 VOFA JustFloat 数据
  * 输入参数:
  * 返 回 值: 
  * 说    明:
  */
void usart_task(void)
{
    tx_data.fdata[0] = (float)mc.foc.duty_cycle_a;
    tx_data.fdata[1] = (float)mc.foc.duty_cycle_b;
    tx_data.fdata[2] = (float)mc.foc.duty_cycle_c;
    tx_data.fdata[3] = mc.sample.ia_real;
    tx_data.fdata[4] = mc.sample.ib_real;
    tx_data.fdata[5] = mc.sample.ic_real;
    tx_data.fdata[6] = mc.spd_pid.ref;
    tx_data.fdata[7] = mc.speed.mechanical_speed;
    tx_data.fdata[8] = mc.spd_pid.ref * OUTPUT_GEAR_RATIO;
    tx_data.fdata[9] = mc.speed.mechanical_speed * OUTPUT_GEAR_RATIO;

    tx_data.tail[0] = 0x00;
    tx_data.tail[1] = 0x00;
    tx_data.tail[2] = 0x80;
    tx_data.tail[3] = 0x7f;

    (void)CDC_Transmit_FS((uint8_t *)&tx_data, sizeof(tx_data));
}

