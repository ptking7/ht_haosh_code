#include "usart_task.h"
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
    u32 tick = HAL_GetTick();
    float phase = (tick % 2000) * 0.001f;

    tx_data.fdata[0] = phase;
    tx_data.fdata[1] = (phase > 1.0f) ? (2.0f - phase) : phase;
    tx_data.fdata[2] = 1.0f;

    tx_data.tail[0] = 0x00;
    tx_data.tail[1] = 0x00;
    tx_data.tail[2] = 0x80;
    tx_data.tail[3] = 0x7f;

    (void)CDC_Transmit_FS((uint8_t *)&tx_data, sizeof(tx_data));
}

