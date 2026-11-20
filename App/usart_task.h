#ifndef __USART_TASK_H
#define __USART_TASK_H


#include "main.h"

#define CH_COUNT 16              //VOFA JustFloat发送通道数
typedef struct
{                                                   
    float fdata[CH_COUNT];         //要发送的浮点数
    u8    tail[4];                 //VOFA的JUSTFLOAT格式的帧尾
}usart_tx_data;


void usart_task(void);







#endif



