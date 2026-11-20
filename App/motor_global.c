/**
  ******************************************************************************
  * @file    motor_global.c
  * @brief   ADC sampling, calibration, LED and VOFA tasks.
  ******************************************************************************
  */

#include "motor_global.h"
#include "adc.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"
#include "motor_publicdata.h"
#include "motor_system.h"
#include "usart_task.h"
#include "motor_tune.h"

static u16 led_task_tim = 0;
static volatile u8 vofa_send_flag = 0;

void global_init(void)
{
    HAL_Delay(100);
    Motor_System_Init();
    motor_tune_init();

    if((HAL_OPAMP_Start(&hopamp1) != HAL_OK) ||
       (HAL_OPAMP_Start(&hopamp2) != HAL_OK) ||
       (HAL_OPAMP_Start(&hopamp3) != HAL_OK))
    {
        Error_Handler();
    }

    if((HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) ||
       (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK))
    {
        Error_Handler();
    }

    if((HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) ||
       (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK))
    {
        Error_Handler();
    }

    __HAL_TIM_MOE_DISABLE(&htim1);
    if(HAL_TIM_Base_Start(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim6);
}

void global_loop(void)
{
    if(vofa_send_flag)
    {
        vofa_send_flag = 0;
        usart_task();
    }
    motor_tune_task();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == htim2.Instance)
    {
        led_task_tim++;
        if(led_task_tim >= 500)
        {
            led_task_tim = 0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
        }
    }

    if(htim->Instance == htim6.Instance)
    {
        vofa_send_flag = 1;
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc->Instance == ADC1)
    {
        mc.sample.ic_raw = ADC1->JDR1;
        mc.sample.ib_raw = ADC2->JDR1;
        mc.sample.bus_raw = ADC1->JDR2;

        Motor_System_Run();
    }
}

u16 encoder_data_get(void)
{
    static s32 encoder_history[3] = {0, 0, 0};
    static u8 history_count = 0U;
    static u8 history_index = 0U;
    static u16 encoder_val_filtered = 0U;
    s32 sample;
    s32 median;
    u8 i;
    u8 j;
    u8 tx_data[2] = {0x00, 0x00};
    u8 rx_data[2] = {0x00, 0x00};

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    if(HAL_SPI_TransmitReceive(&hspi3, tx_data, rx_data, 2, 1) == HAL_OK)
    {
        sample = ((s32)rx_data[0] << 8) | rx_data[1];
        if(history_count != 0U)
        {
            if((sample - encoder_history[history_index]) > PUL_MAX_HALF)
            {
                sample -= ENCODER_COUNTS_PER_REV;
            }
            else if((sample - encoder_history[history_index]) <
                    -(s32)PUL_MAX_HALF)
            {
                sample += ENCODER_COUNTS_PER_REV;
            }
        }

        encoder_history[history_index] = sample;
        history_index = (u8)((history_index + 1U) % 3U);
        if(history_count < 3U)
        {
            history_count++;
        }

        median = sample;
        for(i = 0U; i < history_count; i++)
        {
            u8 less = 0U;
            for(j = 0U; j < history_count; j++)
            {
                if(encoder_history[j] < encoder_history[i])
                {
                    less++;
                }
            }
            if(less == (u8)(history_count / 2U))
            {
                median = encoder_history[i];
                break;
            }
        }
        encoder_val_filtered = (u16)(median % ENCODER_COUNTS_PER_REV);
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    return encoder_val_filtered;
}
