/* 
*
*       Created on : June 10, 2022 
*       Last Update: June 10, 2022
*           Author : massiAv
*

*/
#include <stdio.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_api_driver.h"
#include "stm32f3x_timer_driver.h"


float time_meas = RESET;

void main()
{
        RCC_PCLK_AHBEN(RCC_AHBENR_GPIOA,ENABLE);                /*!< ENABLE PORT A              >*/
        RCC_PCLK_APB1EN(RCC_APB1ENR_TIM2,ENABLE);               /*!< ENABLE TIMER2              >*/
        
        GPIO_MODE(GPIOA,IN_MODE,Px0);                           /*!< SET PA0 IN INPUT_MODE      >*/
        
        while (1)
        {
          
            time_meas = Measure_Time(TIM2,STOP_WATCH);           /*!< Use TIM2 as StopWatch       >*/
            //time_meas = Measure_Time(TIM2,NOT_STOP_WATCH);       /*!< Use TIM2 as NotStopWatch. It measure the time during pressing PA0    >*/
            if(time_meas>0.0001)
              printf("Measurement time is: %f seconds\n", time_meas);
        }
}
