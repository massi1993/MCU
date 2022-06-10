/* 
*
*       Created on : June 10, 2022 
*       Last Update: June 10, 2022
*           Author : massiAv
*
@brief  Start time measurement when the PA0 is pressed; Stop time when PA0 is pressed again.
        Basically, we are implementing a stopwatch.
*/
#include <stdio.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_api_driver.h"
#include "stm32f3x_timer_driver.h"


float time = RESET;                                             /*!< variable to store time measurement >*/

void main()
{
        RCC_PCLK_AHBEN(RCC_AHBENR_GPIOA,ENABLE);                /*!< ENABLE PORT A              >*/
        RCC_PCLK_APB1EN(RCC_APB1ENR_TIM2,ENABLE);               /*!< ENABLE TIMER2              >*/
        
        GPIO_MODE(GPIOA,IN_MODE,Px0);                           /*!< SET PA0 IN INPUT_MODE      >*/
        
        while (1)
        {
            PA0_IDR_PRESSED(0);                                 /*!< WAIT UNTIL PA0 IS PRESSED  TO START TIME MEASUREMENT >*/
            CNT_EN_TIM(TIM2,ENABLE);                            /*!< ENABLE COUNT TIM2          >*/
          
            PA0_IDR_PRESSED(1);                                 /*!< WAIT PA0 IS RELEASE        >*/
          
            PA0_IDR_PRESSED(0);                                 /*!< WAIT UNTIL PA0 IS PRESSED AGAIN TO STOP THE MEASUREMENT >*/
            CNT_EN_TIM(TIM2,DISABLE);                           /*!< DISABLE COUNT TIM2          >*/
          
            time = (float) (TIM2->CNT)/F_CLK;                   /*!< MEASUREMENT TIME           >*/
          
            printf("Measurement time is: %f\n", time);
        }
}
