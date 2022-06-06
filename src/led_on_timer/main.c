/* 
*
*       Created on : June 02, 2022 
*           Author : massiAv
*
*/

#include <stdio.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_api_driver.h"
#include "stm32f3x_timer_driver.h"

int flag = RESET;

void main(){
  
        /*!< ENABLE GPIOE AND TIMER2>*/
        RCC_PCLK_AHBEN(RCC_AHBENR_GPIOE,ENABLE);
        RCC_PCLK_APB1EN(RCC_APB1ENR_TIM2,ENABLE);
                
        /*!< SET PEx (WITH x = 8,9,..15) IN OUTPUT_MODE >*/
        SET_PE_IN_OUT_MODE();
 
        TIM2->ARR = N_CONT(0.5);                /*!< TIME (in seconds) as INPUT > */
        
        /*!< ENABLE COUNTER PULL UP BIT CEN >*/
        TIM2->CR1 |= CEN_EN;
        
        /*!< RESET THE COUNTER >*/
        TIM2->CNT = RESET;                      /*!< WHEN CNT REACHES N_CNT(X) THE UIF IS SET TO 1 >*/
        
        while(1)
        {
            if(CHECK_UIF && !flag)
            {  
              GPIOE->ODR = GPIOE_ALL_LED_ON;    /*!< ON ALL LED                    > */
              TIM2->SR&=~SET_UIF;               /*!< RESET UPDATE INTERRUPT FLAG   > */
              flag = SET;    
            }
            else if(CHECK_UIF && flag)
            {
               GPIOE->ODR = RESET;              /*!< OFF ALL LED                 >*/    
               TIM2->SR &= ~SET_UIF;            /*!< RESET UPDATE INTERRUPT FLAG >*/
               flag = RESET;
            }
        }
}
