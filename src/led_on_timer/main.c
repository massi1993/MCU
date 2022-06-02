#include <stdio.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_gpio_driver.h"
#include "stm32f3x_timer_driver.h"

int flag = RESET;

void main(){
  
        //GPIOE ENABLE
        RCC->AHBENR |= GPIOE_EN;
        //TIM2 ENABLE
        RCC->APB1ENR |= TIM2_EN;
        
        //SET PEx (WITH x = 8,9,..15) IN OUTPUT_MODE
        for(int i = 8; i < 16; i++)
        {
          GPIOE->MODER|= (OUT_MODE << 2*i);
        }
 
        TIM2->ARR = N_CONT(0.5); //TIME (in seconds) IN INPUT
        
        //ENABLE COUNTER PULL UP BIT CEN
        TIM2->CR1 |= CEN_EN;
        //RESET THE COUNTER
        TIM2->CNT = 0;     /*!< WHEN CNT REACHES N_CNT(X) THE UIF IS SET TO 1 >*/
        
        while(1)
        {
            if(CHECK_UIF && !flag)
            {  
              GPIOE->ODR = GPIOE_ALL_LED_ON;        //ON ALL LED
              TIM2->SR&=~SET_UIF;                   //RESET UPDATE INTERRUPT FLAG
              flag = SET;    
            }
            else if(CHECK_UIF && flag)
            {
               GPIOE->ODR = RESET;                 //OFF ALL LED
               TIM2->SR &= ~SET_UIF;               //RESET UPDATE INTERRUPT FLAG
               flag = RESET;
            }
        }
}
