/* 
*
*       Created on : June 8, 2022 
*       Last Update: June 08, 2022
*           Author : massiAv
*
*/

#include <stdio.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_api_driver.h"
#include "stm32f3x_timer_driver.h"

unsigned int cont_max = RESET;
unsigned int time_flash = 1;            /*!< time flashing in seconds                   >*/
float delta_time_flash = 0.01;          /*!< delta time to subtract from cont_max       >*/
unsigned int flag_flash = RESET;

void main()
{
  
        RCC_PCLK_AHBEN(RCC_AHBENR_GPIOE,ENABLE);                /*!< ENABLE CLOCK GPIOE                 >*/
        RCC_PCLK_APB1EN(RCC_APB1ENR_TIM2,ENABLE);               /*!< ENABLE CLOCK TIMER2                >*/
        
        
        GPIOE_OUTMODE(Px8,Px15);                                /*!< FROM PE8 TO PE15 IN OUTPUT MODE    >*/
        
        CNT_EN_TIM(TIM2,ENABLE);                                /*!< ENABLE COUNT FOR TIM2              >*/
        
        cont_max = N_CONT(time_flash);                          /*!< counter max for TIM->ARR           >*/
        
        while (1){
          
          TIM2->ARR = cont_max;
              
          if(CHECK_UIF(TIM2) && !flag_flash)
          {
              TIM2->SR &=~ SET_UIF;                             /*!< when ARR reaches N_CNT TIM2->SR =1. SO, HERE PUT ON 0 TIM2->SR >*/        
              GPIOE->ODR |= GPIOE_ALL_LED_ON;                   /*!< ON ALL LED >*/
              cont_max -= N_CONT(delta_time_flash); 
              flag_flash = SET;
              if(cont_max == 0)
              { 
                  GPIOE->ODR = 0;                              /*!< OFF ALL LED >*/
                  cont_max = N_CONT(time_flash);               /*!< RESTART counter max for TIM->ARR           >*/
              }
          }
          else if(CHECK_UIF(TIM2) && flag_flash){
              TIM2->SR &=~ SET_UIF;                             /*!< when ARR reaches N_CNT TIM2->SR =1. SO, HERE PUT ON 0 TIM2->SR >*/        
              GPIOE->ODR = 0;                                   /*!< OFF ALL LED >*/
              flag_flash = RESET;
          }
        }   
}
