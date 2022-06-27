/*
*
*       Created on : June 25, 2022 
*      Last Update : June 27, 2022
*           Author : massiAv
*               
*  Let's use Timer2 to turn on every 0.5 seconds the red or blue leds.
*  Let's use Timer3 to turn on, every second, the green or orange leds.     
*/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_api_driver.h"
#include "stm32f3x_timer_driver.h"

int flag = RESET;

const char txt[150] = "Type:\r\nB to turn on the Blue Leds\r\nR to turn on the Red Leds\r\nO to turn on the Orange Leds\r\nG to turn on the Green Leds\r\n";
const char error_txt[60] = "\r\nValue not valid, RETRY\r\n";

char data_received; 

float flashing_tim3 = 1;

void main()
{
        RCC_PCLK_AHBEN(RCC_AHBENR_GPIOE,ENABLE);                /*!< ENABLE GPIOE >*/
        RCC_PCLK_AHBEN(RCC_AHBENR_GPIOC,ENABLE);                /*!< ENABLE GPIOC TO SET USART COMMUNICATION >*/
        RCC_PCLK_APB1EN(RCC_APB1ENR_TIM2,ENABLE);               /*!< ENABLE TIMER2 >*/
        RCC_PCLK_APB1EN(RCC_APB1ENR_TIM3,ENABLE);               /*!< ENABLE TIMER3 >*/
        RCC_PCLK_APB2EN(RCC_APB2ENR_USART1,ENABLE);             /*!< ENABLE USART1 >*/
          
        GPIOE_OUTMODE(Px8,Px15);                                /*!< From PE8 to PE15 IN OUTPUT MODE >*/
          
        GPIO_MODE(GPIOC,AF_MODE,Px4);                           /*!< PC4 IN AF MODE TO SET USART1_TX >*/
        GPIO_AFR(GPIOC,AF7,Px4);        
  
        GPIO_MODE(GPIOC,AF_MODE,Px5);                           /*!< PC5 IN AF MODE TO SET USART1_RX >*/
        GPIO_AFR(GPIOC,AF7,Px5);        
          
        CNT_EN_TIM(TIM2,ENABLE);                                /*!< ENABLE TIM2'S COUNTER >*/
        TIM2->ARR = N_CONT_72Mhz(0.5);                          /*!< ARR ARE SETTED IN ORDER TO HAVE 0.5 SECOND USING TIM2 AT 72 MHz*/
        
        CNT_EN_TIM(TIM3,ENABLE);                                /*!< ENABLE TIM3'S COUNTER >*/
        set_PSC_and_ARR_TIM(TIM3,flashing_tim3,F_ck72MHz);      /*!< PSC AND ARR ARE SETTED IN ORDER TO HAVE flashing_tim3 SECOND USING TIM3 AT 72 MHz*/
        
        /*!< SETUP USART>*/
        setup_USART_RX_TX(USART1);
        
        
        usart_tx(USART1,txt,strlen(txt));
        
        while(!(USART1->ISR & USART_ISR_RXNE));                 /*!< Waiting for the data to be ready to be read      >*/
        
        while(1){
          
          data_received = usart_rx(USART1);
          
          if(data_received == 'B'){
            if(CHECK_UIF(TIM2) && !flag)
            {
                GPIOE->ODR = (1<<8);
                TIM2->SR&=~(1<<0);
                flag = SET;
            }
            if(CHECK_UIF(TIM2) && flag)
            {
                GPIOE->ODR = (1<<12);
                TIM2->SR&=~(1<<0);
                flag = RESET;
            }
          }
          if(data_received == 'R'){
            if(CHECK_UIF(TIM2) && !flag)
            {
                GPIOE->ODR = (1<<9);
                TIM2->SR&=~(1<<0);
                flag = SET;
            }
            if(CHECK_UIF(TIM2) && flag)
            {
                GPIOE->ODR = (1<<13);
                TIM2->SR&=~(1<<0);
                flag = RESET;
            }
          }
          if(data_received == 'O'){
            if(CHECK_UIF(TIM3) && !flag)
            {
                GPIOE->ODR = (1<<10);
                TIM3->SR&=~(1<<0);
                flag = SET;
            }
            if(CHECK_UIF(TIM3) && flag)
            {
                GPIOE->ODR = (1<<14);
                TIM3->SR&=~(1<<0);
                flag = RESET;
            }
          }
          if(data_received == 'G'){
            if(CHECK_UIF(TIM3) && !flag)
            {
                GPIOE->ODR = (1<<11);
                TIM3->SR&=~(1<<0);
                flag = SET;
            }
            if(CHECK_UIF(TIM3) && flag)
            {
                GPIOE->ODR = (1<<15);
                TIM3->SR&=~(1<<0);
                flag = RESET;
            }
          }
      
        }
}
