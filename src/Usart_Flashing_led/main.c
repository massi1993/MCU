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

void flashing_led(TIMER_Type* Timer, unsigned int PinNumber);   /*!< Function's prototype for flashing LEDs>*/

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
        
        setup_USART_RX_TX(USART1);                              /*!< SETUP USART>*/
        
        usart_tx(USART1,txt,strlen(txt));                       /*!< txt is the string to transmit via USART            >*/
        
        while(!(USART1->ISR & USART_ISR_RXNE));                 /*!< Waiting for the data to be ready to be read        >*/
        
        while(1){
          
          data_received = usart_rx(USART1);                     /*!< Storing into data_received the data received via USART>*/
          
          /*!<Alghoritm for flashing LEDs>*/
          if(data_received == 'B')
          {
            flashing_led(TIM2,Px8);
          }
          if(data_received == 'R')
          {
            flashing_led(TIM2,Px9);
          }
          if(data_received == 'O')
          {
            flashing_led(TIM3,Px10);
          }
          if(data_received == 'G')
          {
            flashing_led(TIM3,Px11); 
          }
          if(data_received != 'G' && data_received != 'O' && data_received != 'R' && data_received != 'B')
          {
            usart_tx(USART1,error_txt,strlen(error_txt));
            GPIOE-> BRR = 0xFF00;
            while(!(USART1->ISR & USART_ISR_RXNE));                 /*!< Waiting for the data to be ready to be read      >*/
          }
          
        }
}


/*!< Function for flashing LEDs>*/
void flashing_led(TIMER_Type* Timer, unsigned int PinNumber){

  
        if(CHECK_UIF(Timer) && !flag)
        {
            GPIOE->ODR = (1<<PinNumber);
            Timer->SR&=~(1<<0);
            flag = SET;
        }
        if(CHECK_UIF(Timer) && flag)
        {
            GPIOE->ODR = (1<<(PinNumber+4));
            Timer->SR&=~(1<<0);
            flag = RESET;
        }
}