/*
*
*       Created on : June 05, 2022 
*           Author : massiAv
*

*
TIMER 3 IS A 16-BIT TIMER;
    
Before the timer clock signal gets to the counter, it must pass through the prescaler, PSC. 
This is a 16 bit counter that simply counts up to the value in the PSC register and wraps around. 
The overflow from that is what drives the actual counter, CNT.
A value of zero in PSC effectively passes the input clock straight to the counter. 
It is safe to change the prescaler at any time since writes are buffered and the new value will be written at the next overflow.
The prescaler then can divide the input clock by any value from 1 to 65536. Suppose the input clock frequency to the prescaler is 72MHz. 
It would be possible to get the counter input frequency down to as little as 1098 Hz. 

The value written into PSC is the prescale_value – 1. 
Thus, to get a division by 10, the PSC register must get the value 9; 
to get a division by 720, the PSC register gets the value 719.
*/


#include <stdio.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_api_driver.h"
#include "stm32f3x_timer_driver.h"

#define Fck 72000000
 
int time = 1;    //1 secondo

unsigned int _bit_pos_GPIO_MODER = SET;         /*!< Bit position in which write the data >*/ 
unsigned int _index_NVIC_ISER = SET;            /*!< Index of NVIC -> ISER >*/     

void main(){

          RCC_PCLK_AHBEN(RCC_AHBENR_GPIOE,ENABLE);      /*!< ENABLE GPIOE ON AHB BUS >*/        
          RCC_PCLK_APB1EN(RCC_APB1ENR_TIM3,ENABLE);     /*!< ENABLE TIMER2 ON APB1 BUS >*/

          GPIOE_OUTMODE(Px8,Px15);                      /*!< ALL LED IN OUTPUT MODE >*/

          set_PSC_and_ARR_TIM(TIM3, time, Fck);         /*!< SET PSC and ARR for TIMER3 >*/
          
          CNT_EN_TIM(TIM3,ENABLE);                      /*!< RESET CNT ON TIMER3 AND ENABLE COUNTER>*/
          
          TIM3->DIER|=TIM_DIER_UIE;

          /*!< Configure the index of NVIC_ISER and unmasking of TIM3 global interrupt>*/
          _index_NVIC_ISER = index_NVIC_ISER(IRQ_TIM3);
          NVIC->ISER[_index_NVIC_ISER]|= (1 << IRQ_TIM3);    //INTERRUPT SU TIM3

          while(1);
 
}
 
void TIM3_IRQHandler()
{
          TIM3->SR&=~(1<<0);
          GPIOE->ODR^=GPIOE_ALL_LED_ON; //^= INVERTE LO STATO
 
 }

