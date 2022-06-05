/*
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
 
unsigned int prescaler_TIM3;
unsigned int nArr_TIM3;

int time = 1;    //1 secondo

unsigned int _bit_pos_GPIO_MODER = SET;         /*!< Bit position in which write the data >*/ 
unsigned int _index_NVIC_ISER = SET;            /*!< Index of NVIC -> ISER >*/     

void main(){

      /*!< ENABLE GPIOE AND GPIOA ON AHB BUS >*/
      RCC->AHBENR|=GPIOE_EN|GPIOA_EN;
     
      /*!< ENABLE TIMER2 ON APB1 BUS >*/
      RCC->APB1ENR|=TIM3_EN;
     
      /*!< LED IN OUTPUT MODE >*/
      SET_PE_IN_OUT_MODE();

      prescaler_TIM3 = (unsigned int)(time*Fck/65535);
      TIM3->PSC = prescaler_TIM3;
      nArr_TIM3 = (unsigned int)(time*Fck)/(TIM3->PSC +1);
      TIM3->ARR = nArr_TIM3;
     
      TIM3->CNT=0;
      TIM3->CR1|=CEN_EN;   //COUNTER ENABLE
     
      TIM3->DIER|=TIM_DIER_UIE;
      
      /*!< Configure the index of NVIC_ISER and unmasking of TIM3 global interrupt>*/
      _index_NVIC_ISER = index_NVIC_ISER(IRQ_TIM3);
      NVIC->ISER[_index_NVIC_ISER]|= (1 << IRQ_TIM3);    //INTERRUPT SU TIM3
     
      while(1);
 
}
 
void TIM3_IRQHandler(){

  TIM3->SR&=~(1<<0);
  GPIOE->ODR^=GPIOE_ALL_LED_ON; //^= INVERTE LO STATO
  
 }

