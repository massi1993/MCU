#include <stdio.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_gpio_driver.h"
#include "stm32f3x_timer_driver.h"


short int flag = SET;

unsigned int _index_EXTI = RESET;                   /*!< Index of SYSCFG -> EXTI >*/ 
unsigned int _bit_pos_EXTI = RESET;                 /*!< Bit position in which write the data >*/  

void main(){
         
          RCC->AHBENR |= GPIOE_EN;
          RCC->AHBENR |= GPIOA_EN;              /*!< Enable GPIOA and GPIOE >*/ 
         
          SET_PE_IN_OUT_MODE();                 /*!< Enable PEx (x = 8,..15) in output mode >*/ 
          
          
          GPIOA->MODER &=~ (IN_MODE<<0);        /*!< Enable PA0 in input mode >*/         
          
          /*!<CONFIGURE PA0 AS EXTERNAL INTERRUPT. WE HAVE TO SELECT PA0 AS MULTIPLEXER'S INPUT >*/
          _index_EXTI = index_EXTI(Px0);
          _bit_pos_EXTI = bit_pos_EXTI(Px0);
          SYSCFG->EXTICR[_index_EXTI] &=~ (PA0 << _bit_pos_EXTI);
         
          //SMASCHERO LA LINEA 0 SETTANDOLA AD 1
          EXTI->IMR|=(1<<0);
         
          //GENERO INTERRUPT SUL FRONTE DI SALITA TR0
          EXTI->RTSR |= (1<<0);
         
          //CANCELLO LA CAUSA DI INTERRUZIONE
          EXTI->PR |= (1<<0);
         
          //ABILITO DA NVIC LA POSIZIONE DI EXTI LINE 0 INTERRUPT
          NVIC->ISER[0] |= (1<<6);
         
          while(1);
}
 
void EXTI0_IRQHandler()
{
     //RICANCELLO LA CAUSA DI INTERRUZIONE
      EXTI->PR|=(1<<0);
     
      //se flag==1 -> accendo i led
      if(flag == SET)
      {
        GPIOE->ODR|=GPIOE_ALL_LED_ON;
        flag = RESET;
      } else   
      {
        //se flag==0 -> spengo i led
        GPIOE->ODR=0;
        flag=SET;
      }
}
