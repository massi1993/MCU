/* 
*
*       Created on : June 04, 2022 
*      Last Update : June 06, 2022       
*           Author : massiAv
*
*/

#include <stdio.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_api_driver.h"
#include "stm32f3x_timer_driver.h"


short int flag = SET;


unsigned int _index_NVIC_ISER = SET;                /*!< Index of NVIC -> ISER >*/ 

void main(){
         
          RCC_PCLK_AHBEN(RCC_AHBENR_GPIOA,ENABLE);
          RCC_PCLK_AHBEN(RCC_AHBENR_GPIOE,ENABLE);      /*!< Enable GPIOA and GPIOE >*/ 
         
          SET_PE_IN_OUT_MODE();                         /*!< Enable PEx (x = 8,..15) in output mode >*/ 
          
          GPIO_MODE(GPIOA,IN_MODE,Px0);                 /*!< Enable PA0 in input mode >*/         
          
          set_SYSCFG_EXTI(Px0,PA0);                     /*!<CONFIGURE Px0 to get the index and bit position of SYSCFG->EXTI
                                                        AND PA0 AS EXTERNAL INTERRUPT. WE HAVE TO SELECT PA0 AS MULTIPLEXER'S INPUT >*/        
         
          //SMASCHERO LA LINEA 0 SETTANDOLA AD 1
          EXTI->IMR|=(1<<0);
         
          //GENERO INTERRUPT SUL FRONTE DI SALITA TR0
          EXTI->RTSR |= (1<<0);
         
          //CANCELLO LA CAUSA DI INTERRUZIONE
          EXTI->PR |= (1<<0);
         
          /*!< Configure the index of NVIC_ISER and unmasking of EXTI0_Line global interrupt>*/
          _index_NVIC_ISER = index_NVIC_ISER(IRQ_NO_EXTI0);
          NVIC->ISER[_index_NVIC_ISER] |= (1<<IRQ_NO_EXTI0);
         
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
