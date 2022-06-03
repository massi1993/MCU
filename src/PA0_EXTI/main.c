#include <stdio.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_gpio_driver.h"
#include "stm32f3x_timer_driver.h"


short int flag = SET;
 
void main(){
          //ABILITARE GPIOA E GPIOE
          RCC->AHBENR|=GPIOE_EN;
          RCC->AHBENR|=GPIOA_EN;
         
          //IMPOSTO I LED DA PE8 A PE15 COME USCITA
          GPIOE->MODER|=0x55550000;  
          //IMPOSTO PA0 COME INPUT (NON SERVIREBBE PERCHè DI DEFAULT è GIà SETTATO COME INPUT)
          GPIOA->MODER&=~(11<<0);
          
          //CONFIGURO PA0 COME INTERRUPT ESTERNO. STO SELEZIONANDO PA0 COME INGRESSO DEL MULTIPLEXER
          SYSCFG->EXTICR[0]&=~(111<<0);
         
          //SMASCHERO LA LINEA 0 SETTANDOLA AD 1
          EXTI->IMR|=(1<<0);
         
          //GENERO INTERRUPT SUL FRONTE DI SALITA TR0
          EXTI->RTSR|=(1<<0);
         
          //CANCELLO LA CAUSA DI INTERRUZIONE
          EXTI->PR|=(1<<0);
         
          //ABILITO DA NVIC LA POSIZIONE DI EXTI LINE 0 INTERRUPT
          NVIC->ISER[0]|=(1<<6);
         
          while(1);
}
 
void EXTI0_IRQHandler()
{
     //RICANCELLO LA CAUSA DI INTERRUZIONE
      EXTI->PR|=(1<<0);
     
      //se flag==1 -> accendo i led
      if(flag == SET){
        GPIOE->ODR|=GPIOE_ALL_LED_ON;
        flag = RESET;
      }  
      else //se flag==0 -> spengo i led
      {
        GPIOE->ODR=0;
        flag=SET;
      }
}
