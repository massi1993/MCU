/*
*
*       Created on : June 10, 2022 
*      Last Update : June 13, 2022
*           Author : massiAv
*
*/

#include <stdio.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_api_driver.h"
#include "stm32f3x_timer_driver.h"

float voltage = RESET;


void main()
{
          RCC_PCLK_AHBEN(RCC_AHBENR_ADC12,ENABLE);                      /*!< ENABLE ADC12 BECAUSE PA0 (ANALOG_MODE) IS LINKED TO ADC12      >*/
          RCC_PCLK_AHBEN(RCC_AHBENR_GPIOA,ENABLE);                      /*!< ENABLE PORT A      >*/        
          RCC_PCLK_AHBEN(RCC_AHBENR_GPIOE,ENABLE);                      /*!< ENABLE PORT E      >*/
          
          GPIO_MODE(GPIOA,ANALOG_MODE,Px0);                             /*!< ENABLE PA0 in analog mode      >*/
          
          GPIOE_OUTMODE(Px8,Px15);
         
          ADC_Type* ADC = setup_ADC(GPIOA,Px0,CONTINUOUS_MODE);

          ADC->CR |= ADC_CR_ADSTART;                                   /*!< Start CONVERSION pull up bit ADSTART >*/
         
          
          while(1)
          {
              while((ADC->ISR & (ADC_ISR_EOC))!= (ADC_ISR_EOC));       /*!< Wait that EOC change to 1, when EOC=1 can read the result in ADC->DR*/
             
              voltage = (ADC->DR) * (VDD_USB/(get_quantization_level(ADC,ADC_CFG_RES_12bit) - 1));
             
              if(voltage<=3 && voltage>=2.998)
              {
                GPIOE->ODR = GPIOE_ALL_LED_ON;                          /*!< Led ON>*/
                printf("Value of voltage %.3f\n",voltage);
                voltage = RESET;
              }
              else
              {
                GPIOE->BRR = GPIOE_ALL_LED_ON;                          /*!< Using Bit Reset Register to Led OFF>*/
              }
              
           }
}


