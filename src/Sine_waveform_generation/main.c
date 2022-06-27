/*
*
*       Created on : June 27, 2022 
*      Last Update : June 27, 2022
*           Author : massiAv
*               
*/

#define N_SAMPLES 200
#include <stdio.h>
#include <string.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_api_driver.h"
#include "stm32f3x_timer_driver.h"



short int LUT_in[N_SAMPLES];
short int LUT_out[N_SAMPLES];

void main()
{
        RCC_PCLK_AHBEN(RCC_AHBENR_GPIOA,ENABLE);                /*!< GPIOA ENABLE       >*/
        RCC_PCLK_AHBEN(RCC_AHBENR_ADC12,ENABLE);                /*!< ADC12 ENABLE       >*/
        
        RCC_PCLK_APB1EN(RCC_APB1ENR_DAC,ENABLE);                /*!< DAC ENABLE         >*/
        RCC_PCLK_APB1EN(RCC_APB1ENR_TIM2,ENABLE);               /*!< TIM2 ENABLE        >*/
        
        GPIO_MODE(GPIOA,ANALOG_MODE,Px2);                       /*!< PA2 AND PA4 IN ANALOG MODE >*/
        GPIO_MODE(GPIOA,ANALOG_MODE,Px4);
        
        ADC_Type* ADC = setup_ADC(GPIOA,Px2,SINGLE_MODE);       /*!< Setup ADC          >*/
        DAC_trigger_status(DAC1,ENABLE,DAC_TSEL_TIM2_EVENT);    
        
        CNT_EN_TIM(TIM2,ENABLE);
        TIM2->ARR=N_CONT_72Mhz(0.5);                     
        TIM2->CR2|=MMS_UPDATE;                                  /*!< MMS=010->The update event is selected as trigger output (TRGO) > */
        
        sine_waveform(LUT_in,N_SAMPLES);
        
        for(int i=0;i<N_SAMPLES;i++)
        {
          DAC1->DHR12R1=LUT_in[i];
          while((TIM2->SR & (1<<0)) != (1<<0));  //ATTENDO CHE UIF PASSI A 1
  TIM2->SR&=~(1<<0);                    //AZZERO IL UIF-> UIF=0;
  for(int j=0;j<1000;j++);              //attendo la generazione della tensione
        
        
        //adstart=1
	  ADC->CR|=(1<<2);
	  while((ADC->ISR & (1<<2)) != (1<<2)); // ATTENDO LA FINE DELLA CONVERSIONE
	 
	  //MEMORIZZO L'USCITA DELL'ADC IN UNA SECONDA LUT
	  LUT_out[i]=ADC->DR;
	  }
	  while(1);

}
