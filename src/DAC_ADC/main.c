/*
*
*       Created on : June 11, 2022 
*      Last Update : June 11, 2022
*           Author : massiAv
*
*/

#include <stdio.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_api_driver.h"
#include "stm32f3x_timer_driver.h"
#include <math.h>

/*!< DAC Variable       >*/
unsigned int code_in_DAC = 4095;
float voltage_out;
	 
/*!< ADC Variable       >*/
float voltage_in;
int code_out;


void main()
{
        RCC_PCLK_AHBEN(RCC_AHBENR_GPIOA,ENABLE);                /*!< Enable GPIOA       >*/
        RCC_PCLK_AHBEN(RCC_AHBENR_ADC12,ENABLE);                /*!< Enable ADC12       >*/
        RCC_PCLK_APB1EN(RCC_APB1ENR_DAC,ENABLE);                /*!< Enable DAC         >*/
        
        GPIO_MODE(GPIOA,ANALOG_MODE,Px4);                       /*!< PA4 as analog input -> DAC1_OUT1   >*/
        GPIO_MODE(GPIOA,ANALOG_MODE,Px5);                       /*!< PA5 as analog input -> ADC2_IN2    >*/
        
        setup_ADC(GPIOA,Px5);
        setup_DAC(DAC1,code_in_DAC);
        
        ADC_Type* ADC = get_number_ADC(GPIOA,Px5);              /*!< Store into ADC variable the exactly ADC chosen. This function
                                                                     This function discriminates which ADCx is chosen based on GPIOx and pin number >*/
        
        ADC->CR |= ADC_CR_ADSTART;                             /*!< Start CONVERSIONE pull up bit ADSTART >*/
        while((ADC->ISR & ADC_ISR_EOC) != ADC_ISR_EOC );       /*!< Wait that EOC change to 1, when EOC=1 can read the result in ADC->DR*/
        
          
	voltage_out=(DAC1->DHR12R1)*(VDD_USB/(pow(2,12) - 1));                                  /*!< DAC output voltage reading >*/
	printf("DAC\ninput: %d\noutput: %.4f V\n",DAC1->DHR12R1,voltage_out);
	 
	code_out=ADC->DR;                                       
	voltage_in=code_out*(VDD_USB/(get_quantization_level(ADC,ADC_CFG_RES_12bit) - 1));      /*!< ADC output code reading    >*/
	printf("ADC\ninput: %.4f V\noutput: %d \n",voltage_in,code_out);

        ADC_DISABLE(ADC);
        DAC_DISABLE(DAC1);
        
        while(1);

}
