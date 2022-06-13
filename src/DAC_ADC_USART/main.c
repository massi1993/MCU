/*
*
*       Created on : June 11, 2022 
*      Last Update : June 13, 2022
*           Author : massiAv
*               
*
Note: REMEMBER to link PA4 and PA2 via JUMPER
*/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_api_driver.h"
#include "stm32f3x_timer_driver.h"


const char txt1 [60]= {"\nType A to insert code 4095 to the DAC\r\n"};
const char txt2 [60]= {"Type B to insert code 3000 to the DAC\r\n"};
const char txt3 [60]= {"Type C to insert code 2500 to the DAC\r\n"};
const char txt4 [60]= {"Type D to insert code 1750 to the DAC\r\n\n"};
const char txt5 [60]= {"\nYou typed an incorrect value, RETRY! \r\n\n"};
const char txt6 [60]= {"\nOk, correct value\r\n\n"};


char datoRx;                    /*!< variable in which the data read by usart is stored>*/

/*!< DAC's variable >*/
int code_in_dac;                /*!< code between [0: 4095] to be given in input to the DAC >*/
float voltage_out;              /*!< DAC's output voltage                                   >*/    
 
/*!< ADC's variable >*/
float voltage_in;               /*!< ADC's input voltage                                   >*/ 
int code_out;                   /*!< ADC's output code                                     >*/     
 
char text_result[100]; 
void main(){
          
          RCC_PCLK_AHBEN(RCC_AHBENR_GPIOA,ENABLE);              /*!< Enable GPIOA       >*/
          RCC_PCLK_AHBEN(RCC_AHBENR_GPIOC,ENABLE);              /*!< Enable GPIOC       >*/        
          RCC_PCLK_AHBEN(RCC_AHBENR_ADC12,ENABLE);              /*!< Enable ADC12       >*/        
          
          RCC_PCLK_APB1EN(RCC_APB1ENR_DAC,ENABLE);              /*!< Enable DAC         >*/
          
          RCC_PCLK_APB2EN(RCC_APB2ENR_USART1,ENABLE);           /*!< Enable USART1       >*/
 
          GPIO_MODE(GPIOA,ANALOG_MODE,Px4);                     /*!< Set PA2 and PA4 in Analog_mode     >*/
          GPIO_MODE(GPIOA,ANALOG_MODE,Px2);
 
          GPIO_MODE(GPIOC,AF_MODE,Px4);                         /*!< Set PC4 (TX pins) in AF_mode       >*/
          GPIO_AFR(GPIOC,AF7,Px4);                              /*!< Specifies the type of AF: USART1Tx >*/
          
          
          GPIO_MODE(GPIOC,AF_MODE,Px5);                         /*!< Set PC5 (RX pins) in AF_mode       >*/
          GPIO_AFR(GPIOC,AF7,Px5);                              /*!< Specifies the type of AF: USART1Rx >*/

          ADC_Type* ADC = setup_ADC(GPIOA,Px2,SINGLE_MODE);     /*!< Setup ADC          >*/
          setup_USART_RX_TX(USART1);                            /*!< Setup USART        >*/
          
          usart_tx(USART1,txt1,strlen(txt1));                   /*!< String to Transmitt       >*/
          usart_tx(USART1,txt2,strlen(txt2));                   /*!< String to Transmitt       >*/        
          usart_tx(USART1,txt3,strlen(txt3));                   /*!< String to Transmitt       >*/
          usart_tx(USART1,txt4,strlen(txt4));                   /*!< String to Transmitt       >*/
          
          while(1){
                        
              while(!(USART1->ISR & USART_ISR_RXNE));           /*!< The data is expected to be ready to be read        >*/
              datoRx = usart_rx(USART1);  
              
              if(datoRx == 'A')
              {
                usart_tx(USART1,txt6,strlen(txt6));
                code_in_dac = 4095;
              }
              else if(datoRx == 'B')
              { 
                 usart_tx(USART1,txt6,strlen(txt6));
                 code_in_dac = 3000;
              }
              else if(datoRx == 'C')
              {
                  usart_tx(USART1,txt6,strlen(txt6)); 
                  code_in_dac = 2500;
              }
              else if(datoRx == 'D')
              {
                  usart_tx(USART1,txt6,strlen(txt6));
                  code_in_dac = 1750;
              }
              else
              {
                  usart_tx(USART1,txt5,strlen(txt5));
                  continue;  
              }
              
              setup_DAC(DAC1,code_in_dac);                              /*!< Only now call the setup_DAC() function. First at all, it was necessary to set the code_in_dac variable >*/         
                 
              voltage_out=(DAC1->DHR12R1)*(VDD_USB/(pow(2,12)-1.0));    /*!< DAC output voltage reading >*/
              //printf("DAC\ninput: %d\noutput: %f V\n",DAC1->DHR12R1,voltage_out);
             
              ADC->CR|=ADC_CR_ADSTART;                                  /*!< Start CONVERSIONE pull up bit ADSTART >*/
              while( (ADC->ISR & ADC_ISR_EOC) != ADC_ISR_EOC);          /*!< Wait that EOC change to 1, when EOC=1 can read the result in ADC->DR*/
          
              code_out=ADC->DR;                                         /*!< ADC output code reading    >*/
              voltage_in=code_out*(VDD_USB/(get_quantization_level(ADC,ADC_CFG_RES_12bit) - 1));
              //printf("ADC\ninput: %f V\noutput: %d \n\n",voltage_in,code_out);
              
              sprintf(text_result,"DAC's Voltage output : %f\r\nADC's Voltage input : %f\r\n",voltage_out,voltage_in);          /*!< Build result into string to transmitt   >*/
              
              usart_tx(USART1,text_result,strlen(text_result));         /*!< Final String to Transmitt       >*/
              
              code_out = RESET;
              code_in_dac = RESET;
          
              DAC_DISABLE(DAC1);
        
          }
}
 