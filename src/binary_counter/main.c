/* 
*
*       Created on : June 01, 2022 
*       Last Update: June 04, 2022
*           Author : massiAv
*
*/

#include <stdio.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_api_driver.h"

int led_on=0;


void main(){

        RCC_PCLK_AHBEN(RCC_AHBENR_GPIOA,ENABLE);        /*!< ENABLE GPIOE AND GPIOA >*/ 
        RCC_PCLK_AHBEN(RCC_AHBENR_GPIOE,ENABLE);        
	
	SET_PE_IN_OUT_MODE();                           /*!< SET PEx (WITH x = 8,9,..15) IN OUTPUT_MODE >*/
        
        GPIO_MODE(GPIOA,IN_MODE,Px0);                   /*!< SET PA0 IN INPUT MODE (BY DEFAULT PA0 IS ALREADY IN INPUT MODE)> */
        
	while(1)
	{
	  PA0_IDR(0); //WAIT UNTIL PA0 IS NOT PRESSED
	 
	  PA0_IDR(1); //PA0 PRESSED. GO TO NEXT STATEMENT ONLY IF PA0 IS RELEASED
	 
          GPIOE_TURN_LED(++led_on);
	}

}
