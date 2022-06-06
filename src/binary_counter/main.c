/* 
*
*       Created on : June 01, 2022 
*           Author : massiAv
*
*/

#include <stdio.h>
#include "stm32f3x_lib.h"
#include "stm32f3x_api_driver.h"

int led_on=0;
int _bit_pos_GPIO_MODE = SET;

void main(){
  
	// ENABLE GPIOE AND GPIOA
        RCC_PCLK_AHBEN(RCC_AHBENR_GPIOA,ENABLE);
        RCC_PCLK_AHBEN(RCC_AHBENR_GPIOE,ENABLE);
	
	//SET PEx (WITH x = 8,9,..15) IN OUTPUT_MODE
        SET_PE_IN_OUT_MODE();
        
        /*!< SET PA0 IN INPUT MODE (BY DEFAULT PA0 IS ALREADY IN INPUT MODE)> */
        _bit_pos_GPIO_MODE = bit_pos_GPIO_MODER(Px0);
        GPIOA -> MODER &=~ (IN_MODE << _bit_pos_GPIO_MODE);
	 
	while(1)
	{
	  GPIOA_IDR_IN0(); //WAIT UNTIL PA0 IS NOT PRESSED
	 
	  GPIOA_IDR_IN1(); //PA0 PRESSED. GO TO NEXT STATEMENT ONLY IF PA0 IS RELEASED
	 
          GPIOE_TURN_LED(++led_on);
	}

}
