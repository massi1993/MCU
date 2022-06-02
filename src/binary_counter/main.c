#include <stdio.h>
#include "stm32f3x_lib.h"

int contatore=0;

void main(){
  
	// ENABLE GPIOE AND GPIOA
	RCC->AHBENR|=GPIOE_EN|GPIOA_EN;
	 
	//SET PEx (WITH x = 8,9,..15) IN OUTPUT_MODE
        for(int i = 8; i < 16; i++)
        {
          GPIOE->MODER|= (OUT_MODE << 2*i);
        }

        //SET PA0 IN INPUT MODE (BY DEFAULT PA0 IS ALREADY IN INPUT MODE)
        GPIOA -> MODER &=~ (IN_MODE << 0);
	 
	while(1)
	{
	  while(((GPIOA->IDR) & (1)) == 0); //WAIT UNTIL PA0 IS NOT PRESSED
	 
	  while(((GPIOA->IDR) & (1)) == 1); //PA0 PRESSED. GO TO NEXT STATEMENT ONLY IF PA0 IS RELEASED
	  contatore++;
	  GPIOE->ODR=contatore<<8;
	}

}//end main 
