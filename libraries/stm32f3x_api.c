/* STM32_F3X_API
*
*       Created on : June 04, 2022 
*           Author : massiAv
*
*/

#include "stm32f3x_api_driver.h"


/*!-----------------------------------------------------------------------------

                      API FOR General Purpose Input Output 

-------------------------------------------------------------------------------->*/

/*!<
@brief
GPIO->MODER ALLOWS TO CONFIGURE PORTS IN INPUT, OUTPUT, ANALOG  AND ALTERNATE FUNCTION MODE,
WRITING CORRECTLY THE CHOSEN MODE IN THE TWO BITS OF INTEREST.
SO, THIS FUNCTION ALLOWS TO FIND THE BIT POSITION IN WHICH TO WRITE.
For this, we first get the module reported to 2 (PinNumber % 2) and then we multiply by 2.

@param PinNumber pin number chosen from MACROS define PxY, e.g Px0, Px1, etc..
                 If we choose PA0, PinNumber will be Px0;
                 If we choose PB1, PinNumber will be Px1;
                 If we choose PB2, PinNumber will be Px2;
                 and so on...

@return value of bit position

*/

int bit_pos_GPIO_MODER(int PinNumber){

    int bit_pos;
    
    bit_pos = (PinNumber % 2) * 2;
    
    return bit_pos;
}

/*!-----------------------------------------------------------------------------

                      API FOR EXTernal Interrupt 

-------------------------------------------------------------------------------->*/
/*!<
@brief
Get the index of SYS_CFG -> EXTI[index].

@param PinNumber that generate the external interrupt

@return value of EXTI index. It can be EXTI[0], EXTI[1],EXTI[2] or EXTI[3]

*/

int index_EXTI(int PinNumber){
    
    int index;
    
    index = PinNumber/4;

    return index;
}


/*!<
@brief
Each SYS_CFG_EXTICRx is divided into 4 EXTI, composed in its time of 4 bit. 
For this, we first get the module reported to 4 (PinNumber % 4) and then we multiply by 4.

@param PinNumber that generate the external interrupt (NOTE: use the MACROS define PxY, e.g Px0,Px1,etc..)

@return value of bit position

*/

int bit_pos_EXTI(int PinNumber){

    int bit_pos;
    
    bit_pos = (PinNumber % 4) * 4;
    
    return bit_pos;
}