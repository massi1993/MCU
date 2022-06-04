#include "stm32f3x_gpio_driver.h"

/*!<
@brief
Get the index of SYS_CFG -> EXTI[index].

@par_in PinNumber that generate the external interrupt

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

@par_in PinNumber that generate the external interrupt (NOTE: use the MACROS define PxY, e.g Px0,Px1,etc..)

@return value of bit position

*/

int bit_pos_EXTI(int PinNumber){

    int bit_pos;
    
    bit_pos = (PinNumber % 4) * 4;
    
    return bit_pos;
}