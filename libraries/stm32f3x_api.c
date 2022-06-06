/* STM32_F3X_API
*
*       Created on : June 04, 2022 
*      Last Update : June 06, 2022       
*           Author : massiAv
*
*/

#include "stm32f3x_lib.h"
#include "stm32f3x_api_driver.h"
#include "stm32f3x_timer_driver.h"

/*!-----------------------------------------------------------------------------

                      API FOR ENABLE CLOCK PERIPHERAL RCC 

-------------------------------------------------------------------------------->*/
                        
/*!<
@brief
Define if enable or disable the RCC_AHBENR
@param RCC_AHBENR_Periph  specifies the AHB peripheral to gates its clock.
    Can be:
    @arg     RCC_AHBENR_GPIOA   
    @arg     RCC_AHBENR_GPIOB
    @arg     RCC_AHBENR_GPIOC
    @arg     RCC_AHBENR_GPIOD
    @arg     RCC_AHBENR_GPIOE
    @arg     RCC_AHBENR_GPIOF
        
@param status state of specified peripheral clock.
    Can be: ENABLE or DISABLE

@return None

*/

void RCC_PCLK_AHBEN(uint32_t RCC_AHBENR_Periph, int status){

    if(status == ENABLE)
    {
      RCC->AHBENR |= RCC_AHBENR_Periph;
    }
    else
    {
      RCC->AHBENR &=~ RCC_AHBENR_Periph;
    }

}


/*!<
@brief
Define if enable or disable the RCC_APB1ENR
@param RCC_PCLK_APB1EN  specifies the APB1 peripheral to gates its clock.
    Can be:
    @arg     RCC_APB1TIM2   
    @arg     RCC_APB1TIM3
        
@param status state of specified peripheral clock.
    Can be: ENABLE or DISABLE

@return None

*/

void RCC_PCLK_APB1EN(uint32_t RCC_APB1ENR_Periph, int status){

    if(status == ENABLE)
    {
      RCC->APB1ENR |= RCC_APB1ENR_Periph;
    }
    else
    {
      RCC->APB1ENR &=~ RCC_APB1ENR_Periph;
    }

}

/*!-----------------------------------------------------------------------------

                      API FOR General Purpose Input Output 

-------------------------------------------------------------------------------->*/

/*!<
@brief
Allows to enable the mode of GPIO

@param GPIO_Type* GPIO General Purpose I/O Port to enable
@param mode      type of mode of port: Input_mode, Output_mode, Analog_mode, AF_mode
@param PinNumber pin number of port chosen from MACROS define PxY, e.g Px0, Px1, etc..
                 If we choose PA0, PinNumber will be Px0;
                 If we choose PB1, PinNumber will be Px1;
                 If we choose PB2, PinNumber will be Px2;
                 and so on...

@return value of bit position

*/
void GPIO_MODE (GPIO_Type* GPIO, unsigned int mode, unsigned int PinNumber){
  
    if(mode == IN_MODE)
    {
      GPIO->MODER &=~ (mode << bit_pos_GPIO_MODER(PinNumber));
    }
    else
    {
      GPIO->MODER |= (mode << bit_pos_GPIO_MODER(PinNumber));
    }
}



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

                      API FOR TIMER

-------------------------------------------------------------------------------->*/

/*!<
@brief
Get Prescaler and AutoReloadRegister with time and Fck.

@param TIMER_Type* Timer TIMER OF INTEREST
@param unsigned int time
@param usigned int Fck

@return None


*/

void set_PSC_and_ARR_TIM(TIMER_Type* Timer, unsigned int time, unsigned int Fck){
          
    unsigned int PSC, NARR;
    PSC = (unsigned int)(time*Fck/65535);
    Timer ->PSC = PSC ;
    NARR = (unsigned int)(time*Fck)/(TIM3->PSC +1);
    Timer->ARR = NARR;
}

/*!<
@brief
Enable counter

@param TIMER_Type* Timer TIMER OF INTEREST
@param status state of specified timer.
    Can be: ENABLE or DISABLE

@return None


*/

void CNT_EN_TIM(TIMER_Type* Timer, unsigned int status){
          
    Timer->CNT = RESET;
    if(status == ENABLE)
    {
      Timer->CR1 |= CEN_EN;
    }
    else
    {
      Timer->CR1 &=~ CEN_EN;
    }
}


/*!-----------------------------------------------------------------------------

                      API FOR EXTernal Interrupt AND INTERNAL INTERRUPT

-------------------------------------------------------------------------------->*/
/*!<
@brief
CONFIGURE PortNumber AS EXTERNAL INTERRUPT. WE HAVE TO SELECT PortNumver AS MULTIPLEXER'S INPUT 

@param PinNumber only pin number that generate the external interrupt. From it, we get the index of EXTI and bit_position.
                 PinNumber pin number chosen from MACROS define PxY, e.g Px0, Px1, etc..
                                 If we choose PA0, PinNumber will be Px0;
                                 If we choose PB1, PinNumber will be Px1;
                                 If we choose PB2, PinNumber will be Px2;
                                 and so on...

@param PortNumber Port and pin number of GPIO Port that generate the external interrupt

@return None

*/

void set_SYSCFG_EXTI(int PinNumber, int PortNumber){

    unsigned int _index_EXTI = RESET;                   /*!< Index of SYSCFG -> EXTI >*/ 
    unsigned int _bit_pos_EXTI = RESET;                 /*!< Bit position in which write the data >*/  
    
    _index_EXTI = index_EXTI(PinNumber);
    _bit_pos_EXTI = bit_pos_EXTI(PinNumber);
    
    if(PortNumber == PA0)
    {
      SYSCFG->EXTICR[_index_EXTI] &=~ (PortNumber << _bit_pos_EXTI);
    }
    else
    {
      SYSCFG->EXTICR[_index_EXTI] |= (PortNumber << _bit_pos_EXTI);
    }
}



/*!<
@brief
Get the index of SYS_CFG -> EXTI[index].

@param PinNumber that generate the external interrupt
                 PinNumber pin number chosen from MACROS define PxY, e.g Px0, Px1, etc..
                          If we choose PA0, PinNumber will be Px0;
                          If we choose PB1, PinNumber will be Px1;
                          If we choose PB2, PinNumber will be Px2;
                          and so on...

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

/*!<
@brief
Configure the index of NVIC_ISER and unmasking of EXTIx_Line global interrupt>

@param IRQ      number of interrupt request:
                - From 0 to 31 the interrupts are set in the ISER[0] register;
                - from 32 to 63 the interrupts are set in the ISER[1] register;
                - from 64 to 81 the interrupts are set in the ISER[2] register

@return None

*/

void set_NVIC_ISER(int IRQ){
  
    unsigned int _index_NVIC_ISER = SET;                /*!< Index of NVIC -> ISER >*/ 
    
    _index_NVIC_ISER = index_NVIC_ISER(IRQ);
    NVIC->ISER[_index_NVIC_ISER] |= (1<<IRQ);
        
}


/*!<
@brief
Get the index of NVIC -> ISER[index].

@param IRQ      number of interrupt request:
                - From 0 to 31 the interrupts are set in the ISER[0] register;
                - from 32 to 63 the interrupts are set in the ISER[1] register;
                - from 64 to 81 the interrupts are set in the ISER[2] register

@return value of NVIC_ISER index. It can be ISER[0], ISER[1] and ISER[2]

*/

int index_NVIC_ISER(int IRQ){
    
    int index;
    
    index = IRQ/32;

    return index;
}