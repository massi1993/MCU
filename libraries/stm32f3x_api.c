/* STM32_F3X_API
*
*       Created on : June 04, 2022 
*      Last Update : June 11, 2022       
*           Author : massiAv
*
*/

#include <math.h>
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

@return None

*/
void GPIO_MODE (GPIO_Type* GPIO, unsigned int mode, unsigned int PinNumber){
  
    GPIO->MODER |= (mode << bit_pos_GPIO_MODER(PinNumber));

}

/*!<
@brief
Define GPIO Bit Set Reset Register

@param pinEn  Pin to control
@param status It can be Set or Reset

@return None

*/
void GPIO_BSR_REG(GPIO_Type* GPIO, int pinEn, int status){

    if(status == SET)
    {
        GPIO->BSR = (1 << pinEn);
    }
    else if(status == RESET)
    {
        GPIO->BRR = (1 << pinEn);
    }
    
}

/*!<
@brief
Set in OUT_MODE LEds from PEStart to PEStop

@param PEStart  LED on Start
@param PEStop   LED on Stop

@return None

*/
void GPIOE_OUTMODE(int PEstart, int PEstop){

    if(PEstart != Px8 && PEstop != Px15)
    {
        for(int i=PEstart; i<=PEstop; i++)
        {
            GPIOE->MODER |= (OUT_MODE << 2*i);
        
        }
    }
    else
    {
        GPIOE->MODER|=0x55550000;
    }
}




/*!<
@brief
GPIO->MODER ALLOWS TO CONFIGURE PORTS IN INPUT, OUTPUT, ANALOG  AND ALTERNATE FUNCTION MODE,
WRITING CORRECTLY THE CHOSEN MODE IN THE TWO BITS OF INTEREST.
SO, THIS FUNCTION ALLOWS TO FIND THE BIT POSITION IN WHICH TO WRITE.
For this, we first get the module reported to 16 (PinNumber % 16) and then we multiply by 2.

@param PinNumber pin number chosen from MACROS define PxY, e.g Px0, Px1, etc..
                 If we choose PA0, PinNumber will be Px0;
                 If we choose PB1, PinNumber will be Px1;
                 If we choose PB2, PinNumber will be Px2;
                 and so on...

@return value of bit position

*/

int bit_pos_GPIO_MODER(int PinNumber){

    int bit_pos;
    
    bit_pos = (PinNumber % 16) * 2;
    
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
          
    if(status == ENABLE)
    {
      Timer->CNT = RESET;
      Timer->CR1 |= CEN_EN;
    }
    else
    {
      Timer->CR1 &=~ CEN_EN;
    }
}

/*!<
@brief  Start time measurement when the PA0 is pressed; Stop time when PA0 is pressed again if the mode is STOPWATCH.
        Otherwise, if the mode is NOT_STOP_WATCH, it measure the time betwen two pressing of PA0.
        Basically, we are implementing a stopwatch or not_stop_watch.

@param TIMER_Type * Timer Timer to use as stopwatch

@param unsigned int mode it can be STOP_WATCH or NOT_STOP_WATCH

@return time    measurement time

>*/
float Measure_Time(TIMER_Type* Timer, unsigned int mode){
  
    float time = RESET;                                         /*!< variable to store time measurement >*/
    
    if(mode == STOP_WATCH)
    {
        PA0_IDR_PRESSED(0);                                     /*!< WAIT UNTIL PA0 IS PRESSED  TO START TIME MEASUREMENT >*/
        CNT_EN_TIM(Timer,ENABLE);                               /*!< ENABLE COUNT TIM2          >*/
      
        PA0_IDR_PRESSED(1);                                     /*!< WAIT PA0 IS RELEASE        >*/
      
        PA0_IDR_PRESSED(0);                                     /*!< WAIT UNTIL PA0 IS PRESSED AGAIN TO STOP THE MEASUREMENT >*/
        CNT_EN_TIM(Timer,DISABLE);                              /*!< DISABLE COUNT TIM2          >*/
        time = (float) (Timer->CNT)/F_CLK;                      /*!< MEASUREMENT TIME           >*/
    }
    
    else if(mode == NOT_STOP_WATCH)
    {
      /*!<Check if PA0 is PRESSED>*/
      if(((GPIOA->IDR) & (1<<0))== (1<<0))
      {
            CNT_EN_TIM(Timer,ENABLE);                           /*!< ENABLE COUNT TIM2          >*/
            PA0_IDR_PRESSED(1);                                 /*!< WAIT PA0 IS RELEASE        >*/
            CNT_EN_TIM(Timer,DISABLE);                          /*!< DISABLE COUNT TIM2          >*/
            time = (float) (Timer->CNT)/F_CLK;                  /*!< MEASUREMENT TIME           >*/
        }
      
    }
    
    return time;
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

/*!-----------------------------------------------------------------------------

                                API FOR ADC

-------------------------------------------------------------------------------->*/
/*!<
@brief
Before performing any operation such as launching a calibration or enabling the ADC, the ADC
voltage regulator must first be enabled and the software must wait for the regulator start-up time.
The default state is '10': to enable the regulator we have to change the default state with intermediate state '00' and then enable ADC regulator writing '01'

@param ADC_Type* ADC 

@return None

*/
void  ADC_Voltage_Regulator_EN(ADC_Type* ADC){
  
    ADC->CR &=~ (1<<29);                /*!< ADVREGEN from '10' to '00' >*/
    
    ADC->CR |= ADC_CR_REG_EN;           /*!< ADVREGEN from '00' to '01' >*/
    
    for(int i=0;i<1000;i++);            /*!< WAIT 10 us                 >*/
}

/*!<
@brief
Set ADC_CFGR_RES. Setting of data resolution of adc and get quantization levels. 
For example:
  res = 12-bit
  res = 10-bit
  res = 8-bit
  res = 6-bit

  quantization level is:
  QL = 2^res

@param unsigned int resolution  
        One between:
        - ADC_CFG_RES_12bit  
        - ADC_CFG_RES_10bit 
        - ADC_CFG_RES_8bit  
        - ADC_CFG_RES_6bit  

@param ADC_Type* ADC

@return float quantization level (for example 4096.0)

*/
float get_quantization_level(ADC_Type* ADC, unsigned int ADC_res){
    
    float QL = RESET;
    int ADC_res_bit = RESET;
    
    ADC ->CFGR |= ADC_res;
    
    if((ADC ->CFGR & (0x3<<3))== ADC_CFG_RES_12bit)
    {
        ADC_res_bit = 12;
    }
    else if((ADC ->CFGR & (0x3<<3))== ADC_CFG_RES_10bit)
    {
        ADC_res_bit = 10;
    }
    else if((ADC ->CFGR & (0x3<<3)) == ADC_CFG_RES_8bit)
    {
        ADC_res_bit = 8;
    }
    else if((ADC ->CFGR & (0x3<<3)) == ADC_CFG_RES_6bit)
    {   
        ADC_res_bit = 6;
    }
        
    QL = pow(2,ADC_res_bit);
    
    return QL;
}


/*!<

@brief
Set configuration for ADC 

@param  ADC_Type* ADC, ADC_Common_Type* ADC_CC

@return None

*/
void config_ADC(ADC_Type* ADC, ADC_Common_Type* ADC_CC){
    
    ADC_Voltage_Regulator_EN(ADC);
     
    ADC_CC->CCR |= ADC_CC_CCR_SYNC_CKMODE1;                     /*!< For detail see the declaration of ADC_CC_CCR_SYNC_CKMODE1  >*/
    
    ADC->CR |= ADC_CR_ADCAL;                                    /*!< Calibration of ADC >*/
    while((ADC->CR  & ADC_CR_ADCAL)== ADC_CR_ADCAL);            /*!< Waiting for ADCAL change to 0. If ADCAL = 0 so the calibration is complete >*/
     
    ADC->CR |= ADC_CR_ADEN;                                     /*!< ADC Enable>*/
    while((ADC->ISR & ADC_ISR_ADRDY)!= ADC_ISR_ADRDY);          /*!< Wait for ADRDY change to 1 and be ready to conversion      >*/
     
    ADC->CFGR |= ADC_CFG_CONT;                                  /*!< Continuous conversion mode         >*/
    ADC->SQR1 |= (1<<6);                                        /*!< SQR1=00001 -> CHANNEL 1 (PA0)      >*/
    ADC->SQR1 &=~ (0x0000000F);                                 /*!< L=0000 -> 1 Conversion             >*/
    ADC->SMPR1 |= ADC_SMP1_601_5CKC;                            /*!< Choose 601.5 CLOCK CYCLES          >*/
} 

