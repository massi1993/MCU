/* STM32_F3X_API
*
*       Created on : June 04, 2022 
*      Last Update : June 27, 2022       
*           Author : massiAv
*
*/
#include <stdio.h>
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

/*!<

@brief
Define if enable or disable the RCC_APB2ENR

@param RCC_PCLK_APB2EN  specifies the APB2 peripheral to gates its clock.
    Can be:
    @arg   RCC_APB2ENR_SYSCFG     
    @arg   RCC_APB2ENR_TIM1  
           RCC_APB2ENR_USART1
           RCC_APB2ENR_TIM17 
        etc..
@param status state of specified peripheral clock.
    Can be: ENABLE or DISABLE

@return None

*/

void RCC_PCLK_APB2EN(uint32_t RCC_APB2ENR_Periph, int status){

    if(status == ENABLE)
    {
      RCC->APB2ENR |= RCC_APB2ENR_Periph;
    }
    else
    {
      RCC->APB2ENR &=~ RCC_APB2ENR_Periph;
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

/*!<
@brief
Set register AFR with the correct value

@param GPIO_Type* GPIO General Purpose I/O Port to enable

@param mode      type of AFR : AF0, AF1, AF2 ETC..

@param PinNumber pin number of port chosen from MACROS define PxY, e.g Px0, Px1, etc..
                 If we choose PA0, PinNumber will be Px0;
                 If we choose PB1, PinNumber will be Px1;
                 If we choose PB2, PinNumber will be Px2;
                 and so on...

@return None

*/
void GPIO_AFR(GPIO_Type* GPIO, unsigned int AF_Type, unsigned int PinNumber){
  
    GPIO->AFR[index_AFR(PinNumber)] |= (AF_Type << bit_pos_GPIO_AFR(PinNumber));

}


/*!<

@brief
Get the index of GPIO -> AFR[index].

@param PinNumber that generate the ALTERNATE FUNCTION
                 PinNumber pin number chosen from MACROS define PxY, e.g Px0, Px1, etc..
                          If we choose PA0, PinNumber will be Px0;
                          If we choose PB1, PinNumber will be Px1;
                          If we choose PB2, PinNumber will be Px2;
                          and so on...

@return value of AFR index. It can be AFR[0] OR AFR[1]

*/

int index_AFR(int PinNumber){

        int index;

        index = PinNumber/8;

        return index;
}

/*!<

@brief
Set the correct position to write the bit.
For this, we first get the module reported to 8 (PinNumber % 8) and then we multiply by 2.

@param PinNumber pin number chosen from MACROS define PxY, e.g Px0, Px1, etc..
                 If we choose PA0, PinNumber will be Px0;
                 If we choose PB1, PinNumber will be Px1;
                 If we choose PB2, PinNumber will be Px2;
                 and so on...

@return value of bit position

*/

int bit_pos_GPIO_AFR(int PinNumber){

    int bit_pos;
    
    bit_pos = (PinNumber % 8) * 4;
    
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

void set_PSC_and_ARR_TIM(TIMER_Type* Timer, float time, unsigned int Fck){
          
    unsigned int PSC, NARR;
    PSC = (unsigned int)(time*Fck/65535);
    Timer ->PSC = PSC ;
    NARR = (unsigned int)(time*Fck)/(Timer->PSC +1);
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

                                API FOR ADC and DAC

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
Set number channel of ADC via Pin number.
For example:
  PA0 -> ADC1_IN1  (CHANNEL1)
  PA1 -> ADC1_IN2  (CHANNEL2)
  PA2 -> ADC1_IN3  (CHANNEL3)
  etc...

@param GPIO_Type* GPIO to specificate the port

@param Pin number       to specificate the number of the port selected



@return 

*/
unsigned int get_Nchannel_ADC(GPIO_Type* GPIO, unsigned int PinNumber){
    
    unsigned int channel = RESET;
    
    if(GPIO == GPIOA)
    {
      if(PinNumber == Px0 || PinNumber == Px4)
      {
          channel = 0x01;
      }
      else if (PinNumber == Px1 || PinNumber == Px5)
      {
          channel = 0x02;
      }
      else if (PinNumber == Px2 || PinNumber == Px6)
      {
          channel = 0x03;
      }
      else if (PinNumber == Px3 || PinNumber == Px7)
      {
          channel = 0x04;
      }    
    }
    
    return channel;
}

/*!<
@brief
SET NUMBER ADC BY PORT AND PIN NUMBER.

For: From PA0 to PA3 -> ADC1
     From PA4 to PA7 -> ADC2
    ETC..

@param GPIO_Type* GPIO to specificate the port

@param Pin number       to specificate the number of the port selected


@return ADC_Type* ADCx

*/
ADC_Type* get_number_ADC(GPIO_Type* GPIO, unsigned int PinNumber){
  
    ADC_Type* ADC_number;
    
    if(GPIO == GPIOA)
    {
      if(PinNumber <= Px3)
      {
          ADC_number = ADC1;
      }
      else if (PinNumber >= Px4 && PinNumber <= Px7)
      {
          ADC_number = ADC2;
      }
      
    }
    
    return ADC_number;
}


/*!<

@brief
Set configuration for ADC 


@param  GPIO_Type* GPIO, unsigned int PinNumber
        specified PORT and pinnumber that enable ADC.
        For example, if I choose PA5 to use ADC2, then I'll use setup_ADC(GPIOA,Px5)

@return None

*/

ADC_Type* setup_ADC(GPIO_Type* GPIO, unsigned int PinNumber, uint8_t conversion_mode){
    
    unsigned int nChannel;
    ADC_Common_Type* ADC_CC;
    ADC_Type* ADC = get_number_ADC(GPIO,PinNumber);
    
    if(ADC == ADC1 || ADC == ADC2)
    {
        ADC_CC = ADC1_2;
    }
    else
    {
        ADC_CC = ADC3_4;
    }
    
    ADC_Voltage_Regulator_EN(ADC);
     
    ADC_CC->CCR |= ADC_CC_CCR_SYNC_CKMODE1;                     /*!< For detail see the declaration of ADC_CC_CCR_SYNC_CKMODE1  >*/
    
    ADC->CR |= ADC_CR_ADCAL;                                    /*!< Calibration of ADC >*/
    while((ADC->CR  & ADC_CR_ADCAL)== ADC_CR_ADCAL);            /*!< Waiting for ADCAL change to 0. If ADCAL = 0 so the calibration is complete >*/
     
    ADC->CR |= ADC_CR_ADEN;                                     /*!< ADC Enable>*/
    while((ADC->ISR & ADC_ISR_ADRDY)!= ADC_ISR_ADRDY);          /*!< Wait for ADRDY change to 1 and be ready to conversion      >*/
    
    if(conversion_mode == SINGLE_MODE)
    {
      ADC->CFGR &=~ ADC_CFG_CONT;                               /*!< Single conversion mode         >*/
    }
    else if (conversion_mode == CONTINUOUS_MODE)
    {
      ADC->CFGR |= ADC_CFG_CONT;                                /*!< Continuous conversion mode         >*/
    }
    
    nChannel = get_Nchannel_ADC(GPIO,PinNumber);
    ADC->SQR1 |= (nChannel<<6);                                 /*!< 1st conversion in regular sequence      >*/
    ADC->SQR1 &=~ (0x0000000F);                                 /*!< L=0000 -> 1 Conversion             >*/
    //ADC->SMPR1 |= ADC_SMP1_601_5CKC(nChannel);                  /*!< Choose 601.5 CLOCK CYCLES          >*/
    ADC->SMPR1 |= ADC_SMP1_61_5CKC(nChannel);                   /*!< Choose 61.5 CLOCK CYCLES FOR USART          >*/
    
    return ADC;

} 


/*!<

@brief
Setup for DAC

@param  DAC_Type* DAC, unsigned int code

@return None

*/
void setup_DAC(DAC_Type* DAC, unsigned int code){

    DAC->CR |= DAC_CR_EN1;
    if(code <= 4095)
      DAC->DHR12R1 = code;
    for(int j=0;j<1000;j++);                                    /*!< Wait voltage generation    >*/

}

/*!<

@brief
Enable Dac trigger 

@param  DAC_Type* DAC, unsigned int status, unsigned int trigger_event

@return None

*/
void DAC_trigger_status(DAC_Type* DAC, unsigned int status,  unsigned int trigger_event){

    int tenIsEnable = RESET;
    
    if(status == ENABLE)
    {
        DAC->CR |= DAC_CR_TEN1;
        tenIsEnable = SET;
    }
    else if (status == DISABLE)
    {
        DAC->CR &=~ DAC_CR_TEN1;
    }
    
    if(tenIsEnable)
    {
        DAC->CR |= trigger_event;
        DAC->CR |= DAC_CR_EN1;    
    }
    

}


/*!<

@brief
DISABLE ADC

@param  ADC_Type* ADC TO DISABLE

@return None

*/
void ADC_DISABLE(ADC_Type* ADC){
    
    ADC->CR |= ADC_CR_ADDIS;
}

/*!<

@brief
DISABLE DAC

@param  DAC_Type* DAC TO DISABLE

@return None

*/
void DAC_DISABLE(DAC_Type* DAC){
  
    DAC->CR &=~ DAC_CR_EN1;
}



/*!-----------------------------------------------------------------------------

                                API FOR USART

-------------------------------------------------------------------------------->*/
/*!<

@brief
SETUP USART IN RX AND TX. 

@param  USART_Type* USART

@return None

*/
void setup_USART_RX_TX(USART_Type* USART){
  
    USART->CR1 &=~ WORD_LENGHT;                            /*!< Set: 1 Start bit, 8 Data bits, n Stop bit       >*/
    USART->BRR |= BR_115_2;                                /*!< Set: baud rate = 115200 bit/s at 72MHz          >*/ 
    USART->CR1 |= USART_EN;                                /*!< USART Enable    >*/   
    USART->CR1 |= RX_EN;                                   /*!< USART in RX     >*/
    USART->CR1 |= TX_EN;                                   /*!< USART in TX     >*/
    
}

/*!<

@brief
Set the data character to be transmitted

@param  USART_Type* USART

@param const char tx[] to transmitt via USART
@param int len         lenght of text to transi

@return None

*/
void usart_tx(USART_Type* USART,const char tx[], int len){
  
  while((USART->ISR & USART_ISR_TXE) != USART_ISR_TXE){};       /*!< Wait until TXE pull up to 1 >*/
  
  for(int i=0;i<len;i++)
  {
      USART->TDR=tx[i];
      while((USART->ISR & USART_ISR_TC) != USART_ISR_TC);                   /*!< Wait until TC pull up to 1 >*/
  }
}
 
/*!<

@brief
Get the received data from Usart

@param  USART_Type* USART

@return None

*/
char usart_rx(USART_Type* USART){      
    
    return ((char)(USART->RDR & 0xFF));
    
}


/*!-----------------------------------------------------------------------------

                                API FOR GENERATION WAVEFORM

-------------------------------------------------------------------------------->*/
/*!<

@brief
GENERATION OF SINE WAVEFORM

@param  short int LUT_in[]      LookUp Table in which are savedesthe samples of waveform

@param  unsigned int nSample       numbers of sample in which is divided the waveform

@return None

*/
 
void sine_waveform(short int LUT_in[], unsigned int nSample)
{
  for(int i=0;i<nSample;i++)
        LUT_in[i]=(short int)(2048+2048*sin(2*3.14*i/nSample));
}
