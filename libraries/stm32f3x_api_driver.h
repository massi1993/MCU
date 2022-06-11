/* STM32_F3X_DRIVER_H
*
*       Created on : June 01, 2022 
*      Last Update : June 11, 2022
*           Author : massiAv
*
*/
#include <stdint.h>
#include "stm32f3x_lib.h"

#ifndef STM32_F3X_DRIVER_H
#define STM32_F3X_DRIVER_H

/*!----------------------------------------------------------
          DEFINE GENERAL PURPOSE MACROS FOR STM32F3DISCOVERY
-------------------------------------------------------------*/

/*!< define GENERAL MACROS >*/
#define IN_MODE                         0         
#define OUT_MODE                        1
#define ANALOG_MODE                     3
                                                      
#define SET                             1
#define RESET                           0
  
#define ENABLE                          1        
#define DISABLE                         0

#define STOP_WATCH                      1
#define NOT_STOP_WATCH                  0

/*!< MACROS TO DEFINE bits that are written by software to select the source input for the EXTIx with x = 0,1,2,3>*/
/*!< WE DEFINE THEM PortNumber for EXTI REGISTER>*/

#define PA0                             3          /*!< TO SELECT PA0 FOR EXTI0 MUST BE WRITTEN 0 BUT DUE TO SOFTWARE NEGATIVE LOGIC 
                                              WE MUST WRITTEN 3. >*/       
#define PB0                             1
#define PC0                             2
#define PD0                             3
#define PE0                             4
#define PF0                             5

/*!< MACROS TO DEFINE NUMBER PIN OF PORT SELECTED.
 -- > PxN is can be:
         - PA0, PB0, PC0, PD0, PE0, PF0;
         - PA1, PB1, PC1, PD1, PE1, PF1;
         - PA2, PB2, PC2, PD2, PE2, PF2;
          ......
          ......
         - PA0, PB0, PC0, PD0, PE0, PF0;
         - PA0, PB0, PC0, PD0, PE0, PF0; 
>*/
/*!<WE DEFINE THE PinNumber >*/
#define Px0                             0
#define Px1                             1
#define Px2                             2
#define Px3                             3
#define Px4                             4
#define Px5                             5
#define Px6                             6
#define Px7                             7
#define Px8                             8
#define Px9                             9
#define Px10                            10
#define Px11                            11
#define Px12                            12
#define Px13                            13
#define Px14                            14
#define Px15                            15

/*!< define MACROS for GPIOA_IDR>*/
#define PA0_IDR_MASK                    1
#define PA1_IDR_MASK                    2
#define PA2_IDR_MASK                    3
#define PA0_IDR_PRESSED(x)              while(((GPIOA->IDR) & (PA0_IDR_MASK)) == x ) /*!< x can be only 0 or 1>*/
#define PA1_IDR_PRESSED(x)              while(((GPIOA->IDR) & (PA1_IDR_MASK)) == x ) /*!< x can be only 0 or 1>*/
#define PA2_IDR_PRESSED(x)              while(((GPIOA->IDR) & (PA2_IDR_MASK)) == x ) /*!< x can be only 0 or 1>*/


/*!< define MACROS for GPIOE_ODR>*/
#define GPIOE_TURN_LED(x)               GPIOE->ODR=(x)<<8
#define GPIOE_ALL_LED_ON                0x0000FF00      

/*!< define MACROS for RCC_AHB ENABLE >*/
#define RCC_AHBENR_GPIOA                GPIOA_EN
#define RCC_AHBENR_GPIOB                GPIOB_EN
#define RCC_AHBENR_GPIOC                GPIOC_EN
#define RCC_AHBENR_GPIOD                GPIOD_EN
#define RCC_AHBENR_GPIOE                GPIOE_EN
#define RCC_AHBENR_GPIOF                GPIOF_EN
#define RCC_AHBENR_ADC12                ADC12_EN
#define RCC_AHBENR_ADC34                ADC34_EN

/*!< define MACROS for RCC_AP1 ENABLE >*/
#define RCC_APB1ENR_TIM2                TIM2_EN
#define RCC_APB1ENR_TIM3                TIM3_EN
#define RCC_APB1ENR_SPI2                SPI2_EN                 
#define RCC_APB1ENR_SPI3                SPI3_EN                 
#define RCC_APB1ENR_USART2              USART2_EN               
#define RCC_APB1ENR_USART3              USART3_EN               
#define RCC_APB1ENR_UART                UART4_EN                
#define RCC_APB1ENR_UART5               UART5_EN                
#define RCC_APB1ENR_I2C1                I2C1_EN                 
#define RCC_APB1ENR_I2C2                I2C2_EN                 
#define RCC_APB1ENR_USB                 USB_EN                  
#define RCC_APB1ENR_CAN                 CAN_EN                  
#define RCC_APB1ENR_PWREN               PWREN_EN                
#define RCC_APB1ENR_DAC                 DAC_EN                  




/*!----------------------------------------------------------
          FUNCTIONS- PROTOTYPE
-------------------------------------------------------------*/

/*!< Enable Peripheral Clock >*/
void RCC_PCLK_AHBEN(uint32_t RCC_AHBENR_Periph, int status);   
void RCC_PCLK_APB1EN(uint32_t RCC_APB1ENR_Periph, int status);

/*!< API For GPIO>*/
void GPIO_MODE (GPIO_Type* GPIO, unsigned int mode, unsigned int PinNumber);
void GPIO_BSR_REG(GPIO_Type* GPIO, int pinEn, int status);
void GPIOE_OUTMODE(int PEstart, int PEstop);
int bit_pos_GPIO_MODER(int PinNumber);

/*!< API FOR TIMER >*/
void set_PSC_and_ARR_TIM(TIMER_Type* Timer, unsigned int time, unsigned int Fck);
void CNT_EN_TIM(TIMER_Type* Timer, unsigned int status);
float Measure_Time(TIMER_Type* Timer, unsigned int mode);

/*!< API FOR INTERRUPT >*/
void set_SYSCFG_EXTI(int PinNumber, int PortNumber);
int index_EXTI(int PinNumber);
int bit_pos_EXTI(int PinNumber);
void set_NVIC_ISER(int IRQ);
int index_NVIC_ISER(int IRQ);

/*!< API FOR ADC AND DAC>*/
void ADC_Voltage_Regulator_EN(ADC_Type* ADC);
float get_quantization_level(ADC_Type* ADC, unsigned int ADC_res);
unsigned int get_Nchannel_ADC(GPIO_Type* GPIO, unsigned int PinNumber);
ADC_Type* get_number_ADC(GPIO_Type* GPIO, unsigned int PinNumber);
void setup_ADC(GPIO_Type* GPIO, unsigned int PinNumber);
void setup_DAC(DAC_Type* DAC, unsigned int code);
void ADC_DISABLE(ADC_Type* ADC);
void DAC_DISABLE(DAC_Type* DAC);

#endif /* STM32_F3X_DRIVER_H */
