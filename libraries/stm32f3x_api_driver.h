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
#define AF_MODE                         2
#define ANALOG_MODE                     3
                                                      
#define SET                             1
#define RESET                           0
  
#define ENABLE                          1        
#define DISABLE                         0

#define STOP_WATCH                      1
#define NOT_STOP_WATCH                  0

#define SINGLE_MODE                     0
#define CONTINUOUS_MODE                 1        

/*!< MACROS TO DEFINE bits that are written by software to select the source input for the EXTIx with x = 0,1,2,3>*/
/*!< WE DEFINE THEM PortNumber for EXTI REGISTER>*/

#define PA0                             3          /*!< TO SELECT PA0 FOR EXTI0 MUST BE WRITTEN 0 BUT DUE TO SOFTWARE NEGATIVE LOGIC 
                                                    WE MUST WRITTEN 3. >*/       
#define PB0                             1
#define PC0                             2
#define PD0                             3
#define PE0                             4
#define PF0                             5

                                                      
                                                      
/*!< MACROS for alternate function>*/   
#define AF0                             0                          
#define AF1                             0x1
#define AF2                             0x2
#define AF3                             0x3
#define AF4                             0x4
#define AF5                             0x5
#define AF6                             0x6                                      
#define AF7                             0x7                                                                                            
#define AF8                             0x8                                      
#define AF9                             0x9
#define AF10                            0x10  
#define AF11                            0x11 
#define AF12                            0x12  
#define AF13                            0x13
#define AF14                            0x14
#define AF15                            0x15

                                                      
/*!< MACROS FOR BAUD RATE USART >*/
#define BR_38_4                         0x753
#define BR_57_6                         0x4E2                                                      
#define BR_115_2                        0x271
#define BR_230_4                        0x139
                                                      
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


/*!< define MACROS for RCC_AP2 ENABLE >*/
#define RCC_APB2ENR_SYSCFG              SYSCFG_EN   
#define RCC_APB2ENR_TIM1                TIM1_EN     
#define RCC_APB2ENR_SPI1                SPI1_EN     
#define RCC_APB2ENR_TIM8                TIM8_EN     
#define RCC_APB2ENR_USART1              USART1_EN   
#define RCC_APB2ENR_TIM15               TIM15_EN    
#define RCC_APB2ENR_TIM16               TIM16_EN    
#define RCC_APB2ENR_TIM17               TIM17_EN    

/*!----------------------------------------------------------
          FUNCTIONS- PROTOTYPE
-------------------------------------------------------------*/

/*!< Enable Peripheral Clock >*/
void RCC_PCLK_AHBEN(uint32_t RCC_AHBENR_Periph, int status);   
void RCC_PCLK_APB1EN(uint32_t RCC_APB1ENR_Periph, int status);
void RCC_PCLK_APB2EN(uint32_t RCC_APB2ENR_Periph, int status);

/*!< API For GPIO>*/
void GPIO_MODE(GPIO_Type* GPIO, unsigned int mode, unsigned int PinNumber);
void GPIO_BSR_REG(GPIO_Type* GPIO, int pinEn, int status);
void GPIOE_OUTMODE(int PEstart, int PEstop);
int bit_pos_GPIO_MODER(int PinNumber);
void GPIO_AFR(GPIO_Type* GPIO, unsigned int AF_Type, unsigned int PinNumber);
int index_AFR(int PinNumber);
int bit_pos_GPIO_AFR(int PinNumber);

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
ADC_Type* setup_ADC(GPIO_Type* GPIO, unsigned int PinNumber, uint8_t conversion_mode);
void setup_DAC(DAC_Type* DAC, unsigned int code);
void ADC_DISABLE(ADC_Type* ADC);
void DAC_DISABLE(DAC_Type* DAC);

/*!< API FOR USART>*/
void setup_USART_RX_TX(USART_Type* USART);
void usart_tx(USART_Type* USART, const char tx[], int len);
char usart_rx(USART_Type* USART);


#endif /* STM32_F3X_DRIVER_H */
