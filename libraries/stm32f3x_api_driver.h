/* STM32_F3X_DRIVER_H
*
*       Created on : June 01, 2022 
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
#define OUT_MODE                1
#define IN_MODE                 3           /*!< IN_MODE DEFINED 3 BECAUSE OF NEGATIVE LOGIC.
                                            TO SET PINY (WITH Y = A,B,C,...,F) IN IN_MODE WE WRITE:   
                                            PYx & = ~ (3 << x) 
                                            INSTEAD OF
                                            PYx | = (0 << x) >*/   
#define SET                     1
#define RESET                   0

#define ENABLE                  1        
#define DISABLE                 0

/*!< MACROS TO DEFINE bits that are written by software to select the source input for the EXTIx with x = 0,1,2,3>*/
/*!< WE DEFINE THEM PortNumber for EXTI REGISTER>*/

#define PA0                     3          /*!< TO SELECT PA0 FOR EXTI0 MUST BE WRITTEN 0 BUT DUE TO SOFTWARE NEGATIVE LOGIC 
                                             WE MUST WRITTEN 3. >*/       
#define PB0                     1
#define PC0                     2
#define PD0                     3
#define PE0                     4
#define PF0                     5

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
#define Px0                     0
#define Px1                     1
#define Px2                     2
#define Px3                     3
#define Px4                     4
#define Px5                     5
#define Px6                     6
#define Px7                     7
#define Px8                     8
#define Px9                     9
#define Px10                    10
#define Px11                    11
#define Px12                    12
#define Px13                    13
#define Px14                    14
#define Px15                    15

/*!< define MACROS for GPIOA_IDR>*/
#define PA0_IDR_MASK            1
#define PA1_IDR_MASK            2
#define PA2_IDR_MASK            3
#define PA0_IDR(x)              while(((GPIOA->IDR) & (PA0_IDR_MASK)) == x ) /*!< x can be only 0 or 1>*/
#define PA1_IDR(x)              while(((GPIOA->IDR) & (PA1_IDR_MASK)) == x ) /*!< x can be only 0 or 1>*/
#define PA2_IDR(x)              while(((GPIOA->IDR) & (PA2_IDR_MASK)) == x ) /*!< x can be only 0 or 1>*/


/*!< define MACROS for GPIOE_ODR>*/
#define GPIOE_TURN_LED(x)       GPIOE->ODR=(x)<<8
#define GPIOE_ALL_LED_ON        0x0000FF00      

/*!< define MACROS for RCC_AHB ENABLE >*/
#define RCC_AHBENR_GPIOA                GPIOA_EN
#define RCC_AHBENR_GPIOB                GPIOB_EN
#define RCC_AHBENR_GPIOC                GPIOC_EN
#define RCC_AHBENR_GPIOD                GPIOD_EN
#define RCC_AHBENR_GPIOE                GPIOE_EN
#define RCC_AHBENR_GPIOF                GPIOF_EN

/*!< define MACROS for RCC_AP1 ENABLE >*/
#define RCC_APB1ENR_TIM2                TIM2_EN
#define RCC_APB1ENR_TIM3                TIM3_EN



/*!< prototypes FUNCTIONS >*/
void RCC_PCLK_AHBEN(uint32_t RCC_AHBENR_Periph, int status);   
void RCC_PCLK_APB1EN(uint32_t RCC_APB1ENR_Periph, int status);
void GPIO_MODE (GPIO_Type* GPIO, unsigned int mode, unsigned int PinNumber);
void GPIO_BSR_REG(GPIO_Type* GPIO, int pinEn, int status);
void GPIOE_OUTMODE(int PEstart, int PEstop);
int bit_pos_GPIO_MODER(int PinNumber);
void set_PSC_and_ARR_TIM(TIMER_Type* Timer, unsigned int time, unsigned int Fck);
void CNT_EN_TIM(TIMER_Type* Timer, unsigned int status);
void set_SYSCFG_EXTI(int PinNumber, int PortNumber);
int index_EXTI(int PinNumber);
int bit_pos_EXTI(int PinNumber);
void set_NVIC_ISER(int IRQ);
int index_NVIC_ISER(int IRQ);

#endif /* STM32_F3X_DRIVER_H */
