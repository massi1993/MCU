/* STM32_F3X_LIB_H
*
*       Created on : June 01, 2022 
*           Author : massiAv
*
*/

#ifndef STM32_F3X_LIB_H
#define STM32_F3X_LIB_H

#include <stdint.h>



#define __vo            volatile                   /*!< The volatile keyword is intended to prevent 
                                                    the compiler from applying any optimizations on 
                                                    objects that can change in ways that cannot be 
                                                    determined by the compiler.
                                                    E.g. A variable can be modified by external 
                                                    interrupt.>*/    

#define FLASH_BASE      ((uint32_t)0x08000000)     /*!< FLASH base address in the alias region >*/

/*!-----------------------------------------------------------------------------
                    PROCESSOR SPECIFIC DETAILS: ARM CORTEX M4-PROCESSOR
-------------------------------------------------------------------------------->*/
 
/*!<  Structure type to access the Nested Vectored Interrupt Controller (NVIC) >*/

typedef struct
{
  __vo uint32_t ISER[8];                 /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register >*/
       uint32_t RESERVED0[24];
  __vo uint32_t ICER[8];                 /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register >*/
       uint32_t RSERVED1[24];
  __vo uint32_t ISPR[8];                 /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register >*/
       uint32_t RESERVED2[24];
  __vo uint32_t ICPR[8];                 /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register >*/
       uint32_t RESERVED3[24];
  __vo uint32_t IABR[8];                 /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register >*/
       uint32_t RESERVED4[56];
  __vo uint8_t  IPR[240];                 /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) >*/
       uint32_t RESERVED5[644];
  __vo uint32_t STIR;                    /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register >*/
}  NVIC_Type;

#define NVIC                ((NVIC_Type*)0xE000E100U)   
                                         


typedef struct
{
  __vo uint32_t CPUID;                   /*!< Offset: 0x000 (R/ )  CPUID Base Register                                   */
  __vo uint32_t ICSR;                    /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register                  */
  __vo uint32_t VTOR;                    /*!< Offset: 0x008 (R/W)  Vector Table Offset Register                          */
  __vo uint32_t AIRCR;                   /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register      */
  __vo uint32_t SCR;                     /*!< Offset: 0x010 (R/W)  System Control Register                               */
  __vo uint32_t CCR;                     /*!< Offset: 0x014 (R/W)  Configuration Control Register                        */
  __vo uint8_t  SHP[12];                 /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __vo uint32_t SHCSR;                   /*!< Offset: 0x024 (R/W)  System Handler Control and State Register             */
  __vo uint32_t CFSR;                    /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register                    */
  __vo uint32_t HFSR;                    /*!< Offset: 0x02C (R/W)  HardFault Status Register                             */
  __vo uint32_t DFSR;                    /*!< Offset: 0x030 (R/W)  Debug Fault Status Register                           */
  __vo uint32_t MMFAR;                   /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register                      */
  __vo uint32_t BFAR;                    /*!< Offset: 0x038 (R/W)  BusFault Address Register                             */
  __vo uint32_t AFSR;                    /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register                       */
  __vo uint32_t PFR[2];                  /*!< Offset: 0x040 (R/ )  Processor Feature Register                            */
  __vo uint32_t DFR;                     /*!< Offset: 0x048 (R/ )  Debug Feature Register                                */
  __vo uint32_t ADR;                     /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register                            */
  __vo uint32_t MMFR[4];                 /*!< Offset: 0x050 (R/ )  Memory Model Feature Register                         */
  __vo uint32_t ISAR[5];                 /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register                   */
  __vo uint32_t RESERVED0[5];
  __vo uint32_t CPACR;                   /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register                   */
} SCB_Type;


#define SCB                 ((SCB_Type*) 0xE000E000U )   /*!< SCB configuration struct>*/  


/** 
  * @brief FLASH Registers
  */

typedef struct
{
  __vo uint32_t ACR;          /*!< FLASH access control register,              Address offset: 0x00 */
  __vo uint32_t KEYR;         /*!< FLASH key register,                         Address offset: 0x04 */
  __vo uint32_t OPTKEYR;      /*!< FLASH option key register,                  Address offset: 0x08 */
  __vo uint32_t SR;           /*!< FLASH status register,                      Address offset: 0x0C */
  __vo uint32_t CR;           /*!< FLASH control register,                     Address offset: 0x10 */
  __vo uint32_t AR;           /*!< FLASH address register,                     Address offset: 0x14 */
       uint32_t RESERVED;     /*!< Reserved, 0x18                                                   */
  __vo uint32_t OBR;          /*!< FLASH Option byte register,                 Address offset: 0x1C */
  __vo uint32_t WRPR;         /*!< FLASH Write register,                       Address offset: 0x20 */
  
} FLASH_TypeDef;


#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Peripheral base address in the alias region */
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x00002000) /*!< Flash registers base address */
#define FLASH                 ((FLASH_TypeDef *) FLASH_R_BASE)

/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for FLASH_ACR register  ******************/
#define  FLASH_ACR_LATENCY                   ((uint8_t)0x03)               /*!< LATENCY[2:0] bits (Latency) */
#define  FLASH_ACR_LATENCY_0                 ((uint8_t)0x01)               /*!< Bit 0 */
#define  FLASH_ACR_LATENCY_1                 ((uint8_t)0x02)               /*!< Bit 1 */

#define  FLASH_ACR_HLFCYA                    ((uint8_t)0x08)               /*!< Flash Half Cycle Access Enable */
#define  FLASH_ACR_PRFTBE                    ((uint8_t)0x10)               /*!< Prefetch Buffer Enable */
#define  FLASH_ACR_PRFTBS                    ((uint8_t)0x20)  


/*!-----------------------------------------------------------------------------
                    DEFINE  STRUCT PERIPHERAL OF STM32F3DISCOVERY
-------------------------------------------------------------------------------->*/

/*!< define RCC REGISTERS >*/
typedef struct{
        __vo uint32_t CR;               /*!< its address is (RCC_base_address + 0x00) >*/                    
        __vo uint32_t CFGR;             /*!< its address is (RCC_base_address + 0x04) >*/
        __vo uint32_t CIR;              /*!< its address is (RCC_base_address + 0x08) >*/
        __vo uint32_t APB2RSTR;         /*!< its address is (RCC_base_address + 0x0C) >*/
        __vo uint32_t APB1RSTR;         /*!< its address is (RCC_base_address + 0x10) >*/        
        __vo uint32_t AHBENR;           /*!< its address is (RCC_base_address + 0x14) >*/   
        __vo uint32_t APB2ENR;          /*!< its address is (RCC_base_address + 0x18) >*/
        __vo uint32_t APB1ENR;          /*!< its address is (RCC_base_address + 0x1C) >*/   
        __vo uint32_t BDCR;             /*!< its address is (RCC_base_address + 0x20) >*/   
        __vo uint32_t CSR;              /*!< its address is (RCC_base_address + 0x24) >*/   
        __vo uint32_t AHBRSTR;          /*!< its address is (RCC_base_address + 0x28) >*/   
        __vo uint32_t CFGR2;             /*!< its address is (RCC_base_address + 0x2C) >*/
        __vo uint32_t CFGR3;             /*!< its address is (RCC_base_address + 0x30) >*/
}RCC_Type;


typedef struct{
       __vo uint32_t MODER;             /*!< its address is (GPIOx_base_address + 0x00) >*/
       __vo uint32_t OTYPER;            /*!< its address is (GPIOx_base_address + 0x04) >*/
       __vo uint32_t OSPEEDR;           /*!< its address is (GPIOx_base_address + 0x08) >*/
       __vo uint32_t PUPDR;             /*!< its address is (GPIOx_base_address + 0x0C) >*/
       __vo uint32_t IDR;               /*!< its address is (GPIOx_base_address + 0x10) >*/
       __vo uint32_t ODR;               /*!< its address is (GPIOx_base_address + 0x14) >*/
}GPIO_Type;


typedef struct{
      __vo uint32_t CFGR1;              /*!< its address is (SYSCFG_base_address + 0x00) >*/
      __vo uint32_t RCR;                /*!< its address is (SYSCFG_base_address + 0x04) >*/
      __vo uint32_t EXTICR[4];          /*!< its address is (SYSCFG_base_address + 0x08) >*/
      __vo uint32_t CFGR2;              /*!< its address is (SYSCFG_base_address + 0x14) >*/
 }SYSCFG_Type;



typedef struct{
      __vo uint32_t IMR;               /*!< its address is (EXTI_base_address + 0x00) >*/
      __vo uint32_t EMR;               /*!< its address is (EXTI_base_address + 0x04) >*/
      __vo uint32_t RTSR;              /*!< its address is (EXTI_base_address + 0x08) >*/
      __vo uint32_t FTSR;              /*!< its address is (EXTI_base_address + 0x0C) >*/
      __vo uint32_t SWIER;             /*!< its address is (EXTI_base_address + 0x10) >*/
      __vo uint32_t PR;                /*!< its address is (EXTI_base_address + 0x14) >*/
 }EXTI_Type;


typedef struct {
      __vo uint32_t CR1;                /*!< its address is (TIMER_base_address + 0x00) >*/
      __vo uint32_t CR2;                /*!< its address is (TIMER_base_address + 0x04) >*/
      __vo uint32_t SMCR;               /*!< its address is (TIMER_base_address + 0x08) >*/
      __vo uint32_t DIER;               /*!< its address is (TIMER_base_address + 0x0C) >*/
      __vo uint32_t SR;                 /*!< its address is (TIMER_base_address + 0x10) >*/
      __vo uint32_t EGR;                /*!< its address is (TIMER_base_address + 0x14) >*/
      __vo uint32_t CCMR1;              /*!< its address is (TIMER_base_address + 0x18) >*/
      __vo uint32_t CCMR2;              /*!< its address is (TIMER_base_address + 0x1C) >*/
      __vo uint32_t CCER;               /*!< its address is (TIMER_base_address + 0x20) >*/
      __vo uint32_t CNT;                /*!< its address is (TIMER_base_address + 0x24) >*/
      __vo uint32_t PSC;                /*!< its address is (TIMER_base_address + 0x28) >*/
      __vo uint32_t ARR;                /*!< its address is (TIMER_base_address + 0x2C) >*/
      __vo uint32_t RESERVED1;          /*!< its address is (TIMER_base_address + 0x30) -> RESERVED >*/
      __vo uint32_t CCR1;               /*!< its address is (TIMER_base_address + 0x34) >*/
      __vo uint32_t CCR2;               /*!< its address is (TIMER_base_address + 0x38) >*/
      __vo uint32_t CCR3;               /*!< its address is (TIMER_base_address + 0x3C) >*/
      __vo uint32_t CCR4;               /*!< its address is (TIMER_base_address + 0x40) >*/
      __vo uint32_t RESERVED2;          /*!< its address is (TIMER_base_address + 0x44) -> RESERVED >*/
      __vo uint32_t DCR;                /*!< its address is (TIMER_base_address + 0x48) >*/
      __vo uint32_t DMAR;               /*!< its address is (TIMER_base_address + 0x4C) >*/
}TIMER_Type;

/*!<----------------------------------------------------------
          END DEFINE STRUCT PERIPHERAL OF STM32F3DISCOVERY
-------------------------------------------------------------*/

/*!< define of peripheral base address >*/
#define RCC             ((RCC_Type*)    0x40021000U)
#define GPIOE           ((GPIO_Type*)   0x48001000U)
#define GPIOA           ((GPIO_Type*)   0x48000000U)
#define TIM2            ((TIMER_Type*)  0x40000000U)
#define EXTI            ((EXTI_Type*)   0x40010400U)
#define SYSCFG          ((SYSCFG_Type*) 0x40010000U)

/*!< define MACROS of GPIOx_EN into RCC_AHBENR >*/
#define GPIOA_EN                (1<<17)
#define GPIOB_EN                (1<<18)
#define GPIOC_EN                (1<<19)
#define GPIOD_EN                (1<<20)
#define GPIOE_EN                (1<<21)
#define GPIOF_EN                (1<<22)



/*!< define macros for INTERRUPT REQUEST FOR STM32F3DISCOVERY >*/
#define IRQ_NO_EXTI0            6
#define IRQ_NO_EXTI1            7
#define IRQ_NO_EXTI2            8
#define IRQ_NO_EXTI3            9
#define IRQ_NO_EXTI4            10
#define IRQ_NO_EXTI9_5          23
#define IRQ_NO_EXTI15_10        40        



/*!< define MACROS of TIM2  >*/
#define TIM2_EN                 (1<<0)
#define TIM3_EN                 (1<<1)
#define CEN_EN                  (1<<0)


/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_HSION                        ((uint32_t)0x00000001)
#define  RCC_CR_HSIRDY                       ((uint32_t)0x00000002)

#define  RCC_CR_HSITRIM                      ((uint32_t)0x000000F8)
#define  RCC_CR_HSITRIM_0                    ((uint32_t)0x00000008)/*!<Bit 0 */
#define  RCC_CR_HSITRIM_1                    ((uint32_t)0x00000010)/*!<Bit 1 */
#define  RCC_CR_HSITRIM_2                    ((uint32_t)0x00000020)/*!<Bit 2 */
#define  RCC_CR_HSITRIM_3                    ((uint32_t)0x00000040)/*!<Bit 3 */
#define  RCC_CR_HSITRIM_4                    ((uint32_t)0x00000080)/*!<Bit 4 */

#define  RCC_CR_HSICAL                       ((uint32_t)0x0000FF00)
#define  RCC_CR_HSICAL_0                     ((uint32_t)0x00000100)/*!<Bit 0 */
#define  RCC_CR_HSICAL_1                     ((uint32_t)0x00000200)/*!<Bit 1 */
#define  RCC_CR_HSICAL_2                     ((uint32_t)0x00000400)/*!<Bit 2 */
#define  RCC_CR_HSICAL_3                     ((uint32_t)0x00000800)/*!<Bit 3 */
#define  RCC_CR_HSICAL_4                     ((uint32_t)0x00001000)/*!<Bit 4 */
#define  RCC_CR_HSICAL_5                     ((uint32_t)0x00002000)/*!<Bit 5 */
#define  RCC_CR_HSICAL_6                     ((uint32_t)0x00004000)/*!<Bit 6 */
#define  RCC_CR_HSICAL_7                     ((uint32_t)0x00008000)/*!<Bit 7 */

#define  RCC_CR_HSEON                        ((uint32_t)0x00010000)
#define  RCC_CR_HSERDY                       ((uint32_t)0x00020000)
#define  RCC_CR_HSEBYP                       ((uint32_t)0x00040000)
#define  RCC_CR_CSSON                        ((uint32_t)0x00080000)

#define  RCC_CR_PLLON                        ((uint32_t)0x01000000)
#define  RCC_CR_PLLRDY                       ((uint32_t)0x02000000)


/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define  RCC_CFGR_SW                         ((uint32_t)0x00000003)        /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  RCC_CFGR_SW_1                       ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  RCC_CFGR_SW_HSI                     ((uint32_t)0x00000000)        /*!< HSI selected as system clock */
#define  RCC_CFGR_SW_HSE                     ((uint32_t)0x00000001)        /*!< HSE selected as system clock */
#define  RCC_CFGR_SW_PLL                     ((uint32_t)0x00000002)        /*!< PLL selected as system clock */

/*!< SWS configuration */
#define  RCC_CFGR_SWS                        ((uint32_t)0x0000000C)        /*!< SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  RCC_CFGR_SWS_1                      ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  RCC_CFGR_SWS_HSI                    ((uint32_t)0x00000000)        /*!< HSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    ((uint32_t)0x00000004)        /*!< HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    ((uint32_t)0x00000008)        /*!< PLL used as system clock */

/*!< HPRE configuration */
#define  RCC_CFGR_HPRE                       ((uint32_t)0x000000F0)        /*!< HPRE[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE_0                     ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  RCC_CFGR_HPRE_1                     ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  RCC_CFGR_HPRE_2                     ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  RCC_CFGR_HPRE_3                     ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  RCC_CFGR_HPRE_DIV1                  ((uint32_t)0x00000000)        /*!< SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  ((uint32_t)0x00000080)        /*!< SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  ((uint32_t)0x00000090)        /*!< SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  ((uint32_t)0x000000A0)        /*!< SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 ((uint32_t)0x000000B0)        /*!< SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 ((uint32_t)0x000000C0)        /*!< SYSCLK divided by 64 */
#define  RCC_CFGR_HPRE_DIV128                ((uint32_t)0x000000D0)        /*!< SYSCLK divided by 128 */
#define  RCC_CFGR_HPRE_DIV256                ((uint32_t)0x000000E0)        /*!< SYSCLK divided by 256 */
#define  RCC_CFGR_HPRE_DIV512                ((uint32_t)0x000000F0)        /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define  RCC_CFGR_PPRE1                      ((uint32_t)0x00000700)        /*!< PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_CFGR_PPRE1_0                    ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE1_1                    ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE1_2                    ((uint32_t)0x00000400)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE1_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((uint32_t)0x00000400)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 ((uint32_t)0x00000500)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 ((uint32_t)0x00000600)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                ((uint32_t)0x00000700)        /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define  RCC_CFGR_PPRE2                      ((uint32_t)0x00003800)        /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    ((uint32_t)0x00000800)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE2_1                    ((uint32_t)0x00001000)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE2_2                    ((uint32_t)0x00002000)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE2_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 ((uint32_t)0x00002000)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 ((uint32_t)0x00002800)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 ((uint32_t)0x00003000)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                ((uint32_t)0x00003800)        /*!< HCLK divided by 16 */

#define  RCC_CFGR_PLLSRC                     ((uint32_t)0x00010000)        /*!< PLL entry clock source */

#define  RCC_CFGR_PLLXTPRE                   ((uint32_t)0x00020000)        /*!< HSE divider for PLL entry */

/*!< PLLMUL configuration */
#define  RCC_CFGR_PLLMULL                    ((uint32_t)0x003C0000)        /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define  RCC_CFGR_PLLMULL_0                  ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  RCC_CFGR_PLLMULL_1                  ((uint32_t)0x00080000)        /*!< Bit 1 */
#define  RCC_CFGR_PLLMULL_2                  ((uint32_t)0x00100000)        /*!< Bit 2 */
#define  RCC_CFGR_PLLMULL_3                  ((uint32_t)0x00200000)        /*!< Bit 3 */

#define  RCC_CFGR_PLLSRC_HSI_Div2            ((uint32_t)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
#define  RCC_CFGR_PLLSRC_PREDIV1             ((uint32_t)0x00010000)        /*!< PREDIV1 clock selected as PLL entry clock source */

#define  RCC_CFGR_PLLXTPRE_PREDIV1           ((uint32_t)0x00000000)        /*!< PREDIV1 clock not divided for PLL entry */
#define  RCC_CFGR_PLLXTPRE_PREDIV1_Div2      ((uint32_t)0x00020000)        /*!< PREDIV1 clock divided by 2 for PLL entry */

#define  RCC_CFGR_PLLMULL2                   ((uint32_t)0x00000000)        /*!< PLL input clock*2 */
#define  RCC_CFGR_PLLMULL3                   ((uint32_t)0x00040000)        /*!< PLL input clock*3 */
#define  RCC_CFGR_PLLMULL4                   ((uint32_t)0x00080000)        /*!< PLL input clock*4 */
#define  RCC_CFGR_PLLMULL5                   ((uint32_t)0x000C0000)        /*!< PLL input clock*5 */
#define  RCC_CFGR_PLLMULL6                   ((uint32_t)0x00100000)        /*!< PLL input clock*6 */
#define  RCC_CFGR_PLLMULL7                   ((uint32_t)0x00140000)        /*!< PLL input clock*7 */
#define  RCC_CFGR_PLLMULL8                   ((uint32_t)0x00180000)        /*!< PLL input clock*8 */
#define  RCC_CFGR_PLLMULL9                   ((uint32_t)0x001C0000)        /*!< PLL input clock*9 */
#define  RCC_CFGR_PLLMULL10                  ((uint32_t)0x00200000)        /*!< PLL input clock10 */
#define  RCC_CFGR_PLLMULL11                  ((uint32_t)0x00240000)        /*!< PLL input clock*11 */
#define  RCC_CFGR_PLLMULL12                  ((uint32_t)0x00280000)        /*!< PLL input clock*12 */
#define  RCC_CFGR_PLLMULL13                  ((uint32_t)0x002C0000)        /*!< PLL input clock*13 */
#define  RCC_CFGR_PLLMULL14                  ((uint32_t)0x00300000)        /*!< PLL input clock*14 */
#define  RCC_CFGR_PLLMULL15                  ((uint32_t)0x00340000)        /*!< PLL input clock*15 */
#define  RCC_CFGR_PLLMULL16                  ((uint32_t)0x00380000)        /*!< PLL input clock*16 */

/*!< USB configuration */
#define  RCC_CFGR_USBPRE                     ((uint32_t)0x00400000)        /*!< USB prescaler */

/*!< I2S configuration */
#define  RCC_CFGR_I2SSRC                     ((uint32_t)0x00800000)        /*!< I2S external clock source selection */

/*!< MCO configuration */
#define  RCC_CFGR_MCO                        ((uint32_t)0x07000000)        /*!< MCO[2:0] bits (Microcontroller Clock Output) */
#define  RCC_CFGR_MCO_0                      ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  RCC_CFGR_MCO_1                      ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  RCC_CFGR_MCO_2                      ((uint32_t)0x04000000)        /*!< Bit 2 */

#define  RCC_CFGR_MCO_NOCLOCK                ((uint32_t)0x00000000)        /*!< No clock */
#define  RCC_CFGR_MCO_LSI                    ((uint32_t)0x02000000)        /*!< LSI clock selected as MCO source */
#define  RCC_CFGR_MCO_LSE                    ((uint32_t)0x03000000)        /*!< LSE clock selected as MCO source */
#define  RCC_CFGR_MCO_SYSCLK                 ((uint32_t)0x04000000)        /*!< System clock selected as MCO source */
#define  RCC_CFGR_MCO_HSI                    ((uint32_t)0x05000000)        /*!< HSI clock selected as MCO source */
#define  RCC_CFGR_MCO_HSE                    ((uint32_t)0x06000000)        /*!< HSE clock selected as MCO source  */
#define  RCC_CFGR_MCO_PLL                    ((uint32_t)0x07000000)        /*!< PLL clock divided by 2 selected as MCO source */

#define  RCC_CFGR_MCOF                       ((uint32_t)0x10000000)        /*!< Microcontroller Clock Output Flag */




/*******************  Bit definition for RCC_CFGR2 register  ******************/
/*!< PREDIV1 configuration */
#define  RCC_CFGR2_PREDIV1                   ((uint32_t)0x0000000F)        /*!< PREDIV1[3:0] bits */
#define  RCC_CFGR2_PREDIV1_0                 ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  RCC_CFGR2_PREDIV1_1                 ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  RCC_CFGR2_PREDIV1_2                 ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  RCC_CFGR2_PREDIV1_3                 ((uint32_t)0x00000008)        /*!< Bit 3 */

#define  RCC_CFGR2_PREDIV1_DIV1              ((uint32_t)0x00000000)        /*!< PREDIV1 input clock not divided */
#define  RCC_CFGR2_PREDIV1_DIV2              ((uint32_t)0x00000001)        /*!< PREDIV1 input clock divided by 2 */
#define  RCC_CFGR2_PREDIV1_DIV3              ((uint32_t)0x00000002)        /*!< PREDIV1 input clock divided by 3 */
#define  RCC_CFGR2_PREDIV1_DIV4              ((uint32_t)0x00000003)        /*!< PREDIV1 input clock divided by 4 */
#define  RCC_CFGR2_PREDIV1_DIV5              ((uint32_t)0x00000004)        /*!< PREDIV1 input clock divided by 5 */
#define  RCC_CFGR2_PREDIV1_DIV6              ((uint32_t)0x00000005)        /*!< PREDIV1 input clock divided by 6 */
#define  RCC_CFGR2_PREDIV1_DIV7              ((uint32_t)0x00000006)        /*!< PREDIV1 input clock divided by 7 */
#define  RCC_CFGR2_PREDIV1_DIV8              ((uint32_t)0x00000007)        /*!< PREDIV1 input clock divided by 8 */
#define  RCC_CFGR2_PREDIV1_DIV9              ((uint32_t)0x00000008)        /*!< PREDIV1 input clock divided by 9 */
#define  RCC_CFGR2_PREDIV1_DIV10             ((uint32_t)0x00000009)        /*!< PREDIV1 input clock divided by 10 */
#define  RCC_CFGR2_PREDIV1_DIV11             ((uint32_t)0x0000000A)        /*!< PREDIV1 input clock divided by 11 */
#define  RCC_CFGR2_PREDIV1_DIV12             ((uint32_t)0x0000000B)        /*!< PREDIV1 input clock divided by 12 */
#define  RCC_CFGR2_PREDIV1_DIV13             ((uint32_t)0x0000000C)        /*!< PREDIV1 input clock divided by 13 */
#define  RCC_CFGR2_PREDIV1_DIV14             ((uint32_t)0x0000000D)        /*!< PREDIV1 input clock divided by 14 */
#define  RCC_CFGR2_PREDIV1_DIV15             ((uint32_t)0x0000000E)        /*!< PREDIV1 input clock divided by 15 */
#define  RCC_CFGR2_PREDIV1_DIV16             ((uint32_t)0x0000000F)        /*!< PREDIV1 input clock divided by 16 */

/*!< ADCPRE12 configuration */
#define  RCC_CFGR2_ADCPRE12                  ((uint32_t)0x000001F0)        /*!< ADCPRE12[8:4] bits */
#define  RCC_CFGR2_ADCPRE12_0                ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  RCC_CFGR2_ADCPRE12_1                ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  RCC_CFGR2_ADCPRE12_2                ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  RCC_CFGR2_ADCPRE12_3                ((uint32_t)0x00000080)        /*!< Bit 3 */
#define  RCC_CFGR2_ADCPRE12_4                ((uint32_t)0x00000100)        /*!< Bit 4 */

#define  RCC_CFGR2_ADCPRE12_NO               ((uint32_t)0x00000000)        /*!< ADC12 clock disabled, ADC12 can use AHB clock */
#define  RCC_CFGR2_ADCPRE12_DIV1             ((uint32_t)0x00000100)        /*!< ADC12 PLL clock divided by 1 */
#define  RCC_CFGR2_ADCPRE12_DIV2             ((uint32_t)0x00000110)        /*!< ADC12 PLL clock divided by 2 */
#define  RCC_CFGR2_ADCPRE12_DIV4             ((uint32_t)0x00000120)        /*!< ADC12 PLL clock divided by 4 */
#define  RCC_CFGR2_ADCPRE12_DIV6             ((uint32_t)0x00000130)        /*!< ADC12 PLL clock divided by 6 */
#define  RCC_CFGR2_ADCPRE12_DIV8             ((uint32_t)0x00000140)        /*!< ADC12 PLL clock divided by 8 */
#define  RCC_CFGR2_ADCPRE12_DIV10            ((uint32_t)0x00000150)        /*!< ADC12 PLL clock divided by 10 */
#define  RCC_CFGR2_ADCPRE12_DIV12            ((uint32_t)0x00000160)        /*!< ADC12 PLL clock divided by 12 */
#define  RCC_CFGR2_ADCPRE12_DIV16            ((uint32_t)0x00000170)        /*!< ADC12 PLL clock divided by 16 */
#define  RCC_CFGR2_ADCPRE12_DIV32            ((uint32_t)0x00000180)        /*!< ADC12 PLL clock divided by 32 */
#define  RCC_CFGR2_ADCPRE12_DIV64            ((uint32_t)0x00000190)        /*!< ADC12 PLL clock divided by 64 */
#define  RCC_CFGR2_ADCPRE12_DIV128           ((uint32_t)0x000001A0)        /*!< ADC12 PLL clock divided by 128 */
#define  RCC_CFGR2_ADCPRE12_DIV256           ((uint32_t)0x000001B0)        /*!< ADC12 PLL clock divided by 256 */

/*!< ADCPRE34 configuration */
#define  RCC_CFGR2_ADCPRE34                  ((uint32_t)0x00003E00)        /*!< ADCPRE34[13:5] bits */
#define  RCC_CFGR2_ADCPRE34_0                ((uint32_t)0x00000200)        /*!< Bit 0 */
#define  RCC_CFGR2_ADCPRE34_1                ((uint32_t)0x00000400)        /*!< Bit 1 */
#define  RCC_CFGR2_ADCPRE34_2                ((uint32_t)0x00000800)        /*!< Bit 2 */
#define  RCC_CFGR2_ADCPRE34_3                ((uint32_t)0x00001000)        /*!< Bit 3 */
#define  RCC_CFGR2_ADCPRE34_4                ((uint32_t)0x00002000)        /*!< Bit 4 */

#define  RCC_CFGR2_ADCPRE34_NO               ((uint32_t)0x00000000)        /*!< ADC34 clock disabled, ADC34 can use AHB clock */
#define  RCC_CFGR2_ADCPRE34_DIV1             ((uint32_t)0x00002000)        /*!< ADC34 PLL clock divided by 1 */
#define  RCC_CFGR2_ADCPRE34_DIV2             ((uint32_t)0x00002200)        /*!< ADC34 PLL clock divided by 2 */
#define  RCC_CFGR2_ADCPRE34_DIV4             ((uint32_t)0x00002400)        /*!< ADC34 PLL clock divided by 4 */
#define  RCC_CFGR2_ADCPRE34_DIV6             ((uint32_t)0x00002600)        /*!< ADC34 PLL clock divided by 6 */
#define  RCC_CFGR2_ADCPRE34_DIV8             ((uint32_t)0x00002800)        /*!< ADC34 PLL clock divided by 8 */
#define  RCC_CFGR2_ADCPRE34_DIV10            ((uint32_t)0x00002A00)        /*!< ADC34 PLL clock divided by 10 */
#define  RCC_CFGR2_ADCPRE34_DIV12            ((uint32_t)0x00002C00)        /*!< ADC34 PLL clock divided by 12 */
#define  RCC_CFGR2_ADCPRE34_DIV16            ((uint32_t)0x00002E00)        /*!< ADC34 PLL clock divided by 16 */
#define  RCC_CFGR2_ADCPRE34_DIV32            ((uint32_t)0x00003000)        /*!< ADC34 PLL clock divided by 32 */
#define  RCC_CFGR2_ADCPRE34_DIV64            ((uint32_t)0x00003200)        /*!< ADC34 PLL clock divided by 64 */
#define  RCC_CFGR2_ADCPRE34_DIV128           ((uint32_t)0x00003400)        /*!< ADC34 PLL clock divided by 128 */
#define  RCC_CFGR2_ADCPRE34_DIV256           ((uint32_t)0x00003600)        /*!< ADC34 PLL clock divided by 256 */

/**
 * @brief In the following line adjust the value of External High Speed oscillator (HSE)
   used in your application 
   
   Tip: To avoid modifying this file each time you need to use different HSE, you
        can define the HSE value in your toolchain compiler preprocessor.
  */           
#if !defined  (HSE_VALUE) 
 #define HSE_VALUE            ((uint32_t)8000000) /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

/**
 * @brief In the following line adjust the External High Speed oscillator (HSE) Startup 
   Timeout value 
   */
#if !defined  (HSE_STARTUP_TIMEOUT) 
 #define HSE_STARTUP_TIMEOUT  ((uint16_t)0x0500)   /*!< Time out for HSE start up */
#endif /* HSE_STARTUP_TIMEOUT */


/**
 * @brief In the following line adjust the Internal High Speed oscillator (HSI) Startup 
   Timeout value 
   */
#if !defined  (HSI_STARTUP_TIMEOUT) 
 #define HSI_STARTUP_TIMEOUT   ((uint16_t)0x0500) /*!< Time out for HSI start up */
#endif /* HSI_STARTUP_TIMEOUT */  

#if !defined  (HSI_VALUE) 
 #define HSI_VALUE  ((uint32_t)8000000)
#endif /* HSI_VALUE */                      /*!< Value of the Internal High Speed oscillator in Hz.
                                            The real value may vary depending on the variations
                                             in voltage and temperature.  */
#if !defined  (LSI_VALUE) 
 #define LSI_VALUE  ((uint32_t)40000)    
#endif /* LSI_VALUE */                      /*!< Value of the Internal Low Speed oscillator in Hz
                                             The real value may vary depending on the variations
                                             in voltage and temperature.  */
#if !defined  (LSE_VALUE)
 #define LSE_VALUE  ((uint32_t)32768)    /*!< Value of the External Low Speed oscillator in Hz */
#endif /* LSE_VALUE */     



#endif /* STM32_F3X_LIB_H */
