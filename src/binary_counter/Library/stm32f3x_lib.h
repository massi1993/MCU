#ifndef STM32_F3X_LIB_H
#define STM32_F3X_LIB_H

#include <stdint.h>

#define __vo volatile                   /*!< The volatile keyword is intended to prevent 
                                        the compiler from applying any optimizations on 
                                        objects that can change in ways that cannot be 
                                        determined by the compiler.
                                        E.g. A variable can be modified by external 
                                        interrupt.>*/    

/*!----------------------------------------------------------
          DEFINE  STRUCT PERIPHERAL OF STM32F3DISCOVERY
------------------------------------------------------------->*/

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

/*!<----------------------------------------------------------
          END DEFINE STRUCT PERIPHERAL OF STM32F3DISCOVERY
-------------------------------------------------------------*/

/*!< define of peripheral base address >*/
#define RCC             ((RCC_Type*)  0x40021000U)
#define GPIOE           ((GPIO_Type*) 0x48001000U)
#define GPIOA           ((GPIO_Type*) 0x48000000U)


/*!< define MACROS of GPIOx_EN into RCC_AHBENR >*/
#define GPIOA_EN                (1<<17)
#define GPIOB_EN                (1<<18)
#define GPIOC_EN                (1<<19)
#define GPIOD_EN                (1<<20)
#define GPIOE_EN                (1<<21)
#define GPIOF_EN                (1<<22)



#endif /* STM32_F3X_LIB_H */
