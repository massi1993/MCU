/*!----------------------------------------------------------
          DEFINE  STRUCT PERIPHERAL OF STM32F3DISCOVERY
-------------------------------------------------------------*/

/*!< define RCC REGISTERS >*/
typedef struct{
        unsigned int CR;
        unsigned int CFGR;
        unsigned int CIR;
        unsigned int APB2RSTR;
        unsigned int APB1RSTR;
        unsigned int AHBENR;
}RCC_Type;

typedef struct{
        unsigned int MODER;
        unsigned int OTYPER;
        unsigned int OSPEEDR;
        unsigned int PUPDR;
        unsigned int IDR;
        unsigned int ODR;
}GPIO_Type;

/*!< define GENERAL MACROS >*/
#define OUT_MODE     1
#define IN_MODE      3           /*!< IN_MODE DEFINED 3 BECAUSE OF NEGATIVE LOGIC.
                                      TO SET PINY (WITH Y = A,B,C,...,F) IN IN_MODE WE WRITE:   
                                      PYx & = ~ (3 << x) 
                                      INSTEAD OF
                                      PYx | = (0 << x) >*/       


/*!< define of peripheral base address >*/
#define RCC             ((RCC_Type*)  0x40021000)
#define GPIOE           ((GPIO_Type*) 0x48001000)
#define GPIOA           ((GPIO_Type*) 0x48000000)


/*!< define MACROS of GPIOx_EN into RCC_AHBENR >*/
#define GPIOA_EN                (1<<17)
#define GPIOB_EN                (1<<18)
#define GPIOC_EN                (1<<19)
#define GPIOD_EN                (1<<20)
#define GPIOE_EN                (1<<21)
#define GPIOF_EN                (1<<22)




/*!< define MACROS for GPIOA_IDR>*/
#define PA0_IDR_MASK            1
#define PA1_IDR_MASK            2
#define PA2_IDR_MASK            3
#define GPIOA_IDR_IN0()         while(((GPIOA->IDR) & (PA0_IDR_MASK)) == 0)
#define GPIOA_IDR_IN1()         while(((GPIOA->IDR) & (PA0_IDR_MASK)) == 1)



/*!< define MACROS for GPIOE_ODR>*/
#define GPIOE_TURN_LED(x)       GPIOE->ODR=(x)<<8

