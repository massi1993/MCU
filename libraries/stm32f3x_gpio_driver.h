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


/*!< define MACROS for GPIOA_IDR>*/
#define PA0_IDR_MASK            1
#define PA1_IDR_MASK            2
#define PA2_IDR_MASK            3
#define GPIOA_IDR_IN0()         while(((GPIOA->IDR) & (PA0_IDR_MASK)) == 0)
#define GPIOA_IDR_IN1()         while(((GPIOA->IDR) & (PA0_IDR_MASK)) == 1)



/*!< define MACROS for GPIOE_ODR>*/
#define GPIOE_TURN_LED(x)       GPIOE->ODR=(x)<<8
#define GPIOE_ALL_LED_ON        0x0000FF00      


#endif /* STM32_F3X_DRIVER_H */
