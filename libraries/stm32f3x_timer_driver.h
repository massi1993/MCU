#ifndef STM32_F3X_T_DRIVER_H
#define STM32_F3X_T_DRIVER_H

/*!----------------------------------------------------------
          DEFINE GENERAL PURPOSE MACROS FOR STM32F3DISCOVERY
-------------------------------------------------------------*/

/*!< define GENERAL MACROS >*/
#define F_CLK   8000000
                                         
#define N_CONT(x)       x*F_CLK                          /*!< x is time time between one flash and the next >*/

/*!< define MACROS for TIMx->SR (STATUS REGISTER)>*/
#define SET_UIF         (1<<0)                          /*!< WHEN CNT REACHES N_CNT(X) THE UIF IS SET TO 1 BY HARDWARE >*/                  
#define CHECK_UIF       ((TIM2->SR)&(1<<0)) == (1<<0)   /*!< CHECK THAT UIF IS 1 >*/


/*!< define MACROS for TIMx->DIER (DMA INTERRUPT ENABLE REGISTER)>*/
#define TIM_DIER_UIE    (1<<0)

#endif /* STM32_F3X_T_DRIVER_H */
