/* STM32_F3X_T_DRIVER_H
*
*       Created on : June 01, 2022 
*      Last Update : June 10, 2022
*           Author : massiAv
*
*/

#ifndef STM32_F3X_T_DRIVER_H
#define STM32_F3X_T_DRIVER_H

/*!----------------------------------------------------------
          DEFINE GENERAL PURPOSE MACROS FOR STM32F3DISCOVERY
-------------------------------------------------------------*/

/*!< define GENERAL MACROS >*/
#define F_CLK   8000000
                                         
#define N_CONT(x)       x*F_CLK                          /*!< x is time time between one flash and the next >*/

/*!< define MACROS for TIMx->SR (STATUS REGISTER)>*/
#define SET_UIF                 (1<<0)                                  /*!< WHEN CNT REACHES N_CNT(X) THE UIF IS SET TO 1 BY HARDWARE >*/                  
#define CHECK_UIF(TIM)          ((TIM->SR)&(1<<0)) == SET_UIF           /*!< CHECK THAT UIF IS 1 >*/


/*!< define MACROS for TIMx->DIER (DMA INTERRUPT ENABLE REGISTER)>*/
#define TIM_DIER_UIE    (1<<0)

/*!< define MACROS  >*/
#define TIM2_EN                 (1<<0)
#define TIM3_EN                 (1<<1)
#define CEN_EN                  (1<<0)

#endif /* STM32_F3X_T_DRIVER_H */
