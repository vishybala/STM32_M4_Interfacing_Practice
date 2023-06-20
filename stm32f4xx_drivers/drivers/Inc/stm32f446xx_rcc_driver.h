/*
 * smt32f446xx_rcc_driver.h
 *
 *  Created on: Mar. 29, 2023
 *      Author: altai
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Peripheral Clock Control and Fetching
 */
uint32_t RCC_GetPCLK1Val(void);
uint32_t RCC_GetPCLK2Val(void);
uint32_t RCC_GetPLLOutputClk(void);

#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
