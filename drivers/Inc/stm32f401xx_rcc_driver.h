/*
 * stm32f401xx_rcc_driver.h
 *
 *  Created on: Sep 30, 2025
 *      Author: dalya
 */

#ifndef INC_STM32F401XX_RCC_DRIVER_H_
#define INC_STM32F401XX_RCC_DRIVER_H_

#include "stm32f401xx.h"

uint32_t RCC_GetPllOutputClkValue();
uint32_t RCC_GetPClk1Value();
uint32_t RCC_GetPClk2Value();

#endif /* INC_STM32F401XX_RCC_DRIVER_H_ */
