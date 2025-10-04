/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Sep 15, 2025
 *      Author: dalya
 */

#include "stm32f401xx_gpio_driver.h"

/*
 * 	Clock Enable
 * */

/***************************************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		-this function enable or disable a GPIO Port Clock
 *
 * @param		- GPIO Periph Base address
 * @param		- ENABLE or DISABLE Macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * 	Init DeInit
 * */

/***************************************************************
 * @fn			- GPIO_Init
 *
 * @brief		-this function initializes a GPIO Port
 *
 * @param		- GPIO Periph Base address
 *
 * @return		- none
 *
 * @Note		- none
 */

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle) {
	uint32_t temp = 0;

	// configure the mode
	GPIO_PeriClockControl(pGPIO_Handle->pGPIOx, ENABLE);
	if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// non interrupt mode
		temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode<< (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIO_Handle->pGPIOx->MODER |= temp;

	} else {
		// Rising / falling / Rising and Falling configuration
		if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			EXTI->FTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			EXTI->FTSR &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			EXTI->FTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//EXTICR configuration
		uint8_t temp1 = 0;
		uint8_t temp2 = 0;
		temp1 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 4;
		temp2 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 4;
		//SYSCFG->EXTICR[temp1] |= 1 << temp2;
		SYSCFG_PCLK_EN();
		uint8_t port_code = GPIO_BASEADDR_TO_CODE(pGPIO_Handle->pGPIOx);
		SYSCFG->EXTICR[temp1] = port_code << ( temp2 * 4);

		//IMR Enabling
		EXTI->IMR |= 1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber;
		//interrupt mode
	}
	//2. configure the speed
	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIO_Handle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIO_Handle->pGPIOx->OSPEEDR |= temp;

	//3. configure the pull up pull down settings
	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIO_Handle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIO_Handle->pGPIOx->PUPDR |= temp;


	//4. configure the out put type
	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinOPType << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIO_Handle->pGPIOx->OTYPER &= ~( 0x1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIO_Handle->pGPIOx->OTYPER |= temp;
	// configure the Alternate function mode
	if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		uint8_t temp1 = 0 ;
		uint8_t temp2 = 0;
		temp1 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIO_Handle->pGPIOx->AF[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
		pGPIO_Handle->pGPIOx->AF[temp1] |= pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);
	}
}

/***************************************************************
 * @fn			- GPIO_Init
 *
 * @brief		-this function Deinitializes(Resets) a GPIO Port
 *
 * @param		- GPIO Periph Base address
 *
 * @return		- none
 *
 * @Note		- none
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}
}

/*
 * 	Read Write
 * */

/***************************************************************
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- this function Reads Input Value of a specific pin of Port
 *
 * @param		- GPIO Periph Base address
 * @param		- Pin number
 *
 * @return		- uint8_t
 *
 * @Note		- none
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value = 0;
	value = (uint8_t)(((pGPIOx->IDR) >> PinNumber) & (0x01));
	return value;
}

/***************************************************************
 * @fn			- GPIO_ReadFromInputPort
 *
 * @brief		- this function Reads Input Value of a entire GPIO PORT
 *
 * @param		- GPIO Periph Base address
 *
 *
 * @return		- uint16_t
 *
 * @Note		- none
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value = 0;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/***************************************************************
 * @fn			- GPIO_WriteToOutputPin
 *
 * @brief		- this function Writes a Value to a specific pin of Port
 *
 * @param		- GPIO Periph Base address
 * @param		- Pin number
 * @param		- Output Value
 *
 * @return		- none
 *
 * @Note		- none
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value) {
	if (Value == GPIO_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else{ // here values that differs from 0 are interpreted as 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/***************************************************************
 * @fn			- GPIO_WriteToOutputPort
 *
 * @brief		- this function Writes a Value to an entire GPIO Port
 *
 * @param		- GPIO Periph Base address
 * @param		- Pin number
 * @param		- Output Value
 *
 * @return		- none
 *
 * @Note		- none
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

/***************************************************************
 * @fn			- GPIO_ToggleOutputPin
 *
 * @brief		- this function Toggles a pin of a GPIO Port
 *
 * @param		- GPIO Periph Base address
 * @param		- Pin number
 *
 * @return		- none
 *
 * @Note		- none
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * 	Interrupt
 * */

/***************************************************************
 * @fn			- GPIO_IRQConfig
 *
 * @brief		- this function Configures an Interruption
 *
 * @param		- Number of the interruption in Vector Table
 * @param		- Chosen Priority number
 * @param		- ENABLE  or  DISABLE
 *
 * @return		- none
 *
 * @Note		- none
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDi) {

	if (EnorDi == ENABLE){
		if (IRQNumber <= 31){
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber < 64){
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber < 96){
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else {
		if (IRQNumber <= 31){
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber < 64){
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber < 96){
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
    uint8_t iprx = IRQNumber / 4;                  // Determine IPR register
    uint8_t iprx_section = IRQNumber % 4;          // Determine priority field
    uint8_t shift = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASE_ADDR + iprx )  |= (IRQPriority << shift); // Set prioritypriority
}

/***************************************************************
 * @fn			- GPIO_IRGHandling
 *
 * @brief		- interruption service routine
 *
 * @param		- Pin Number
 *
 *
 * @return		- none
 *
 * @Note		- none
 */

void GPIO_IRQHandling(uint8_t PinNumber) {
	if (EXTI->PR & (1 << PinNumber)){
		EXTI->PR |= (1 << PinNumber);
	}
}
