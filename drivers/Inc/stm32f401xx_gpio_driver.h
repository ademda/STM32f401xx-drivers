/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Sep 15, 2025
 *      Author: dalya
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401xx.h"

/*
 * @GPIO_PIN_MODES
 * GPIO pin modes
 */
#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT 	1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT 	4
#define GPIO_MODE_IT_RT 	5
#define GPIO_MODE_IT_RFT 	6

/*
 * @GPIO_OUTPUT_TYPES
 * GPIO pin OUTPUT type
 */

#define GPIO_OP_TYPE_PP 	0
#define GPIO_OP_TYPE_OD 	1

/*
 * @GPIO_SPEEDS
 * GPIO pin OUTPUT Speeds
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PU_PD
 * GPIO pin PullUp PullDown config
 */

#define GPIO_NO_PUPD 		0
#define GPIO_PIN_PU 		1
#define GPIO_PIN_PD			2

/*
 * @GPIO_PIN_NUMBER
 * GPIO pin Numbers
 */

#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * Handle for GPIO pin
 */


typedef struct {
	uint8_t GPIO_PinNumber;        	//values from @GPIO_PIN_NUMBER
	uint8_t GPIO_PinMode;  			//values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			//values from @GPIO_SPEEDS
	uint8_t GPIO_PinPuPdControl;	//values from @GPIO_PU_PD
	uint8_t GPIO_PinOPType;			//values from @GPIO_OUTPUT_TYPES
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t * pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/***********************************************************
 * 				API supported by this driver
 * *********************************************************/

/*
 * 	Clock Enable
 * */

void GPIO_PeriClockControl(GPIO_RegDef_t * pGPIOx, uint8_t EnorDi);

/*
 * 	Init DeInit
 * */
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t * pGPIOx);

/*
 * 	Read Write
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t * pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t * pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t * pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t * pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t * pGPIOx, uint8_t PinNumber);

/*
 * 	Interrupt
 * */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */
