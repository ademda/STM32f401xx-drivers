/*
 * 002_let_button_interrupt.c
 *
 *  Created on: Sep 18, 2025
 *      Author: dalya
 */
#include "stm32f401xx.h"


int main(void){
	GPIO_Handle_t Gpio_led;
	Gpio_led.pGPIOx = GPIOA;
	Gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	Gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	Gpio_led.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	Gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	Gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_Handle_t Gpio_button;
	Gpio_button.pGPIOx = GPIOC;
	Gpio_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	Gpio_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	Gpio_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	Gpio_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&Gpio_button);
	GPIO_Init(&Gpio_led);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI10_15, ENABLE);
	//GPIO_IRQPriorityConfig(IRQ_NO_EXTI10_15,6);
	while (1){
	}
	return 0;
}

void EXTI15_10_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, SET);
}
