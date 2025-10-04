/*
 * 001led_toggle.c
 *
 *  Created on: Sep 18, 2025
 *      Author: dalya
 */
#include "stm32f401xx.h"

void delay(void){

	for (uint32_t i=0;i<600000;i++);
}

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
	Gpio_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	Gpio_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	Gpio_button.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	Gpio_button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpio_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	Gpio_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&Gpio_button);
	GPIO_Init(&Gpio_led);
	while (1){
		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)){
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, SET);
		}
		else {
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, RESET);
		}


	}
	return 0;
}
