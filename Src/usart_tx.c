/*
 * usart_tx.c
 *
 *  Created on: Oct 3, 2025
 *      Author: dalya
 */

#include "stm32f401xx.h" 
#include <string.h> 
#include <stdlib.h> 
USART_Handle_t USARTHandle; 
volatile int a; 
uint8_t data[] = "Hello World";
uint8_t rxBuffer[30];
void USART2_GPIOInits(void){ 
    GPIO_Handle_t USART2Pins; 
    USART2Pins.pGPIOx = GPIOA;
    USART2Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN; 
    USART2Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 8;  // AF8 for USART6
    USART2Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; 
    USART2Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; 
    USART2Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST; 
    //TX 
    USART2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;  // PA11
    GPIO_Init(&USART2Pins); 
    //RX 
    USART2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;  // PA12
    GPIO_Init(&USART2Pins); 
}
void USART2_Init(){ 
    USARTHandle.pUSARTx = USART6; 
    USARTHandle.USART_Config.USART_Baud = USART_STD_BAUD_9600; 
    USARTHandle.USART_Config.USART_Mode = USART_MODE_TXRX; 
    USARTHandle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE; 
    USARTHandle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS; 
    USARTHandle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1 ; 
    USARTHandle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE; 
    USART_Init(&USARTHandle); 
} 
int main(void){ 
    USART2_GPIOInits(); 
    USART2_Init(); 
    USART_PeripheralControl(USARTHandle.pUSARTx, ENABLE); 
    USART_SendData(&USARTHandle, data, strlen((char *)data)); 
    while(1){
    	USART_ReceiveData(&USARTHandle, rxBuffer, 4);
    	while(1);
    }
    return 1; 
}
