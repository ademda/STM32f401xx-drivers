/*
 * 002_spi_send_data.c
 *
 *  Created on: Sep 19, 2025
 *      Author: dalya
 */

//SPI2
// PB15 -> MOSI
// PB14 --> MISO
//PB13 -> SCLK
//PB12: NSS
//AF: 5

#include "stm32f401xx.h"
#include <string.h>

SPI_Handle_t SPI2Handle;


void SPI2_GPIOInits(void){
	GPIO_Handle_t SPI2Pins;

	SPI2Pins.pGPIOx = GPIOB;
	SPI2Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI2Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI2Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI2Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPI2Pins);
	//MOSI
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPI2Pins);
	//MISO
	//SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPI2Pins);
	//NSS
	//SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPI2Pins);
}
void SPI2_Inits(void){

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CGF_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_SECOND;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_LSB_FIRST = SPI_MSB;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256; //8MHZ
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_SW;

	SPI_Init(&SPI2Handle);
}
int main(){
	char data[] ="AEllo\n";
	SPI2_GPIOInits();
	SPI2_Inits();

	SPI_PeriphiralControl(SPI2, ENABLE);
	SPI_SendData(SPI2,(uint8_t *) data, strlen(data));
	while(1);
	SPI_PeriphiralControl(SPI2, DISABLE);
	return 0;
}
