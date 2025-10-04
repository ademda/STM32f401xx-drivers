/*
 * i2c_master_tx_testing.c
 *
 *  Created on: Oct 2, 2025
 *      Author: dalya
 */

/*
 * PB6 SCL
 * */

/*
 * PB9 SDA
 * */
#include "stm32f401xx.h"
#include <string.h>
#include <stdlib.h>

#define MY_ADDR 0x61
#define OTHER_ADDR 0x62
uint8_t *pRxbuffer;
I2C_Handle_t I2C1Handle;
volatile int a;
uint8_t data[] = "adem";
void I2C1_GPIOInits(void){
	GPIO_Handle_t I2C1Pins;

	I2C1Pins.pGPIOx = GPIOB;
	I2C1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2C1Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2C1Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2C1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2C1Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2C1Pins);

	//SDA
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2C1Pins);

}

void I2C1_Inits(void){
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

int main(){
	I2C1_GPIOInits();
	I2C1_Inits();
	I2C_PeriphiralControl(I2C1Handle.pI2Cx, ENABLE);
	I2C_MasterSendData(&I2C1Handle, data, strlen((char *)data), OTHER_ADDR);
	//I2C_MasterReceiveData(&I2C1Handle, pRxbuffer, 4, OTHER_ADDR);
	while (1){

		if (strcmp((char *)pRxbuffer, "dali") == 0){
			a++;
		}
	}
	return 1;
}
