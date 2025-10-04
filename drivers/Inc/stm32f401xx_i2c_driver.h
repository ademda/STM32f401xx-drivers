/*
 * stm32f401xx_i2c_driver.h
 *
 *  Created on: Sep 22, 2025
 *      Author: dalya
 */

#ifndef INC_STM32F401XX_I2C_DRIVER_H_
#define INC_STM32F401XX_I2C_DRIVER_H_

#include "stm32f401xx.h"

typedef struct {
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;


typedef struct {
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint8_t DevAddr;
	uint32_t RxSize;
	uint8_t Sr;
}I2C_Handle_t;

/*
 * I2C Reapeted state condtion
 * */


#define I2C_ENABLE_SR	SET
#define I2C_DISABLE_SR	RESET

/*
 * I2C application states
 * */

#define I2C_READY			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2

/*
 * @I2C Read Write
 * */

#define I2C_READ 	1
#define I2C_WRITE 	0

/*
 * @I2C_SPEED
 * */
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM4K	400000
#define I2C_SCL_SPEED_FM2K	200000

/*
 * @I2C_AckControl
 * */

#define I2C_ACK_ENABLE		1
#define I2C_AK_DISABLE		0


/*
 * @I2C_FMDutyCycle
 * */

#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * I2C Related status Flags
 * */
#define I2C_SB_FLAG (1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG (1 << I2C_SR1_ADDR)
#define I2C_TXE_FLAG (1 << I2C_SR1_TxE)
#define I2C_RXNE_FLAG (1 << I2C_SR1_RxNE)
#define I2C_BTF_FLAG (1 << I2C_SR1_BTF)
#define I2C_STOPF_FLAG (1 << I2C_SR1_STOPF)
#define I2C_BERR_FLAG (1 << I2C_SR1_BERR)
#define I2C_ARLO_FLAG (1 << I2C_SR1_ARLO)
#define I2C_AF_FLAG (1 << I2C_SR1_AF)
#define I2C_OVR_FLAG (1 << I2C_SR1_OVR)
#define I2C_TIMEOUT_FLAG (1 << I2C_SR1_TIMEOUT)

/*
 * I2C application event macros
 * */

#define I2C_EV_RX_CPLT		0
#define I2C_EV_STOPF 		1
#define I2C_EV_TX_CMPLT		2
#define I2C_ERROR_BERR		3
#define I2C_ERROR_ARLO		4
#define I2C_ERROR_AF		5
#define I2C_ERROR_OVR		6
#define I2C_ERROR_TIMEOUT	7
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV 	9

/***********************************************************
 * 				API supported by this driver
 * *********************************************************/


/*
 * 	Clock Enable
 * */

void I2C_PeriClockControl(I2C_RegDef_t * pI2Cx, uint8_t EnorDi);
void I2C_PeriphiralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
/*
 * 	Init DeInit
 * */
void I2C_Init(I2C_Handle_t *pI2C_Handle);
void I2C_DeInit(I2C_RegDef_t * pI2Cx);

/*
 * Send and Receive data
 * */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slave_addr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t slave_addr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slave_addr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t slave_addr, uint8_t Sr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

/*
 * IRQ Configuratuni and ISR Handling
 * */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2C_Handle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2C_Handle);

/*
 * IT Case: close send/receive data
 * */

void I2C_CloseSendData(I2C_Handle_t *pI2C_Handle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2C_Handle);

/*
 * Other Periphiral Control APIs
 * */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t AppEvent);



#endif /* INC_STM32F401XX_I2C_DRIVER_H_ */
