/*
 * stm32f401xx_spi_driver.h
 *
 *  Created on: Sep 19, 2025
 *      Author: dalya
 */

#ifndef INC_STM32F401XX_SPI_DRIVER_H_
#define INC_STM32F401XX_SPI_DRIVER_H_

#include "stm32f401xx.h"

typedef struct {
	uint8_t SPI_DeviceMode; 	//@SPI_Device_Mode
	uint8_t SPI_BusConfig; 		//@SPI_BusConfig
	uint8_t SPI_SclkSpeed; 		//@SPI_SclkSpeed
	uint8_t SPI_DFF; 			//@SPI_DFF
	uint8_t SPI_CPOL;			//@SPI_CPOL
	uint8_t SPI_CPHA;			//@SPI_CPHA
	uint8_t SPI_SSM;			//@SPI_SSM
	uint8_t SPI_LSB_FIRST ;		//@SPI_LSB_FIRST
}SPI_Config_t;

typedef struct {
	SPI_Config_t SPIConfig;
	SPI_RegDef_t *pSPIx;
	uint8_t *pTxBuffer;	//used to store the data to be sent and it
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;


/*
 * 	@SPI_Device_Mode
 * */

#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * 	@SPI_BusConfig
 * */

#define SPI_BUS_CGF_FD 				1
#define SPI_BUS_CGF_HD 				2
#define SPI_BUS_CGF_SIMPLEX_TXONLY 	3
#define SPI_BUS_CGF_SIMPLEX_RXONLY 	4

/*
 * 	@SPI_SclkSpeed
 * */

#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * 	@SPI_DFF
 * */

#define SPI_DFF_8BITS 	0
#define SPI_DFF_16BITS 	1

/*
 * 	@SPI_CPHA
 * */

#define SPI_CPHA_FIRST		0
#define SPI_CPHA_SECOND 	1

/*
 * 	@SPI_CPOL
 * */

#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW 	0

/*
 * 	@SPI_SSM
 * */

#define SPI_SSM_HW 	0
#define SPI_SSM_SW 	1

/*
 * 	@SPI_LSB_FIRST
 * */

#define SPI_MSB 	0
#define SPI_LSB		1

/***********************************************************
 * 				API supported by this driver
 * *********************************************************/


/*
 * 	Clock Enable
 * */

void SPI_PeriClockControl(SPI_RegDef_t * pSPIx, uint8_t EnorDi);

/*
 * 	Init DeInit
 * */
void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_RegDef_t * pSPIx);

/*
 * Send and Receive data
 * */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *TxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *RxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *TxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *RxBuffer, uint32_t len);

/*
 * IRQ Configuratuni and ISR Handling
 * */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPI_Handle);

/*
 * Other Periphiral Control APIs
 * */
void SPI_PeriphiralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPI_Handle, uint8_t AppEvent);
void SPI_SSIConfig(SPI_Handle_t *pSPI_Handle, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_Handle_t *pSPI_Handle, uint8_t EnorDi);
/***********************************************************
 * 				USEFUL Defines
 * *********************************************************/

#define SPI_READY 		0
#define SPI_BUSY_TX		1
#define SPI_BUSY_RX		2

/*
 * Possible SPI application events
 * */

#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERROR		3


#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */
