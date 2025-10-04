/*
 * stm32f041xx_spi_driver.c
 *
 *  Created on: Sep 19, 2025
 *      Author: dalya
 */

#include "stm32f401xx_spi_driver.h"


/***************************************************************
 * @fn			- SPI_PeriClockControl
 *
 * @brief		- this function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param		- pSPIx : SPI peripheral base address (SPI1/SPI2/SPI3/SPI4)
 * @param		- EnorDi : ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_PeriClockControl(SPI_RegDef_t * pSPIx, uint8_t EnorDi){
	if (EnorDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_DI();
		}
	}
}

/**
 ***************************************************************
 * @fn			- SPI_Init
 *
 * @brief		- this function initializes the SPI peripheral according to the specified parameters in SPI_Handle_t
 *
 * @param		- pSPI_Handle : pointer to SPI_Handle_t structure that contains configuration information for SPI module
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_Init(SPI_Handle_t *pSPI_Handle){
	uint32_t temp_reg = 0;

	SPI_PeriClockControl(pSPI_Handle->pSPIx, ENABLE);
	//Slave master select
	temp_reg |= pSPI_Handle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	//Full duplex half or simplex rx only
	if (pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CGF_FD){
		temp_reg &= ~(1 << 15);
	}
	else if (pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CGF_HD){
		temp_reg |= (1 <<15 );
	}
	else if (pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CGF_SIMPLEX_RXONLY){
		temp_reg |= (1 << 10);
		temp_reg &= ~(1 << 15);
	}
	//polarity
	temp_reg |= (pSPI_Handle->SPIConfig.SPI_CPOL << SPI_CR1_CPHA);
	//phase
	temp_reg |= (pSPI_Handle->SPIConfig.SPI_CPHA << SPI_CR1_CPOL);
	//clock speed
	temp_reg |= (pSPI_Handle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR0);
	//LSB o MSB first sent
	temp_reg |= (pSPI_Handle->SPIConfig.SPI_LSB_FIRST << SPI_CR_LBSFIRST);
	//data frame format(8 16 bits)
	temp_reg |= (pSPI_Handle->SPIConfig.SPI_DFF << SPI_CR1_DFF);
	//software nss
	temp_reg |= (pSPI_Handle->SPIConfig.SPI_SSM << SPI_CR1_SSM);
	//pulling SSi high if SSM mode is set
	pSPI_Handle->pSPIx->CR1 = temp_reg;
	SPI_SSOEConfig(pSPI_Handle, ENABLE);
	SPI_SSIConfig(pSPI_Handle, ENABLE);
}
/**
 ***************************************************************
 * @fn			- SPI_DeInit
 *
 * @brief		- this function de-initializes (resets) the given SPI peripheral registers
 *
 * @param		- pSPIx : SPI peripheral base address (SPI1/SPI2/SPI3/SPI4)
 *
 * @return		- none
 *
 * @Note		- none
 */

void SPI_DeInit(SPI_RegDef_t * pSPIx){
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	} else if (pSPIx == SPI4) {
		SPI4_REG_RESET();
	}
}

/***************************************************************
 * @fn			- SPI_PeriphiralControl
 *
 * @brief		- this function enables or disables the SPI peripheral (SPE bit)
 *
 * @param		- pSPIx : SPI peripheral base address
 * @param		- EnorDi : ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @Note		- none
 */

void SPI_PeriphiralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
/***************************************************************
 * @fn			- SPI_SSIConfig
 *
 * @brief		- this function configures the SSI bit when using software slave management (SSM)
 *
 * @param		- pSPI_Handle : pointer to SPI handle
 * @param		- EnorDi : ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @Note		- only has effect when SSM mode (software slave management) is selected
 */
void SPI_SSIConfig(SPI_Handle_t *pSPI_Handle, uint8_t EnorDi){
	if (pSPI_Handle->SPIConfig.SPI_SSM == SPI_SSM_SW){
		if (EnorDi == ENABLE){
			pSPI_Handle->pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		}
		else {
			pSPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
	}
}
/***************************************************************
 * @fn			- SPI_SSOEConfig
 *
 * @brief		- this function configures the SSOE bit (SS output enable) for hardware NSS management
 *
 * @param		- pSPI_Handle : pointer to SPI handle
 * @param		- EnorDi : ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @Note		- only has effect when hardware NSS management is used
 */
void SPI_SSOEConfig(SPI_Handle_t *pSPI_Handle, uint8_t EnorDi){
	if (pSPI_Handle->SPIConfig.SPI_SSM == SPI_SSM_HW){
		if (EnorDi == ENABLE){
			pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
		}
		else {
			pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
		}
	}
}
/***************************************************************
 * @fn			- SPI_SendData
 *
 * @brief		- this function sends data in blocking mode (polling) over SPI
 *
 * @param		- pSPIx : SPI peripheral base address
 * @param		- TxBuffer : pointer to data buffer to send
 * @param		- len : length of data in bytes (for 16-bit DFF, len should be even)
 *
 * @return		- none
 *
 * @Note		- this is a blocking call; waits for TXE flag before writing data to DR
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *TxBuffer, uint32_t len){
	uint8_t dataformat_value = (pSPIx->CR1 >> SPI_CR1_DFF) & (0x01);
	while (len > 0){
		while (!((pSPIx->SR) & (1 << SPI_SR_TXE)));
		if (dataformat_value == SPI_DFF_8BITS){
			pSPIx->DR = *((uint8_t *)TxBuffer);
			TxBuffer++;
			len--;
		}
		else{
			pSPIx->DR = *((uint16_t *)TxBuffer);
			(uint16_t *)TxBuffer++;
			len -= 2;
		}
	}
}

/***************************************************************
 * @fn			- SPI_ReceiveData
 *
 * @brief		- this function receives data in blocking mode (polling) over SPI
 *
 * @param		- pSPIx : SPI peripheral base address
 * @param		- RxBuffer : pointer to buffer where received data will be stored
 * @param		- len : length of data in bytes (for 16-bit DFF, len should be even)
 *
 * @return		- none
 *
 * @Note		- this is a blocking call; waits for RXNE flag before reading data from DR
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *RxBuffer, uint32_t len){
	uint8_t dataformat_value = (pSPIx->CR1 >> SPI_CR1_DFF) & (0x01);
	while (len > 0){
		while (!((pSPIx->SR) & (1 << SPI_SR_RXNE)));
		if (dataformat_value == SPI_DFF_8BITS){
			*RxBuffer = (uint8_t) pSPIx->DR;
			RxBuffer++;
			len--;
		}
		else{
			*((uint16_t *)RxBuffer) = pSPIx->DR;
			(uint16_t *)RxBuffer++;
			len -= 2;
		}
	}
}

/***************************************************************
 * @fn			- SPI_SendDataIT
 *
 * @brief		- this function starts SPI transmission in interrupt mode (non-blocking)
 *
 * @param		- pSPIHandle : pointer to SPI handle
 * @param		- TxBuffer : pointer to data buffer to send
 * @param		- len : length of data in bytes
 *
 * @return		- uint8_t : current TxState (SPI_READY or SPI_BUSY_TX)
 *
 * @Note		- enables TXEIE interrupt; application will be notified via callback when transmission completes
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *TxBuffer, uint32_t len){
	if (pSPIHandle->TxState != SPI_BUSY_TX){
		pSPIHandle->pTxBuffer = TxBuffer;
		pSPIHandle->TxLen = len;
		pSPIHandle->TxState = SPI_BUSY_TX;
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return pSPIHandle->TxState;
}
/***************************************************************
 * @fn			- SPI_ReceiveDataIT
 *
 * @brief		- this function starts SPI reception in interrupt mode (non-blocking)
 *
 * @param		- pSPIHandle : pointer to SPI handle
 * @param		- RxBuffer : pointer to buffer where received data will be stored
 * @param		- len : length of data in bytes
 *
 * @return		- uint8_t : current RxState (SPI_READY or SPI_BUSY_RX)
 *
 * @Note		- enables RXNEIE interrupt; application will be notified via callback when reception completes
 */

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *RxBuffer, uint32_t len){
	if (pSPIHandle->RxState != SPI_BUSY_RX){
		pSPIHandle->pRxBuffer = RxBuffer;
		pSPIHandle->RxLen = len;
		pSPIHandle->RxState = SPI_BUSY_RX;
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return pSPIHandle->RxState;
}

/***************************************************************
 * @fn			- SPI_IRQInterruptConfig
 *
 * @brief		- this function configures IRQ in NVIC (enable/disable)
 *
 * @param		- IRQNumber : IRQ number as present in the vector table
 * @param		- EnorDi : ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @Note		- none
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

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
/**
 ***************************************************************
 * @fn			- SPI_IRQPriorityConfig
 *
 * @brief		- this function sets the priority for an IRQ number
 *
 * @param		- IRQNumber : IRQ number
 * @param		- IRQPriority : priority to set
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;                  // Determine IPR register
	uint8_t iprx_section = IRQNumber % 4;          // Determine priority field
	uint8_t shift = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx )  |= (IRQPriority << shift); // Set priority
}
/**
 ***************************************************************
 * @fn			- SPI_IRQHandling
 *
 * @brief		- this function handles SPI interrupt events (TXE, RXNE, OVR)
 *
 * @param		- pSPI_Handle : pointer to SPI handle structure
 *
 * @return		- none
 *
 * @Note		- calls application callback on events
 */

void SPI_IRQHandling(SPI_Handle_t *pSPI_Handle){
	uint8_t dataformat_value = (pSPI_Handle->pSPIx->CR1 >> SPI_CR1_DFF) & (0x01);
	uint8_t sr_txe_status = (pSPI_Handle->pSPIx->SR >> SPI_SR_TXE) & (0x01);
	uint8_t cr2_txeie_status = (pSPI_Handle->pSPIx->CR2 >> SPI_CR2_TXEIE) & (0x01);
	uint8_t sr_rxne_status = (pSPI_Handle->pSPIx->SR >> SPI_SR_RXNE) & (0x01);
	uint8_t cr2_rxneie_status = (pSPI_Handle->pSPIx->CR2 >> SPI_CR2_RXNEIE) & (0x01);
	uint8_t sr_ovrerror_status = (pSPI_Handle->pSPIx->SR >> SPI_SR_OVR) & (0x01);
	uint8_t cr2_errorie_status = (pSPI_Handle->pSPIx->CR2 >> SPI_CR2_ERRIE) & (0x01);
	if (sr_txe_status && cr2_txeie_status){
		while (pSPI_Handle->TxLen > 0){
			if (dataformat_value == SPI_DFF_8BITS){
				pSPI_Handle->pSPIx->DR = *((uint8_t *)pSPI_Handle->pTxBuffer);
				pSPI_Handle->pTxBuffer++;
				pSPI_Handle->TxLen--;
			}
			else{
				pSPI_Handle->pSPIx->DR = *((uint16_t *)pSPI_Handle->pTxBuffer);
				(uint16_t *)pSPI_Handle->pTxBuffer++;
				pSPI_Handle->TxLen -= 2;
			}
			if (! pSPI_Handle->TxLen){
				//preventing interrupts for txe
				pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
				pSPI_Handle->pTxBuffer = NULL;
				pSPI_Handle->TxState = SPI_READY;
				SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_TX_CMPLT);
			}
		}
	}
	else if (sr_rxne_status && cr2_rxneie_status){
		while (pSPI_Handle->RxLen > 0){
			if (dataformat_value == SPI_DFF_8BITS){
				*pSPI_Handle->pRxBuffer = (uint8_t) pSPI_Handle->pSPIx->DR;
				pSPI_Handle->pRxBuffer++;
				pSPI_Handle->RxLen--;
			}
			else{
				*((uint16_t *)pSPI_Handle->pRxBuffer) = pSPI_Handle->pSPIx->DR;
				(uint16_t *)pSPI_Handle->pRxBuffer++;
				pSPI_Handle->RxLen -= 2;
			}
			if (! pSPI_Handle->RxLen){
				//preventing interrupts for txe
				pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
				pSPI_Handle->pRxBuffer = NULL;
				pSPI_Handle->RxState = SPI_READY;
				SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_RX_CMPLT);
			}
		}
	}
	else if (sr_ovrerror_status && cr2_errorie_status){
		uint8_t temp;
		if (pSPI_Handle->TxState != SPI_BUSY_TX){
			temp = pSPI_Handle->pSPIx->DR;
			temp = pSPI_Handle->pSPIx->SR;
		}
		(void)temp;
		SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_OVR_ERROR);
	}

}
/***************************************************************
 * @fn			- SPI_ApplicationEventCallback
 *
 * @brief		- weak application callback that the driver calls to notify about SPI events
 *
 * @param		- pSPI_Handle : pointer to SPI handle
 * @param		- AppEvent : event id (e.g., SPI_EVENT_TX_CMPLT, SPI_EVENT_RX_CMPLT, SPI_EVENT_OVR_ERROR)
 *
 * @return		- none
 *
 * @Note		- implement this function in application code to handle events; driver provides a weak stub
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPI_Handle, uint8_t AppEvent){

}

