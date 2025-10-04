/*
 * stm32fxx_usart_driver.c
 *
 *  Created on: Sep 29, 2025
 *      Author: dalya
 */



#include "stm32f401xx_usart_driver.h"

uint8_t * debug;

/***************************************************************
 * @fn		- USART_PeriClockControl
 *
 * @brief		- this function enables or disables peripheral clock for the given USART peripheral
 *
 * @param		- pUSARTx : USART peripheral base address (USART1/USART2/USART6)
 * @param		- EnorDi : ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @Note		- none
 */

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		if (pUSARTx == USART1){
			USART1_PCLK_EN();
		}else if (pUSARTx == USART2){
			USART2_PCLK_EN();
		}else if (pUSARTx == USART6){
			USART6_PCLK_EN();
		}
	}
	else if (EnorDi == DISABLE){
		if (pUSARTx == USART1){
			USART1_PCLK_DI();
		}else if (pUSARTx == USART2){
			USART2_PCLK_DI();
		}else if (pUSARTx == USART6){
			USART6_PCLK_DI();
		}
	}
}

/**
 ***************************************************************
 * @fn		- USART_SetBaudRate
 *
 * @brief		- this function configures the USART BRR register to set the desired baud rate
 *
 * @param		- pUSARTx : USART peripheral base address
 * @param		- BaudRate : desired baud rate in bits per second
 *
 * @return		- none
 *
 * @Note		- uses PCLK1 or PCLK2 depending on USART instance and accounts for OVER8 bit
 */

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
  uint32_t PCLKx;
  uint32_t usartdiv;
  uint32_t M_part,F_part;
  uint32_t tempreg=0;
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   PCLKx = RCC_GetPClk2Value();
  }else
  {
	   PCLKx = RCC_GetPClk1Value();
  }
  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	  usartdiv = ((25 * PCLKx) / (4 *BaudRate));
  }
  M_part = usartdiv/100;
  tempreg |= M_part << 4;
  F_part = (usartdiv - (M_part * 100));
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);
   }else
   {
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);
   }
  tempreg |= F_part;
  pUSARTx->BRR = tempreg;
}

/***************************************************************
 * @fn		- USART_Init
 *
 * @brief		- this function initializes the USART peripheral according to the configuration in USART_Handle_t
 *
 * @param		- pUSARTHandle : pointer to USART handle containing configuration
 *
 * @return		- none
 *
 * @Note		- enables peripheral clock and configures CR1/CR2/CR3 and baud rate
 */
void USART_Init(USART_Handle_t *pUSARTHandle){
	uint32_t temp_reg = 0;
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);
	/****************  CR1 Configuration  ***********************/
	// TX RX mode selection
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX){
		//pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RE);
		temp_reg |= 1 << USART_CR1_RE;
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX){
		//pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TE);
		temp_reg |= 1 << USART_CR1_TE;
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX){
		//pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RE);
		//pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TE);
		temp_reg |= (1 << USART_CR1_RE) | (1 << USART_CR1_TE);
	}

	temp_reg |= (pUSARTHandle->USART_Config.USART_WordLength) << USART_CR1_M;
	//Parity selection
	if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN){
		//pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PCE);
		//pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_PS);
		temp_reg |= 1 << USART_CR1_PCE;
		temp_reg &= ~(1 << USART_CR1_PS);
	}
	else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD){
		//pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PCE);
		//pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PS);
		temp_reg |= 1 << USART_CR1_PCE;
		temp_reg |= 1 << USART_CR1_PS;
	}
	else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
		//pUSARTHandle->pUSARTx->CR1 &=  ~(1 << USART_CR1_PCE);
		//pUSARTHandle->pUSARTx->CR1 &=  ~(1 << USART_CR1_PS);
		temp_reg &= ~(1 << USART_CR1_PCE);
		temp_reg &= ~(1 << USART_CR1_PS);
	}

	pUSARTHandle->pUSARTx->CR1 = temp_reg;

	/****************  CR2 Configuration  ***********************/
	temp_reg = 0;
	temp_reg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;
	pUSARTHandle->pUSARTx->CR2 = temp_reg;
	/****************  CR3 Configuration  ************************/
	temp_reg = 0;
	if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS){
		temp_reg |= (1 << USART_CR3_CTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS){
		temp_reg |= (1 << USART_CR3_RTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS){
		temp_reg |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
	}
	pUSARTHandle->pUSARTx->CR3 = temp_reg;
	/**************    Baudrate Configuration  ***************************/
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}
/**
 ***************************************************************
 * @fn		- USART_DeInit
 *
 * @brief		- this function de-initializes (resets) the given USART peripheral registers
 *
 * @param		- pUSARTx : USART peripheral base address
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_DeInit(USART_RegDef_t *pUSARTx){
	if (pUSARTx == USART1){
		USART1_REG_RESET();
	}
	else if (pUSARTx == USART2){
		USART2_REG_RESET();
	}
	else if (pUSARTx == USART6){
		USART6_REG_RESET();
	}
}


/***************************************************************
 * @fn		- USART_SendData
 *
 * @brief		- this function sends data in blocking mode (polling) over USART
 *
 * @param		- pUSARTHandle : pointer to USART handle
 * @param		- pTxBuffer : pointer to data buffer to send
 * @param		- Len : number of bytes to send
 *
 * @return		- none
 *
 * @Note		- this is blocking; waits for TXE and TC flags as needed
 */

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint16_t *pdata;
	for(uint32_t i = 0 ; i < Len; i++)
	{
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				pTxBuffer++;
			}
		}
		else
		{
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
			debug =  (*pTxBuffer  & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}
/***************************************************************
 * @fn		- USART_ReceiveData
 *
 * @brief		- this function receives data in blocking mode (polling) over USART
 *
 * @param		- pUSARTHandle : pointer to USART handle
 * @param		- pRxBuffer : buffer to store received data
 * @param		- Len : number of bytes to receive
 *
 * @return		- none
 *
 * @Note		- this is blocking; waits for RXNE flag before reading DR
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	for(uint32_t i = 0 ; i < Len; i++)
	{
		while (! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x1FF);
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				 pRxBuffer++;
			}
		}
		else
		{
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				 *pRxBuffer =(uint8_t) pUSARTHandle->pUSARTx->DR ;
			}
			else
			{
				 *pRxBuffer = ((uint8_t) pUSARTHandle->pUSARTx->DR) & (uint8_t)0x7F;
			}
			pRxBuffer++;
		}
	}

}
/**
 ***************************************************************
 * @fn		- USART_SendDataIT
 *
 * @brief		- this function starts USART transmission in interrupt mode (non-blocking)
 *
 * @param		- pUSARTHandle : pointer to USART handle
 * @param		- pTxBuffer : data buffer
 * @param		- Len : length to transmit
 *
 * @return		- uint8_t : previous Tx state
 *
 * @Note		- enables TXEIE and TCIE interrupts
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;
	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}
	return txstate;

}
/**
 ***************************************************************
 * @fn		- USART_ReceiveDataIT
 *
 * @brief		- this function starts USART reception in interrupt mode (non-blocking)
 *
 * @param		- pUSARTHandle : pointer to USART handle
 * @param		- pRxBuffer : buffer to store received data
 * @param		- Len : number of bytes to receive
 *
 * @return		- uint8_t : previous Rx state
 *
 * @Note		- enables RXNEIE interrupt
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;
	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}
	return rxstate;
}

/**
 ***************************************************************
 * @fn		- USART_IRQInterruptConfig
 *
 * @brief		- this function enables or disables given IRQ number in NVIC
 *
 * @param		- IRQNumber : IRQ number
 * @param		- EnorDi : ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @Note		- none
 */

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
 * @fn		- USART_IRQPriorityConfig
 *
 * @brief		- this function sets the priority for given IRQ number
 *
 * @param		- IRQNumber : IRQ number
 * @param		- IRQPriority : priority value
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;                  // Determine IPR register
	uint8_t iprx_section = IRQNumber % 4;          // Determine priority field
	uint8_t shift = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx )  |= (IRQPriority << shift); // Set priority
}

/***************************************************************
 * @fn		- USART_IRQHandling
 *
 * @brief		- this function handles USART interrupts and dispatches events to application callback
 *
 * @param		- pUSARTHandle : pointer to USART handle
 *
 * @return		- none
 *
 * @Note		- checks multiple USART flags and invokes USART_ApplicationEventCallback
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t temp1 , temp2, temp3;
	uint16_t *pdata;

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);
	if(temp1 && temp2 )
	{
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			if(! pUSARTHandle->TxLen )
			{
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);
				pUSARTHandle->TxBusyState = USART_READY;
				pUSARTHandle->pTxBuffer = NULL;
				pUSARTHandle->TxLen = 0;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}


	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);
	if(temp1 && temp2 )
	{
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			if(pUSARTHandle->TxLen > 0)
			{
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						pUSARTHandle->pTxBuffer ++;
						pUSARTHandle->pTxBuffer ++;
						pUSARTHandle->TxLen -= 2;
					}
					else
					{
						pUSARTHandle->pTxBuffer ++;
						pUSARTHandle->TxLen--;
					}
				}
				else
				{
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);
					pUSARTHandle->pTxBuffer ++;
					pUSARTHandle->TxLen--;
				}
			}
			if (pUSARTHandle->TxLen == 0 )
			{
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}


	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);
						pUSARTHandle->pRxBuffer ++;
						pUSARTHandle->pRxBuffer ++;
						pUSARTHandle->RxLen -= 2;
					}
					else
					{
						 *(pUSARTHandle->pRxBuffer)  = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						 pUSARTHandle->pRxBuffer ++;
						 pUSARTHandle->RxLen--;
					}
				}
				else
				{
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						 *(pUSARTHandle->pRxBuffer) = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}

					else
					{
						 *(pUSARTHandle->pRxBuffer) = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
					}
					pUSARTHandle->pRxBuffer ++;
					pUSARTHandle->RxLen--;
				}
			}
			if(! pUSARTHandle->RxLen)
			{
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}

	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);
	(void) temp3;
	if(temp1  && temp2 )
	{
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS );
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}


	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);

	if(temp1 && temp2)
	{
		uint32_t temp_read = pUSARTHandle->pUSARTx->SR;
		temp_read = pUSARTHandle->pUSARTx->DR;
		(void)temp_read;
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if(temp1 & ( 1 << USART_SR_NF) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}
}

/**
 ***************************************************************
 * @fn		- USART_PeripheralControl
 *
 * @brief		- this function enables or disables the USART peripheral (UE bit)
 *
 * @param		- pUSARTx : USART peripheral base address
 * @param		- EnOrDi : ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @Note		- none
 */

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){
	if (EnOrDi == ENABLE){
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else if (EnOrDi == DISABLE){
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}
/**
 ***************************************************************
 * @fn		- USART_GetFlagStatus
 *
 * @brief		- this function returns the status of a USART status flag
 *
 * @param		- pUSARTx : USART peripheral base address
 * @param		- FlagName : status flag mask to check
 *
 * @return		- uint8_t : FLAG_SET or FLAG_RESET
 *
 * @Note		- none
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName){
	if (pUSARTx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/**
 ***************************************************************
 * @fn		- USART_ClearFlag
 *
 * @brief		- this function clears the specified status flag in USART SR register
 *
 * @param		- pUSARTx : USART peripheral base address
 * @param		- StatusFlagName : status flag bit position/name
 *
 * @return		- none
 *
 * @Note		- clearing some flags requires specific read/write sequences; this function clears by masking SR
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName){
	pUSARTx->SR &= ~(1 << StatusFlagName);
}
