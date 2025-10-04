/*
 * stm32f401xx_i2c_driver.c
 *
 *  Created on: Sep 22, 2025
 *      Author: dalya
 */

#include "stm32f401xx_i2c_driver.h"


/***********************************Helper Functions*******************************/
/**
 * @fn		- I2C_GenerateStartCondition
 *
 * @brief		- sets the START bit to generate a START condition on the I2C bus
 *
 * @param		- pI2Cx : I2C peripheral base address
 * @return		- none
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
/**
 * @fn		- I2C_GenerateStopCondition
 *
 * @brief		- sets the STOP bit to generate a STOP condition on the I2C bus
 *
 * @param		- pI2Cx : I2C peripheral base address
 * @return		- none
 */
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
/**
 * @fn		- I2C_MasterHandlerRXNEIT
 *
 * @brief		- helper to handle RXNE interrupt in master receive mode
 *
 * @param		- pI2C_Handle : pointer to I2C handle
 * @return		- none
 */
static void I2C_MasterHandlerRXNEIT(I2C_Handle_t *pI2C_Handle);
/**
 * @fn		- I2C_MasterHandlerTXEIT
 *
 * @brief		- helper to handle TXE interrupt in master transmit mode
 *
 * @param		- pI2C_Handle : pointer to I2C handle
 * @return		- none
 */
static void I2C_MasterHandlerTXEIT(I2C_Handle_t *pI2C_Handle);

/**
 * @fn		- I2C_SendSlaveAddr
 *
 * @brief		- helper to send the 7-bit slave address with R/W bit
 *
 * @param		- pI2Cx : I2C peripheral base address
 * @param		- slave_addr : 7-bit slave address
 * @param		- read_write : I2C_READ or I2C_WRITE
 * @return		- none
 */
static void I2C_SendSlaveAddr(I2C_RegDef_t *pI2Cx, uint16_t slave_addr, uint8_t read_write);
/**
 * @fn		- I2C_Get_FlagStatus
 *
 * @brief		- returns status of an I2C flag from SR1
 *
 * @param		- pI2Cx : I2C peripheral base address
 * @param		- FlagName : flag mask to check
 * @return		- uint8_t : FLAG_SET or FLAG_RESET
 */
uint8_t I2C_Get_FlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
/**
 * @fn		- I2C_Clear_AddrFlag
 *
 * @brief		- clears the ADDR flag by reading SR1 and SR2; handles master/slave differences
 *
 * @param		- pI2CHandle : pointer to I2C handle
 * @return		- none
 */
static void I2C_Clear_AddrFlag(I2C_Handle_t *pI2CHandle);

/**************************************Implementation*****************************/
static void I2C_Clear_AddrFlag(I2C_Handle_t *pI2CHandle){
	uint32_t dummy_read ;
	dummy_read = pI2CHandle->pI2Cx->SR1;
	dummy_read = pI2CHandle->pI2Cx->SR2;
	(void)dummy_read;

	if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
		// master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			if (pI2CHandle->RxSize == 1){
				//disable ack
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else {
		//slave mode
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}


static void I2C_SendSlaveAddr(I2C_RegDef_t *pI2Cx, uint16_t slave_addr, uint8_t read_write){
	slave_addr = slave_addr << 1;
	if (read_write == I2C_WRITE){
		slave_addr &= ~(1); // r/nw bit
	}
	else {
		slave_addr |= 1;
	}
	pI2Cx->DR = slave_addr;
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

uint8_t I2C_Get_FlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
	if (pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/***************************************************************
 * @fn		- I2C_CloseSendData
 *
 * @brief		- disables I2C transmit-related interrupts and resets transmit state
 *
 * @param		- pI2C_Handle : pointer to I2C handle
 *
 * @return		- none
 *
 * @Note		- used to clean up after interrupt-driven master transmit
 */
void I2C_CloseSendData(I2C_Handle_t *pI2C_Handle){
	//disable ITBUFEN
	pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//disable ITEVFEN
	pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2C_Handle->TxRxState = I2C_READY;
	pI2C_Handle->pTxBuffer = NULL;
	pI2C_Handle->TxLen = 0;
}
/***************************************************************
 * @fn		- I2C_CloseReceiveData
 *
 * @brief		- disables I2C receive-related interrupts and resets receive state
 *
 * @param		- pI2C_Handle : pointer to I2C handle
 *
 * @return		- none
 *
 * @Note		- used to clean up after interrupt-driven master receive
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2C_Handle){
	//disable ITBUFEN
	pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//disable ITEVFEN
	pI2C_Handle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2C_Handle->TxRxState = I2C_READY;
	pI2C_Handle->pRxBuffer = NULL;
	pI2C_Handle->RxLen = 0;
	pI2C_Handle->RxSize = 0;
	if (pI2C_Handle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
		pI2C_Handle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
}
/************************************************************************************/


/***************************************************************
 * @fn		- I2C_PeriClockControl
 *
 * @brief		- enables or disables peripheral clock for the specified I2C peripheral
 *
 * @param		- pI2Cx : I2C peripheral base address (I2C1/I2C2/I2C3)
 * @param		- EnorDi : ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @Note		- none
 */

void I2C_PeriClockControl(I2C_RegDef_t * pI2Cx, uint8_t EnorDi){
	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}

/***************************************************************
 * @fn		- I2C_PeriphiralControl
 *
 * @brief		- enables or disables the I2C peripheral (PE bit)
 *
 * @param		- pI2Cx : I2C peripheral base address
 * @param		- EnorDi : ENABLE or DISABLE macro
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_PeriphiralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/***************************************************************
 * @fn		- I2C_DeInit
 *
 * @brief		- initializes the specified I2C peripheral registers
 *
 * @param		- pI2C_Handle : I2C Handle struct
 *
 * @return		- none
 *
 * @Note		- none
 */


void I2C_Init(I2C_Handle_t *pI2C_Handle){
	uint32_t temp_reg = 0;
	I2C_PeriClockControl(pI2C_Handle->pI2Cx, ENABLE);
	//Configure CR1
	temp_reg |= (pI2C_Handle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2C_Handle->pI2Cx->CR1 = temp_reg;
	temp_reg = 0;
	//Configure CR2
	temp_reg |= RCC_GetPClk1Value(RCC)/1000000U;
	pI2C_Handle->pI2Cx->CR2 = temp_reg;
	temp_reg = 0;
	//Configure OAR1
	//address in case of slave mode
	temp_reg |= (pI2C_Handle->I2C_Config.I2C_DeviceAddress << 1);
	temp_reg |= (1<< 14);
	pI2C_Handle->pI2Cx->OAR1 = temp_reg;
	//CCR calculation
	uint16_t ccr_value = 0;
	temp_reg = 0;
	if (pI2C_Handle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//standard mode
		ccr_value = (RCC_GetPClk1Value())/(2 * pI2C_Handle->I2C_Config.I2C_SCLSpeed);
		temp_reg |= (ccr_value ) & (0xFFF);
	}
	else{
		//fast mode
		temp_reg |= (1 << 15);
		temp_reg |= (pI2C_Handle->I2C_Config.I2C_FMDutyCycle << 14);
		if (pI2C_Handle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2 ){
			ccr_value = (RCC_GetPClk1Value())/(3 * pI2C_Handle->I2C_Config.I2C_SCLSpeed);
		}
		else if (pI2C_Handle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9 ){
			ccr_value = (RCC_GetPClk1Value())/(25 * pI2C_Handle->I2C_Config.I2C_SCLSpeed);
		}
		temp_reg |= (ccr_value ) & (0xFFF);
	}
	pI2C_Handle->pI2Cx->CCR = temp_reg;
	//TRISE configuration
	if (pI2C_Handle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//strandard mode

		temp_reg = (RCC_GetPClk1Value() /1000000U) + 1;
	}
	else{
		//fast mode
		temp_reg = ((RCC_GetPClk1Value() *300)/1000000000U) + 1;
	}
	pI2C_Handle->pI2Cx->TRISE = temp_reg & 0x3F ;
}
/***************************************************************
 * @fn		- I2C_DeInit
 *
 * @brief		- de-initializes (resets) the specified I2C peripheral registers
 *
 * @param		- pI2Cx : I2C peripheral base address
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_DeInit(I2C_RegDef_t * pI2Cx){
	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
	} else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}

/*
 * I2C Send/Receive data slave mode
 * */
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data){
	pI2C->DR = data;
}

/***************************************************************
 * @fn		- I2C_SlaveReceiveData
 *
 * @brief		- reads a received byte from the I2C data register (slave mode)
 *
 * @param		- pI2C : I2C peripheral base address
 *
 * @return		- uint8_t : received byte
 *
 * @Note		- caller in slave event should use this to fetch data
 */

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C){
	return (uint8_t) pI2C->DR;
}
/***************************************************************
 * @fn		- I2C_MasterSendData
 *
 * @brief		- master blocking send to a slave device
 *
 * @param		- pI2CHandle : pointer to I2C handle
 *
 * @param		- pTxbuffer : data buffer
 * 
 * @param 		- len : length to send
 * 
 * @param		- slave_addr : 7-bit slave address
 * 
 * @return		- uint8_t : received byte
 *
 * @Note		- caller in slave event should use this to fetch data
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slave_addr){
	//Genertae start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	//check SB flag in SR1
	while (! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));

	//send address to slave with r/nw set to 0
	I2C_SendSlaveAddr(pI2CHandle->pI2Cx, slave_addr, I2C_WRITE);
	//Confirm address ny checking addr flag in SR
	while (! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));
	//clear ADDR flag
	I2C_Clear_AddrFlag(pI2CHandle);
	//send data until len = 0
	while (len > 0) {
		while (! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		len --;
		pTxbuffer++ ;
	}
	//wait until BTF = 1 and TXE = 1 and than generate stop bit
	while (! (I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG) && (I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG))));
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

/**
 ***************************************************************
 * @fn		- I2C_MasterReceiveData
 *
 * @brief		- master blocking receive from a slave device
 *
 * @param		- pI2CHandle : pointer to I2C handle
 * @param		- pRxbuffer : buffer to store received data
 * @param		- len : number of bytes to receive
 * @param		- slave_addr : 7-bit slave address
 *
 * @return		- none
 *
 * @Note		- generates start, sends address, reads bytes and generates stop when done
 */

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t slave_addr){
	// Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	// confirm that start generation is completet by checking the SB flag in the SR1
	while (! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));
	// send slave addr + r/nw
	I2C_SendSlaveAddr(pI2CHandle->pI2Cx, slave_addr, I2C_READ);
	//check address phase is completed by checking ADDR flag
	while (! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));

	//reading only one byte procedure
	if (len == 1){
		//disable acking
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
		//clear addr flag
		I2C_Clear_AddrFlag(pI2CHandle);
		// wait until RXNE becomes 1
		while (! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));
		//generate stop condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		//read data in to buffer
		*pRxbuffer = pI2CHandle->pI2Cx->DR;
		return ;
	}
	//procedure if data lenth > 1
	if (len > 1){
		//clear the ADDR flag
		I2C_Clear_AddrFlag(pI2CHandle);
		//read the data until len becaomes zero
		for (uint32_t i = len; i>0; i--){
			//wait until RXNE becomes 1
			while (! I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));
			if (i == 2){ //if last 2 bytes are remaining
				//clear the ack bit
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
				//generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			//read the data from data register in to buffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;
			//increment the buffer address
			pRxbuffer++;
		}
	}

	//re_enable ACKing
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	return ;
}



/*
 * enabling /disabling interrupts in processor
 * */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
/***************************************************************
 * @fn		- I2C_IRQPriorityConfig
 *
 * @brief		- sets NVIC priority for given I2C IRQ number
 *
 * @param		- IRQNumber : IRQ number
 * @param		- IRQPriority : priority value
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;                  // Determine IPR register
	uint8_t iprx_section = IRQNumber % 4;          // Determine priority field
	uint8_t shift = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx )  |= (IRQPriority << shift); // Set priority
}
/***************************************************************
 * @fn		- I2C_MasterHandlerTXEIT
 *
 * @brief		- helper to handle TXE interrupt in interrupt-driven master transmit
 *
 * @param		- pI2C_Handle : pointer to I2C handle
 *
 * @return		- none
 */
void I2C_MasterHandlerTXEIT(I2C_Handle_t *pI2C_Handle){
	if (pI2C_Handle->TxLen > 0 ){
		pI2C_Handle->pI2Cx->DR = *(pI2C_Handle->pTxBuffer);
		pI2C_Handle->TxLen--;
		pI2C_Handle->pTxBuffer++;
	}
}
/***************************************************************
 * @fn		- I2C_MasterHandlerRXNEIT
 *
 * @brief		- helper to handle RXNE interrupt in interrupt-driven master receive
 *
 * @param		- pI2C_Handle : pointer to I2C handle
 *
 * @return		- none
 */
void I2C_MasterHandlerRXNEIT(I2C_Handle_t *pI2C_Handle){
	if (pI2C_Handle->RxSize == 1){
		pI2C_Handle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

		//read data in to buffer
		*(pI2C_Handle->pRxBuffer) = pI2C_Handle->pI2Cx->DR;
		pI2C_Handle->RxLen--;
	}
	else if(pI2C_Handle->RxSize > 1){
		if (pI2C_Handle->RxLen == 2){
			//lear ack bit
			pI2C_Handle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
		}
		//read DR
		*(pI2C_Handle->pRxBuffer) = pI2C_Handle->pI2Cx->DR;
		pI2C_Handle->RxLen--;
		pI2C_Handle->pRxBuffer++;
	}
	if (pI2C_Handle->RxLen == 0){
		//Close connection
		//generate stop
		if (pI2C_Handle->Sr == I2C_DISABLE_SR){
			I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
		}
		//close I2C Rx
		I2C_CloseReceiveData(pI2C_Handle);
		//notify app
		I2C_ApplicationEventCallback(pI2C_Handle, I2C_EV_RX_CPLT);
	}
}
/***************************************************************
 * @fn		- I2C_EV_IRQHandling
 *
 * @brief		- I2C event interrupt handler (PB events: SB, ADDR, BTF, STOPF, TxE, RxNE)
 *
 * @param		- pI2C_Handle : pointer to I2C handle
 *
 * @return		- none
 *
 * @Note		- dispatches to appropriate helper routines and application callbacks
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2C_Handle){

	uint8_t temp1, temp2, temp3;
	temp1 = (pI2C_Handle->pI2Cx->CR2 << I2C_CR2_ITBUFEN ) & (0x01);
	temp2 = (pI2C_Handle->pI2Cx->CR2 << I2C_CR2_ITEVTEN) & (0x01);
	temp3 = (pI2C_Handle->pI2Cx->CR2 << I2C_SR1_SB) & (0x01);

	if (temp1 && temp3){
		//send byte case:flag only applicable in master mode
		if (pI2C_Handle->TxRxState == I2C_BUSY_IN_TX){
			I2C_SendSlaveAddr(pI2C_Handle->pI2Cx, pI2C_Handle->DevAddr, I2C_WRITE);
		}
		else if (pI2C_Handle->TxRxState == I2C_BUSY_IN_RX){
			I2C_SendSlaveAddr(pI2C_Handle->pI2Cx, pI2C_Handle->DevAddr, I2C_READ);
		}
	}

	temp3 = (pI2C_Handle->pI2Cx->CR2 << I2C_SR1_ADDR) & (0x01);

	if (temp1 && temp3){
		//Master mode:Address is sent
		//slave mode: address received matches with own address
		I2C_Clear_AddrFlag(pI2C_Handle);
	}

	temp3 = (pI2C_Handle->pI2Cx->CR2 << I2C_SR1_BTF) & (0x01);

	if (temp1 && temp3){
		//Byte transfer finished
		if (pI2C_Handle->TxRxState == I2C_BUSY_IN_TX){
			if ((pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_TxE)) && (pI2C_Handle->TxLen == 0)){
				//BTF , TXE = 1
				//Generate stop condition
				if (pI2C_Handle->Sr == I2C_DISABLE_SR){
					I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
				}
				//reset all members of handle stcut
				I2C_CloseSendData(pI2C_Handle);
				//notify application
				I2C_ApplicationEventCallback(pI2C_Handle, I2C_EV_TX_CMPLT);
			}
		}
		else if (pI2C_Handle->TxRxState == I2C_BUSY_IN_RX){
			;
		}

	}

	temp3 = (pI2C_Handle->pI2Cx->CR2 << I2C_SR1_STOPF) & (0x01);

	if (temp1 && temp3){
		//slave mode: stop detected

		pI2C_Handle->pI2Cx->CR1 |= 0x0000;
		I2C_ApplicationEventCallback(pI2C_Handle, I2C_EV_STOPF);
	}

	temp3 = (pI2C_Handle->pI2Cx->CR2 << I2C_SR1_TxE) & (0x01);

	if (temp1 && temp2 && temp3){
		if (pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			//data regiter(in transmit mode) empty
			if (pI2C_Handle->TxRxState == I2C_BUSY_IN_TX){
				I2C_MasterHandlerTXEIT(pI2C_Handle);
			}
		}
		else {
			//slave
			if (pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)){
				//transmitter mode
				I2C_ApplicationEventCallback(pI2C_Handle, I2C_EV_DATA_REQ);
			}
		}
	}

	temp3 = (pI2C_Handle->pI2Cx->CR2 << I2C_SR1_RxNE) & (0x01);

	if (temp1 && temp2 && temp3){
		//data regiter(in receive mode) empty
		if (!(pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))){
			if (pI2C_Handle->TxRxState == I2C_BUSY_IN_RX){
				I2C_MasterHandlerRXNEIT(pI2C_Handle);
			}
		}
		else {
			//slave mode
			if (!(pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))){
				I2C_ApplicationEventCallback(pI2C_Handle, I2C_EV_DATA_RCV);
			}
		}
	}

}
/***************************************************************
 * @fn		- I2C_ER_IRQHandling
 *
 * @brief		- I2C error interrupt handler (BERR, ARLO, AF, OVR, TIMEOUT)
 *
 * @param		- pI2CHandle : pointer to I2C handle
 *
 * @return		- none
 *
 * @Note		- clears error flags and notifies application via callback
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}


	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}

/***************************************************************
 * @fn		- I2C_MasterSendDataIT
 *
 * @brief		- start master transmit in interrupt mode (non-blocking)
 *
 * @param		- pI2CHandle : pointer to I2C handle
 * @param		- pTxbuffer : data buffer
 * @param		- len : length to send
 * @param		- slave_addr : 7-bit slave address
 * @param		- Sr : whether to generate STOP or not (Sr)
 *
 * @return		- uint8_t : previous busystate
 *
 * @Note		- enables event and buffer and error interrupts
 */

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slave_addr, uint8_t Sr){

	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = slave_addr;
		pI2CHandle->Sr = Sr ;
		//start generation
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//ITBUFEN enable
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		//ITEVEN enable
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		//ITERREN enable
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busystate;
}
/***************************************************************
 * @fn		- I2C_MasterReceiveDataIT
 *
 * @brief		- start master receive in interrupt mode (non-blocking)
 *
 * @param		- pI2CHandle : pointer to I2C handle
 * @param		- pRxbuffer : buffer to store received data
 * @param		- len : number of bytes to receive
 * @param		- slave_addr : 7-bit slave address
 * @param		- Sr : whether to generate STOP or not (Sr)
 *
 * @return		- uint8_t : previous busystate
 *
 * @Note		- enables event and buffer and error interrupts
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t slave_addr, uint8_t Sr){
	uint8_t busystate = pI2CHandle->TxRxState;
	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){
		*(pI2CHandle->pRxBuffer) =pI2CHandle->pI2Cx->DR;
		(pI2CHandle->RxLen)++;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = len;
		pI2CHandle->DevAddr = slave_addr;
		(pI2CHandle->Sr)++;
		//start generation
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//ITBUFEN enable
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		//ITEVEN enable
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		//ITERREN enable
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busystate;
}
/**
 ***************************************************************
 * @fn		- I2C_ApplicationEventCallback
 *
 * @brief		- weak application callback invoked by driver on events (TX complete, RX complete, errors)
 *
 * @param		- pI2C_Handle : pointer to I2C handle
 * @param		- AppEvent : application event identifier
 * @return		- none
 *
 * @Note		- implement this function in application code to handle events; driver provides a weak stub
 */
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t AppEvent){}

