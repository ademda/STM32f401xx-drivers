/*
 * stm32f401xx_rcc_driver.c
 *
 *  Created on: Sep 30, 2025
 *      Author: dalya
 */

#include "stm32f401xx_rcc_driver.h"
uint16_t AHB_Prescaler[9] = {2,4,8,16,32,64,128,256,512};
uint16_t APB_Prescaler[4] = {2, 4, 8, 16};

/**
 ***************************************************************
 * @fn		- RCC_GetPllOutputClkValue
 *
 * @brief		- this function computes and returns the PLL output clock frequency
 *
 * @param		- none
 *
 * @return		- uint32_t : PLL output clock frequency in Hz
 *
 * @Note		- currently a stub returning 0; implement PLL calculation if PLL used
 */
uint32_t RCC_GetPllOutputClkValue(){
	return 0;
}
/**
 ***************************************************************
 * @fn		- RCC_GetPClk1Value
 *
 * @brief		- this function returns the APB1 peripheral clock (PCLK1) frequency
 *
 * @param		- none
 *
 * @return		- uint32_t : PCLK1 frequency in Hz
 *
 * @Note		- calculates PCLK1 based on system clock and prescalers in RCC->CFGR
 */
uint32_t RCC_GetPClk1Value(){
	uint8_t pclk_type = (RCC->CFGR >> 2) & (0x3);
	uint32_t sys_clock, pclk1;;
	uint8_t temp,ahb_prescaler, apb1_prescaler;
	if (pclk_type == 0){
		sys_clock = 16000000;
	}else if (pclk_type == 1){
		sys_clock = 8000000;
	}else if (pclk_type == 2){
		sys_clock = RCC_GetPllOutputClkValue();
	}
	temp = (RCC->CFGR >> 4) & (0xF);
	if (temp < 8){
		ahb_prescaler = 1;
	}else {
		ahb_prescaler = AHB_Prescaler[temp-8];
	}
	temp = (RCC->CFGR >> 10) & (0x7);
	if (temp < 4){
		apb1_prescaler = 1;
	}else {
		apb1_prescaler = APB_Prescaler[temp - 4];
	}
	pclk1 = (sys_clock / ahb_prescaler) / apb1_prescaler;
	return pclk1;
}
/**
 ***************************************************************
 * @fn		- RCC_GetPClk2Value
 *
 * @brief		- this function returns the APB2 peripheral clock (PCLK2) frequency
 *
 * @param		- none
 *
 * @return		- uint32_t : PCLK2 frequency in Hz
 *
 * @Note		- calculates PCLK2 based on system clock and prescalers in RCC->CFGR
 */
uint32_t RCC_GetPClk2Value(){
	uint8_t pclk_type = (RCC->CFGR >> 2) & (0x3);
	uint32_t sys_clock, pclk2;;
	uint8_t temp,ahb_prescaler, apb2_prescaler;
	if (pclk_type == 0){
		sys_clock = 16000000;
	}else if (pclk_type == 1){
		sys_clock = 8000000;
	}else if (pclk_type == 2){
		sys_clock = RCC_GetPllOutputClkValue();
	}
	temp = (RCC->CFGR >> 4) & (0xF);
	if (temp < 8){
		ahb_prescaler = 1;
	}else {
		ahb_prescaler = AHB_Prescaler[temp-8];
	}
	temp = (RCC->CFGR >> 13) & (0x7);
	if (temp < 4){
		apb2_prescaler = 1;
	}else {
		apb2_prescaler = APB_Prescaler[temp - 4];
	}
	pclk2 = (sys_clock / ahb_prescaler) / apb2_prescaler;
	return pclk2;
}
