/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Mar. 29, 2023
 *      Author: altai
 */

#include <stm32f446xx_rcc_driver.h>

/*
 * Peripheral Clock Control and Fetching
 */
uint32_t RCC_GetPLLOutputClk(void){
	uint32_t pllClk = 0;
	return pllClk;
}

uint32_t RCC_GetPCLK1Val(void){
	uint32_t pclk1, sysClk,apb1p;
	uint16_t ahbp;
	uint8_t clksrc, preScale;

	uint16_t AHB_PreScalarArray[8] = {2,4,8,16,128,256,512};
	uint16_t APB1_PreScalarArray[4] = {2,4,8,16};

	// Find Clock Source Rate
	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0)
	{
		sysClk = 16000000;
	} else if(clksrc == 1)
	{
		sysClk = 8000000;
	} else if(clksrc == 2)
	{
		sysClk = RCC_GetPLLOutputClk();
	}

	// Fetching AHB Prescaler value
	preScale = ((RCC->CFGR >> 4) & 0xF);

	if(preScale < 8)
	{
		ahbp = 1;
	} else {
		ahbp = AHB_PreScalarArray[preScale - 8];
	}

	// Fetching APB1 Prescaler Value
	preScale = ((RCC->CFGR >> 10) & 0x7);

	if(preScale < 4)
	{
		apb1p = 1;
	} else {
		apb1p = APB1_PreScalarArray[preScale - 4];
	}

	// Calculate pClk value with prescaler now found
	pclk1 = (sysClk / ahbp) / apb1p;

	return pclk1;
}

uint32_t RCC_GetPCLK2Val(void){
	uint32_t pclk2, sysClk,apb2p;
	uint16_t ahbp;
	uint8_t clksrc, preScale;

	uint16_t AHB_PreScalarArray[8] = {2,4,8,16,128,256,512};
	uint16_t APB2_PreScalarArray[4] = {2,4,8,16};

	// Find Clock Source Rate
	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0)
	{
		sysClk = 16000000;
	} else if(clksrc == 1)
	{
		sysClk = 8000000;
	} else if(clksrc == 2)
	{
		sysClk = RCC_GetPLLOutputClk();
	}

	// Fetching AHB Prescaler value
	preScale = ((RCC->CFGR >> 4) & 0xF);

	if(preScale < 8)
	{
		ahbp = 1;
	} else {
		ahbp = AHB_PreScalarArray[preScale - 8];
	}

	// Fetching APB1 Prescaler Value
	preScale = ((RCC->CFGR >> 10) & 0x7);

	if(preScale < 4)
	{
		apb2p = 1;
	} else {
		apb2p = APB2_PreScalarArray[preScale - 4];
	}

	// Calculate pClk value with prescaler now found
	pclk2 = (sysClk / ahbp) / apb2p;

	return pclk2;
}
