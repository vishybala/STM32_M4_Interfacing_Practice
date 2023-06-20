/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Feb. 24, 2023
 *      Author: altai
 */

#include "stm32f446xx_spi_driver.h"

/*
 * SPI Interrupt Handling Methods Definitions
 */
static void SPI_TXE_ITHandler(SPI_Handler_t *pSPIHandler);
static void SPI_RXNE_ITHandler(SPI_Handler_t *pSPIHandler);
static void SPI_OVR_ITHandler(SPI_Handler_t *pSPIHandler);

/*
 * SPI Peripheral Clock Control
 */
void SPI_ClockCtrl(SPI_RegDef_t *pSPIx, uint8_t EnORDi){
	if (EnORDi == ENABLE){
		if (pSPIx == SPI1){
			SPI1_CLK_EN();
		}else if (pSPIx == SPI2){
			SPI2_CLK_EN();
		}else if (pSPIx == SPI3){
			SPI3_CLK_EN();
		}else if (pSPIx == SPI4){
			SPI4_CLK_EN();
		}
	}else if (EnORDi == DISABLE){
		if (pSPIx == SPI1){
			SPI1_CLK_DI();
		}else if (pSPIx == SPI2){
			SPI2_CLK_DI();
		}else if (pSPIx == SPI3){
			SPI3_CLK_DI();
		}else if (pSPIx == SPI4){
			SPI4_CLK_DI();
		}
	}
}

/*
 * SPI Initialize and De-Initialize
 */
void SPI_Init(SPI_Handler_t *pSPIHandler)
{
	// Configuring SPI_CR1 Register
	uint32_t tempreg = 0;

	SPI_ClockCtrl(pSPIHandler->pSPIx, ENABLE);

	// 1. Device Mode
	tempreg |= pSPIHandler->SPI_Config.SPI_DeviceMode << 2;

	// 2. Bus Config
	if(pSPIHandler->SPI_Config.SPI_BusConfig == SPI_BUS_FULLDUPLX){
		// BIDI Clear
		tempreg &= ~(1 << 15);
	} else if (pSPIHandler->SPI_Config.SPI_BusConfig == SPI_BUS_HALFDUPLX){
		// BIDI Set
		tempreg |= (1 << 15);
	} else if (pSPIHandler->SPI_Config.SPI_BusConfig == SPI_BUS_SIMPLX_RX){
		// BIDI Clear
		tempreg &= ~(1 << 15);
		// RXONLY Set
		tempreg |= (1 << 10);
	}

	// 3. Clock Speed (Baud Rate)
	tempreg |= pSPIHandler->SPI_Config.SPI_ClkSpd << 3;

	// 4. Data Format
	tempreg |= pSPIHandler->SPI_Config.SPI_DFF << 11;

	// 5. Clock Polarity
	tempreg |= pSPIHandler->SPI_Config.SPI_CPOL << 1;

	// 6. Clock Phase
	tempreg |= pSPIHandler->SPI_Config.SPI_CPHA << 0;

	// 7. Software Slave Management
	tempreg |= pSPIHandler->SPI_Config.SPI_SSM << 9;

	pSPIHandler->pSPIx->CR1 = tempreg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1){
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2){
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3){
		SPI3_REG_RESET();
	} else if (pSPIx == SPI4){
		SPI4_REG_RESET();
	}
}

/*
 * Data Send and Receive
 */
/******************************************************************************
 *
 * Function			- SPI_SendData
 *
 * Brief			- Sends Data in the TXBuffer to Slave
 *
 * Notes			- Blocking Call as processor waits on this function to complete
 * 					sending specified 'Len' bytes on TXBuffer
 *******************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		// 1. Wait for TX to be empty (TXE Set)
		while (!(pSPIx->SR & SPI_TXE_FLAG));

		// 2. Check DFF bit
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			// 16 Bit Depth
			pSPIx->DR = *((uint16_t*)pTXBuffer);
			Len--;
			Len--;
			(uint16_t*)pTXBuffer++;
		} else {
			// 8 Bit Depth
			pSPIx->DR = *pTXBuffer;
			Len--;
			pTXBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		// 1. Wait for RX to be empty (RXNE Set)
		while (!(pSPIx->SR & SPI_RXNE_FLAG));

		// 2. Check DFF bit
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 Bit Depth
			*((uint16_t*)pRXBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRXBuffer++;
		} else {
			// 8 Bit Depth
			*pRXBuffer = pSPIx->DR;
			Len--;
			pRXBuffer++;
		}
	}
}

/******************************************************************************
 *
 * Function			- SPI_SendData_IT
 *
 * Brief			- Sends Data in the TXBuffer to Slave using Interrupts
 *
 * Notes			- Non Blocking as this API saves state and pointers and
 * 						lets the ISR handler send the data by setting TXEIE bit
 *******************************************************************************/
uint8_t SPI_SendData_IT(SPI_Handler_t *pSPIHandler, uint8_t *pTXBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandler->TXState;

	if(state != SPI_BUSY_TX)
	{
		// 1. Save TX buffer address and Len info into global variable
		pSPIHandler->pTXBuffer = pTXBuffer;
		pSPIHandler->TXLen = Len;

		// 2. Mark SPI state as Busy Transmitting to block others
		pSPIHandler->TXState = SPI_BUSY_TX;

		// 3. Enable TXEIE control bit to get interrupt when TXE flag set in SR
		pSPIHandler->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// 4. Data transmission handled by ISR Handler
	}
	return state;
}

uint8_t SPI_ReceiveData_IT(SPI_Handler_t *pSPIHandler, uint8_t *pRXBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandler->RXState;

	if(state != SPI_BUSY_RX)
	{
		// 1. Save TX buffer address and Len info into global variable
		pSPIHandler->pRXBuffer = pRXBuffer;
		pSPIHandler->RXLen = Len;

		// 2. Mark SPI state as Busy Transmitting to block others
		pSPIHandler->RXState = SPI_BUSY_RX;

		// 3. Enable TXEIE control bit to get interrupt when TXE flag set in SR
		pSPIHandler->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// 4. Data transmission handled by ISR Handler
	}
	return state;
}

/*
 * SPI Interrupt Configuration and ISR Handling
 */
void SPI_IRQEnableConfig(uint8_t IRQNumber, uint8_t EnORDi){
	if(EnORDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// Program ISER0 Register of NVIC
			*NVIC_ISER0_ADDR |= (1 << IRQNumber);

		} else if(IRQNumber > 31 && IRQNumber <= 64)
		{
			// Program ISER1 Register of NVIC
			*NVIC_ISER1_ADDR |= (1 << (IRQNumber % 32));

		} else if(IRQNumber > 64 && IRQNumber <= 96)
		{
			// Program ISER2 Register of NVIC
			*NVIC_ISER2_ADDR |= (1 << (IRQNumber % 64));

		}
	} else
	{
		if(IRQNumber <= 31)
		{
			// Program ICER0 Register of NVIC
			*NVIC_ICER0_ADDR |= (1 << IRQNumber);

		} else if(IRQNumber > 31 && IRQNumber <= 64)
		{
			// Program ICER1 Register of NVIC
			*NVIC_ICER1_ADDR |= (1 << (IRQNumber % 32));

		} else if(IRQNumber > 64 && IRQNumber <= 96)
		{
			// Program ICER2 Register of NVIC
			*NVIC_ICER2_ADDR |= (1 << (IRQNumber % 64));

		}
	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx_reg = IRQNumber / 4;
	uint8_t iprx_sec = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_sec) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_ADDR + (iprx_reg)) |= (IRQPriority << shift_amount);
}

void SPI_IRQHandling(SPI_Handler_t *pSPIHandler)
{
	uint8_t temp1, temp2;
	 // Check for TXE
	temp1 = pSPIHandler->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandler->pSPIx->SR & (1 << SPI_CR2_TXEIE);

	if(temp1 & temp2)
	{
		// Handle TXE
		SPI_TXE_ITHandler(pSPIHandler);
	}

	 // Check for RXNE
	temp1 = pSPIHandler->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandler->pSPIx->SR & (1 << SPI_CR2_RXNEIE);

	if(temp1 & temp2)
	{
		// Handle RXNE
		SPI_RXNE_ITHandler(pSPIHandler);
	}

	 // Check for OVR
	temp1 = pSPIHandler->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandler->pSPIx->SR & (1 << SPI_CR2_ERRIE);

	if(temp1 & temp2)
	{
		// Handle OVR
		SPI_OVR_ITHandler(pSPIHandler);
	}

}
/*
 * SPI Interrupt Handling Methods Implementation
 */

static void SPI_TXE_ITHandler(SPI_Handler_t *pSPIHandler)
{
	// 2. Check DFF bit
	if(pSPIHandler->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		// 16 Bit Depth
		pSPIHandler->pSPIx->DR = *((uint16_t*)pSPIHandler->pTXBuffer);
		pSPIHandler->TXLen--;
		pSPIHandler->TXLen--;
		(uint16_t*)pSPIHandler->pTXBuffer++;
	} else {
		// 8 Bit Depth
		pSPIHandler->pSPIx->DR = *pSPIHandler->pTXBuffer;
		pSPIHandler->TXLen--;
		pSPIHandler->pTXBuffer++;
	}

	if(! pSPIHandler->TXLen)
	{
		// Disable TXEIE flag to stop interrupts after transmission
		// Also reset SPIHandler TX members
		SPI_CloseTransmission(pSPIHandler);

		// Inform the application that the TX interrupt has been handled
		SPI_ApplicationEventCB(pSPIHandler, SPI_EVT_TX_CMPL);
	}
}

static void SPI_RXNE_ITHandler(SPI_Handler_t *pSPIHandler)
{
	// 2. Check DFF bit
	if(pSPIHandler->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		// 16 Bit Depth
		pSPIHandler->pSPIx->DR = *((uint16_t*)pSPIHandler->pRXBuffer);
		pSPIHandler->RXLen--;
		pSPIHandler->RXLen--;
		(uint16_t*)pSPIHandler->pRXBuffer++;
	} else {
		// 8 Bit Depth
		pSPIHandler->pSPIx->DR = *pSPIHandler->pRXBuffer;
		pSPIHandler->RXLen--;
		pSPIHandler->pRXBuffer++;
	}

	if(! pSPIHandler->RXLen)
	{
		// Disable TXEIE flag to stop interrupts after transmission
		// Also reset SPIHandler TX members
		SPI_CloseReception(pSPIHandler);

		// Inform the application that the TX interrupt has been handled
		SPI_ApplicationEventCB(pSPIHandler, SPI_EVT_RX_CMPL);
	}
}

static void SPI_OVR_ITHandler(SPI_Handler_t *pSPIHandler)
{

	// Clear the OVR Flag
	if(pSPIHandler->TXState != SPI_BUSY_TX)
	{
		SPI_ClearOVRFlag(pSPIHandler);
	}

	// Inform the application that the TX interrupt has been handled
	SPI_ApplicationEventCB(pSPIHandler, SPI_EVT_OVR_ERR);
}

/*
 * Other Peripheral Control APIs
 */

void SPI_PeripheralConfig(SPI_RegDef_t *pSPIx, uint8_t EnORDi)
{
	if (EnORDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnORDi)
{
	if (EnORDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnORDi)
{
	if (EnORDi == ENABLE){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void SPI_ClearOVRFlag(SPI_Handler_t *pSPIHandler)
{
	uint8_t temp;
	temp = pSPIHandler->pSPIx->DR;
	temp = pSPIHandler->pSPIx->SR;
	(void)temp;
}

void SPI_CloseTransmission(SPI_Handler_t *pSPIHandler)
{
	pSPIHandler->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandler->pTXBuffer = NULL;
	pSPIHandler->TXLen = 0;
	pSPIHandler->TXState = SPI_READY;
}

void SPI_CloseReception(SPI_Handler_t *pSPIHandler)
{
	pSPIHandler->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandler->pRXBuffer = NULL;
	pSPIHandler->RXLen = 0;
	pSPIHandler->RXState = SPI_READY;
}

/*
 * Application Callback
 */
__weak void SPI_ApplicationEventCB(SPI_Handler_t *pSPIHandler, uint8_t AppEv)
{
	/*
	 * Weak Implementation. Application that uses SPI may override this
	 * function to add features for function callback from Interrupt enabled
	 * SPI request handlers.
	 */

}
