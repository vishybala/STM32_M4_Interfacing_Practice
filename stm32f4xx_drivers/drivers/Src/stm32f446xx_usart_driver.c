/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: Mar. 28, 2023
 *      Author: altai
 */

#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"

/*
 * USART Peripheral Clock Control
 */
void USART_ClockCtrl(USART_RegDef_t *pUSARTx, uint8_t EnORDi){
	if (EnORDi == ENABLE){
		if (pUSARTx == USART1){
			USART1_CLK_EN();
		} else if (pUSARTx == USART2){
			USART2_CLK_EN();
		} else if (pUSARTx == USART3){
			USART3_CLK_EN();
		} else if (pUSARTx == UART4){
			UART4_CLK_EN();
		} else if (pUSARTx == UART5){
			UART5_CLK_EN();
		} else if (pUSARTx == USART6){
			USART6_CLK_EN();
		}
	}else if (EnORDi == DISABLE){
		if (pUSARTx == USART1){
			USART1_CLK_DI();
		} else if (pUSARTx == USART2){
			USART2_CLK_DI();
		} else if (pUSARTx == USART3){
			USART3_CLK_DI();
		} else if (pUSARTx == UART4){
			UART4_CLK_DI();
		} else if (pUSARTx == UART5){
			UART5_CLK_DI();
		} else if (pUSARTx == USART6){
			USART6_CLK_DI();
		}
	}
}

/*
 * USART Initialize and De-Initialize
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t Baud){

	// Variable to hold the APB clock
	uint32_t pclk;

	uint32_t usartdiv;

	// Variables to hold Mantissa and Fraction values
	uint32_t mantissa,fractional;

	uint32_t tempreg=0;

	// Fetch APBx clock values according to USART/UARTx bus lines
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		// USART1 and USART6 are on APB2 bus
		pclk = RCC_GetPCLK2Val();
	} else
	{
		// USART2 to UART5 are on APB1 bus
		pclk = RCC_GetPCLK1Val();
	}

	// Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		// OVER8 = 1 , oversampling by 8
		usartdiv = ((25 * pclk) / (2 * Baud));
	} else
	{
		// OVER8 = 0, oversampling by 16
		usartdiv = ((25 * pclk) / (4 * Baud));
	}

	// Calculate the Mantissa part
	mantissa = usartdiv/100;

	// Copy mantissa into tempreg
	tempreg |= mantissa << 4;

	// Extract the fraction part
	fractional = (usartdiv - (mantissa * 100));

	// Calculate the final fractional according to oversampling
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		// OVER8 = 1, oversampling by 8
		fractional = (((fractional * 8) + 50) / 100) & ((uint8_t)0x07);

	} else
	{
		// OVER8 = 0, oversampling by 16
		fractional = (((fractional * 16) + 50) / 100) & ((uint8_t)0x0F);

	}

	// Copy fractional into tempreg
	tempreg |= fractional;

	// Program the mantissa and fractional into the BRR register
	pUSARTx->BRR = tempreg;
}

void USART_Init(USART_Handler_t *pUSARTHandler)
{
	uint32_t tempreg=0;
	USART_ClockCtrl(pUSARTHandler->pUSARTx, ENABLE);

	/*
	 * Configuring USART_CR1 Register
	 */
	// Enable USART TX and RX engines according to the USART_Mode
	if (pUSARTHandler->USART_Config.USART_Mode == USART_MODE_RX)
	{
		// Enable Receiver Bit field
		tempreg |= (1 << USART_CR1_RE);
	} else if (pUSARTHandler->USART_Config.USART_Mode == USART_MODE_TX)
	{
		// Enable Transmitter Bit field
		tempreg |= ( 1 << USART_CR1_TE );
	} else if (pUSARTHandler->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		// Enable both TE and RE
		tempreg |= ( 1 << USART_CR1_RE);
		tempreg |= ( 1 << USART_CR1_TE);
	}

    // Configure Word length
	tempreg |= pUSARTHandler->USART_Config.USART_WordLen << USART_CR1_M ;

	//Configuration of parity control bit fields
	if (pUSARTHandler->USART_Config.USART_Parity == USART_PAR_EVEN)
	{
		// Enable the parity control bit
		tempreg |= ( 1 << USART_CR1_PCE);

		// By default, Parity set to EVEN. Enabling the Parity Bit enough for even parity checking
	}else if (pUSARTHandler->USART_Config.USART_Parity == USART_PAR_ODD )
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 << USART_CR1_PS);
	}

	// Program the configuration into CR1 register
	pUSARTHandler->pUSARTx->CR1 = tempreg;

	/*
	 *  Configuring USART_CR2 Register
	 */
	tempreg=0;

	// Configure number of stop bits inserted at end of USART frame
	tempreg |= (pUSARTHandler->USART_Config.USART_NumStopbits << USART_CR2_STOP);

	// Program the configuration into CR2 register
	pUSARTHandler->pUSARTx->CR2 = tempreg;

	/*
	 *  Configuring USART_CR3 Register
	 */
	tempreg=0;

	// Configure Hardware Flow Control
	if (pUSARTHandler->USART_Config.USART_HWFlowCtrl == USART_HW_FLOW_CTRL_CTS)
	{
		// Enable CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);
	}else if (pUSARTHandler->USART_Config.USART_HWFlowCtrl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	}else if (pUSARTHandler->USART_Config.USART_HWFlowCtrl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= (1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);
	}

	// Program the configuration into CR3 register
	pUSARTHandler->pUSARTx->CR3 = tempreg;

	/*
	 *  Configuring USART_BRR Register
	 */
	//Implement the code to configure the baud rate
	USART_SetBaudRate(pUSARTHandler->pUSARTx, pUSARTHandler->USART_Config.USART_Baud);
}

void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if (pUSARTx == USART1){
		USART1_REG_RESET();
	} else if (pUSARTx == USART2){
		USART2_REG_RESET();
	} else if (pUSARTx == USART3){
		USART3_REG_RESET();
	} else if (pUSARTx == UART4){
		UART4_REG_RESET();
	} else if (pUSARTx == UART5){
		UART5_REG_RESET();
	} else if (pUSARTx == USART6){
		USART6_REG_RESET();
	}
}

/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handler_t *pUSARTHandler, uint8_t *pTXBuffer, uint32_t Len){
	uint16_t *pdata;

	// Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		// Wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandler->pUSARTx, USART_FLAG_TXE));

		// Check the USART_WordLength for 9 bits or 8 bits in a frame
		if(pUSARTHandler->USART_Config.USART_WordLen == USART_WORDLEN_9BIT)
		{
			// If 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTXBuffer;
			pUSARTHandler->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			// Check for Parity configuration
			if(pUSARTHandler->USART_Config.USART_Parity == USART_PAR_OFF)
			{
				// 9Bits of data being sent, with no parity bit
				// 16Bit buffer being used hence increment pTXBuffer twice
				pTXBuffer++;
				pTXBuffer++;
			}
			else
			{
				// 8Bits of data being sent, 9th bit is parity bit
				// Parity bit is auto set by hardware, hence increment pTXBuffer once
				pTXBuffer++;
			}
		}
		else
		{
			// 8BIT transfer of data, load pTXBuffer into DR
			pUSARTHandler->pUSARTx->DR = (*pTXBuffer & (uint8_t)0xFF);

			// Increment the buffer address
			pTXBuffer++;

			// We don't check for parity bit here as if it enabled, the 8th bit will
			// automatically be replaced by parity bit by the hardware. Masking the TXBuffer data
			// to 7bits is redundant.
		}
	}

	// Wait till Transmission is complete (TC Bit in SR)
	while( ! USART_GetFlagStatus(pUSARTHandler->pUSARTx,USART_FLAG_TC));
}


void USART_ReceiveData(USART_Handler_t *pUSARTHandler, uint8_t *pRXBuffer, uint32_t Len){

	// Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		// Wait until RXNE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandler->pUSARTx, USART_FLAG_RXNE));

		// Check the USART_WordLength for 9 bits or 8 bits in a frame
		if(pUSARTHandler->USART_Config.USART_WordLen == USART_WORDLEN_9BIT)
		{
			// Receiving 9bit data in a frame
			// Check for Parity configuration
			if(pUSARTHandler->USART_Config.USART_Parity == USART_PAR_OFF)
			{
				// No parity is used. Read all 9Bits of data
				// Mask 16bit buffer length with 0x01FF to invalidate 10-16th bit
				*((uint16_t*)pRXBuffer) = (pUSARTHandler->pUSARTx->DR & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRXBuffer++;
				pRXBuffer++;
			}
			else
			{
				// Parity enabled, read 8Bits of data, 9th is Parity
				*pRXBuffer = (pUSARTHandler->pUSARTx->DR  & (uint8_t)0xFF);

				//Increment the pRxBuffer
				pRXBuffer++;
			}
		}
		else
		{
			// Receiving 8bit data in a frame
			// Check for Parity configuration
			if(pUSARTHandler->USART_Config.USART_Parity == USART_PAR_OFF)
			{
				// No parity, all 8Bits are user data
				// Load 8Bits from DR to RXBuffer
				*pRXBuffer = pUSARTHandler->pUSARTx->DR;
			} else
			{
				// Parity is used, 7Bits are user data, 8th is parity bit
				// Mask 8bit buffer length with 0x7F to invalidate 8th parity bit
				*pRXBuffer = (uint8_t) (pUSARTHandler->pUSARTx->DR & (uint8_t)0x7F);
			}
			// Increment the RXBuffer
			pRXBuffer++;
		}
	}
}

uint8_t USART_SendData_IT(USART_Handler_t *pUSARTHandler, uint8_t *pTXBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandler->TXState;

	if(txstate != USART_BUSY_TX)
	{
		pUSARTHandler->TXLen = Len;
		pUSARTHandler->pTXBuffer = pTXBuffer;
		pUSARTHandler->TXState = USART_BUSY_TX;

		// Enable interrupt for TXE
		pUSARTHandler->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		// Enable interrupt for TC
		pUSARTHandler->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}
	return txstate;
}

uint8_t USART_ReceiveData_IT(USART_Handler_t *pUSARTHandler, uint8_t *pRXBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandler->RXState;

	if(rxstate != USART_BUSY_RX)
	{
		pUSARTHandler->RXLen = Len;
		pUSARTHandler->pRXBuffer = pRXBuffer;
		pUSARTHandler->RXState = USART_BUSY_RX;

		// Enable interrupt for RXNE
		pUSARTHandler->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}
	return rxstate;
}


/*
 * USART Interrupt Configuration and ISR Handling
 */
void USART_IRQEnableConfig(uint8_t IRQNumber, uint8_t EnORDi)
{
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

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx_reg = IRQNumber / 4;
	uint8_t iprx_sec = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_sec) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_ADDR + (iprx_reg)) |= (IRQPriority << shift_amount);
}

void USART_IRQHandling(USART_Handler_t *pUSARTHandler)
{
	uint32_t temp1 , temp2, temp3;
	uint16_t *pdata;

	/*
	 *  TC FLAG triggered interrupt
	 */
	// Check the state of TC bit in the SR
	temp1 = pUSARTHandler->pUSARTx->SR & ( 1 << USART_SR_TC);

	 // Check the state of TCIE bit
	temp2 = pUSARTHandler->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2)
	{
		// Interrupt indeed caused by TC

		// If TXLen = 0, transmission complete. Close connection and call AppCB
		if (pUSARTHandler->TXState == USART_BUSY_TX)
		{
			if(!pUSARTHandler->TXLen)
			{
				// Clear the TC flag
				pUSARTHandler->pUSARTx->SR &= ~(1 << USART_SR_TC);

				// Clear the TCIE control bit (disable interrupt flag for TC)
				pUSARTHandler->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);

				// Reset the application state
				pUSARTHandler->TXState = USART_READY;

				// Reset TXBuffer address to NULL
				pUSARTHandler->pTXBuffer = NULL;

				// Reset the TXLen to zero
				pUSARTHandler->TXLen = 0;

				// Application call back for transmission complete
				USART_ApplicationEventCB(pUSARTHandler, USART_EVT_TX_CMPL);
			}
		}
	}

	/*
	 *  TXE FLAG triggered interrupt
	 */
	// Check the state of TXE bit in the SR
	temp1 = pUSARTHandler->pUSARTx->SR & ( 1 << USART_SR_TXE);

	 // Check the state of TXEIE bit
	temp2 = pUSARTHandler->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);
	if(temp1 && temp2)
	{
		// Interrupt indeed caused by TXE
		if(pUSARTHandler->TXState == USART_BUSY_TX)
		{
			// Keep sending data until TXLen reaches to zero
			if(pUSARTHandler->TXLen > 0)
			{
				// Check the USART_WordLength for 9 bits or 8 bits in a frame
				if(pUSARTHandler->USART_Config.USART_WordLen == USART_WORDLEN_9BIT)
				{
					// If 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandler->pTXBuffer;
					pUSARTHandler->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					// Check for Parity configuration
					if(pUSARTHandler->USART_Config.USART_Parity == USART_PAR_OFF)
					{
						// 9Bits of data being sent, with no parity bit
						// 16Bit buffer being used hence increment pTXBuffer twice
						pUSARTHandler->pTXBuffer++;
						pUSARTHandler->pTXBuffer++;
						pUSARTHandler->TXLen--;
						pUSARTHandler->TXLen--;
					}
					else
					{
						// 8Bits of data being sent, 9th bit is parity bit
						// Parity bit is auto set by hardware, hence increment pTXBuffer once
						pUSARTHandler->pTXBuffer++;
						pUSARTHandler->TXLen--;
					}
				}
				else
				{
					// 8BIT transfer of data, load pTXBuffer into DR
					pUSARTHandler->pUSARTx->DR = (*pUSARTHandler->pTXBuffer & (uint8_t)0xFF);

					// Increment the buffer address
					pUSARTHandler->pTXBuffer++;
					pUSARTHandler->TXLen--;

					// We don't check for parity bit here as if it enabled, the 8th bit will
					// automatically be replaced by parity bit by the hardware. Masking the TXBuffer data
					// to 7bits is redundant.
				}
			}

			// If TXLen is equal to zero
			if (pUSARTHandler->TXLen == 0 )
			{
				// Clear the TXEIE bit (disable interrupt for TXE flag)
				pUSARTHandler->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}

		// We don't need to clear the state as TC flag is set when TXE is complete.
		// The TC interrupt handler finishes up the TXE reset and closing connection.
	}

	/*
	 *  RXNE FLAG triggered interrupt
	 */
	// Check the state of RXNE bit in the SR
	temp1 = pUSARTHandler->pUSARTx->SR & ( 1 << USART_SR_RXNE);

	 // Check the state of RXNEIE bit
	temp2 = pUSARTHandler->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);
	if(temp1 && temp2)
	{
		// Interrupt indeed caused by RXNE
		if(pUSARTHandler->RXState == USART_BUSY_RX)
		{
			// Keep receiving data until RXLen reaches to zero
			if(pUSARTHandler->RXLen > 0)
			{
				if(pUSARTHandler->USART_Config.USART_WordLen == USART_WORDLEN_9BIT)
				{
					// Receiving 9bit data in a frame
					// Check for Parity configuration
					if(pUSARTHandler->USART_Config.USART_Parity == USART_PAR_OFF)
					{
						// No parity is used. Read all 9Bits of data
						// Mask 16bit buffer length with 0x01FF to invalidate 10-16th bit
						*((uint16_t*)pUSARTHandler->pRXBuffer) = (pUSARTHandler->pUSARTx->DR & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandler->pRXBuffer++;
						pUSARTHandler->pRXBuffer++;
						pUSARTHandler->RXLen--;
						pUSARTHandler->RXLen--;
					}
					else
					{
						// Parity enabled, read 8Bits of data, 9th is Parity
						*pUSARTHandler->pRXBuffer = (pUSARTHandler->pUSARTx->DR  & (uint8_t)0xFF);

						//Increment the pRxBuffer
						pUSARTHandler->pRXBuffer++;
						pUSARTHandler->RXLen--;
					}
				}
				else
				{
					// Receiving 8bit data in a frame
					// Check for Parity configuration
					if(pUSARTHandler->USART_Config.USART_Parity == USART_PAR_OFF)
					{
						// No parity, all 8Bits are user data
						// Load 8Bits from DR to RXBuffer
						*pUSARTHandler->pRXBuffer = pUSARTHandler->pUSARTx->DR;
					} else
					{
						// Parity is used, 7Bits are user data, 8th is parity bit
						// Mask 8bit buffer length with 0x7F to invalidate 8th parity bit
						*pUSARTHandler->pRXBuffer = (uint8_t) (pUSARTHandler->pUSARTx->DR & (uint8_t)0x7F);
					}
					// Increment the RXBuffer
					pUSARTHandler->pRXBuffer++;
					pUSARTHandler->RXLen--;
				}
			}

			// If RXLen is equal to zero
			if (pUSARTHandler->RXLen == 0 )
			{
				// Clear the RXNEIE bit (disable interrupt for RXNE flag )
				pUSARTHandler->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
				pUSARTHandler->RXState = USART_READY;

				// Application call back for reception complete
				USART_ApplicationEventCB(pUSARTHandler, USART_EVT_RX_CMPL);
			}
		}
	}

	/*
	 *  CTS FLAG triggered interrupt
	 *  (Unavailable for UART4 and UART5
	 */
	// Check the state of CTS bit in the SR
	temp1 = pUSARTHandler->pUSARTx->SR & ( 1 << USART_SR_CTS);

	 // Check the state of CTSE bit in CR3 register
	temp2 = pUSARTHandler->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	 // Check the state of CTSIE bit in CR3 register
	temp3 = pUSARTHandler->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);

	if((temp1 && temp2) & (temp1 && temp3))
	{
		// Clear the CTS flag in SR
		pUSARTHandler->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		// Application call back informing interrupt thrown by CTS
		USART_ApplicationEventCB(pUSARTHandler, USART_EVT_CTS);
	}

	/*
	 *  IDLE Detection Flag triggered interrupt
	 */
	// Check the state of IDLE bit in the SR
	temp1 = pUSARTHandler->pUSARTx->SR & ( 1 << USART_SR_IDLE);

	 // Check the state of CTSE bit in CR1 register
	temp2 = pUSARTHandler->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);

	if(temp1 && temp2)
	{
		// Cleared by read to SR followed by read to DR
		temp3 = pUSARTHandler->pUSARTx->SR;
		temp3 = pUSARTHandler->pUSARTx->DR;

		// NOTE: ONLY IMPLEMENTED BECAUSE UDEMY COURSE HAS IT LIKE THIS
		pUSARTHandler->pUSARTx->SR &= ~(1 << USART_SR_IDLE);

		// Application call back informing of the idle line
		USART_ApplicationEventCB(pUSARTHandler, USART_ERR_IDLE);
	}

	/*
	 *  OVERRUN Detection Flag triggered interrupt
	 */
	// Check the state of ORE bit in the SR
	temp1 = pUSARTHandler->pUSARTx->SR & ( 1 << USART_SR_ORE);

	 // Check the state of CTSE bit in CR1 register
	temp2 = pUSARTHandler->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);

	if(temp1 && temp2)
	{
		// Inform the application that a overrun error has occurred. Implement API in application
		// layer to handle the clearing of the bit.
		USART_ApplicationEventCB(pUSARTHandler, USART_ERR_OVR);
	}

	/*
	 *  ERROR Detection Flag triggered interrupt
	 *  NOTE: Only triggered when multi-buffer mode is used.
	 */
	// Check if Error interrupt bit is enabled in CR3
	temp1 = pUSARTHandler->pUSARTx->CR3 & ( 1 << USART_CR3_EIE);

	if(temp1)
	{
		temp2 = pUSARTHandler->pUSARTx->SR;
		if(temp1 & (1 << USART_SR_FE))
		{
			// Set by hardware when a de-synchronization, excessive noise or a break character
			// is detected. It is cleared by a software sequence (an read to the USART_SR register
			// followed by a read to the USART_DR register).

			// Inform the application that a framing error has occurred. Implement API in application
			// layer to handle the clearing of the bit.
			USART_ApplicationEventCB(pUSARTHandler, USART_ERR_FE);
		}

		if(temp1 & (1 << USART_SR_NF))
		{
			// Set by hardware when noise is detected on a received frame. It is cleared by a
			// software sequence (an read to the USART_SR register followed by a read to the
			// USART_DR register).

			// Inform the application that a noise error has occurred. Implement API in application
			// layer to handle the clearing of the bit.
			USART_ApplicationEventCB(pUSARTHandler, USART_ERR_NF);
		}

		if(temp1 & (1 << USART_SR_ORE))
		{
			// Inform the application that a overrun error has occurred. Implement API in application
			// layer to handle the clearing of the bit.
			USART_ApplicationEventCB(pUSARTHandler, USART_ERR_OVR);
		}
	}

}

/*
 * Other Peripheral Control APIs
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	if (pUSARTx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	pUSARTx->SR &= ~(FlagName);
}

void USART_PeripheralConfig(USART_RegDef_t *pUSARTx, uint8_t EnORDi)
{
	if (EnORDi == ENABLE){
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	} else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/*
 * Application Callback
 */
__weak void USART_ApplicationEventCB(USART_Handler_t *pUSARTHandler, uint8_t AppEv)
{
	/*
	 * Weak Implementation. Application that uses USART may override this
	 * function to add features for function callback from Interrupt enabled
	 * USART request handlers.
	 */

}
