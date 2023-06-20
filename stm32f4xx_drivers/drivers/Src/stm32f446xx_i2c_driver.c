/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Mar. 16, 2023
 *      Author: altai
 */

#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_rcc_driver.h"

/*
 *  Event Related Interrupt Handlers
 */
static void I2C_SB_EVHandler(I2C_Handler_t *pI2CHandler);
static void I2C_ADDR_EVHandler(I2C_Handler_t *pI2CHandler);
static void I2C_BTF_EVHandler(I2C_Handler_t *pI2CHandler);
static void I2C_STOPF_EVHandler(I2C_Handler_t *pI2CHandler);
static void I2C_RXNE_EVHandler(I2C_Handler_t *pI2CHandler);
static void I2C_TXE_EVHandler(I2C_Handler_t *pI2CHandler);

/*
 *  Error Related Interrupt Handlers
 */
static void I2C_BUS_ERRHandler(I2C_Handler_t *pI2CHandler);
static void I2C_ARLO_ERRHandler(I2C_Handler_t *pI2CHandler);
static void I2C_ACK_ERRHandler(I2C_Handler_t *pI2CHandler);
static void I2C_OVR_ERRHandler(I2C_Handler_t *pI2CHandler);
static void I2C_TMEOUT_ERRHandler(I2C_Handler_t *pI2CHandler);

static void I2C_GenStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddrPhase_RW(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t ReadORWrte){
	if(ReadORWrte == WRITE){
		SlaveAddr = SlaveAddr << 1;
		SlaveAddr &= ~(1);
		pI2Cx->DR = SlaveAddr;
	} else if(ReadORWrte == READ){
		SlaveAddr = SlaveAddr << 1;
		SlaveAddr |= (1);
		pI2Cx->DR = SlaveAddr;
	}
}

static void I2C_ClearADDRFlag(I2C_Handler_t *pI2CHandler){
	uint32_t dummyRead;
	if(pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
		// Device in Master mode
		if(pI2CHandler->TXRXState == I2C_BUSY_RX){
			// Check size before clearing ADDR for disabling ACK when RXSize = 1
			if(pI2CHandler->RXSize == 1){
				// Disable ACK
				I2C_SetORClr_ACK(pI2CHandler->pI2Cx,DISABLE);

				// Clear ADDR Flag
				dummyRead = pI2CHandler->pI2Cx->SR1;
				dummyRead = pI2CHandler->pI2Cx->SR2;
			}
		} else {
			// Device not receiving the last byte, Clear ADDR Flag
			dummyRead = pI2CHandler->pI2Cx->SR1;
			dummyRead = pI2CHandler->pI2Cx->SR2;
		}
	} else {
		// Device in Slave mode, Clear ADDR Flag
		dummyRead = pI2CHandler->pI2Cx->SR1;
		dummyRead = pI2CHandler->pI2Cx->SR2;
	}
	(void)dummyRead;
}

void I2C_GenStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_SetORClr_ACK(I2C_RegDef_t *pI2Cx, uint8_t EnORDi){
	if(EnORDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else if(EnORDi == DISABLE)
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/*
 * I2C Peripheral Clock Control
 */
void I2C_ClockCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnORDi){
	if (EnORDi == ENABLE){
		if (pI2Cx == I2C1){
			I2C1_CLK_EN();
		}else if (pI2Cx == I2C2){
			I2C2_CLK_EN();
		}else if (pI2Cx == I2C3){
			I2C3_CLK_EN();
		}
	}else if (EnORDi == DISABLE){
		if (pI2Cx == I2C1){
			I2C1_CLK_DI();
		}else if (pI2Cx == I2C2){
			I2C2_CLK_DI();
		}else if (pI2Cx == I2C3){
			I2C3_CLK_DI();
		}
	}
}

/*
 * I2C Initialize and De-Initialize
 */
void I2C_Init(I2C_Handler_t *pI2CHandler)
{
	uint32_t tempreg = 0;

	// Enable the Peripheral Clock
	I2C_ClockCtrl(pI2CHandler->pI2Cx, ENABLE);

	// Enable or Disable Acking on CR1
	tempreg |= (pI2CHandler->I2C_Config.I2C_ACKCtrl << 10);
	pI2CHandler->pI2Cx->CR1 |= tempreg;

	// Configure the FREQ Field on CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Val() / 1000000U;
	pI2CHandler->pI2Cx->CR2 = (tempreg & 0x3F);

	// Load the Device's Own Address into OAR1
	tempreg = 0;
	tempreg = (pI2CHandler->I2C_Config.I2C_DeviceAddr << 1);
	tempreg |= (1 << 14);
	pI2CHandler->pI2Cx->OAR1 = tempreg;

	// CCR Calculations for Clock Speeds
	uint16_t ccrVal = 0;
	tempreg = 0;
	if(pI2CHandler->I2C_Config.I2C_SCLSpd <= I2C_SCL_SPD_SM)
	{
		// Standard mode
		ccrVal = RCC_GetPCLK1Val() / (2 * pI2CHandler->I2C_Config.I2C_SCLSpd);
		tempreg |= (ccrVal & 0xFFF);
	} else
	{
		// Fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandler->I2C_Config.I2C_FMDutyCycl << 14);

		if (pI2CHandler->I2C_Config.I2C_FMDutyCycl == I2C_DUTY_2)
		{
			ccrVal = RCC_GetPCLK1Val() / (3 * pI2CHandler->I2C_Config.I2C_SCLSpd);
		} else if (pI2CHandler->I2C_Config.I2C_FMDutyCycl == I2C_DUTY_16_9)
		{
			ccrVal = RCC_GetPCLK1Val() / (25 * pI2CHandler->I2C_Config.I2C_SCLSpd);
		}

		tempreg |= (ccrVal & 0xFFF);
	}
	pI2CHandler->pI2Cx->CCR = tempreg;


	// Calculation for TRISE Configuration
	if(pI2CHandler->I2C_Config.I2C_SCLSpd <= I2C_SCL_SPD_SM)
		{
			// Standard mode
			tempreg = (RCC_GetPCLK1Val() / 1000000U) + 1;

		} else
		{
			// Fast mode
			tempreg = ((RCC_GetPCLK1Val() * 300)/ 1000000000U) + 1;
		}
	pI2CHandler->pI2Cx->TRISE = (tempreg & 0x3F);
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1){
		SPI1_REG_RESET();
	} else if (pI2Cx == I2C2){
		SPI2_REG_RESET();
	} else if (pI2Cx == I2C3){
		SPI3_REG_RESET();
	}
}

/*
 * MASTER Data Send and Receive without Interrupts
 */

void I2C_MasterSendData(I2C_Handler_t *pI2CHandler, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS)
{
	// Generate start condition
	I2C_GenStartCondition(pI2CHandler->pI2Cx);

	// Wait till SB flag is set
	while(! I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_SB));

	// Send address of slave with R/W bit set to w(0)
	I2C_ExecuteAddrPhase_RW(pI2CHandler->pI2Cx, SlaveAddr, WRITE);

	// Confirm address phase is complete by checking ADDR flag
	while(! I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_ADDR));

	// Clear ADDR flag according to software sequence
	I2C_ClearADDRFlag(pI2CHandler);

	// Send data from DR till Len becomes 0
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_TXE));
		pI2CHandler->pI2Cx->DR = *pTXBuffer;
		pTXBuffer++;
		Len--;
	}

	// When Len is 0, wait for TXE and BTF before generating STOP cond
	// Note: TXE and BTF set = SR and DR are empty and next transmission can begin
	// Note: When BTF set, SCL is stretched
	while(! I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_BTF));

	// Generate STOP cond. Master need not wait for completion of STOP cond.
	// Note: Generating STOP clears BTF
	if (RS == I2C_NO_RS){
		I2C_GenStopCondition(pI2CHandler->pI2Cx);
	}
}

void I2C_MasterRecvData(I2C_Handler_t *pI2CHandler, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS){

	// Generate start condition
	I2C_GenStartCondition(pI2CHandler->pI2Cx);

	// Wait till SB flag is set
	while(! I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_SB));

	// Send address of slave with R/W bit set to 1
	I2C_ExecuteAddrPhase_RW(pI2CHandler->pI2Cx, SlaveAddr, READ);

	// Confirm address phase is complete by checking ADDR flag
	while(! I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_ADDR));

	// Reading data when one byte of information is to be sent
	if(Len == 1){

		// Disable ACKING
		I2C_SetORClr_ACK(pI2CHandler->pI2Cx, DISABLE);

		// Clear ADDR flag according to software sequence
		I2C_ClearADDRFlag(pI2CHandler);

		// Wait till RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_RXNE));

		// Generate STOP condition
		if (RS == I2C_NO_RS){
			I2C_GenStopCondition(pI2CHandler->pI2Cx);
		}

		// Read the data into buffer
		*pRXBuffer = pI2CHandler->pI2Cx->DR;

	}

	// Reading data when information is more than one byte
	if(Len > 1){

		// Clear ADDR flag according to software sequence
		I2C_ClearADDRFlag(pI2CHandler);

		// Read data until Len becomes zero
		for(uint32_t i = Len; i > 0; i--){

			// Wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2){
				// When last two bytes are left, clear ACK bit
				I2C_SetORClr_ACK(pI2CHandler->pI2Cx, DISABLE);

				// Generate STOP condition
				if (RS == I2C_NO_RS){
					I2C_GenStopCondition(pI2CHandler->pI2Cx);
				}

			}

			// Read Data from the DR into buffer
			*pRXBuffer = pI2CHandler->pI2Cx->DR;

			// Increment the buffer address
			pRXBuffer++;
		}
	}

	// Re-enable ACKING
	I2C_SetORClr_ACK(pI2CHandler->pI2Cx, ENABLE);
}

/*
 * MASTER Data Send and Receive with Interrupts
 */

uint8_t I2C_MasterSendData_IT(I2C_Handler_t *pI2CHandler, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS)
{
	uint8_t busystate = pI2CHandler->TXRXState;

	if( (busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX))
	{
		pI2CHandler->pTXBuffer = pTXBuffer;
		pI2CHandler->TXLen = Len;
		pI2CHandler->TXRXState = I2C_BUSY_TX;
		pI2CHandler->DevAddr = SlaveAddr;
		pI2CHandler->RS = RS;

		//Implement code to Generate START Condition
		I2C_GenStartCondition(pI2CHandler->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	return busystate;

}

uint8_t I2C_MasterRecvData_IT(I2C_Handler_t *pI2CHandler, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS)
{

	uint8_t busystate = pI2CHandler->TXRXState;

	if( (busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX))
	{
		pI2CHandler->pRXBuffer = pRXBuffer;
		pI2CHandler->RXLen = Len;
		pI2CHandler->TXRXState = I2C_BUSY_RX;
		pI2CHandler->RXSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandler->DevAddr = SlaveAddr;
		pI2CHandler->RS = RS;

		//Implement code to Generate START Condition
		I2C_GenStartCondition(pI2CHandler->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandler->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

/*
 *  SLAVE Data Send and Receive
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}

uint8_t I2C_SlaveRecvData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}


/*
 * I2C INTERRUPT CONFIGURATION AND HANDLING
 */

void I2C_IRQEnableConfig(uint8_t IRQNumber, uint8_t EnORDi)
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx_reg = IRQNumber / 4;
	uint8_t iprx_sec = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_sec) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_ADDR + (iprx_reg)) |= (IRQPriority << shift_amount);

}

void I2C_EV_IRQHandling(I2C_Handler_t *pI2CHandler)
{
	uint8_t temp,evtEN,bufEN;
	evtEN = pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	bufEN = pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	// Handle SB EVENT
	// Note: Only applic. in Master Mode
	temp = pI2CHandler->pI2Cx->SR1 & (I2C_FLAG_SB);
	if(evtEN && temp)
	{
		I2C_SB_EVHandler(pI2CHandler);
	}

	// Handle ADDR EVENT
	// Note: Master Mode: Addr is sent
	//		 Slave Mode	: Addr matched with own address
	temp = pI2CHandler->pI2Cx->SR1 & (I2C_FLAG_ADDR);
	if(evtEN && temp)
	{
		I2C_ADDR_EVHandler(pI2CHandler);
	}

	// Handle BTF (Byte Transfer Finish) EVENT
	temp = pI2CHandler->pI2Cx->SR1 & (I2C_FLAG_BTF);
	if(evtEN && temp)
	{
		I2C_BTF_EVHandler(pI2CHandler);
	}

	// Handle STOPF EVENT
	// Note: Stop detection only in Slave Mode.
	temp = pI2CHandler->pI2Cx->SR1 & (I2C_FLAG_STOPF);
	if(evtEN && temp)
	{
		I2C_STOPF_EVHandler(pI2CHandler);
	}

	// Handle RXNE EVENT
	temp = pI2CHandler->pI2Cx->SR1 & (I2C_FLAG_RXNE);
	if((evtEN & bufEN) && temp)
	{
		I2C_RXNE_EVHandler(pI2CHandler);
	}

	// Handle TXE EVENT
	temp = pI2CHandler->pI2Cx->SR1 & (I2C_FLAG_TXE);
	if((evtEN & bufEN) && temp)
	{
		I2C_TXE_EVHandler(pI2CHandler);
	}
}

void I2C_ER_IRQHandling(I2C_Handler_t *pI2CHandler)
{
	uint32_t temp,errEN;
	errEN = (pI2CHandler->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

	// Handle BUS ERROR
	temp = (pI2CHandler->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
	if(temp && errEN)
	{
		I2C_BUS_ERRHandler(pI2CHandler);
	}

	// Handle ARBITRATION LOST ERROR
	temp = (pI2CHandler->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
	if(temp && errEN)
	{
		I2C_ARLO_ERRHandler(pI2CHandler);
	}

	// Handle ACK ERROR
	temp = (pI2CHandler->pI2Cx->SR1) & (1 << I2C_SR1_AF);
	if(temp && errEN)
	{
		I2C_ACK_ERRHandler(pI2CHandler);
	}

	// Handle OVERRUN/UNDERRUN ERROR
	temp = (pI2CHandler->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
	if(temp && errEN)
	{
		I2C_OVR_ERRHandler(pI2CHandler);
	}

	// Handle TIMEOUT ERROR
	temp = (pI2CHandler->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
	if(temp && errEN)
	{
		I2C_TMEOUT_ERRHandler(pI2CHandler);
	}
}

/*
 *  EVENT RELATED INTERRUPT HANDLERS
 */
static void I2C_SB_EVHandler(I2C_Handler_t *pI2CHandler){
	// SB set when Start Condition successfully generated. Can initiate address transfer.
	if(pI2CHandler->TXRXState == I2C_BUSY_TX){
		I2C_ExecuteAddrPhase_RW(pI2CHandler->pI2Cx, pI2CHandler->DevAddr, WRITE);
	} else if(pI2CHandler->TXRXState == I2C_BUSY_RX){
		I2C_ExecuteAddrPhase_RW(pI2CHandler->pI2Cx, pI2CHandler->DevAddr, READ);
	}
}

static void I2C_ADDR_EVHandler(I2C_Handler_t *pI2CHandler){
	// Set when ADDR Phase is complete
	I2C_ClearADDRFlag(pI2CHandler);
}

static void I2C_BTF_EVHandler(I2C_Handler_t *pI2CHandler){
	if(pI2CHandler->TXRXState == I2C_BUSY_TX){
		// Verify if TXE is also set
		if(pI2CHandler->pI2Cx->SR1 & (I2C_FLAG_TXE)){
			if(pI2CHandler->TXLen == 0){
				// BTF = 1, TXE = 1; Transfer done, close connection.
				// Generate stop condition
				if (pI2CHandler->RS == I2C_NO_RS){
					I2C_GenStopCondition(pI2CHandler->pI2Cx);
				}
				// Reset all member elements of handle structure for TX
				I2C_ResetHandler_TX(pI2CHandler);

				// Notify application of event completion
				I2C_ApplicationEventCB(pI2CHandler, I2C_EVT_TX_CMPL);
			}
		}

	} else if(pI2CHandler->TXRXState == I2C_BUSY_RX)
	{
		;
	}
}

static void I2C_STOPF_EVHandler(I2C_Handler_t *pI2CHandler){
	pI2CHandler->pI2Cx->CR1 |= 0x0000;

	// Notify application of STOP event
	I2C_ApplicationEventCB(pI2CHandler, I2C_EVT_STOP);
}

static void I2C_RXNE_EVHandler(I2C_Handler_t *pI2CHandler){
	// Check if the device is in Master mode
	if(pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
		// Verify status and perform data transmission
		if(pI2CHandler->TXRXState == I2C_BUSY_RX){
			if(pI2CHandler->RXSize == 1){
				// Receive one byte of data
				*pI2CHandler->pRXBuffer = pI2CHandler->pI2Cx->DR;
				pI2CHandler->RXLen--;
			}

			if(pI2CHandler->RXSize > 1){
				if(pI2CHandler->RXLen == 2){
					// Clear ACK bit
					I2C_SetORClr_ACK(pI2CHandler->pI2Cx, DISABLE);
				}
				// Read DR
				*pI2CHandler->pRXBuffer = pI2CHandler->pI2Cx->DR;
				pI2CHandler->pRXBuffer++;
				pI2CHandler->RXLen--;
			}

			if(pI2CHandler->RXLen == 0){
				// All data received. Can close connection and notify application
				if (pI2CHandler->RS == I2C_NO_RS){
					I2C_GenStopCondition(pI2CHandler->pI2Cx);
				}

				// Reset all member elements of handle structure for RX
				I2C_ResetHandler_RX(pI2CHandler);

				// Notify application of event completion
				I2C_ApplicationEventCB(pI2CHandler, I2C_EVT_RX_CMPL);
			}
		}
	} else {
		// Device in slave mode, check for if in Receiver mode
		if(!(pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
		{
			I2C_ApplicationEventCB(pI2CHandler, I2C_EVT_DATA_RCV);
		}
	}
}

static void I2C_TXE_EVHandler(I2C_Handler_t *pI2CHandler){
	// Check if device mode is Master
	if(pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
		// Verify status and perform data transmission
		if(pI2CHandler->TXRXState == I2C_BUSY_TX){
			if(pI2CHandler->TXLen > 0){
				// Load data into DR
				pI2CHandler->pI2Cx->DR = *(pI2CHandler->pTXBuffer);

				// Decrement TXLen
				pI2CHandler->TXLen--;

				// Increment TXBuffer address
				pI2CHandler->pTXBuffer++;
			}
		}
	} else {
		// Device is in Slave mode, check for if in transmitter mode
		if(pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
		{
			I2C_ApplicationEventCB(pI2CHandler, I2C_EVT_DATA_REQ);
		}
	}
	// We don't close the connection here as when TX is complete, the BTF flag
	// is set. The IRQ handler for BTF closes the connection then.
}

/*
 *  ERROR RELATED INTERRUPT HANDLERS
 */
static void I2C_BUS_ERRHandler(I2C_Handler_t *pI2CHandler){
	//Implement the code to clear the buss error flag
	pI2CHandler->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

	//Implement the code to notify the application about the error
	I2C_ApplicationEventCB(pI2CHandler,I2C_ERR_BERR);
}

static void I2C_ARLO_ERRHandler(I2C_Handler_t *pI2CHandler){
	//Implement the code to clear the arbitration lost error flag
	pI2CHandler->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

	//Implement the code to notify the application about the error
	I2C_ApplicationEventCB(pI2CHandler,I2C_ERR_ARLO);
}

static void I2C_ACK_ERRHandler(I2C_Handler_t *pI2CHandler){
    //Implement the code to clear the ACK failure error flag
	pI2CHandler->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

	//Implement the code to notify the application about the error
	I2C_ApplicationEventCB(pI2CHandler,I2C_ERR_AF);
}

static void I2C_OVR_ERRHandler(I2C_Handler_t *pI2CHandler){
    //Implement the code to clear the Overrun/Underrun error flag
	pI2CHandler->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

	//Implement the code to notify the application about the error
	I2C_ApplicationEventCB(pI2CHandler,I2C_ERR_OVR);
}

static void I2C_TMEOUT_ERRHandler(I2C_Handler_t *pI2CHandler){
    //Implement the code to clear the Time out error flag
	pI2CHandler->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

	//Implement the code to notify the application about the error
	I2C_ApplicationEventCB(pI2CHandler,I2C_ERR_TIMEOUT);
}

/*
 * Other Peripheral Control APIs
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_PeripheralConfig(I2C_RegDef_t *pI2Cx, uint8_t EnORDi)
{
	if (EnORDi == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

void I2C_Slave_EnORDiCallback(I2C_RegDef_t *pI2Cx, uint8_t EnORDi)
{
	if(EnORDi == ENABLE){
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	} else {
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}

/*
 * Application Callback
 */

void I2C_ResetHandler_TX(I2C_Handler_t *pI2CHandler){
	// Disable ITBUFEN Control Bit
	pI2CHandler->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Disable ITEVTEN Control Bit
	pI2CHandler->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// Reset Handler for TX members
	pI2CHandler->TXRXState = I2C_READY;
	pI2CHandler->pTXBuffer = NULL;
	pI2CHandler->TXLen = 0;
	I2C_SetORClr_ACK(pI2CHandler->pI2Cx, ENABLE);
}

void I2C_ResetHandler_RX(I2C_Handler_t *pI2CHandler){
	// Disable ITBUFEN Control Bit
	pI2CHandler->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Disable ITEVTEN Control Bit
	pI2CHandler->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// Reset Handler for RX members
	pI2CHandler->TXRXState = I2C_READY;
	pI2CHandler->pRXBuffer = NULL;
	pI2CHandler->RXLen = 0;
	pI2CHandler->RXSize = 0;
	I2C_SetORClr_ACK(pI2CHandler->pI2Cx, ENABLE);
}

