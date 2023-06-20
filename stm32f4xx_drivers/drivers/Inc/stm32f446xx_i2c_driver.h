/*
 * stm32f446xx_i2c_drivers.h
 *
 *  Created on: Mar. 16, 2023
 *      Author: altai
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"

/*****************************************************************************************
 * 								STRUCTS FOR I2C CONFIG AND HANDLER
 *****************************************************************************************/

/*
 * Configuration Structure for I2C Pin
 */

typedef struct
{
	uint32_t 	I2C_SCLSpd;				// Possible Values @12C_SCLSPEED
	uint8_t 	I2C_DeviceAddr;			// User Defined
	uint8_t 	I2C_ACKCtrl;			// Possible Values @12C_DUTYCYCLE
	uint16_t 	I2C_FMDutyCycl;			// Possible Values @12C_DUTYCYCLE
}I2C_Config;

/*
 * I2C Handler Structure
 */

typedef struct
{
	// Pointer to hold base address of I2C peripheral
	I2C_RegDef_t 	*pI2Cx; 			// Holds the base address of I2C port x of connected pin
	I2C_Config 		I2C_Config;			// Holds I2C configuration settings
	uint8_t			*pTXBuffer;			// Pointer to TX buffer
	uint8_t			*pRXBuffer;			// Pointer to RX buffer
	uint32_t		TXLen;				// Length of send data
	uint32_t		RXLen;				// Length of recv data
	uint8_t			TXRXState;			// Communication state
	uint8_t			DevAddr;			// Slave/Device address
	uint32_t		RXSize;				// RX size
	uint8_t			RS;					// Repeated start configuration
}I2C_Handler_t;

/*****************************************************************************************
 * 								MACROS FOR I2C REGISTER INIT
 *****************************************************************************************/
/*
 *  @12C_SCLSPEED
 *  I2C Possible SCL Speeds
 */
#define I2C_SCL_SPD_SM			100000
#define I2C_SCL_SPD_FM2K		200000
#define I2C_SCL_SPD_FM4K		400000

/*
 *  @12C_ACK_CONTROL
 *  I2C ACK Enable/Disable
 */
#define I2C_ACK_EN				1
#define I2C_ACK_DI				0

/*
 *  @12C_DUTYCYCLE
 *  I2C Possible Duty Cycles
 */
#define I2C_DUTY_2				0
#define I2C_DUTY_16_9			1

/*
 *  I2C_SR Flag/Masking Definitions
 */
#define I2C_FLAG_SB				(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR			(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF			(1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF			(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE			(1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE			(1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR			(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO			(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF				(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR			(1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT		(1 << I2C_SR1_TIMEOUT)

/*
 *  I2C Repeated Start Configuration
 */
#define I2C_NO_RS				0
#define I2C_RS					1

/*
 *  I2C Possible Communication States
 */
#define I2C_READY				0
#define I2C_BUSY_RX				1
#define I2C_BUSY_TX				2

/*
 *  I2C Possible Application Events
 */
#define I2C_EVT_TX_CMPL			0
#define I2C_EVT_RX_CMPL			1
#define I2C_EVT_STOP			2
#define I2C_EVT_DATA_REQ		3
#define I2C_EVT_DATA_RCV		4

#define I2C_ERR_BERR  			5
#define I2C_ERR_ARLO  			6
#define I2C_ERR_AF    			7
#define I2C_ERR_OVR   			8
#define I2C_ERR_TIMEOUT 		9

/*****************************************************************************************
 * 								APIs SUPPORTED BY THIS DRIVER
 * 					Refer to function definitions for more information
 *****************************************************************************************/

/*
 * I2C Peripheral Clock Control and Fetching
 */
void I2C_ClockCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnORDi);

/*
 * I2C Initialize and De-Initialize
 */
void I2C_Init(I2C_Handler_t *pI2CHandler);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data Send and Receive
 */
void I2C_GenStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_MasterSendData(I2C_Handler_t *pI2CHandler, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS);
void I2C_MasterRecvData(I2C_Handler_t *pI2CHandler, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS);

uint8_t I2C_MasterSendData_IT(I2C_Handler_t *pI2CHandler, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS);
uint8_t I2C_MasterRecvData_IT(I2C_Handler_t *pI2CHandler, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveRecvData(I2C_RegDef_t *pI2Cx);

/*
 * I2C Interrupt Configuration and ISR Handling
 */
void I2C_IRQEnableConfig(uint8_t IRQNumber, uint8_t EnORDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handler_t *pI2CHandler);
void I2C_ER_IRQHandling(I2C_Handler_t *pI2CHandler);

/*
 * Other Peripheral Control APIs
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_PeripheralConfig(I2C_RegDef_t *pI2Cx, uint8_t EnORDi);
void I2C_SetORClr_ACK(I2C_RegDef_t *pI2Cx, uint8_t EnORDi);
void I2C_Slave_EnORDiCallback(I2C_RegDef_t *pI2Cx, uint8_t EnORDi);

/*
 * Application Callback
 */
void I2C_ApplicationEventCB(I2C_Handler_t *pI2CHandler, uint8_t AppEv);
void I2C_ResetHandler_TX(I2C_Handler_t *pI2CHandler);
void I2C_ResetHandler_RX(I2C_Handler_t *pI2CHandler);


#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
