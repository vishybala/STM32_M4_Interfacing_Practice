/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: Mar. 28, 2023
 *      Author: altai
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include "stm32f446xx.h"

/*****************************************************************************************
 * 								STRUCTS FOR USART CONFIG AND HANDLER
 *****************************************************************************************/

/*
 * Configuration Structure for USART Pin
 */

typedef struct
{
	uint8_t 	USART_Mode;					// Possible Values @USART_MODES
	uint32_t 	USART_Baud;					// Possible Values @USART_BAUD
	uint8_t 	USART_NumStopbits;			// Possible Values @USART_NUMSTOPBITS
	uint8_t 	USART_WordLen;				// Possible Values @USART_WORDLENGTH
	uint8_t 	USART_Parity;				// Possible Values @USART_PARITY
	uint8_t 	USART_HWFlowCtrl;			// Possible Values @USART_HARDWARECONTROL

}USART_Config;

/*
 * USART Handler Structure
 */

typedef struct
{
	// Pointer to hold base address of USART peripheral
	USART_RegDef_t 	*pUSARTx; 				// Holds the base address of USART port x of connected pin
	USART_Config 	USART_Config;			// Holds USART configuration settings
	uint8_t 		*pTXBuffer;
	uint8_t			*pRXBuffer;
	uint8_t			TXLen;
	uint8_t			RXLen;
	uint8_t			TXState;
	uint8_t			RXState;
}USART_Handler_t;

/*****************************************************************************************
 * 								MACROS FOR USART REGISTER INIT
 *****************************************************************************************/
/*
 * @USART_MODES
 * Possible modes for USART communication
 */
#define USART_MODE_TX 					0
#define USART_MODE_RX 					1
#define USART_MODE_TXRX  				2

/*
 * @USART_BAUD
 * Possible baud rates (data transfer speed)
 */
#define USART_BAUD_1200					1200
#define USART_BAUD_2400					2400
#define USART_BAUD_9600					9600
#define USART_BAUD_19200 				19200
#define USART_BAUD_38400 				38400
#define USART_BAUD_57600 				57600
#define USART_BAUD_115200 				115200
#define USART_BAUD_230400 				230400
#define USART_BAUD_460800 				460800
#define USART_BAUD_921600 				921600
#define USART_BAUD_2M 					2000000
#define SUART_BAUD_3M 					3000000

/*
 * @USART_NUMSTOPBITS
 * Number of bits for stop condition generation
 */
#define USART_STOPBIT_1     			0
#define USART_STOPBIT_0_5   			1
#define USART_STOPBIT_2     			2
#define USART_STOPBIT_1_5   			3

/*
 * @USART_WORDLENGTH
 * Possible word lengths
 */
#define USART_WORDLEN_8BIT  			0
#define USART_WORDLEN_9BIT  			1

/*
 * @USART_PARITY
 * Even or odd parity bit for error checking
 */
#define USART_PAR_ODD   				2
#define USART_PAR_EVEN  				1
#define USART_PAR_OFF   				0

/*
 * @USART_HARDWARECONTROL
 * USART Hardware based flow control
 */
#define USART_HW_FLOW_CTRL_NONE    		0
#define USART_HW_FLOW_CTRL_CTS    		1
#define USART_HW_FLOW_CTRL_RTS    		2
#define USART_HW_FLOW_CTRL_CTS_RTS		3

/*
 *  USART Flag/Masking Macros
 */
#define USART_FLAG_TXE					(1 << USART_SR_TXE)
#define USART_FLAG_RXNE					(1 << USART_SR_RXNE)
#define USART_FLAG_PE					(1 << USART_SR_PE)
#define USART_FLAG_FE					(1 << USART_SR_FE)
#define USART_FLAG_NF					(1 << USART_SR_NF)
#define USART_FLAG_ORE					(1 << USART_SR_ORE)
#define USART_FLAG_IDLE					(1 << USART_SR_IDLE)
#define USART_FLAG_TC					(1 << USART_SR_TC)
#define USART_FLAG_LBD					(1 << USART_SR_LBD)
#define USART_FLAG_CTS					(1 << USART_SR_CTS)

/*
 *  USART Possible States for ISR handler
 */
#define USART_READY					0
#define USART_BUSY_RX				1
#define USART_BUSY_TX				2

/*
 *  USART Possible Application Events
 */
#define USART_EVT_TX_CMPL			1
#define USART_EVT_RX_CMPL			2
#define USART_EVT_CTS				3

#define USART_ERR_IDLE				4
#define USART_ERR_OVR				5
#define USART_ERR_FE				6
#define USART_ERR_NF				7

/*****************************************************************************************
 * 								APIs SUPPORTED BY THIS DRIVER
 * 					Refer to function definitions for more information
 *****************************************************************************************/

/*
 * USART Peripheral Clock Control and Fetching
 */
void USART_ClockCtrl(USART_RegDef_t *pUSARTx, uint8_t EnORDi);

/*
 * USART Initialize and De-Initialize
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t Baud);
void USART_Init(USART_Handler_t *pUSARTHandler);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handler_t *pUSARTHandler, uint8_t *pTXBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handler_t *pUSARTHandler, uint8_t *pRXBuffer, uint32_t Len);

uint8_t USART_SendData_IT(USART_Handler_t *pUSARTHandler, uint8_t *pTXBuffer, uint32_t Len);
uint8_t USART_ReceiveData_IT(USART_Handler_t *pUSARTHandler, uint8_t *pRXBuffer, uint32_t Len);

/*
 * USART Interrupt Configuration and ISR Handling
 */
void USART_IRQEnableConfig(uint8_t IRQNumber, uint8_t EnORDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handler_t *pUSARTHandler);

/*
 * Other Peripheral Control APIs
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void USART_PeripheralConfig(USART_RegDef_t *pUSARTx, uint8_t EnORDi);

/*
 * Application Callback
 */
void USART_ApplicationEventCB(USART_Handler_t *pUSARTHandler, uint8_t AppEv);

#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
