/*
 * stm32f446xx_spi_drivers.h
 *
 *  Created on: Mar. 6, 2023
 *      Author: altai
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/*****************************************************************************************
 * 								STRUCTS FOR SPI CONFIG AND HANDLER
 *****************************************************************************************/

/*
 * Configuration Structure for SPI Pin
 */

typedef struct
{
	uint8_t SPI_DeviceMode;			// Possible Values @SPI_DEVICEMODES
	uint8_t SPI_BusConfig;			// Possible Values @SPI_BUSCONFIG
	uint8_t SPI_ClkSpd;				// Possible Values @SPI_CLOCKSPEEDS
	uint8_t SPI_DFF;				// Possible Values @SPI_DFF
	uint8_t SPI_CPOL;				// Possible Values @SPI_CLKPOLARITY
	uint8_t SPI_CPHA;				// Possible Values @SPI_CLKPHASE
	uint8_t SPI_SSM;				// Possible Values @SPI_SSM
}SPI_Config;

/*
 * SPI Handler Structure
 */

typedef struct
{
	// Pointer to hold base address of SPI peripheral
	SPI_RegDef_t 	*pSPIx; 					//Holds the base address of SPI port x of connected pin
	SPI_Config 		SPI_Config;			//Holds SPI configuration settings
	uint8_t 		*pTXBuffer;
	uint8_t			*pRXBuffer;
	uint8_t			TXLen;
	uint8_t			RXLen;
	uint8_t			TXState;
	uint8_t			RXState;
}SPI_Handler_t;

/*****************************************************************************************
 * 								MACROS FOR SPI REGISTER INIT
 *****************************************************************************************/
/*
 *  @SPI_DEVICEMODES
 *  SPI Pin Possible Modes
 */
#define SPI_MODE_SLAVE			0
#define SPI_MODE_MASTER			1

/*
 *  @SPI_BUSCONFIG
 *  SPI Bus Possible Modes
 */
#define SPI_BUS_FULLDUPLX		1
#define SPI_BUS_HALFDUPLX		2
#define SPI_BUS_SIMPLX_RX		3

/*
 *  @SPI_CLOCKSPEEDS
 *  SPI Baud Rate
 */
#define SPI_BAUD_CLK_DIV2		0
#define SPI_BAUD_CLK_DIV4		1
#define SPI_BAUD_CLK_DIV8		2
#define SPI_BAUD_CLK_DIV16		3
#define SPI_BAUD_CLK_DIV32		4
#define SPI_BAUD_CLK_DIV64		5
#define SPI_BAUD_CLK_DIV128		6
#define SPI_BAUD_CLK_DIV256		7

/*
 *  @SPI_DFF
 *  SPI Data Frame Format Possible Bit Depth
 */
#define SPI_DFF_8BIT			0
#define SPI_DFF_16BIT			1

/*
 *  @SPI_CLKPOLARITY
 *  SPI Clock Polarity (0 or 1 on Idle)
 */
#define SPI_CPOL_LO				0
#define SPI_CPOL_HI				1

/*
 *  @SPI_CLKPHASE
 *  SPI Clock Phase (Leading or Trailing Edge Sampling)
 */
#define SPI_CPHA_LEAD			0
#define SPI_CPHA_TRAIL			1

/*
 *  @SPI_SSM
 *  SPI Software Slave Management
 */
#define SPI_SSM_DI				0
#define SPI_SSM_EN				1

/*
 *  SPI_SR Flag Definitions
 */
#define SPI_RXNE_FLAG			(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG			(1 << SPI_SR_TXE)
#define SPI_BSY_FLAG			(1 << SPI_SR_BSY)

/*
 *  SPI Possible States for ISR handler
 */
#define SPI_READY				0
#define SPI_BUSY_RX				1
#define SPI_BUSY_TX				2

/*
 *  SPI Possible Application Events
 */
#define SPI_EVT_TX_CMPL			1
#define SPI_EVT_RX_CMPL			2
#define SPI_EVT_OVR_ERR			3
#define SPI_EVT_CRC_ERR			4

/*****************************************************************************************
 * 								APIs SUPPORTED BY THIS DRIVER
 * 					Refer to function definitions for more information
 *****************************************************************************************/

/*
 * SPI Peripheral Clock Control
 */
void SPI_ClockCtrl(SPI_RegDef_t *pSPIx, uint8_t EnORDi);

/*
 * SPI Initialize and De-Initialize
 */
void SPI_Init(SPI_Handler_t *pSPIHandler);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len);

uint8_t SPI_SendData_IT(SPI_Handler_t *pSPIHandler, uint8_t *pTXBuffer, uint32_t Len);
uint8_t SPI_ReceiveData_IT(SPI_Handler_t *pSPIHandler, uint8_t *pRXBuffer, uint32_t Len);

/*
 * SPI Interrupt Configuration and ISR Handling
 */
void SPI_IRQEnableConfig(uint8_t IRQNumber, uint8_t EnORDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handler_t *pSPIHandler);

/*
 * Other Peripheral Control APIs
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnORDi);
void SPI_PeripheralConfig(SPI_RegDef_t *pSPIx, uint8_t EnORDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnORDi);
void SPI_ClearOVRFlag(SPI_Handler_t *pSPIHandler);
void SPI_CloseTransmission(SPI_Handler_t *pSPIHandler);
void SPI_CloseReception(SPI_Handler_t *pSPIHandler);

/*
 * Application Callback
 */
void SPI_ApplicationEventCB(SPI_Handler_t *pSPIHandler, uint8_t AppEv);


#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
