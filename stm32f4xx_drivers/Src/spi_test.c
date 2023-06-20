/*
 * spi_test.c
 *
 *  Created on: Mar. 7, 2023
 *      Author: altai
 */

#include "stm32f446xx.h"
#include <string.h>

/*
 * SPI2 PINS CONFIG
 * Alt Funct -> 5
 * PB15 MOSI
 * PB14 MISO
 * PB12 NSS
 * PB13 SCK
 */

void SPI2_GPIOInit(void){
	GPIO_Handler_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpd = GPIO_OUT_SPD_FAST;

	//SCLK Init
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI Init
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO Init
	//SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS Init
	//SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);
}

void SPI2_Init(void){
	SPI_Handler_t SPI2Handler;

	SPI2Handler.pSPIx = SPI2;
	SPI2Handler.SPI_Config.SPI_BusConfig = SPI_BUS_FULLDUPLX;
	SPI2Handler.SPI_Config.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI2Handler.SPI_Config.SPI_ClkSpd = SPI_BAUD_CLK_DIV2;
	SPI2Handler.SPI_Config.SPI_DFF = SPI_DFF_8BIT;
	SPI2Handler.SPI_Config.SPI_CPOL = SPI_CPOL_LO;
	SPI2Handler.SPI_Config.SPI_CPHA = SPI_CPHA_LEAD;
	SPI2Handler.SPI_Config.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2Handler);
}

int main(void)
{
	char user_data[] = "Hello World";

	SPI2_GPIOInit();

	SPI2_Init();

	SPI_SSIConfig(SPI2, ENABLE);

	SPI_PeripheralConfig(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	SPI_PeripheralConfig(SPI2, DISABLE);

	while(1);
	return 0;
}
