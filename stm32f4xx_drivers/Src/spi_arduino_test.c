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

void delay1(void)
{
	for(uint32_t i = 0; i < 100000; i++);
}

void SPI2_GPIOInit(void){
	GPIO_Handler_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpd = GPIO_OUT_SPD_FAST;

	//SCLK Init
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_10;
	GPIO_Init(&SPIPins);

	//MOSI Init
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO Init
	//SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS Init
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void GPIO_ButtonInit(void){

	GPIO_Handler_t GPIOBtn;
	//GPIO Button INIT
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpd = GPIO_OUT_SPD_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}

void SPI2_Init(void){
	SPI_Handler_t SPI2Handler;

	SPI2Handler.pSPIx = SPI2;
	SPI2Handler.SPI_Config.SPI_BusConfig = SPI_BUS_FULLDUPLX;
	SPI2Handler.SPI_Config.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI2Handler.SPI_Config.SPI_ClkSpd = SPI_BAUD_CLK_DIV8;
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

	SPI_SSOEConfig(SPI2, ENABLE);

	GPIO_ButtonInit();

	while(1)
	{
		// Block till button press
		while (GPIO_ReadInputPin(GPIOC, GPIO_PIN_NO_13));
		// Delay for debouncing of button
		//delay1();
		SPI_PeripheralConfig(SPI2, ENABLE);

		// Send length of data first to the slave
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		// Send actual data to slave
		SPI_SendData(SPI2, (uint8_t*)user_data, dataLen);

		// Confirm if SPI is Busy
		while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
		SPI_PeripheralConfig(SPI2, DISABLE);
	}

while(1);
return 0;
}
