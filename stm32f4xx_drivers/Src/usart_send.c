/*
 * usart_send.c
 *
 *  Created on: Mar. 29, 2023
 *      Author: altai
 */

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

void delay1(void)
{
	for(uint32_t i = 0; i < 200000; i++);
}

// Global Variables
USART_Handler_t USARTHandler;
uint8_t TXData[32] = "UART TX TESTING 123";

/*
 * USART PINS CONFIG
 * Alt Funct ->
 * P USART_
 * P USART_
 */

// TODO CHANGE THESE TO USART SPEC
void USART_GPIOInit(void){
	GPIO_Handler_t USARTPins;

	USARTPins.pGPIOx = GPIOB;
	USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 7;
	USARTPins.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_TYPE_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU;
	USARTPins.GPIO_PinConfig.GPIO_PinSpd = GPIO_OUT_SPD_FAST;

	// TODO CHANGE THESE TO USART SPEC
	// RX Pin
	USARTPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_10;
	GPIO_Init(&USARTPins);

	// TX Pin
	USARTPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_11;
	GPIO_Init(&USARTPins);
}

void USARTx_Init(void){
	USARTHandler.pUSARTx = USART2;
	USARTHandler.USART_Config.USART_Mode = USART_MODE_TX;
	USARTHandler.USART_Config.USART_Baud = USART_BAUD_115200;
	USARTHandler.USART_Config.USART_NumStopbits = USART_STOPBIT_1;
	USARTHandler.USART_Config.USART_Parity = USART_PAR_OFF;
	USARTHandler.USART_Config.USART_WordLen = USART_WORDLEN_8BIT;
	USARTHandler.USART_Config.USART_HWFlowCtrl = USART_HW_FLOW_CTRL_NONE;

	USART_Init(&USARTHandler);
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

int main(void)
{
	USART_GPIOInit();

	USARTx_Init();

	USART_PeripheralConfig(USART2, ENABLE);

	while(1)
	{
		// Block till button press
		while (GPIO_ReadInputPin(GPIOC, GPIO_PIN_NO_13));
		// Delay for debouncing of button
		delay1();

		USART_SendData(&USARTHandler, (uint8_t*)TXData, strlen((char *)TXData));
	}

	return 0;
}
