/*
 * i2c_YESint_recv.c
 *
 *  Created on: Mar. 27, 2023
 *      Author: altai
 */

// TODO SEMIHOSTING

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles();

uint8_t rxCmplt = RESET;

#define MY_ADDR		0x61
#define SLAVE_ADDR	0x68

void delay1(void)
{
	for(uint32_t i = 0; i < 200000; i++);
}

// Global Variables
I2C_Handler_t I2C2Handler;
uint8_t recvData[32];

/*
 * I2C PINS CONFIG
 * Alt Funct -> 4
 * PB10 I2C2_SCL
 * PB11 I2C2_SDA
 */

void I2C2_GPIOInit(void){
	GPIO_Handler_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpd = GPIO_OUT_SPD_FAST;

	// SCL Pin
	I2CPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_10;
	GPIO_Init(&I2CPins);

	// SDA Pin
	I2CPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_11;
	GPIO_Init(&I2CPins);
}

void I2C2_Init(void){
	I2C2Handler.pI2Cx = I2C2;
	I2C2Handler.I2C_Config.I2C_ACKCtrl = I2C_ACK_EN;
	I2C2Handler.I2C_Config.I2C_DeviceAddr = MY_ADDR;
	I2C2Handler.I2C_Config.I2C_FMDutyCycl = I2C_DUTY_2;
	I2C2Handler.I2C_Config.I2C_SCLSpd = I2C_SCL_SPD_SM;

	I2C_Init(&I2C2Handler);

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
	// Create command variable
	uint8_t commandcode, len;

	//initialise_monitor_handles();
	printf("Application is running \n");

	// Init the GPIO Pins for I2C
	I2C2_GPIOInit();

	// Configure the push-button
	GPIO_ButtonInit();

	// Configure I2C
	I2C2_Init();

	// Configure the IRQ Handler
	I2C_IRQEnableConfig(IRQ_I2C2_EV, ENABLE);
	I2C_IRQEnableConfig(IRQ_I2C2_ER, ENABLE);

	// Peripheral Control
	I2C_PeripheralConfig(I2C2, ENABLE);

	I2C_SetORClr_ACK(I2C2, ENABLE);

	while(1)
	{
		// Block till button press
		while (GPIO_ReadInputPin(GPIOC, GPIO_PIN_NO_13));
		// Delay for debouncing of button
		delay1();

		commandcode = 0x51;
		// Data write of command to slave
		while (I2C_MasterSendData_IT(&I2C2Handler, (uint8_t *)&commandcode, 1, SLAVE_ADDR, I2C_RS) != I2C_READY);

		// Recv Length of upcoming data from slave
		while (I2C_MasterRecvData_IT(&I2C2Handler, &len, 1, SLAVE_ADDR, I2C_RS) != I2C_READY);
		rxCmplt = RESET;

		commandcode = 0x52;
		// Data write of command to slave
		while (I2C_MasterSendData_IT(&I2C2Handler, (uint8_t *)&commandcode, 1, SLAVE_ADDR, I2C_RS) != I2C_READY);

		// Now request to receive Len bytes from slave
		while (I2C_MasterRecvData_IT(&I2C2Handler, (uint8_t *)&recvData, len, SLAVE_ADDR, I2C_NO_RS) != I2C_READY);

		while(rxCmplt != SET){

		}

		recvData[len+1] = '\0';
		printf("Data: %s", recvData);
		rxCmplt = RESET;
	}
}

void I2C2_EV_IRQHandler(void){
	I2C_EV_IRQHandling(&I2C2Handler);
}

void I2C2_ER_IRQHandler(void){
	I2C_ER_IRQHandling(&I2C2Handler);
}

void I2C_ApplicationEventCB(I2C_Handler_t *pI2CHandler, uint8_t AppEv){
	if(AppEv == I2C_EVT_TX_CMPL)
	{
		printf("TX IS COMPLETE\n");
	}
	else if(AppEv == I2C_EVT_RX_CMPL)
	{
		printf("RX IS COMPLETE\n");
		rxCmplt = SET;
	}
	else if(AppEv == I2C_ERR_BERR)
	{
		printf("Error: Bus Fault\n");
	}
	else if(AppEv == I2C_ERR_ARLO)
	{
		printf("Error: Arbitration Lost\n");
	}
	else if(AppEv == I2C_ERR_AF)
	{
		printf("Error: Ack Failure\n");
		// For master, ack failure occurs when the slave fails to send ack for byte sent by master
		// Close the connection when this happens
		I2C_ResetHandler_RX(pI2CHandler);

		// Generate stop condition and release the bus
		I2C_GenStopCondition(pI2CHandler->pI2Cx);

		// Hang here for this error
		while(1);
	}
	else if(AppEv == I2C_ERR_OVR)
	{
		printf("Error: Overrun/Underrun\n");
	}
	else if(AppEv == I2C_ERR_TIMEOUT)
	{
		printf("TX IS COMPLETE\n");
	}

}
