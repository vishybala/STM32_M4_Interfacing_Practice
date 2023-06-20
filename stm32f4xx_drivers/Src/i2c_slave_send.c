/*
 * i2c_slave.c
 *
 *  Created on: Mar. 27, 2023
 *      Author: altai
 */

// TODO SEMIHOSTING

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

#define SLAVE_ADDR	0x68
#define MY_ADDR		SLAVE_ADDR

void delay1(void)
{
	for(uint32_t i = 0; i < 200000; i++);
}

// Global Variables
I2C_Handler_t I2C2Handler;
uint8_t txData[32] = ".STM32 Slave Testing.";

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
	// Init the GPIO Pins for I2C
	I2C2_GPIOInit();

	// Configure the push-button
	GPIO_ButtonInit();

	// Configure I2C
	I2C2_Init();

	// Configure the IRQ Handler
	I2C_IRQEnableConfig(IRQ_I2C2_EV, ENABLE);
	I2C_IRQEnableConfig(IRQ_I2C2_ER, ENABLE);

	I2C_Slave_EnORDiCallback(I2C2, ENABLE);

	// Peripheral Control
	I2C_PeripheralConfig(I2C2, ENABLE);

	// Manage ACKing
	I2C_SetORClr_ACK(I2C2, ENABLE);

	while(1);
}

void I2C2_EV_IRQHandler(void){
	I2C_EV_IRQHandling(&I2C2Handler);
}

void I2C2_ER_IRQHandler(void){
	I2C_ER_IRQHandling(&I2C2Handler);
}



void I2C_ApplicationEventCB(I2C_Handler_t *pI2CHandler, uint8_t AppEv)
{
	static uint8_t commandCode = 0;
	static uint8_t byteCount = 0;

	if(AppEv == I2C_EVT_DATA_REQ){
		// Master wants some data, slave sends it
		if(commandCode == 0x51){
			// Send the length of data to be transferred
			I2C_SlaveSendData(pI2CHandler->pI2Cx, strlen((char*)txData));
		} else if (commandCode == 0x52){
			// Send the data
			I2C_SlaveSendData(pI2CHandler->pI2Cx, txData[byteCount++]);
		}
	} else if (AppEv == I2C_EVT_DATA_RCV){
		// Master is sending data for slave to read
		commandCode = I2C_SlaveRecvData(pI2CHandler->pI2Cx);
	} else if (AppEv == I2C_ERR_AF){
		// Happens only if slave is transmitting
		// Master wants slave to stop sending data
		commandCode = 0xff;
		byteCount = 0;
	} else if (AppEv == I2C_EVT_STOP){
		// Happens only if slave is receiving
		// Master has ended i2c communication with the slave
	}

}
