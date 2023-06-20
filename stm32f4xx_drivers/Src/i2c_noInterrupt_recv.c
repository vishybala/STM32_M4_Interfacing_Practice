/*
 * i2c_noInterrupt_button_recv.c
 *
 *  Created on: Mar. 1, 2023
 *      Author: altai
 */

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

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

	// Init the GPIO Pins for I2C
	I2C2_GPIOInit();

	// Configure the push-button
	GPIO_ButtonInit();

	// Configure I2C
	I2C2_Init();

	// Peripheral Control
	I2C_PeripheralConfig(I2C2, ENABLE);

	while(1)
	{
		// Block till button press
		while (GPIO_ReadInputPin(GPIOC, GPIO_PIN_NO_13));
		// Delay for debouncing of button
		delay1();

		commandcode = 0x51;
		// Data write of command to slave
		I2C_MasterSendData(&I2C2Handler, (uint8_t *)&commandcode, 1, SLAVE_ADDR, I2C_NO_RS);

		// Recv Length of upcoming data from slave
		I2C_MasterRecvData(&I2C2Handler, &len, 1, SLAVE_ADDR, I2C_NO_RS);

		commandcode = 0x52;
		// Data write of command to slave
		I2C_MasterSendData(&I2C2Handler, (uint8_t *)&commandcode, 1, SLAVE_ADDR, I2C_NO_RS);

		// Now request to receive Len bytes from slave
		I2C_MasterRecvData(&I2C2Handler, (uint8_t *)&recvData, len, SLAVE_ADDR, I2C_NO_RS);
	}
}
/*
 * i2c_noInterrupt_recv.c
 *
 *  Created on: Mar. 22, 2023
 *      Author: altai
 */


