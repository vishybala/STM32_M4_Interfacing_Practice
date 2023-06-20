/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Feb. 24, 2023
 *      Author: altai
 */

#ifndef STM32F446XX_GPIO_DRIVER_H_
#define STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/*****************************************************************************************
 * 								STRUCTS FOR GPIO CONFIG AND HANDLER
 *****************************************************************************************/

/*
 * Configuration Structure for GPIO Pin
 */

typedef struct{
	uint8_t GPIO_PinNum;			// Possible Values @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;			// Possible Values @GPIO_PIN_MODES
	uint8_t GPIO_PinSpd;			// Possible Values @GPIO_PIN_OUTPUT_SPEEDS
	uint8_t GPIO_PinPuPdCtrl;		// Possible Values @GPIO_PIN_PUPD_CONFIG
	uint8_t GPIO_PinOutType;		// Possible Values @GPIO_PIN_OUTPUT_MODES
	uint8_t GPIO_PinAltFuncMode;
}GPIO_PinConfig;

/*
 * GPIO Handler Structure
 */

typedef struct
{
	// Pointer to hold base address of GPIO peripheral
	GPIO_RegDef_t *pGPIOx; 					//Holds the base address of GPIO port x of connected pin
	GPIO_PinConfig GPIO_PinConfig;		//Holds GPIO pin configuration settings

}GPIO_Handler_t;

/*****************************************************************************************
 * 								MACROS FOR GPIO REGISTER INIT
 *****************************************************************************************/
/*
 *  @GPIO_PIN_NUMBERS
 *  GPIO Pin Possible Numbers
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/*
 *  @GPIO_PIN_MODES
 *  GPIO Pin Possible Modes
 */
#define GPIO_MODE_IN			0		// Input Mode
#define GPIO_MODE_OUT			1		// Output Mode
#define GPIO_MODE_ALTFN			2		// Alternate Function Mode
#define GPIO_MODE_ANALOG		3		// Analog Mode
#define GPIO_MODE_INT_FT		4		// Falling Edge Trigger for Input
#define GPIO_MODE_INT_RT		5		// Rising Edge Trigger for Input
#define GPIO_MODE_INT_RFT		6		// Rising and Falling Edge Trigger for Input

/*
 *  @GPIO_PIN_OUTPUT_SPEEDS
 *  GPIO Pin Possible Output Speeds
 */
#define GPIO_OUT_SPD_LO			0
#define GPIO_OUT_SPD_MED		1
#define GPIO_OUT_SPD_FAST		2
#define GPIO_OUT_SPD_HIGH		3

/*
 *  @GPIO_PIN_PUPD_CONFIG
 *  GPIO Pull Up/ Pull Down Configuration
 */
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

/*
 * 	@GPIO_PIN_OUTPUT_MODES
 *  GPIO Pin Possible Output Modes
 */
#define GPIO_OUT_TYPE_PP		0		// Output Push/Pull
#define GPIO_OUT_TYPE_OD		1		// Output Open Drain

/*****************************************************************************************
 * 								APIs SUPPORTED BY THIS DRIVER
 * 					Refer to function definitions for more information
 *****************************************************************************************/

/*
 * GPIO Peripheral Clock Control
 */
void GPIO_ClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnORDi);

/*
 * GPIO Initialize and De-Initialize
 */
void GPIO_Init(GPIO_Handler_t *pGPIOHandler);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * GPIO Read/Write
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t Value);
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);

/*
 * GPIO Interrupt Configuration and ISR Handling
 */
void GPIO_IRQEnableConfig(uint8_t IRQNumber, uint8_t EnORDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNum);

#endif /* STM32F446XX_GPIO_DRIVER_H_ */
