/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Feb. 24, 2023
 *      Author: altai
 */

#include "stm32f446xx_gpio_driver.h"

/******************************************************************************
 *
 * Function			- GPIO_ClockCtrl
 *
 * Brief			- Enables or disables the perihperal clock for given GPIOx
 *
 * Notes			-
 ******************************************************************************/
void GPIO_ClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnORDi)
{
	if (EnORDi == ENABLE){
		if (pGPIOx == GPIOA){
			GPIOA_CLK_EN();
		}else if (pGPIOx == GPIOB){
			GPIOB_CLK_EN();
		}else if (pGPIOx == GPIOC){
			GPIOC_CLK_EN();
		}else if (pGPIOx == GPIOD){
			GPIOD_CLK_EN();
		}else if (pGPIOx == GPIOE){
			GPIOE_CLK_EN();
		}else if (pGPIOx == GPIOF){
			GPIOF_CLK_EN();
		}else if (pGPIOx == GPIOG){
			GPIOG_CLK_EN();
		}else if (pGPIOx == GPIOH){
			GPIOH_CLK_EN();
		}

	}else if (EnORDi == DISABLE){
		if (pGPIOx == GPIOA){
			GPIOA_CLK_DI();
		}else if (pGPIOx == GPIOB){
			GPIOB_CLK_DI();
		}else if (pGPIOx == GPIOC){
			GPIOC_CLK_DI();
		}else if (pGPIOx == GPIOD){
			GPIOD_CLK_DI();
		}else if (pGPIOx == GPIOE){
			GPIOE_CLK_DI();
		}else if (pGPIOx == GPIOF){
			GPIOF_CLK_DI();
		}else if (pGPIOx == GPIOG){
			GPIOG_CLK_DI();
		}else if (pGPIOx == GPIOH){
			GPIOH_CLK_DI();
		}
	}
}

/*
 * GPIO Initialize and De-Initialize
 */
void GPIO_Init(GPIO_Handler_t *pGPIOHandler)
{
	uint32_t temp = 0;

	GPIO_ClockCtrl(pGPIOHandler->pGPIOx, ENABLE);
	// Configure NON-INTERRUPT MODES of GPIO Pin

	if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) // Non Interrupt Modes
	{
		temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNum));
		// Clear Bit-Field of MODERx
		pGPIOHandler->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNum));
		// Set Bit-Field of MODERx
		pGPIOHandler->pGPIOx->MODER |= temp;

	} else // Configure INTERRUPT MODES of GPIO Pin
	{
		if(pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_FT)
		{
			// Configure Falling Trigger Selection Register
			EXTI->FTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNum);
			EXTI->RTSR &= ~(1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNum);

		} else if(pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RT)
		{
			// Configure Rising Trigger Selection Register
			EXTI->RTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNum);
			EXTI->FTSR &= ~(1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNum);

		} else if(pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RFT)
		{
			// Configure FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNum);
			EXTI->RTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNum);

		}

		// Configure the GPIO Port Selection in SYSCFG_EXTI Control Register
		uint8_t temp1 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNum / 4;
		uint8_t temp2 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNum % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_EXTICODE(pGPIOHandler->pGPIOx);

		SYSCFG_CLK_EN();
		SYSCFG->EXTICR[temp1] = (portcode << (temp2 * 4));

		// Enable the EXTI Interrupt Delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNum);
	}
	temp = 0;

	// 2. Configure Speed of GPIO Pin

	temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinSpd << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNum));
	// Clear Bit-Field of OSPEEDRx
	pGPIOHandler->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNum));
	// Set Bit-Field of OSPEEDRx
	pGPIOHandler->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3. Configure Pull Up/ Pull Down Registers

	temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinPuPdCtrl << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNum));
	// Clear Bit-Field of PUPDRx
	pGPIOHandler->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNum));
	// Set Bit-Field of PUPDRx
	pGPIOHandler->pGPIOx->PUPDR |= temp;
	temp = 0;

	// 4. Configure the Output Type

	temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinOutType << (pGPIOHandler->GPIO_PinConfig.GPIO_PinNum));
	// Clear Bit-Field of OTYPERx
	pGPIOHandler->pGPIOx->OTYPER &= ~(0x3 << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNum));
	// Set Bit-Field of OTYPERx
	pGPIOHandler->pGPIOx->OTYPER |= temp;
	temp = 0;

	// 5. Configure Alternate Function

	uint8_t temp1 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNum / 8;
	uint8_t temp2 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNum % 8;
	if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		if (temp1 == 0)
		{
			pGPIOHandler->pGPIOx->AFRL &= ~(0xF << (4 * temp2));
			pGPIOHandler->pGPIOx->AFRL |= (pGPIOHandler->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * temp2));

		} else if (temp1 == 1)
		{
			pGPIOHandler->pGPIOx->AFRH &= ~(0xF << (4 * temp2));
			pGPIOHandler->pGPIOx->AFRH |= (pGPIOHandler->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * temp2));
		}
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
}

/*
 * GPIO Read/Write
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
	uint8_t value;
	value = ((pGPIOx->IDR >> PinNum) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNum);

	} else
	{
		pGPIOx->ODR &= ~(1 << PinNum);
	}
}

void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value)
{
	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
	pGPIOx->ODR ^= (1 << PinNum);
}

/*
 * GPIO Interrupt SET and RESET Configuration
 */
void GPIO_IRQEnableConfig(uint8_t IRQNumber, uint8_t EnORDi)
{
	if(EnORDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// Program ISER0 Register of NVIC
			*NVIC_ISER0_ADDR |= (1 << IRQNumber);

		} else if(IRQNumber > 31 && IRQNumber <= 64)
		{
			// Program ISER1 Register of NVIC
			*NVIC_ISER1_ADDR |= (1 << (IRQNumber % 32));

		} else if(IRQNumber > 64 && IRQNumber <= 96)
		{
			// Program ISER2 Register of NVIC
			*NVIC_ISER2_ADDR |= (1 << (IRQNumber % 64));

		}
	} else
	{
		if(IRQNumber <= 31)
		{
			// Program ICER0 Register of NVIC
			*NVIC_ICER0_ADDR |= (1 << IRQNumber);

		} else if(IRQNumber > 31 && IRQNumber <= 64)
		{
			// Program ICER1 Register of NVIC
			*NVIC_ICER1_ADDR |= (1 << (IRQNumber % 32));

		} else if(IRQNumber > 64 && IRQNumber <= 96)
		{
			// Program ICER2 Register of NVIC
			*NVIC_ICER2_ADDR |= (1 << (IRQNumber % 64));

		}
	}
}

/*
 * GPIO Interrupt Priority Configuration
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx_reg = IRQNumber / 4;
	uint8_t iprx_sec = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_sec) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_ADDR + (iprx_reg)) |= (IRQPriority << shift_amount);
}

/*
 * GPIO Interrupt Handler
 */
void GPIO_IRQHandling(uint8_t PinNum)
{
	// Clear the EXTI PR Register Corresponding to PinNum
	if (EXTI->PR & (1 << PinNum))
	{
		EXTI->PR |= (1 << PinNum);
	}
}
