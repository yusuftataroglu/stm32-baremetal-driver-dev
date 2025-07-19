/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Jul 18, 2025
 *      Author: yusuf
 */

#include "stm32f103xx_gpio_driver.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *GPIOx, uint8_t EnorDi)
{
	if (EnorDi == 1)
	{
		switch ((uint32_t) GPIOx)
		{
		case (uint32_t) GPIOA:
			GPIOA_PCLK_EN();
			break;
		case (uint32_t) GPIOB:
			GPIOB_PCLK_EN();
			break;
		case (uint32_t) GPIOC:
			GPIOC_PCLK_EN();
			break;
		default:
			break;
		}
	}
	else
	{
		switch ((uint32_t) GPIOx)
		{
		case (uint32_t) GPIOA:
			GPIOA_PCLK_DI();
			break;
		case (uint32_t) GPIOB:
			GPIOB_PCLK_DI();
			break;
		case (uint32_t) GPIOC:
			GPIOC_PCLK_DI();
			break;
		default:
			break;
		}
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
// 1. Enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

// 2. Configure pin mode (CNF + MODE)
	GPIO_ConfigMode(pGPIOHandle->pGPIOx, &(pGPIOHandle->GPIO_PinConfig));

// 3. Configure speed if needed (only for output/AF)
	GPIO_ConfigSpeed(pGPIOHandle->pGPIOx, &(pGPIOHandle->GPIO_PinConfig));

// 4. Configure pull-up/pull-down if needed (only for input PU/PD)
	GPIO_ConfigPuPd(pGPIOHandle->pGPIOx, &(pGPIOHandle->GPIO_PinConfig));
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	return 0;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return 0;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value)
{

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{

}

static void GPIO_ConfigMode(GPIO_RegDef_t *pGPIOx, GPIO_PinConfig_t *pPinConfig)
{
	uint32_t pinNumber = pPinConfig->GPIO_PinNumber;
	uint32_t mode = 0;
	uint32_t cnf = 0;
	uint32_t shiftAmount = (pinNumber % 8) * 4;// Her pin için 4 bit

	volatile uint32_t *pConfigReg;

	if (pinNumber <= 7)
		pConfigReg = &pGPIOx->CRL;
	else
		pConfigReg = &pGPIOx->CRH;

// MODE ve CNF değerlerini belirle
	switch (pPinConfig->GPIO_PinMode)
	{
	case GPIO_MODE_INPUT:
		mode = 0x0;
		cnf = 0x1;// Input floating
		break;
	case GPIO_MODE_ANALOG:
		mode = 0x0;
		cnf = 0x0;
		break;
	case GPIO_MODE_OUTPUT_PP:
		mode = 0x1;// Medium speed (10 MHz)
		cnf = 0x0;
		break;
	case GPIO_MODE_OUTPUT_OD:
		mode = 0x1;
		cnf = 0x1;
		break;
	case GPIO_MODE_AF_PP:
		mode = 0x1;
		cnf = 0x2;
		break;
	case GPIO_MODE_AF_OD:
		mode = 0x1;
		cnf = 0x3;
		break;
	case GPIO_MODE_IT_FT:
	case GPIO_MODE_IT_RT:
	case GPIO_MODE_IT_RFT:
		mode = 0x0;
		cnf = 0x1;// Input floating, EXTI ayrıca yapılandırılacak
		break;
	default:
// Geçersiz mod → burada hata loglanabilir
		return;
	}

// Eski 4 biti temizle
	*pConfigReg &= ~(0xF << shiftAmount);

// Yeni değeri yaz
	*pConfigReg |= ((cnf << 2) | mode) << shiftAmount;
}

static void GPIO_ConfigSpeed(GPIO_RegDef_t *pGPIOx,
		GPIO_PinConfig_t *pPinConfig)
{
	uint32_t pinNumber = pPinConfig->GPIO_PinNumber;
	uint32_t shiftAmount = (pinNumber % 8) * 4;
	uint32_t modeBits = 0;

	if (pPinConfig->GPIO_PinMode == GPIO_MODE_OUTPUT_PP
			|| pPinConfig->GPIO_PinMode == GPIO_MODE_OUTPUT_OD
			|| pPinConfig->GPIO_PinMode == GPIO_MODE_AF_PP
			|| pPinConfig->GPIO_PinMode == GPIO_MODE_AF_OD)
	{
// Map enum to actual MODE[1:0] bits
		switch (pPinConfig->GPIO_PinSpeed)
		{
		case GPIO_SPEED_LOW:
			modeBits = 0b10;// 2 MHz
			break;
		case GPIO_SPEED_MEDIUM:
			modeBits = 0b01;// 10 MHz
			break;
		case GPIO_SPEED_HIGH:
			modeBits = 0b11;// 50 MHz
			break;
		default:
			return;// Invalid speed
		}

		if (pinNumber < 8)
		{
			pGPIOx->CRL &= ~(0x3 << shiftAmount);// clear MODE[1:0]
			pGPIOx->CRL |= (modeBits << shiftAmount);// set MODE[1:0]
		}
		else
		{
			pGPIOx->CRH &= ~(0x3 << shiftAmount);// clear MODE[1:0]
			pGPIOx->CRH |= (modeBits << shiftAmount);// set MODE[1:0]
		}
	}
}

static void GPIO_ConfigPuPd(GPIO_RegDef_t *pGPIOx, GPIO_PinConfig_t *pPinConfig)
{
	uint32_t pinNumber = pPinConfig->GPIO_PinNumber;

// Check if the pin mode is input with pull-up or pull-down
	if (pPinConfig->GPIO_PinMode == GPIO_MODE_INPUT_PU_PD)
	{
		switch (pPinConfig->GPIO_PinPuPdControl)
		{
		case GPIO_PULLUP:
			pGPIOx->ODR |= (1 << pinNumber);// Set bit for pull-up
			break;
		case GPIO_PULLDOWN:
			pGPIOx->ODR &= ~(1 << pinNumber);// Clear bit for pull-down
			break;
		case GPIO_NOPULL:
		default:
// Do nothing
			break;
		}
	}
}

