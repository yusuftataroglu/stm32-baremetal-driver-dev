/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Jul 18, 2025
 *      Author: yusuf
 */

#include "stm32f103xx_gpio_driver.h"

static void GPIO_ConfigMode(GPIO_RegDef_t *pGPIOx, GPIO_PinConfig_t *pPinConfig);
static void GPIO_ConfigSpeed(GPIO_RegDef_t *pGPIOx, GPIO_PinConfig_t *pPinConfig);
static void GPIO_ConfigPuPd(GPIO_RegDef_t *pGPIOx, GPIO_PinConfig_t *pPinConfig);
static void GPIO_ConfigInterrupt(GPIO_Handle_t *pGPIOHandle);

void GPIO_PeriClockControl(GPIO_Handle_t *pGPIOHandle, uint8_t enorDi)
{
	GPIO_Mode_t pinMode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;

	if (pinMode == GPIO_MODE_IT_RT || pinMode == GPIO_MODE_IT_FT
			|| pinMode == GPIO_MODE_IT_RFT)
	{
		RCC->APB2ENR |= (1 << 0);
	}

	if (enorDi == 1)
	{
		switch ((uint32_t) pGPIOHandle->pGPIOx)
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
		switch ((uint32_t) pGPIOHandle->pGPIOx)
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
	GPIO_PeriClockControl(pGPIOHandle, ENABLE);

// 2. Configure pin mode (CNF + MODE)
	GPIO_ConfigMode(pGPIOHandle->pGPIOx, &(pGPIOHandle->GPIO_PinConfig));

// 3. Configure speed if needed (only for output/AF)
	GPIO_ConfigSpeed(pGPIOHandle->pGPIOx, &(pGPIOHandle->GPIO_PinConfig));

// 4. Configure pull-up/pull-down if needed (only for input PU/PD)
	GPIO_ConfigPuPd(pGPIOHandle->pGPIOx, &(pGPIOHandle->GPIO_PinConfig));

// 5. Configure AFIO & EXTI registers if needed (only for interrupt modes)
	GPIO_ConfigInterrupt(pGPIOHandle);

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		RCC->APB2RSTR |= (1 << 2);
		RCC->APB2RSTR &= ~(1 << 2);
	}
	else if (pGPIOx == GPIOB)
	{
		RCC->APB2RSTR |= (1 << 3);
		RCC->APB2RSTR &= ~(1 << 3);
	}
	else if (pGPIOx == GPIOC)
	{
		RCC->APB2RSTR |= (1 << 4);
		RCC->APB2RSTR &= ~(1 << 4);
	}
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, GPIO_PinNumber_t pinNumber)
{
	return (uint8_t) ((pGPIOx->IDR >> pinNumber) & 0x1);
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t) pGPIOx->IDR;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, GPIO_PinNumber_t pinNumber, uint8_t value)
{
	if (value == ENABLE)
		pGPIOx->ODR |= (1 << pinNumber);
	else
		pGPIOx->ODR &= ~(1 << pinNumber);
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, GPIO_PinNumber_t pinNumber)
{
	pGPIOx->ODR ^= (1 << pinNumber);
}

/**
 * @brief  Configures the mode and configuration (CNF) bits of a given GPIO pin.
 *
 * This function sets the MODE[1:0] and CNF[1:0] bits located in the GPIO port’s
 * CRL or CRH register, depending on the pin number (0–7 → CRL, 8–15 → CRH).
 * It interprets the desired mode from the GPIO_PinConfig_t structure
 * and applies the appropriate 4-bit configuration for the selected pin.
 *
 * @note
 * - Output mode defaults to 10 MHz speed. To configure other speeds, call GPIO_ConfigSpeed() separately.
 * - For interrupt modes (e.g., GPIO_MODE_IT_FT), only the pin configuration is applied here.
 *   The EXTI/AFIO configuration must be handled by another function (e.g., GPIO_ConfigInterrupt()).
 *
 * @param[in] pGPIOx     Pointer to the GPIO port base address (e.g., GPIOA, GPIOB).
 * @param[in] pPinConfig Pointer to the user-defined configuration structure for the pin.
 *
 * @return None
 */
static void GPIO_ConfigMode(GPIO_RegDef_t *pGPIOx, GPIO_PinConfig_t *pPinConfig)
{
	uint32_t pinNumber = pPinConfig->GPIO_PinNumber;
	uint32_t mode = 0;
	uint32_t cnf = 0;
	uint32_t shiftAmount = (pinNumber % 8) * 4;

	volatile uint32_t *pConfigReg;

	if (pinNumber <= 7)
		pConfigReg = &pGPIOx->CRL;
	else
		pConfigReg = &pGPIOx->CRH;

	switch (pPinConfig->GPIO_PinMode)
	{
	case GPIO_MODE_INPUT:
		mode = 0x0;
		cnf = 0x1;// Input floating
		break;
	case GPIO_MODE_INPUT_PU_PD:
		mode = 0x0;
		cnf = 0x2;
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
		cnf = 0x2;
		break;
	default:
		return;
	}
	*pConfigReg &= ~(0xF << shiftAmount);
	*pConfigReg |= ((cnf << 2) | mode) << shiftAmount;
}

/**
 * @brief  Configures the output speed for the given GPIO pin.
 *
 * This function sets the MODE[1:0] bits in CRL or CRH to control
 * the switching speed of an output or alternate function pin.
 *
 * @note   Only valid for output and alternate function modes.
 *         This function does nothing for input mode pins.
 *
 * @param  pGPIOx: Pointer to the GPIO port base address
 * @param  pPinConfig: Pointer to the pin configuration structure
 *
 * @return None
 */
static void GPIO_ConfigSpeed(GPIO_RegDef_t *pGPIOx, GPIO_PinConfig_t *pPinConfig)
{
	if (pPinConfig->GPIO_PinMode == GPIO_MODE_OUTPUT_PP
			|| pPinConfig->GPIO_PinMode == GPIO_MODE_OUTPUT_OD
			|| pPinConfig->GPIO_PinMode == GPIO_MODE_AF_PP
			|| pPinConfig->GPIO_PinMode == GPIO_MODE_AF_OD)
	{
		uint32_t pinNumber = pPinConfig->GPIO_PinNumber;
		uint32_t shiftAmount = (pinNumber % 8) * 4;
		uint32_t modeBits = 0;
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

/**
 * @brief Configures the pull-up or pull-down resistor for a GPIO pin.
 *
 * Only applicable for input mode with pull-up/pull-down configuration (GPIO_MODE_INPUT_PU_PD).
 * Uses the ODR register to enable internal pull-up or pull-down resistors:
 * - ODR bit set   → Pull-up
 * - ODR bit clear → Pull-down
 *
 * @param pGPIOx Pointer to GPIO peripheral base address.
 * @param pPinConfig Pointer to GPIO pin configuration structure.
 *
 * @return None
 */
static void GPIO_ConfigPuPd(GPIO_RegDef_t *pGPIOx, GPIO_PinConfig_t *pPinConfig)
{
// Check if the pin mode is input with pull-up or pull-down
	GPIO_Mode_t pinMode = pPinConfig->GPIO_PinMode;

	if (pinMode == GPIO_MODE_INPUT_PU_PD || pinMode == GPIO_MODE_IT_FT
			|| pinMode == GPIO_MODE_IT_RT || pinMode == GPIO_MODE_IT_RFT)
	{
		uint32_t pinNumber = pPinConfig->GPIO_PinNumber;
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

/**
 * @brief  Configures the EXTI peripheral and AFIO registers for GPIO interrupt.
 * @param  pGPIOHandle: Pointer to the GPIO handle structure
 * @note   This function only performs AFIO and EXTI configuration.
 *         NVIC configuration must be done separately using GPIO_IRQInterruptConfig().
 * @return None
 */
static void GPIO_ConfigInterrupt(GPIO_Handle_t *pGPIOHandle)
{
	uint8_t mode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;

	if (mode != GPIO_MODE_IT_FT && mode != GPIO_MODE_IT_RT
			&& mode != GPIO_MODE_IT_RFT)
	{
// Not an interrupt mode, exit early
		return;
	}
	/* 1. Map GPIO port to appropriate EXTI line via AFIO_EXTICR */
	uint8_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	uint8_t extiCrIndex = pinNumber / 4;
	uint8_t fieldPosition = (pinNumber % 4) * 4;

// Clear the target field first
	AFIO->EXTICR[extiCrIndex] &= ~(0xF << fieldPosition);

// Set the port code into the field
	uint8_t port_code = GPIO_PORT_CODE(pGPIOHandle->pGPIOx);
	AFIO->EXTICR[extiCrIndex] |= (port_code << fieldPosition);

	/* 2. Unmask the interrupt line in EXTI_IMR */
	EXTI->IMR |= (1 << pinNumber);

	/* 3. Configure rising/falling trigger */
	if (mode == GPIO_MODE_IT_FT)
	{
		EXTI->FTSR |= (1 << pinNumber);
		EXTI->RTSR &= ~(1 << pinNumber);
	}
	else if (mode == GPIO_MODE_IT_RT)
	{
		EXTI->RTSR |= (1 << pinNumber);
		EXTI->FTSR &= ~(1 << pinNumber);
	}
	else if (mode == GPIO_MODE_IT_RFT)
	{
		EXTI->RTSR |= (1 << pinNumber);
		EXTI->FTSR |= (1 << pinNumber);
	}
}

void GPIO_IRQHandling(GPIO_PinNumber_t pinNumber)
{
	if (EXTI->PR & (1 << pinNumber))
		EXTI->PR |= (1 << pinNumber);
}
