/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: Jul 23, 2025
 *      Author: yusuf
 */

#include "stm32f103xx_spi_driver.h"
#include "stm32f103xx_gpio_driver.h"

static void SPI_GPIO_Config(SPI_Handle_t *pSPIHandle);

void SPI_PeriClockControl(SPI_Handle_t *pSPIHandle, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pSPIHandle->pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if (pSPIHandle->pSPIx == SPI2)
			SPI2_PCLK_EN();
	}
	else
	{
		if (pSPIHandle->pSPIx == SPI1)
			SPI1_PCLK_DI();
		else if (pSPIHandle->pSPIx == SPI2)
			SPI2_PCLK_DI();
	}
}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg = 0;

// 1. Configure device mode (Master/Slave)
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << 2;

// 2. Configure bus configuration
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_FULL_DUPLEX)
	{
// Clear BIDIMODE (bit 15) for full-duplex
		tempreg &= ~(1 << 15);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_HALF_DUPLEX)
	{
// Set BIDIMODE for half-duplex
		tempreg |= (1 << 15);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_SIMPLEX_RXONLY)
	{
// Clear BIDIMODE and set RXONLY for simplex RX
		tempreg &= ~(1 << 15);
		tempreg |= (1 << 10);
	}

// 3. Configure SPI clock speed (baud rate control BR[2:0])
	tempreg |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << 3);

// 4. Configure DFF (data frame format)
	tempreg |= (pSPIHandle->SPI_Config.SPI_DFF << 11);

// 5. Configure CPOL
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPOL << 1);

// 6. Configure CPHA
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPHA << 0);

// 7. Configure software slave management (SSM)
	tempreg |= (pSPIHandle->SPI_Config.SPI_SSM << 9);

// Write the final CR1 value
	pSPIHandle->pSPIx->CR1 = tempreg;

	uint32_t tempreg2 = 0;

	if (pSPIHandle->SPI_Config.SPI_SSM == SPI_SSM_DISABLED)
	{
		tempreg2 |= (1 << 2);// SSOE enable
	}

	pSPIHandle->pSPIx->CR2 = tempreg2;

	SPI_GPIO_Config(pSPIHandle);
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		RCC->APB2RSTR |= (1 << 12);
		RCC->APB2RSTR &= ~(1 << 12);
	}
	else if (pSPIx == SPI2)
	{
		RCC->APB1RSTR |= (1 << 14);
		RCC->APB1RSTR &= ~(1 << 14);
	}
}

/**
 * @brief  Configures the GPIO pins used for SPI functionality.
 *
 * This function configures the appropriate GPIO pins in alternate function
 * push-pull mode or input mode based on SPI peripheral instance and configuration.
 *
 * @param[in] pSPIHandle Pointer to SPI handle structure.
 *
 * @note  This function assumes standard pin mapping:
 *        - SPI1: PA5 (SCK), PA6 (MISO), PA7 (MOSI)
 *        - SPI2: PB13 (SCK), PB14 (MISO), PB15 (MOSI)
 *
 * @return None
 */
static void SPI_GPIO_Config(SPI_Handle_t *pSPIHandle)
{
	GPIO_Handle_t SPIPins;

	if (pSPIHandle->pSPIx == SPI1)
	{
// SCK
		SPIPins.pGPIOx = GPIOA;
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_PP;
		SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
		GPIO_Init(&SPIPins);

// MISO
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
		GPIO_Init(&SPIPins);

// MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_PP;
		GPIO_Init(&SPIPins);
	}
	else if (pSPIHandle->pSPIx == SPI2)
	{
		SPIPins.pGPIOx = GPIOB;
		SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

// SCK
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_PP;
		GPIO_Init(&SPIPins);

// MISO
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
		GPIO_Init(&SPIPins);

// MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_PP;
		GPIO_Init(&SPIPins);
	}
}
