/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: Jul 23, 2025
 *      Author: yusuf
 */

#include "stm32f103xx_spi_driver.h"
#include "stm32f103xx_gpio_driver.h"

static void SPI_GPIO_Config(SPI_Handle_t *pSPIHandle);

void SPI_PeriClockControl(SPI_Handle_t *pSPIHandle, uint8_t enOrDi)
{
	if (enOrDi == ENABLE)
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
// 1. Enable clock for SPI peripheral
	SPI_PeriClockControl(pSPIHandle, ENABLE);

	uint32_t tempreg = 0;

// 2. Configure device mode (Master/Slave)
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << 2;

// 3. Configure bus configuration
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

// 4. Configure SPI clock speed (baud rate control BR[2:0])
	tempreg |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << 3);

// 5. Configure DFF (data frame format)
	tempreg |= (pSPIHandle->SPI_Config.SPI_DFF << 11);

// 6. Configure CPOL
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPOL << 1);

// 7. Configure CPHA
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPHA << 0);

// 8. Configure software slave management (SSM)
	tempreg |= (pSPIHandle->SPI_Config.SPI_SSM << 9);

// 9. Configure internal slave select bit
	if (pSPIHandle->SPI_Config.SPI_SSM == SPI_SSM_ENABLED)
	{
	    tempreg |= (1 << 8);
	}

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

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while (len > 0)
	{
// Wait until TXE (Transmit buffer empty) is set
		while (!(pSPIx->SR & (1 << 1)));

// Check DFF (data frame format)
		if (pSPIx->CR1 & (1 << 11))
		{
// 16-bit DFF
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			pTxBuffer += 2;
			len -= 2;
		}
		else
		{
// 8-bit DFF
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
			len--;
		}
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
		SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
		GPIO_Init(&SPIPins);

// MISO
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
		GPIO_Init(&SPIPins);

// MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
		GPIO_Init(&SPIPins);
	}
	else if (pSPIHandle->pSPIx == SPI2)
	{
// SCK
		SPIPins.pGPIOx = GPIOB;
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_PP;
		SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
		GPIO_Init(&SPIPins);

// MISO
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
		GPIO_Init(&SPIPins);

// MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
		GPIO_Init(&SPIPins);
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t enOrDi)
{
	if (enOrDi == ENABLE)
	{
// Enable SPE bit
		pSPIx->CR1 |= (1 << 6);
	}
	else
	{
// Disable SPE bit
		pSPIx->CR1 &= ~(1 << 6);
	}
}

