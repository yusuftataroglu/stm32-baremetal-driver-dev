/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: Jul 23, 2025
 *      Author: yusuf
 */

#include "stm32f103xx_spi_driver.h"

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
	    tempreg2 |= (1 << 2); // SSOE enable
	}

	pSPIHandle->pSPIx->CR2 = tempreg2;

}
