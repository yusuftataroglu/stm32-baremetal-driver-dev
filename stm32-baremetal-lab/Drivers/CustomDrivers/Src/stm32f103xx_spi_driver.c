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
