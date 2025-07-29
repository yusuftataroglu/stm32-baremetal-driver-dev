/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: Jul 23, 2025
 *      Author: yusuf
 */

#include "stm32f103xx_spi_driver.h"
#include "stm32f103xx_gpio_driver.h"

static void SPI_GPIO_Config(SPI_Handle_t *pSPIHandle);
static void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
static void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);

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

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while (len > 0)
	{
// 1. Wait until TXE is set (Transmit buffer empty)
		while (!(pSPIx->SR & (1 << 1)));

		if (pSPIx->CR1 & (1 << 11))// 16-bit DFF
		{
// 2. Send dummy byte to generate clock and receive data
			pSPIx->DR = 0xFFFF;

// 3. Wait until RXNE is set (Receive buffer not empty)
			while (!(pSPIx->SR & (1 << 0)));

// 4. Read received data from DR
			*((uint16_t*) pRxBuffer) = pSPIx->DR;

			pRxBuffer += 2;
			len -= 2;
		}
		else// 8-bit DFF
		{
			pSPIx->DR = 0xFF;

			while (!(pSPIx->SR & (1 << 0)));

			*pRxBuffer = pSPIx->DR;

			pRxBuffer++;
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

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
{
	if (pSPIx->SR & flagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
// Check for TXE
	uint8_t txe_flag = pSPIHandle->pSPIx->SR & SPI_FLAG_TXE;
	uint8_t txe_ie = pSPIHandle->pSPIx->CR2 & SPI_CR2_TXEIE;

	if (txe_flag && txe_ie)
		spi_txe_interrupt_handle(pSPIHandle);// Handle TXE interrupt

// Check for RXNE
	uint8_t rxne_flag = pSPIHandle->pSPIx->SR & SPI_FLAG_RXNE;
	uint8_t rxne_ie = pSPIHandle->pSPIx->CR2 & SPI_CR2_RXNEIE;

	if (rxne_flag && rxne_ie)
		spi_rxne_interrupt_handle(pSPIHandle);// Handle RXNE interrupt

// Check for OVR
	uint8_t ovr_flag = pSPIHandle->pSPIx->SR & SPI_FLAG_OVR;
	uint8_t err_ie = pSPIHandle->pSPIx->CR2 & SPI_CR2_ERRIE;

	if (ovr_flag && err_ie)
	{
// Handle OVR error
//spi_ovr_err_interrupt_handle(pSPIHandle); /* OVR'yi öğrendikten sonra ekleyebilirim */
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
	}
}

uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX)
	{
// Save the Tx buffer address and length
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

// Enable TXEIE control bit to trigger interrupt
		pSPIHandle->pSPIx->CR2 |= (SPI_CR2_TXEIE);
	}
	return state;
}

uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX)
	{
// Save RX transfer details to the handle
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

// Enable RXNE interrupt
		pSPIHandle->pSPIx->CR2 |= (SPI_CR2_RXNEIE);
	}
	return state;
}

/**
 * @brief  Handles the SPI transmit buffer empty (TXE) interrupt.
 *
 * Called when the TXE flag is set and TXEIE interrupt is enabled.
 * Sends the next byte/word of data if available, otherwise ends transmission.
 *
 * @param[in] pSPIHandle Pointer to SPI handle structure.
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if (pSPIHandle->TxLen > 0)
	{
		if (pSPIHandle->SPI_Config.SPI_DFF)
		{
// 16-bit data
			pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen -= 2;
			pSPIHandle->pTxBuffer++;
			pSPIHandle->pTxBuffer++;
		}
		else
		{
// 8-bit data
			pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
		}
	}

	if (pSPIHandle->TxLen == 0)
	{
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

/**
 * @brief  Handles the SPI receive buffer not empty (RXNE) interrupt.
 *
 * Called when the RXNE flag is set and RXNEIE interrupt is enabled.
 * Reads the received data and stores it in the Rx buffer.
 *
 * @param[in] pSPIHandle Pointer to SPI handle structure.
 */
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if (pSPIHandle->RxLen > 0)
	{
		if (pSPIHandle->SPI_Config.SPI_DFF)
		{
			*((uint16_t*) pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen -= 2;
			pSPIHandle->pRxBuffer++;
			pSPIHandle->pRxBuffer++;
		}
		else
		{
			*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer++;
		}
	}

	if (pSPIHandle->RxLen == 0)
	{
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

/**
 * @brief  Closes the SPI transmission in interrupt mode.
 *
 * Disables the TXE interrupt and resets internal state variables related to SPI transmission.
 *
 * @param[in] pSPIHandle  Pointer to the SPI handle structure.
 *
 * @note This function is intended to be called from the IRQ handler after the transmission is complete.
 */
static void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
// Disable TXE interrupt
	pSPIHandle->pSPIx->CR2 &= ~(SPI_CR2_TXEIE);

// Reset application buffer and state
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/**
 * @brief  Closes the SPI reception in interrupt mode.
 *
 * Disables the RXNE interrupt and resets internal state variables related to SPI reception.
 *
 * @param[in] pSPIHandle  Pointer to the SPI handle structure.
 *
 * @note This function is intended to be called from the IRQ handler after the reception is complete.
 */
static void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
// Disable RXNE interrupt
	pSPIHandle->pSPIx->CR2 &= ~(SPI_CR2_RXNEIE);

// Reset application buffer and state
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{

}
