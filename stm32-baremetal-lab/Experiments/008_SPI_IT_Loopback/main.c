#include "stm32f103xx_spi_driver.h"
#include "stm32f103xx_gpio_driver.h"
#include "core_nvic.h"
#include "string.h"

#define SPI_BUFFER_SIZE  sizeof(txData)
#define LED_TOGGLE_COUNT  10

void delay(uint32_t count)
{
	for (uint32_t i = 0; i < count; i++);
}

uint8_t txData[] = "Interrupt SPI!";
uint8_t rxData[SPI_BUFFER_SIZE] = { 0 };
GPIO_Handle_t ledTxIndicator;
GPIO_Handle_t ledRxIndicator;
SPI_Handle_t SPI1Handle;

int main(void)
{
	ledTxIndicator.pGPIOx = GPIOA;
	ledTxIndicator.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP;
	ledTxIndicator.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	ledTxIndicator.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

	ledRxIndicator.pGPIOx = GPIOA;
	ledRxIndicator.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP;
	ledRxIndicator.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	ledRxIndicator.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	GPIO_Init(&ledTxIndicator);
	GPIO_Init(&ledRxIndicator);

	SPI1Handle.pSPIx = SPI1;
	SPI1Handle.SPI_Config.SPI_BusConfig = SPI_BUS_FULL_DUPLEX;
	SPI1Handle.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST_EDGE;
	SPI1Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPI_Config.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI1Handle.SPI_Config.SPI_SSM = SPI_SSM_ENABLED;
	SPI1Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI_Init(&SPI1Handle);
	NVIC_EnableIRQ(SPI1_IRQn);// Enable SPI1 IRQ
	SPI_PeripheralControl(SPI1Handle.pSPIx, ENABLE);// Enable SPI1
	SPI_ReceiveData_IT(&SPI1Handle, rxData, sizeof(rxData));// Start RX first
	SPI_SendData_IT(&SPI1Handle, txData, sizeof(txData));// Then start TX

	while (1);

	return 0;
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	if (AppEv == SPI_EVENT_TX_CMPLT)
	{
// Transmission complete (can toggle LED or set flag)
		for (uint8_t i = 0; i < LED_TOGGLE_COUNT; ++i)
		{
			GPIO_ToggleOutputPin(ledTxIndicator.pGPIOx, ledTxIndicator.GPIO_PinConfig.GPIO_PinNumber);
			delay(30000);
		}
	}
	else if (AppEv == SPI_EVENT_RX_CMPLT)
	{

		if (memcmp(txData, rxData, sizeof(txData)) == 0)// Compare data
		{
// Loopback successful
			for (uint8_t i = 0; i < LED_TOGGLE_COUNT; ++i)
			{
				GPIO_ToggleOutputPin(ledRxIndicator.pGPIOx, ledRxIndicator.GPIO_PinConfig.GPIO_PinNumber);
				delay(30000);
			}
		}
		else
		{
// Mismatch detected
			GPIO_WriteToOutputPin(ledRxIndicator.pGPIOx, ledRxIndicator.GPIO_PinConfig.GPIO_PinNumber, 1);
		}
	}
}

void SPI1_IRQHandler(void)
{
	SPI_IRQHandling(&SPI1Handle);
}
