#include "stm32f103xx_spi_driver.h"
#include "stm32f103xx_gpio_driver.h"

void delay(uint32_t count)
{
	for (uint32_t i = 0; i < count; i++);
}

int main(void)
{
	SPI_Handle_t SPI1Handle;
	SPI1Handle.pSPIx = SPI1;
	SPI1Handle.SPI_Config.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI1Handle.SPI_Config.SPI_BusConfig = SPI_BUS_FULL_DUPLEX;
	SPI1Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST_EDGE;
	SPI1Handle.SPI_Config.SPI_SSM = SPI_SSM_ENABLED;

	SPI_Init(&SPI1Handle);

	GPIO_Handle_t led;
	led.pGPIOx = GPIOA;
	led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP;
	led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	GPIO_Init(&led);

	uint8_t tx_data = 0xAB;
	uint8_t rx_data = 0;

	while (1)
	{
		SPI_PeripheralControl(SPI1Handle.pSPIx, ENABLE);
		SPI_SendData(SPI1Handle.pSPIx, &tx_data, 1);
		SPI_ReceiveData(SPI1Handle.pSPIx, &rx_data, 1);
		SPI_PeripheralControl(SPI1Handle.pSPIx, DISABLE);

		if (rx_data == tx_data)
		{
			GPIO_WriteToOutputPin(led.pGPIOx, GPIO_PIN_0, 1);
			delay(200000);
		}
		else
		{
			for (uint32_t i = 0; i < 10; ++i)
			{
				GPIO_ToggleOutputPin(led.pGPIOx, GPIO_PIN_0);
				delay(20000);
			}
		}
		GPIO_WriteToOutputPin(led.pGPIOx, GPIO_PIN_0, 0);
		delay(500000);
	}
}
