#include "stm32f103xx_spi_driver.h"
#include "stm32f103xx_gpio_driver.h"

void delay(uint32_t count)
{
	for (uint32_t i = 0; i < count; i++);
}

int main(void)
{
	SPI_Handle_t SPI1Handle;

// 1. SPI Peripheral Config
	SPI1Handle.pSPIx = SPI1;
	SPI1Handle.SPI_Config.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI1Handle.SPI_Config.SPI_BusConfig = SPI_BUS_FULL_DUPLEX;
	SPI1Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST_EDGE;
	SPI1Handle.SPI_Config.SPI_SSM = SPI_SSM_ENABLED;
	SPI_Init(&SPI1Handle);

// 2. RCLK (latch pin) as GPIO Output
	GPIO_Handle_t LatchPin;
	LatchPin.pGPIOx = GPIOA;
	LatchPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	LatchPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP;
	LatchPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	GPIO_Init(&LatchPin);

	uint8_t data = 0;
	while (1)
	{
		data = 0xAA;
		SPI_PeripheralControl(SPI1Handle.pSPIx, ENABLE);// SPI Enable
		GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_0, 0);
		SPI_SendData(SPI1Handle.pSPIx, &data, 1);
		SPI_PeripheralControl(SPI1Handle.pSPIx, DISABLE);// SPI Disable
		GPIO_WriteToOutputPin(LatchPin.pGPIOx, GPIO_PIN_0, 1);
		delay(500000);

		data = 0x55;
		SPI_PeripheralControl(SPI1Handle.pSPIx, ENABLE);// SPI Enable
		GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_0, 0);
		SPI_SendData(SPI1Handle.pSPIx, &data, 1);
		SPI_PeripheralControl(SPI1Handle.pSPIx, DISABLE);// SPI Disable
		GPIO_WriteToOutputPin(LatchPin.pGPIOx, GPIO_PIN_0, 1);
		delay(500000);
	}
}
