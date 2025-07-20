/*
 * main.c
 *
 *  Created on: Jul 20, 2025
 *      Author: yusuf
 */


#include <stm32f103xx_gpio_driver.h>

void delay()
{
	for (uint32_t i = 0; i < 500000; ++i);
}

int main(void)
{
	GPIO_Handle_t onboardLed;
	onboardLed.pGPIOx = GPIOA;
	onboardLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	onboardLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP;
	onboardLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	onboardLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NOPULL;
	GPIO_Init(&onboardLed);

	while (1)
	{
		GPIO_ToggleOutputPin(onboardLed.pGPIOx, onboardLed.GPIO_PinConfig.GPIO_PinNumber);
		delay();
	}
	return 0;
}
