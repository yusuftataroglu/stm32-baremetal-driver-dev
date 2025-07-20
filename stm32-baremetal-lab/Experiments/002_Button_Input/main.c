/*
 * main.c
 *
 *  Created on: Jul 20, 2025
 *      Author: yusuf
 */

#include <stm32f103xx_gpio_driver.h>

#define BTN_PRESSED 0

void delay()
{
	for (uint32_t i = 0; i < 200000; ++i);
}

int main(void)
{
	GPIO_Handle_t onboardLed;
	GPIO_Handle_t onboardBtn;

	onboardLed.pGPIOx = GPIOA;
	onboardLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	onboardLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP;
	onboardLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	onboardLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NOPULL;

	onboardBtn.pGPIOx = GPIOC;
	onboardBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	onboardBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;

	GPIO_Init(&onboardLed);
	GPIO_Init(&onboardBtn);

	while (1)
	{
		if (GPIO_ReadFromInputPin(onboardBtn.pGPIOx, GPIO_PIN_13) == BTN_PRESSED)
		{
			GPIO_ToggleOutputPin(onboardLed.pGPIOx, onboardLed.GPIO_PinConfig.GPIO_PinNumber);
			delay();
		}
	}
	return 0;
}
