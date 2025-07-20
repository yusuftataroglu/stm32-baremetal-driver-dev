/*
 * main.c
 *
 *  Created on: Jul 21, 2025
 *      Author: yusuf
 */

#include <stm32f103xx_gpio_driver.h>

#define BTN_PRESSED 0
#define LEDS_ON_MASK  ( (1 << 10) | (1 << 11) | (1 << 12) )

void delay()
{
	for (uint32_t i = 0; i < 100000; ++i);
}

int main(void)
{

	uint16_t led_pins[] = { GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12 };
	for (int i = 0; i < 3; i++)
	{
		GPIO_Handle_t led;
		led.pGPIOx = GPIOC;
		led.GPIO_PinConfig.GPIO_PinNumber = led_pins[i];
		led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP;
		led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
		led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NOPULL;
		GPIO_Init(&led);
	}

	GPIO_Handle_t btn;
	btn.pGPIOx = GPIOA;
	btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT_PU_PD;
	btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULLUP;

	GPIO_Init(&btn);

	while (1)
	{
		if (GPIO_ReadFromInputPin(btn.pGPIOx, btn.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED)
		{
			GPIO_WriteToOutputPort(GPIOC, LEDS_ON_MASK);
			delay();
		}
		else
		{
			GPIO_WriteToOutputPort(GPIOC, 0x0);
		}
	}
	return 0;
}
