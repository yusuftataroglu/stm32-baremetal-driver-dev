/*
 * main.c
 *
 *  Created on: Jul 22, 2025
 *      Author: yusuf
 */

#include <stm32f103xx_gpio_driver.h>

void delay()
{
	for (uint32_t i = 0; i < 150000; ++i);
}
GPIO_Handle_t onboardLed;
GPIO_Handle_t onboardBtn;
int main(void)
{

	onboardLed.pGPIOx = GPIOA;
	onboardLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	onboardLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP;
	onboardLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	onboardLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NOPULL;

	onboardBtn.pGPIOx = GPIOC;
	onboardBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	onboardBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;

	GPIO_Init(&onboardLed);
	GPIO_Init(&onboardBtn);
	NVIC_EnableIRQ(40);// EXTI15_10_IRQHandler IRQ number
	while (1);

	return 0;
}

void EXTI15_10_IRQHandler(void)
{
	EXTI->PR |= (1 << 13);
	GPIO_ToggleOutputPin(onboardLed.pGPIOx, onboardLed.GPIO_PinConfig.GPIO_PinNumber);
	delay();
}
