/*
 * main.c
 *
 *  Created on: Jul 22, 2025
 *      Author: yusuf
 */

#include "stm32f103xx_gpio_driver.h"
#include "core_nvic.h"

void delay()
{
	for (uint32_t i = 0; i < 200000; ++i);
}
GPIO_Handle_t led1;
GPIO_Handle_t led2;
GPIO_Handle_t btn1;
GPIO_Handle_t btn2;
int main(void)
{
	led1.pGPIOx = GPIOA;
	led1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	led1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP;

	led2.pGPIOx = GPIOA;
	led2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	led2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP;

	btn1.pGPIOx = GPIOB;
	btn1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	btn1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	btn1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULLUP;

	btn2.pGPIOx = GPIOB;
	btn2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
	btn2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	btn2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULLUP;

	GPIO_Init(&led1);
	GPIO_Init(&led2);
	GPIO_Init(&btn1);
	GPIO_Init(&btn2);
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_SetPriority(EXTI0_IRQn, 1);
	NVIC_SetPriority(EXTI4_IRQn, 2);
	while (1);

	return 0;
}

void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(btn1.GPIO_PinConfig.GPIO_PinNumber);
	GPIO_ToggleOutputPin(led1.pGPIOx, led1.GPIO_PinConfig.GPIO_PinNumber);
}

void EXTI4_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(btn2.GPIO_PinConfig.GPIO_PinNumber);
	GPIO_ToggleOutputPin(led2.pGPIOx, led2.GPIO_PinConfig.GPIO_PinNumber);
}
