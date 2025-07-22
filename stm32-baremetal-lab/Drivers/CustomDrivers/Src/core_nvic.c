/*
 * core_nvic.c
 *
 *  Created on: Jul 22, 2025
 *      Author: yusuf
 */

#include "core_nvic.h"

void NVIC_EnableIRQ(uint8_t IRQNumber)
{
	if (IRQNumber < 32)
	{
		NVIC_ISER0 |= (1 << IRQNumber);
	}
	else if (IRQNumber < 64)
	{
		NVIC_ISER1 |= (1 << (IRQNumber % 32));
	}
	else if (IRQNumber < 96)
	{
		NVIC_ISER2 |= (1 << (IRQNumber % 32));
	}
}

void NVIC_DisableIRQ(uint8_t IRQNumber)
{
	if (IRQNumber < 32)
	{
		NVIC_ICER0 |= (1 << IRQNumber);
	}
	else if (IRQNumber < 64)
	{
		NVIC_ICER1 |= (1 << (IRQNumber % 32));
	}
	else if (IRQNumber < 96)
	{
		NVIC_ICER2 |= (1 << (IRQNumber % 32));
	}
}

void NVIC_SetPriority(uint8_t IRQNumber, uint8_t priority)
{
	uint8_t iprIndex = IRQNumber / 4;
	uint8_t iprSection = IRQNumber % 4;
	uint8_t beginningOfValidBitsPos = 4;
	uint8_t shiftAmount = (iprSection * 8) + beginningOfValidBitsPos;

	*(NVIC_PR_BASEADDR + iprIndex) &= ~(0XFF << shiftAmount);
	*(NVIC_PR_BASEADDR + iprIndex) |= (priority << shiftAmount);
}
