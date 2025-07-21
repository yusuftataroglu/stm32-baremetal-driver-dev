/*
 * core_nvic.h
 *
 *  Created on: Jul 22, 2025
 *      Author: yusuf
 */

#ifndef CUSTOMDRIVERS_INC_CORE_NVIC_H_
#define CUSTOMDRIVERS_INC_CORE_NVIC_H_

#include <stdint.h>

/***********************************************/
/* NVIC Registers (Interrupt Controller)       */
/***********************************************/

/**
 * @brief Interrupt Set Enable Registers (ISER)
 */
#define NVIC_ISER0   (*(volatile uint32_t*)0xE000E100)
#define NVIC_ISER1   (*(volatile uint32_t*)0xE000E104)
#define NVIC_ISER2   (*(volatile uint32_t*)0xE000E108)

/**
 * @brief Interrupt Clear Enable Registers (ICER)
 */
#define NVIC_ICER0   (*(volatile uint32_t*)0xE000E180)
#define NVIC_ICER1   (*(volatile uint32_t*)0xE000E184)
#define NVIC_ICER2   (*(volatile uint32_t*)0xE000E188)


/**
 * @brief  Enables a specific interrupt in the NVIC.
 * @param  IRQNumber: Interrupt number of the EXTI line (e.g., EXTI0_IRQn, EXTI1_IRQn).
 * @return None
 */
void NVIC_EnableIRQ(uint8_t IRQNumber);

/**
 * @brief  Disables a specific interrupt in the NVIC.
 * @param  IRQNumber: Interrupt number of the EXTI line (e.g., EXTI0_IRQn, EXTI1_IRQn).
 * @return None
 */
void NVIC_DisableIRQ(uint8_t IRQNumber);

#endif /* CUSTOMDRIVERS_INC_CORE_NVIC_H_ */
