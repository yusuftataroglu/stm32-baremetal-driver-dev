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
#define NVIC_ISER0   (*(volatile uint32_t*)0xE000E100U)
#define NVIC_ISER1   (*(volatile uint32_t*)0xE000E104U)
#define NVIC_ISER2   (*(volatile uint32_t*)0xE000E108U)

/**
 * @brief Interrupt Clear Enable Registers (ICER)
 */
#define NVIC_ICER0   (*(volatile uint32_t*)0xE000E180U)
#define NVIC_ICER1   (*(volatile uint32_t*)0xE000E184U)
#define NVIC_ICER2   (*(volatile uint32_t*)0xE000E188U)

/**
 * @brief Base address of NVIC Interrupt Priority Registers (IPR)
 *
 * Each IPR register is 8 bits (1 byte) and corresponds to one IRQ number.
 * For STM32F1 series, only the upper 2 bits of each IPR field are used to set priority.
 */
#define NVIC_PR_BASEADDR   ((uint32_t*)0xE000E400)

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

/**
 * @brief  Sets the priority level for a given IRQ number.
 *
 * This function writes the specified priority value to the appropriate IPR register
 * for the given IRQ number. Only the upper bits (e.g., upper 2 bits of 4-bit field)
 * are effective in STM32F103 series.
 *
 * @param[in] IRQNumber IRQ number to configure
 * @param[in] priority  Priority level (0 to 3 recommended)
 *
 * @return None
 */
void NVIC_SetPriority(uint8_t IRQNumber, uint8_t priority);

#endif /* CUSTOMDRIVERS_INC_CORE_NVIC_H_ */
