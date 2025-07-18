/******************************************************************************
 * @file    stm32f103xx_gpio_driver.h
 * @author  Yusuf
 * @brief   GPIO driver header for STM32F103RB (bare-metal implementation)
 *
 * @note    This file contains register-level GPIO driver APIs, written
 *          without any HAL or CMSIS dependency, intended for educational
 *          and portfolio purposes.
 ******************************************************************************/

#ifndef CUSTOMDRIVERS_INC_STM32F103XX_GPIO_DRIVER_H_
#define CUSTOMDRIVERS_INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"

/**************************************
 * @def STRUCTURES USED FOR GPIO DRIVER
 **************************************/

/**
 * @brief Configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber; /*!< Pin number (0 to 15) */
	uint8_t GPIO_PinMode; /*!< Pin mode: Input, Output, Alternate Function, Analog */
	uint8_t GPIO_PinSpeed; /*!< Output speed: Low, Medium, High */
	uint8_t GPIO_PinPuPdControl; /*!< Pull-up/Pull-down configuration */
	uint8_t GPIO_PinOPType; /*!< Output type: Push-pull or Open-drain */
	uint8_t GPIO_PinAltFunMode; /*!< Alternate function mode (if applicable) */
} GPIO_PinConfig_t;

/**
 * @brief GPIO Handle structure
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx; /*!< Pointer to GPIO peripheral base address */
	GPIO_PinConfig_t GPIO_PinConfig; /*!< GPIO pin configuration settings */
} GPIO_Handle_t;

/**************************************
 * @def DRIVER API FUNCTION PROTOTYPES
 **************************************/

/**
 * @brief Initializes the GPIO pin according to specified settings in GPIO_Handle_t
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

/**
 * @brief Resets all GPIO peripheral registers to default values
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/**
 * @brief Reads the state of a single GPIO input pin
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/**
 * @brief Reads the state of all GPIO input port pins
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/**
 * @brief Writes a logic level (HIGH or LOW) to a single GPIO output pin
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber,
		uint8_t value);

/**
 * @brief Writes a 16-bit value to the entire GPIO output port
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);

/**
 * @brief Toggles the logic level of a specific output pin
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

#endif /* CUSTOMDRIVERS_INC_STM32F103XX_GPIO_DRIVER_H_ */
