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

#define GPIO_PORT_CODE(GPIOx)     \
    ((GPIOx == GPIOA) ? 0 :       \
    (GPIOx == GPIOB) ? 1 :        \
    (GPIOx == GPIOC) ? 2 : 0)

#include "stm32f103xx.h"

/**
 * @brief GPIO pin number definitions (0 to 15)
 *
 * These enum values represent the GPIO pin numbers for STM32F103 GPIO ports.
 * Used in GPIO_PinConfig_t to select a specific pin within a GPIO port.
 */
typedef enum
{
	GPIO_PIN_0 = 0, /*!< GPIO Pin 0  */
	GPIO_PIN_1 = 1, /*!< GPIO Pin 1  */
	GPIO_PIN_2 = 2, /*!< GPIO Pin 2  */
	GPIO_PIN_3 = 3, /*!< GPIO Pin 3  */
	GPIO_PIN_4 = 4, /*!< GPIO Pin 4  */
	GPIO_PIN_5 = 5, /*!< GPIO Pin 5  */
	GPIO_PIN_6 = 6, /*!< GPIO Pin 6  */
	GPIO_PIN_7 = 7, /*!< GPIO Pin 7  */
	GPIO_PIN_8 = 8, /*!< GPIO Pin 8  */
	GPIO_PIN_9 = 9, /*!< GPIO Pin 9  */
	GPIO_PIN_10 = 10, /*!< GPIO Pin 10 */
	GPIO_PIN_11 = 11, /*!< GPIO Pin 11 */
	GPIO_PIN_12 = 12, /*!< GPIO Pin 12 */
	GPIO_PIN_13 = 13, /*!< GPIO Pin 13 */
	GPIO_PIN_14 = 14, /*!< GPIO Pin 14 */
	GPIO_PIN_15 = 15 /*!< GPIO Pin 15 */
} GPIO_PinNumber_t;

/**
 * @brief GPIO pin mode configuration.
 *
 * This enum defines the available modes for a GPIO pin:
 * - Input
 * - Output (Push-Pull or Open-Drain)
 * - Alternate Function (Push-Pull or Open-Drain)
 * - Analog
 */
typedef enum
{
	GPIO_MODE_INPUT = 0, /*!< Input floating */
	GPIO_MODE_INPUT_PU_PD, /*!< Input with internal pull up/down resistor */
	GPIO_MODE_OUTPUT_PP, /*!< General purpose output push-pull */
	GPIO_MODE_OUTPUT_OD, /*!< General purpose output open-drain */
	GPIO_MODE_AF_PP, /*!< Alternate function push-pull */
	GPIO_MODE_AF_OD, /*!< Alternate function open-drain */
	GPIO_MODE_ANALOG, /*!< Analog mode */
	GPIO_MODE_IT_FT, /*!< Falling Trigger */
	GPIO_MODE_IT_RT, /*!< Rising Trigger*/
	GPIO_MODE_IT_RFT /*!< Rising-Falling Trigger*/
} GPIO_Mode_t;

/**
 * @brief GPIO output speed configuration.
 *
 * Determines how fast the output signal changes.
 * Used only for output and alternate function modes.
 */
typedef enum
{
	GPIO_SPEED_LOW = 0, /*!< 2 MHz */
	GPIO_SPEED_MEDIUM, /*!< 10 MHz */
	GPIO_SPEED_HIGH /*!< 50 MHz */
} GPIO_Speed_t;

/**
 * @brief GPIO pull-up / pull-down configuration.
 *
 * Selects the internal pull-up or pull-down resistors.
 */
typedef enum
{
	GPIO_NOPULL = 0, /*!< No pull-up or pull-down */
	GPIO_PULLUP, /*!< Pull-up resistor enabled */
	GPIO_PULLDOWN /*!< Pull-down resistor enabled */
} GPIO_PuPd_t;

/**************************************
 * @def STRUCTURES USED FOR GPIO DRIVER
 **************************************/

/**
 * @brief Configuration structure for a GPIO pin
 */
typedef struct
{
	GPIO_PinNumber_t GPIO_PinNumber; /*!< Pin number (0 to 15) */
	GPIO_Mode_t GPIO_PinMode; /*!< Pin mode: Input, Output, Alternate Function, Analog */
	GPIO_Speed_t GPIO_PinSpeed; /*!< Output speed: Low, Medium, High */
	GPIO_PuPd_t GPIO_PinPuPdControl; /*!< Pull-up/Pull-down configuration */
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
 * @brief  Configures the clock signal for a given GPIO port.
 *
 * This function enables or disables the peripheral clock associated with
 * the specified GPIO port using the EnorDi flag.
 *
 * @param[in] pGPIOHandle   Pointer to a GPIO handle structure containing
 *                         the configuration information for the specified GPIO pin.
 * @param[in] EnorDi  ENABLE to activate clock, DISABLE to deactivate
 *
 * @return None
 */
void GPIO_PeriClockControl(GPIO_Handle_t *pGPIOHandle, uint8_t enorDi);

/**
 * @brief  Initializes the specified GPIO pin based on user configuration.
 *
 * This function configures the GPIO pin direction, speed, pull-up/pull-down settings,
 * output type, and alternate functionality based on the values defined in the provided
 * GPIO handle structure.
 *
 * @param[in] pGPIOHandle  Pointer to a GPIO handle structure containing
 *                         the configuration information for the specified GPIO pin.
 *
 * @return None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

/**
 * @brief  Resets all the configuration registers of the specified GPIO port.
 *
 * This function de-initializes the GPIO port by resetting it to its default state.
 * It affects the entire port, not individual pins.
 *
 * @param[in] GPIOx  GPIO port base address (e.g., GPIOA, GPIOB, ...)
 *
 * @return None
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/**
 * @brief  Reads the logic level of the specified GPIO input pin.
 *
 * This function returns the logic level (0 or 1) of a single pin in a given GPIO port.
 *
 * @param[in] GPIOx     GPIO port base address
 * @param[in] PinNumber GPIO pin number (0 to 15)
 *
 * @return uint8_t      Logic level (0 or 1)
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, GPIO_PinNumber_t pinNumber);

/**
 * @brief  Reads the entire input value of a GPIO port.
 *
 * This function returns the 16-bit value representing the logic levels of all pins
 * in the specified GPIO port.
 *
 * @param[in] GPIOx  GPIO port base address
 *
 * @return uint16_t  16-bit value of input port
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/**
 * @brief  Sets or clears the specified GPIO output pin.
 *
 * This function writes a logic level (0 or 1) to the output data register for
 * the selected pin in the given GPIO port.
 *
 * @param[in] GPIOx     GPIO port base address
 * @param[in] PinNumber GPIO pin number (0 to 15)
 * @param[in] Value     Logic level (0 = reset, 1 = set)
 *
 * @return None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, GPIO_PinNumber_t pinNumber, uint8_t value);

/**
 * @brief  Writes a 16-bit value to the entire GPIO output port.
 *
 * This function sets the output levels of all pins in the specified GPIO port
 * simultaneously using the provided value.
 *
 * @param[in] GPIOx  GPIO port base address
 * @param[in] Value  16-bit value to write to output port
 *
 * @return None
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);

/**
 * @brief  Toggles the current output state of the specified GPIO pin.
 *
 * This function inverts the logic level of the selected output pin.
 *
 * @param[in] GPIOx     GPIO port base address
 * @param[in] PinNumber GPIO pin number (0 to 15)
 *
 * @return None
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, GPIO_PinNumber_t pinNumber);

#endif /* CUSTOMDRIVERS_INC_STM32F103XX_GPIO_DRIVER_H_ */
