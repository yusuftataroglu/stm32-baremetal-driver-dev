/**************************************************************************************
 * @file    core_nvic.h
 * @author  Yusuf
 * @brief   NVIC driver header file (bare-metal, STM32F103RB)
 **************************************************************************************/

#ifndef CUSTOMDRIVERS_INC_CORE_NVIC_H_
#define CUSTOMDRIVERS_INC_CORE_NVIC_H_

#include <stdint.h>

/**
 * @brief  IRQ numbers for all peripherals (STM32F103RB specific)
 *
 * These values correspond to the position in the vector table.
 */
typedef enum
{
    /* Cortex-M3 Processor Exceptions (internal use) */
    NonMaskableInt_IRQn     = -14,
    MemoryManagement_IRQn   = -12,
    BusFault_IRQn           = -11,
    UsageFault_IRQn         = -10,
    SVCall_IRQn             = -5,
    DebugMonitor_IRQn       = -4,
    PendSV_IRQn             = -2,
    SysTick_IRQn            = -1,

    /* STM32F103RB specific IRQ numbers */
    WWDG_IRQn               = 0,
    PVD_IRQn                = 1,
    TAMPER_IRQn             = 2,
    RTC_IRQn                = 3,
    FLASH_IRQn              = 4,
    RCC_IRQn                = 5,
    EXTI0_IRQn              = 6,
    EXTI1_IRQn              = 7,
    EXTI2_IRQn              = 8,
    EXTI3_IRQn              = 9,
    EXTI4_IRQn              = 10,
    DMA1_Channel1_IRQn      = 11,
    DMA1_Channel2_IRQn      = 12,
    DMA1_Channel3_IRQn      = 13,
    DMA1_Channel4_IRQn      = 14,
    DMA1_Channel5_IRQn      = 15,
    DMA1_Channel6_IRQn      = 16,
    DMA1_Channel7_IRQn      = 17,
    ADC1_2_IRQn             = 18,
    USB_HP_CAN_TX_IRQn      = 19,
    USB_LP_CAN_RX0_IRQn     = 20,
    CAN_RX1_IRQn            = 21,
    CAN_SCE_IRQn            = 22,
    EXTI9_5_IRQn            = 23,
    TIM1_BRK_IRQn           = 24,
    TIM1_UP_IRQn            = 25,
    TIM1_TRG_COM_IRQn       = 26,
    TIM1_CC_IRQn            = 27,
    TIM2_IRQn               = 28,
    TIM3_IRQn               = 29,
    TIM4_IRQn               = 30,
    I2C1_EV_IRQn            = 31,
    I2C1_ER_IRQn            = 32,
    I2C2_EV_IRQn            = 33,
    I2C2_ER_IRQn            = 34,
    SPI1_IRQn               = 35,
    SPI2_IRQn               = 36,
    USART1_IRQn             = 37,
    USART2_IRQn             = 38,
    USART3_IRQn             = 39,
    EXTI15_10_IRQn          = 40,
    RTCAlarm_IRQn           = 41,
    USBWakeUp_IRQn          = 42

} IRQNumber_t;


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
void NVIC_EnableIRQ(IRQNumber_t IRQNumber);

/**
 * @brief  Disables a specific interrupt in the NVIC.
 * @param  IRQNumber: Interrupt number of the EXTI line (e.g., EXTI0_IRQn, EXTI1_IRQn).
 * @return None
 */
void NVIC_DisableIRQ(IRQNumber_t IRQNumber);

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
void NVIC_SetPriority(IRQNumber_t IRQNumber, uint8_t priority);

#endif /* CUSTOMDRIVERS_INC_CORE_NVIC_H_ */
