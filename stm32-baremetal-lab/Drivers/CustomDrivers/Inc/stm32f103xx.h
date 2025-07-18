/*
 * stm32f103xx.h
 *
 *  Created on: Jul 17, 2025
 *      Author: yusuf
 */

#ifndef CUSTOMDRIVERS_INC_STM32F103XX_H_
#define CUSTOMDRIVERS_INC_STM32F103XX_H_

#include <stdint.h>

/********************************** START: Processor Specific Details **********************************/
#define __vo    volatile

/********************************** BASE ADDRESSES OF FLASH AND SRAM **********************************/
#define FLASH_BASEADDR          0x08000000U
#define SRAM_BASEADDR           0x20000000U

/********************************** BASE ADDRESSES OF AHB AND APB BUS **********************************/
#define PERIPH_BASEADDR         0x40000000U
#define APB1PERIPH_BASEADDR     PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR     (PERIPH_BASEADDR + 0x10000U)
#define AHBPERIPH_BASEADDR      (PERIPH_BASEADDR + 0x18000U)

/********************************** BASE ADDRESSES OF PERIPHERALS ON AHB **********************************/
#define RCC_BASEADDR            (AHBPERIPH_BASEADDR  + 0x9000U)
//...

/********************************** BASE ADDRESSES OF PERIPHERALS ON APB2 *********************************/
#define AFIO_BASEADDR        	(APB2PERIPH_BASEADDR)
#define EXTI_BASEADDR        	(APB2PERIPH_BASEADDR + 0x0400U)
#define GPIOA_BASEADDR          (APB2PERIPH_BASEADDR + 0x0800U)
#define GPIOB_BASEADDR          (APB2PERIPH_BASEADDR + 0x0C00U)
#define GPIOC_BASEADDR          (APB2PERIPH_BASEADDR + 0x1000U)
#define SPI1_BASEADDR          	(APB2PERIPH_BASEADDR + 0x3000U)
#define USART1_BASEADDR         (APB2PERIPH_BASEADDR + 0x3800U)
// ...

/********************************** BASE ADDRESSES OF PERIPHERALS ON APB1 *********************************/
#define TIM2_BASEADDR			(APB1PERIPH_BASEADDR)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400U)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800U)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00U)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000U)
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800U)
//...

/****************************** PERIPHERAL REGISTER STRUCTURE DEFINITIONS ******************************/

/********************************** RCC REGISTER STRUCTURE DEFINITION **********************************
 * Base Address : 0x40021000 (AHBPERIPH_BASEADDR + 0x9000)
 * Reference Manual : RM0008, Section 7 (Reset and clock control (RCC))
 ********************************************************************************************************/
typedef struct
{
	__vo uint32_t CR; /*!< Clock Control Register,               			Address offset: 0x00 */
	__vo uint32_t CFGR; /*!< Clock Configuration Register,         			Address offset: 0x04 */
	__vo uint32_t CIR; /*!< Clock Interrupt Register,             			Address offset: 0x08 */
	__vo uint32_t APB2RSTR; /*!< APB2 Peripheral Reset Register,       		Address offset: 0x0C */
	__vo uint32_t APB1RSTR; /*!< APB1 Peripheral Reset Register,       		Address offset: 0x10 */
	__vo uint32_t AHBENR; /*!< AHB Peripheral Clock Enable Register, 		Address offset: 0x14 */
	__vo uint32_t APB2ENR; /*!< APB2 Peripheral Clock Enable Register,		Address offset: 0x18 */
	__vo uint32_t APB1ENR; /*!< APB1 Peripheral Clock Enable Register,		Address offset: 0x1C */
	__vo uint32_t BDCR; /*!< Backup Domain Control Register,       			Address offset: 0x20 */
	__vo uint32_t CSR; /*!< Control/Status Register,              			Address offset: 0x24 */
} RCC_RegDef_t;

/********************************** GPIO REGISTER STRUCTURE DEFINITION **********************************
 * Base Addresses:
 *    GPIOA : 0x40010800
 *    GPIOB : 0x40010C00
 *    GPIOC : 0x40011000
 *
 * Reference Manual: RM0008, Section 9 (General-purpose I/Os)
 ********************************************************************************************************/
typedef struct
{
	__vo uint32_t CRL; /*!< Port Configuration Register Low (pins 0–7),  	 	Address offset: 0x00 */
	__vo uint32_t CRH; /*!< Port Configuration Register High (pins 8–15), 		Address offset: 0x04 */
	__vo uint32_t IDR; /*!< Input Data Register,                          		Address offset: 0x08 */
	__vo uint32_t ODR; /*!< Output Data Register,                         		Address offset: 0x0C */
	__vo uint32_t BSRR; /*!< Bit Set/Reset Register,                     	  	Address offset: 0x10 */
	__vo uint32_t BRR; /*!< Bit Reset Register,                          	 	Address offset: 0x14 */
	__vo uint32_t LCKR; /*!< Port Configuration Lock Register,           	 	Address offset: 0x18 */
} GPIO_RegDef_t;

/************************************** PERIPHERAL REGISTERS **************************************/
#define GPIOA 	((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB   ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC   ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define RCC     ((RCC_RegDef_t*)  RCC_BASEADDR)

/******************************** RCC PERIPHERAL CLOCK ENABLE MACROS ********************************/

#define GPIOA_PCLK_EN()     (RCC->APB2ENR |= (1 << 2))   /* Enable clock for GPIOA */
#define GPIOB_PCLK_EN()     (RCC->APB2ENR |= (1 << 3))   /* Enable clock for GPIOB */
#define GPIOC_PCLK_EN()     (RCC->APB2ENR |= (1 << 4))   /* Enable clock for GPIOC */

#define AFIO_PCLK_EN()      (RCC->APB2ENR |= (1 << 0))   /* Enable clock for AFIO */

#define USART1_PCLK_EN()    (RCC->APB2ENR |= (1 << 14))  /* Enable clock for USART1 (on APB2) */
#define USART2_PCLK_EN()    (RCC->APB1ENR |= (1 << 17))  /* Enable clock for USART2 (on APB1) */
#define USART3_PCLK_EN()    (RCC->APB1ENR |= (1 << 18))  /* Enable clock for USART3 (on APB1) */
#define UART4_PCLK_EN()    	(RCC->APB1ENR |= (1 << 19))  /* Enable clock for UART4 (on APB1) */
#define UART5_PCLK_EN()    	(RCC->APB1ENR |= (1 << 20))  /* Enable clock for UART5 (on APB1) */

#define SPI1_PCLK_EN()      (RCC->APB2ENR |= (1 << 12))  /* Enable clock for SPI1 */

#define I2C1_PCLK_EN()      (RCC->APB1ENR |= (1 << 21))  /* Enable clock for I2C1 */
#define I2C2_PCLK_EN()      (RCC->APB1ENR |= (1 << 22))  /* Enable clock for I2C2 */

/******************************** RCC PERIPHERAL CLOCK DISABLE MACROS ********************************/

#define GPIOA_PCLK_DI()     (RCC->APB2ENR &= ~(1 << 2))   /* Disable clock for GPIOA */
#define GPIOB_PCLK_DI()     (RCC->APB2ENR &= ~(1 << 3))   /* Disable clock for GPIOB */
#define GPIOC_PCLK_DI()     (RCC->APB2ENR &= ~(1 << 4))   /* Disable clock for GPIOC */

#define AFIO_PCLK_DI()      (RCC->APB2ENR &= ~(1 << 0))   /* Disable clock for AFIO */

#define USART1_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 14))  /* Disable clock for USART1 (on APB2) */
#define USART2_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 17))  /* Disable clock for USART2 (on APB1) */
#define USART3_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 18))  /* Disable clock for USART3 (on APB1) */
#define UART4_PCLK_DI()    	(RCC->APB1ENR &= ~(1 << 19))  /* Disable clock for UART4 (on APB1) */
#define UART5_PCLK_DI()    	(RCC->APB1ENR &= ~(1 << 20))  /* Disable clock for UART5 (on APB1) */

#define SPI1_PCLK_DI()      (RCC->APB2ENR &= ~(1 << 12))  /* Disable clock for SPI1 */

#define I2C1_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 21))  /* Disable clock for I2C1 */
#define I2C2_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 22))  /* Disable clock for I2C2 */

#endif /* CUSTOMDRIVERS_INC_STM32F103XX_H_ */
