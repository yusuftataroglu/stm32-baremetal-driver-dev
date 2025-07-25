/**************************************************************************************
 * @file    stm32f103xx_spi_driver.h
 * @author  Yusuf
 * @brief   SPI peripheral driver header file (bare-metal, STM32F103RB)
 **************************************************************************************/

#ifndef CUSTOMDRIVERS_INC_STM32F103XX_SPI_DRIVER_H_
#define CUSTOMDRIVERS_INC_STM32F103XX_SPI_DRIVER_H_

#include "stm32f103xx.h"

/*********************************************************************************************
 * @section ENUMERATIONS
 *********************************************************************************************/

/**
 * @brief SPI operating mode selection (Master or Slave)
 */
typedef enum
{
	SPI_MODE_SLAVE = 0,
	SPI_MODE_MASTER
} SPI_DeviceMode_t;

/**
 * @brief SPI bus configuration
 */
typedef enum
{
	SPI_BUS_FULL_DUPLEX = 0,
	SPI_BUS_HALF_DUPLEX,
	SPI_BUS_SIMPLEX_RXONLY
} SPI_BusConfig_t;

/**
 * @brief SPI data frame format
 */
typedef enum
{
	SPI_DFF_8BITS = 0,
	SPI_DFF_16BITS
} SPI_DFF_t;

/**
 * @brief Clock polarity options
 */
typedef enum
{
	SPI_CPOL_LOW = 0,
	SPI_CPOL_HIGH
} SPI_CPOL_t;

/**
 * @brief Clock phase options
 */
typedef enum
{
	SPI_CPHA_FIRST_EDGE = 0,
	SPI_CPHA_SECOND_EDGE
} SPI_CPHA_t;

/**
 * @brief Software slave management
 */
typedef enum
{
	SPI_SSM_DISABLED = 0,
	SPI_SSM_ENABLED
} SPI_SSM_t;

/*********************************************************************************************
 * @section DATA STRUCTURES
 *********************************************************************************************/

/**
 * @brief SPI configuration structure
 */
typedef struct
{
	SPI_DeviceMode_t SPI_DeviceMode; /*!< Master or Slave mode */
	SPI_BusConfig_t SPI_BusConfig; /*!< Full-duplex, half-duplex, or RX-only */
	uint8_t SPI_SclkSpeed; /*!< Baud rate control: BR[2:0] in CR1 (0 to 7) */
	SPI_DFF_t SPI_DFF; /*!< Data frame format: 8 or 16 bits */
	SPI_CPOL_t SPI_CPOL; /*!< Clock polarity */
	SPI_CPHA_t SPI_CPHA; /*!< Clock phase */
	SPI_SSM_t SPI_SSM; /*!< Software slave management enable/disable */
} SPI_Config_t;

/**
 * @brief SPI handle structure
 */
typedef struct
{
	SPI_RegDef_t *pSPIx; /*!< Base address of the SPI peripheral */
	SPI_Config_t SPI_Config; /*!< SPI configuration settings */
} SPI_Handle_t;

/*********************************************************************************************
 * @section DRIVER API PROTOTYPES
 *********************************************************************************************/

/**
 * @brief  Enables or disables the peripheral clock for the specified SPI module.
 *
 * This function enables or disables the SPI peripheral clock via the RCC APB2 or APB1 bus,
 * depending on which SPI instance is used (SPI1, SPI2, etc.).
 *
 * @param[in] pSPIHandle Pointer to the SPI handle structure for the selected SPI peripheral.
 * @param[in] EnOrDi     ENABLE to enable clock, DISABLE to disable.
 *
 * @return None
 */
void SPI_PeriClockControl(SPI_Handle_t *pSPIHandle, uint8_t enOrDi);

/**
 * @brief  Initializes the given SPI peripheral with specified configurations.
 *
 * Configures the SPI control registers (CR1 and CR2) based on parameters
 * provided in the SPI_Handle_t structure.
 *
 * @param[in] pSPIHandle Pointer to SPI handle structure containing the peripheral base address
 *                       and user-defined configuration settings.
 *
 * @return None
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);

/**
 * @brief  De-initializes the SPI peripheral.
 *
 * Resets all SPI registers to their default values by enabling the reset
 * bit in the RCC APB2RSTR or APB1RSTR register depending on the SPI instance.
 *
 * @param[in] pSPIx Pointer to the base address of the SPI peripheral.
 *
 * @return None
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/**
 * @brief  Sends data through the SPI peripheral.
 *
 * Transfers `len` number of bytes from the given Tx buffer over the SPI interface.
 *
 * @param[in] pSPIx     Base address of the SPI peripheral.
 * @param[in] pTxBuffer Pointer to the transmission buffer.
 * @param[in] len       Length of data to transmit in bytes.
 *
 * @return None
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);

/**
 * @brief  Receives data from the SPI peripheral.
 *
 * Receives `len` number of bytes from the SPI bus into the given Rx buffer.
 *
 * @param[in]  pSPIx     Base address of the SPI peripheral.
 * @param[out] pRxBuffer Pointer to the reception buffer.
 * @param[in]  len       Length of data to receive in bytes.
 *
 * @return None
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/**
 * @brief  Enables or disables the SPI peripheral.
 *
 * Sets or clears the SPE (SPI Enable) bit in the CR1 register.
 *
 * @note This bit must be enabled before communication.
 *
 * @param[in] pSPIx  Base address of the SPI peripheral.
 * @param[in] EnOrDi ENABLE to activate, DISABLE to deactivate the SPI.
 *
 * @return None
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t enOrDi);

/**
 * @brief  Checks the status of a specific SPI flag.
 *
 * Evaluates the status of the specified flag (e.g., TXE, RXNE, BSY) in the SPI SR register.
 *
 * @param[in] pSPIx    Base address of the SPI peripheral.
 * @param[in] FlagName Symbolic constant indicating the flag to check.
 *
 * @return Flag status: SET or RESET.
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName);


#endif /* CUSTOMDRIVERS_INC_STM32F103XX_SPI_DRIVER_H_ */
