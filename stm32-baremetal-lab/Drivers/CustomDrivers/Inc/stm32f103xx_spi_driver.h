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
    SPI_DeviceMode_t SPI_DeviceMode;  /*!< Master or Slave mode */
    SPI_BusConfig_t SPI_BusConfig;    /*!< Full-duplex, half-duplex, or RX-only */
    uint8_t SPI_SclkSpeed;            /*!< Baud rate control: BR[2:0] in CR1 (0 to 7) */
    SPI_DFF_t SPI_DFF;                /*!< Data frame format: 8 or 16 bits */
    SPI_CPOL_t SPI_CPOL;              /*!< Clock polarity */
    SPI_CPHA_t SPI_CPHA;              /*!< Clock phase */
    SPI_SSM_t SPI_SSM;                /*!< Software slave management enable/disable */
} SPI_Config_t;

/**
 * @brief SPI handle structure
 */
typedef struct
{
    SPI_RegDef_t *pSPIx;      /*!< Base address of the SPI peripheral */
    SPI_Config_t SPI_Config;  /*!< SPI configuration settings */
} SPI_Handle_t;

/*********************************************************************************************
 * @section DRIVER API PROTOTYPES
 *********************************************************************************************/

/**
 * @brief  Enables or disables SPI peripheral clock
 */
void SPI_PeriClockControl(SPI_Handle_t *pSPIHandle, uint8_t EnOrDi);

/**
 * @brief  Initializes the SPI peripheral
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);

/**
 * @brief  Resets the SPI peripheral
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/**
 * @brief  Sends data over SPI
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);

/**
 * @brief  Receives data over SPI
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/**
 * @brief  Enables or disables the SPI peripheral
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/**
 * @brief  Returns the status of a specific SPI flag
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

#endif /* CUSTOMDRIVERS_INC_STM32F103XX_SPI_DRIVER_H_ */
