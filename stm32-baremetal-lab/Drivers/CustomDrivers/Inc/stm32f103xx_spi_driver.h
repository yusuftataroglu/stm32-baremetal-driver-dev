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
	uint8_t *pTxBuffer; /*!< Pointer to the transmission buffer.*/
	uint8_t *pRxBuffer; /*!< Pointer to the reception buffer.*/
	uint32_t TxLen; /*!< Length of data to transmit in bytes.*/
	uint32_t RxLen; /*!< Length of data to receive in bytes.*/
	uint8_t TxState; /*!< State of SPI which is checked before transmit data.*/
	uint8_t RxState; /*!< State of SPI which is checked before receive data*/
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

/**
 * @brief  Handles all SPI-related interrupt events.
 *
 * This function processes interrupt requests triggered by the SPI peripheral. It checks the
 * status and control bits of relevant flags (TXE, RXNE, OVR) and delegates the handling to
 * appropriate sub-functions based on the current mode of operation (transmit, receive, or
 * full-duplex).
 *
 * The handler performs the following:
 * - Checks if the Transmit Buffer Empty (TXE) interrupt is enabled and pending, and invokes
 *   the internal transmit ISR handler to continue sending remaining data.
 * - Checks if the Receive Buffer Not Empty (RXNE) interrupt is enabled and pending, and invokes
 *   the internal receive ISR handler to read incoming data into the user buffer.
 * - Checks for Overrun Error (OVR) condition, and if detected, invokes the internal OVR handler
 *   to clear the error by reading the SR and DR registers.
 *
 * This function should be called from the user-defined IRQ handler (e.g., SPI1_IRQHandler) and
 * requires the SPI handle to access the associated transmit and receive buffer context.
 *
 * @param[in] pSPIHandle Pointer to the SPI handle structure containing the peripheral base address
 *                       and transfer state information.
 *
 * @note This handler is interrupt-safe and only processes events when the corresponding interrupt
 *       enable bits are set in the CR2 register. It does not perform any blocking operations.
 *
 * @return None.
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/**
 * @brief  Initiates transmission of data over SPI using interrupt mode.
 *
 * Stores the transmission buffer and length inside the SPI handle and enables
 * the TXEIE (TX buffer empty interrupt) to trigger interrupt-driven data transfer.
 *
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 * @param[in] pTxBuffer  Pointer to the transmission buffer.
 * @param[in] len        Number of bytes to transmit.
 *
 * @return Transmission state: 1 = busy in TX, 0 = ready.
 */
uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);

/**
 * @brief  Initiates SPI data reception using interrupt mode.
 *
 * Configures the SPI peripheral to receive data asynchronously using RXNE interrupt.
 * The actual data handling will be performed inside the SPI interrupt handler.
 *
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 * @param[in] pRxBuffer  Pointer to the buffer where received data will be stored.
 * @param[in] len        Number of bytes to receive.
 *
 * @return  SPI status: SPI_READY if reception started successfully,
 *                      SPI_BUSY_IN_RX if SPI is already receiving.
 */
uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

/**
 * @brief  SPI application event callback (weak implementation).
 *
 * This function is called by the SPI driver to notify the application about events.
 * The user application may override this function to handle specific SPI events.
 *
 * @param[in] pSPIHandle Pointer to the SPI handle.
 * @param[in] AppEv      Event identifier.
 *                        - SPI_EVENT_TX_CMPLT
 *                        - SPI_EVENT_RX_CMPLT
 *                        - SPI_EVENT_OVR_ERR
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* CUSTOMDRIVERS_INC_STM32F103XX_SPI_DRIVER_H_ */
