/*
 * stm32f103xx_i2c_driver.h
 *
 *  Created on: Jul 28, 2025
 *      Author: yusuf
 */

#ifndef CUSTOMDRIVERS_INC_STM32F103XX_I2C_DRIVER_H_
#define CUSTOMDRIVERS_INC_STM32F103XX_I2C_DRIVER_H_

/********************************** I2C STATUS FLAG DEFINITIONS **********************************/

#define I2C_FLAG_TXE       (1 << 7)  /*!< Data register empty (Transmitter) */
#define I2C_FLAG_RXNE      (1 << 6)  /*!< Data register not empty (Receiver) */
#define I2C_FLAG_STOPF     (1 << 4)  /*!< Stop detection flag */
#define I2C_FLAG_ADD10     (1 << 3)  /*!< 10-bit header sent flag */
#define I2C_FLAG_BTF       (1 << 2)  /*!< Byte transfer finished */
#define I2C_FLAG_ADDR      (1 << 1)  /*!< Address sent/matched */
#define I2C_FLAG_SB        (1 << 0)  /*!< Start bit generated */

#define I2C_FLAG_TIMEOUT   (1 << 14) /*!< Timeout or Tlow error flag */
#define I2C_FLAG_AF        (1 << 10) /*!< Acknowledge failure */
#define I2C_FLAG_ARLO      (1 << 9)  /*!< Arbitration lost */
#define I2C_FLAG_BERR      (1 << 8)  /*!< Bus error */

/********************************** I2C CONFIGURATION STRUCTURE **********************************
 * @brief Configuration structure for I2C peripheral initialization.
 * This structure is filled by the user and passed to the driver during initialization.
 *************************************************************************************************/
typedef struct
{
	uint32_t I2C_SCLSpeed; /*!< Specifies the clock speed for I2C (e.g. 100kHz, 400kHz).
	 Must be one of @I2C_SCL_SPEED macros. */

	uint8_t I2C_DeviceAddress; /*!< Specifies the device's own address in 7-bit mode. */

	uint8_t I2C_ACKControl; /*!< Specifies whether ACKing is enabled or disabled.
	 Must be one of @I2C_ACK macros. */

	uint16_t I2C_FMDutyCycle; /*!< Specifies duty cycle in fast mode.
	 Must be one of @I2C_FM_DUTY macros. */
} I2C_Config_t;

/********************************** @I2C_SCL_SPEED **********************************
 * @brief Defines supported clock speeds for I2C communication.
 ***********************************************************************************/
typedef enum
{
    I2C_SCL_SPEED_SM = 100000U,   /*!< Standard Mode: 100 kHz */
    I2C_SCL_SPEED_FM4K = 400000U  /*!< Fast Mode: 400 kHz */
} I2C_SCLSpeed_t;

/********************************** @I2C_ACK_CONTROL **********************************
 * @brief Enables or disables ACK control after receiving a byte.
 *************************************************************************************/
typedef enum
{
    I2C_ACK_DISABLE = 0U,
    I2C_ACK_ENABLE
} I2C_ACKControl_t;

/********************************** @I2C_FM_DUTY_CYCLE **********************************
 * @brief Defines duty cycle configuration for Fast Mode (FM) I2C.
 **************************************************************************************/
typedef enum
{
    I2C_FM_DUTY_2 = 0U,     /*!< Tlow/Thigh = 2 */
    I2C_FM_DUTY_16_9        /*!< Tlow/Thigh = 16/9 */
} I2C_FMDutyCycle_t;

/********************************** @I2C_STATE **********************************
 * @brief Represents current I2C communication state.
 *******************************************************************************/
typedef enum
{
    I2C_READY = 0U,
    I2C_BUSY_IN_RX,
    I2C_BUSY_IN_TX
} I2C_State_t;

/********************************** I2C HANDLE STRUCTURE **********************************
 * @brief Handle structure to manage I2C peripheral and associated configuration/state.
 *****************************************************************************************/
typedef struct
{
	I2C_RegDef_t *pI2Cx; /*!< Pointer to the I2C peripheral base address. */

	I2C_Config_t I2C_Config; /*!< I2C configuration settings provided by the user. */

	uint8_t *pTxBuffer; /*!< Pointer to transmission buffer (used in interrupt mode). */
	uint8_t *pRxBuffer; /*!< Pointer to reception buffer (used in interrupt mode). */

	uint32_t TxLen; /*!< Number of bytes to transmit. */
	uint32_t RxLen; /*!< Number of bytes to receive. */

	uint8_t TxRxState; /*!< Indicates the current communication state: TX, RX, or READY.
	 Must be one of @I2C_STATE macros. */

	uint8_t DevAddr; /*!< Slave address for communication. */

	uint32_t RxSize; /*!< Remaining bytes for reception (used in interrupt mode). */

	uint8_t Sr; /*!< Whether or not to send repeated START after current transmission. */
} I2C_Handle_t;

/*********************************************************************************************
 * @section DRIVER API PROTOTYPES
 *********************************************************************************************/

/**
 * @brief  Enables or disables the peripheral clock for the specified I2C module.
 *
 * This function enables or disables the I2C peripheral clock via the APB1 bus.
 *
 * @param[in] pI2CHandle Pointer to the I2C handle structure for the selected I2C peripheral.
 * @param[in] EnOrDi     ENABLE to enable clock, DISABLE to disable.
 *
 * @return None
 */
void I2C_PeriClockControl(I2C_Handle_t *pI2CHandle, uint8_t enOrDi);

/**
 * @brief Initializes the I2C peripheral with user-defined settings.
 * @param[in] pI2CHandle Pointer to the I2C handle structure.
 * @return None
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);

/**
 * @brief Disables the I2C peripheral and resets configuration.
 * @param[in] pI2CHandle Pointer to the I2C handle structure.
 * @return None
 */
void I2C_DeInit(I2C_Handle_t *pI2CHandle);

/**
 * @brief Sends data from master to slave using blocking mode.
 * @param[in] pI2CHandle Pointer to the I2C handle structure.
 * @param[in] pTxBuffer Pointer to the data buffer to transmit.
 * @param[in] len Number of bytes to transmit.
 * @param[in] slaveAddr 7-bit address of the slave.
 * @param[in] repeatedStart If set, generates repeated start instead of stop.
 * @return None
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
                        uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart);

/**
 * @brief Receives data in master mode using blocking mode.
 * @param[in] pI2CHandle Pointer to the I2C handle structure.
 * @param[out] pRxBuffer Pointer to buffer to store received data.
 * @param[in] len Number of bytes to receive.
 * @param[in] slaveAddr 7-bit address of the slave.
 * @param[in] repeatedStart If set, generates repeated start instead of stop.
 * @return None
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
                           uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart);

/**
 * @brief Initiates I2C master transmission using interrupt-driven mode.
 * @param[in] pI2CHandle Pointer to the I2C handle structure.
 * @param[in] pTxBuffer Pointer to the data buffer to transmit.
 * @param[in] len Number of bytes to transmit.
 * @param[in] slaveAddr 7-bit address of the slave.
 * @param[in] repeatedStart If set, generates repeated start instead of stop.
 * @return I2C_BUSY_IN_TX if already transmitting, I2C_READY otherwise.
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
                             uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart);

/**
 * @brief Initiates I2C master reception using interrupt-driven mode.
 * @param[in] pI2CHandle Pointer to the I2C handle structure.
 * @param[out] pRxBuffer Pointer to the buffer to store received data.
 * @param[in] len Number of bytes to receive.
 * @param[in] slaveAddr 7-bit address of the slave.
 * @param[in] repeatedStart If set, generates repeated start instead of stop.
 * @return I2C_BUSY_IN_RX if already receiving, I2C_READY otherwise.
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
                                uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart);

/**
 * @brief Configures NVIC interrupt for given IRQ number.
 * @param[in] IRQNumber IRQ number for I2C peripheral.
 * @param[in] EnOrDi ENABLE or DISABLE macro.
 * @return None
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);

/**
 * @brief Handles I2C event-related interrupts.
 * @param[in] pI2CHandle Pointer to the I2C handle structure.
 * @return None
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);

/**
 * @brief Handles I2C error-related interrupts.
 * @param[in] pI2CHandle Pointer to the I2C handle structure.
 * @return None
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/**
 * @brief Application callback to handle I2C events like TX complete, RX complete, errors.
 * @param[in] pI2CHandle Pointer to the I2C handle structure.
 * @param[in] AppEv Event type: TX_COMPLETE, RX_COMPLETE, STOP_DETECTED, etc.
 * @return None
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

/**
 * @brief Enables or disables the I2C peripheral.
 * @param[in] pI2Cx Pointer to I2C peripheral.
 * @param[in] EnOrDi ENABLE or DISABLE macro.
 * @return None
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/**
 * @brief Manages ACK bit during communication.
 * @param[in] pI2Cx Pointer to I2C peripheral.
 * @param[in] EnOrDi ENABLE or DISABLE macro.
 * @return None
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/**
 * @brief Returns the status of a given I2C flag.
 * @param[in] pI2Cx Pointer to I2C peripheral.
 * @param[in] flagName Flag macro (e.g., I2C_FLAG_TXE).
 * @return SET or RESET
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName);


#endif /* CUSTOMDRIVERS_INC_STM32F103XX_I2C_DRIVER_H_ */
