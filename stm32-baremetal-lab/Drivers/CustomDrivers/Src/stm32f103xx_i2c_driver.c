/*
 * stm32f103xx_i2c_driver.c
 *
 *  Created on: Jul 28, 2025
 *      Author: yusuf
 */

#include "stm32f103xx.h"
#include "stm32f103xx_i2c_driver.h"

void I2C_PeriClockControl(I2C_Handle_t *pI2CHandle, uint8_t enOrDi)
{
	if (enOrDi == ENABLE)
	{
		if (pI2CHandle->pI2Cx == I2C1)
			I2C1_PCLK_EN();
		else if (pI2CHandle->pI2Cx == I2C2)
			I2C2_PCLK_EN();
	}
	else
	{
		if (pI2CHandle->pI2Cx == I2C1)
			I2C1_PCLK_DI();
		else if (pI2CHandle->pI2Cx == I2C2)
			I2C2_PCLK_DI();
	}
}

/**
 * @brief Initializes the I2C peripheral with user-defined settings.
 * @param[in] pI2CHandle Pointer to the I2C handle structure.
 * @return None
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp = 0;

// 1. Enable the I2C peripheral clock
	I2C_PeriClockControl(pI2CHandle, ENABLE);

// 2. Configure ACK control bit
// (This must be done before enabling PE bit)
	pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);// Clear first
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}

// 3. Set the peripheral clock frequency (in MHz) into CR2[5:0]
	temp = 8000000U / 1000000U;// It is better to calculate dynamically
	pI2CHandle->pI2Cx->CR2 = (temp & 0x3F);

// 4. Set device own address (for slave mode), put in OAR1[7:1]
// Bit 14 must always be kept at 1 (refer to RM)
	temp = 0;
	temp |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	temp |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = temp;

// 5. Set CCR (clock control register) based on SCL speed
	uint16_t ccr_value = 0;
	temp = 0;

	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
// Standard mode
		ccr_value = 8000000U / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		temp |= (ccr_value & 0xFFF);// CCR[11:0]
	}
	else
	{
// Fast mode
		temp |= (1 << 15);// F/S bit = 1 (fast mode)
		temp |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);// Duty bit

		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = 8000000U / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			ccr_value = 8000000U / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}

		temp |= (ccr_value & 0xFFF);
	}

	pI2CHandle->pI2Cx->CCR = temp;

// 6. Set TRISE (maximum rise time)
// For SM: TRISE = (PCLK1 in MHz) + 1
// For FM: TRISE = ((PCLK1 in MHz) * 300ns) + 1
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		temp = 8000000U / 1000000U + 1;
	}
	else
	{
		temp = (8000000U * 300) / 1000000000U + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (temp & 0x3F);
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
                        uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart)
{
    // 1. Generate START condition
    pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

    // 2. Wait until SB is set
    while (! (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB)));

    // 3. Send address with write bit (0)
    slaveAddr = slaveAddr << 1; // LSB = 0 -> write
    pI2CHandle->pI2Cx->DR = slaveAddr;

    // 4. Wait until ADDR is set
    while (! (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR)));

    // 5. Clear ADDR flag by reading SR1 and SR2
    (void)pI2CHandle->pI2Cx->SR1;
    (void)pI2CHandle->pI2Cx->SR2;

    // 6. Send data until Len becomes 0
    while (len > 0)
    {
        while (! (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))); // Wait TXE = 1

        pI2CHandle->pI2Cx->DR = *pTxBuffer;
        pTxBuffer++;
        len--;
    }

    // 7. Wait for BTF = 1 (data transfer completed)
    while (! (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF)));

    // 8. Generate STOP condition if Sr is disabled (Sr = 0)
    if (repeatedStart == 0)
    {
        pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
    }
}


void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t enOrDi)
{
	if (enOrDi == ENABLE)
	{
// Enable SPE bit
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
// Disable SPE bit
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName)
{
	if (pI2Cx->SR1 & flagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
