#include "as5600.h"
#include "SEGGER_RTT.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdint.h>

AS5600_Status_t AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint16_t *pAngle)
{
    HAL_StatusTypeDef status;

    uint8_t data[2] ={0,0};

    if (pAngle == NULL) 
    {
        return AS5600_ERROR;
    }

    status = HAL_I2C_Mem_Read(hi2c,
                            AS5600_ADDR,
                            AS5600_ANGLE_HIGH,
                            AS5600_ADDR_SIZE_8bit,
                            data,
                            2,
                            100
                              );
    if (status != HAL_OK)
    {
        return AS5600_ERROR;
    }

    *pAngle = (((uint16_t)(uint16_t)data[0] << 8U) | (uint16_t)data[1]) & 0x0FFFU;
    SEGGER_RTT_printf(0, "AS5600: Read Sensor OK\r\n");
    return AS5600_OK;
}

AS5600_Status_t AS5600_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *pStatus)
{
    if (HAL_I2C_IsDeviceReady(hi2c,AS5600_ADDR, 5,100 ) != HAL_OK)
    {
        SEGGER_RTT_WriteString(0, "NO ACK/r/n");
    }
    else {
        SEGGER_RTT_WriteString(0, "OK ACK/r/n");
    }
    uint8_t status_reg = 0;
    if (HAL_I2C_Mem_Read(hi2c, AS5600_ADDR, AS5600_STATUS, AS5600_ADDR_SIZE_8bit, &status_reg,1,100) != HAL_OK)
    {
        SEGGER_RTT_printf(0, "AS5600: Read STATUS FAIL\r\n");
        return AS5600_ERROR;
    }
    if(pStatus != NULL)
    {
        *pStatus = status_reg;
    }
    
    if (!(status_reg & AS5600_STATUS_MD))
    {
        SEGGER_RTT_printf(0, "Magnet: KHONG TIM THAY NAM CHAM\r\n");
        return AS5600_ERROR;
    }
     else if (status_reg & AS5600_STATUS_ML)
    {
        SEGGER_RTT_printf(0, "Magnet: NAM CHAM XA\r\n");
    }
    else if (status_reg & AS5600_STATUS_MH)
    {
        SEGGER_RTT_printf(0, "Magnet: NAM CHAM GAN\r\n");
    }
    else
    {
        SEGGER_RTT_printf(0, "Magnet: OK - TOI UU\r\n");
    }
    return AS5600_OK;
}
