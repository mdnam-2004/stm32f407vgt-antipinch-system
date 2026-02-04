#include "as5600.h"

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
    return AS5600_OK;
}