#ifndef AS5600_H
#define AS5600_H
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdint.h>
#include <stdio.h>

#define AS5600_ADDR (uint16_t)(0x36U <<1)
#define AS5600_ANGLE_HIGH (uint16_t)(0x0EU)
#define AS5600_ANGLE_LOW (uint16_t)(0x0FU)
#define AS5600_ADDR_SIZE_8bit (0x00000001U)


typedef enum {
    AS5600_OK = 0,
    AS5600_ERROR,
    AS5600_TIMEOUT
}AS5600_Status_t;

#endif