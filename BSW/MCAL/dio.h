#ifndef DIO_H
#define DIO_H

#include "stm32f4xx_hal.h"

typedef enum {
    DIO_LOW = 0,
    DIO_HIGH = 1
} Dio_LevelType;

typedef struct {
	GPIO_TypeDef  *port;
	uint16_t pin;
}Dio_ConfigType;

typedef enum{
	LED_0 = 0,
	LED_1,
	LED_COUNT
}Dio_NumLed;

void Dio_Write(Dio_NumLed ID, Dio_LevelType level);
void Dio_Toggle(Dio_NumLed ID);
#endif