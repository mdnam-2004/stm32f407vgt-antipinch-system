#include "dio.h"

const Dio_ConfigType  Dio_Map[LED_COUNT] ={
 {GPIOD, GPIO_PIN_15},
 {GPIOD, GPIO_PIN_13}
 };

void Dio_Write(Dio_NumLed ID, Dio_LevelType level)
{
	if (ID >= LED_COUNT) return;
	const Dio_ConfigType *cfg = &Dio_Map[ID];
	HAL_GPIO_WritePin(cfg->port, cfg->pin,
	(level == DIO_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Dio_Toggle(Dio_NumLed ID)
{
	if (ID >= LED_COUNT) return;
	const Dio_ConfigType *cfg = &Dio_Map[ID];
	HAL_GPIO_TogglePin( cfg->port, cfg->pin);
}
	
