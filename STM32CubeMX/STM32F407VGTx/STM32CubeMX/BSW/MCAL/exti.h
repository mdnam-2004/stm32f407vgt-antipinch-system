#ifndef EXTI_H
#define EXTI_H
#include "stm32f4xx_hal.h"

typedef enum{
	EXTI_BTN_0 =0,
	EXTI_BTN_1,
	EXTI_BTN_COUNT
}Exti_NumBtn_t;

typedef struct {
	uint16_t pin;
}Exti_Cfg_t;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
uint8_t Exti_GetAndClear (Exti_NumBtn_t BTN_ID);
#endif
