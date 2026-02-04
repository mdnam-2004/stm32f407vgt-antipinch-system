#include "exti.h"

#define TIME_NOISE 10
const Exti_Cfg_t Exti_Map[EXTI_BTN_COUNT] = {
	{GPIO_PIN_15},
	{GPIO_PIN_9}
};

static volatile uint8_t Exti_Flag[EXTI_BTN_COUNT] = {0};

static uint32_t Exti_LastTime[EXTI_BTN_COUNT] = {0};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t current_time = HAL_GetTick();
	int i = 0;
	for(i =0; i<EXTI_BTN_COUNT; i++)
	{
		if(GPIO_Pin == Exti_Map[i].pin)
		{
			if ((current_time - Exti_LastTime[i]) >= TIME_NOISE)
			{
				Exti_Flag[i] =1;
				Exti_LastTime[i] = current_time;
			}
			break;
		}
	}
}

uint8_t Exti_GetAndClear (Exti_NumBtn_t BTN_ID)
{
	uint8_t ret = 0;
	if (BTN_ID < EXTI_BTN_COUNT)
	{
		__disable_irq();
		if(Exti_Flag[BTN_ID])
		{
		ret =1;
		Exti_Flag[BTN_ID] = 0;
		}
		__enable_irq();
	}
	return ret;
}