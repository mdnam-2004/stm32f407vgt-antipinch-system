#include "button.h"
#define DEBOUNCE_TIME 5

static BTN_Config_t Map_BTN[BTN_COUNT] ={
		{GPIOB,GPIO_PIN_9,0,0,0},
		{GPIOC, GPIO_PIN_15,0,0,0}
	};

void Button_Scan()
{
	for( int i =0; i<BTN_COUNT;i++)
	{
		if (HAL_GPIO_ReadPin(Map_BTN[i].port, Map_BTN[i].pin) == GPIO_PIN_RESET) // Dang nhan
		{
			if(Map_BTN[i].Debounce_Cnt < DEBOUNCE_TIME)
			{
				Map_BTN[i].Debounce_Cnt ++;
			}
			if( Map_BTN[i].Debounce_Cnt == DEBOUNCE_TIME)
			{
				if ( Map_BTN[i].Is_Pressed == 0)
				{
					Map_BTN[i].Is_Pressed = 1;
					Map_BTN[i].Flag = 1;
			}
		}
		}
		else 
		{
			Map_BTN[i].Debounce_Cnt =0;
			Map_BTN[i].Is_Pressed = 0;
		}
	}
}

uint8_t Button_IsHeld(BTN_ID_t ID_BTN)
{
	if (ID_BTN >= BTN_COUNT) return 0;
	return Map_BTN[ID_BTN].Is_Pressed;
}

uint8_t Button_GetFlag (BTN_ID_t ID_BTN)
{
	if (ID_BTN >= BTN_COUNT) return 0;
	
	uint8_t ret = 0;
	
	if (Map_BTN[ID_BTN].Flag ==1)
	{
		ret = 1;
		Map_BTN[ID_BTN].Flag =0;
	}
	return ret;
}
