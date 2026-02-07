#ifndef BUTTON_H
#define BUTTON_H

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "math.h"

typedef enum{
	BTN_UP = 0,
	BTN_DOWN,
	BTN_COUNT
}BTN_ID_t;
typedef struct{
	GPIO_TypeDef *port;
	uint16_t pin;
	uint8_t Debounce_Cnt; // Bien dem chong rung
	uint8_t Is_Pressed;	// Trang thai hien tai
	uint8_t Flag;				//Co bao hieu 
}BTN_Config_t;

void Button_Scan();
uint8_t Button_GetFlag (BTN_ID_t ID_BTN); 
uint8_t Button_IsHeld(BTN_ID_t ID_BTN);

#endif