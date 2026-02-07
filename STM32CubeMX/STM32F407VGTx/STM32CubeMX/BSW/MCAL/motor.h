#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"
#include "stdio.h"

typedef enum{
	MOTOR_UP = 0,
	MOTOR_DOWN,
	MOTOR_STOP
} Motor_Dir_t;

void Motor_Init();
void Motor_Control (Motor_Dir_t dir, uint8_t speed);
void Motor_Update (void);
void Motor_EStop(void); // STOP SOS
#endif