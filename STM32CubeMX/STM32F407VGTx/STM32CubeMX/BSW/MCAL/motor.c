#include "motor.h"

#define MOTOR_TIM htim2
#define MOTOR_CH_UP   TIM_CHANNEL_1
#define MOTOR_CH_DOWN TIM_CHANNEL_2

#define PWM_PERIOD_ARR 8399
#define SPEED_MAX 100
#define SCALING 84

#define START_STEP 2
#define UPDATE_INTERVAL 10

static uint8_t target_speed =0;
static uint8_t current_speed = 0;
static Motor_Dir_t current_dir = MOTOR_STOP;
extern TIM_HandleTypeDef MOTOR_TIM;

static Motor_Dir_t pending_dir = MOTOR_STOP;
static uint8_t pending_speed =0;

void Motor_Init()
{
	HAL_TIM_PWM_Start(&MOTOR_TIM,MOTOR_CH_UP);
	HAL_TIM_PWM_Start(&MOTOR_TIM,MOTOR_CH_DOWN);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIM,MOTOR_CH_UP,0);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR_CH_DOWN, 0);
}

void Motor_Control (Motor_Dir_t dir, uint8_t speed)
{
		if (speed > SPEED_MAX) {speed = SPEED_MAX;}
		if ( dir == MOTOR_STOP)
		{
			target_speed =0;
			pending_dir = MOTOR_STOP;
			pending_speed =0;
			return;
		}
		if (current_dir != MOTOR_STOP && dir != current_dir && current_speed >0)
		{
			pending_dir = dir;
			pending_speed =speed;
			target_speed =0;
			return;
		}
		pending_dir = MOTOR_STOP;
		pending_speed =0;
		target_speed =speed;
		current_dir = dir;
}

void Motor_Update (void)
{
	
	if(current_speed < target_speed)
	{
		current_speed += START_STEP;
		if (current_speed > target_speed) 
		{
			current_speed = target_speed;
		}
	}
	else if (current_speed > target_speed)  
	{
		current_speed -= START_STEP ;
	}
		uint32_t pwm_pulse = (uint32_t)current_speed *SCALING;
	if ( current_speed ==0 & pending_dir != MOTOR_STOP)
	{
		current_dir = pending_dir;
		target_speed = pending_speed;
		pending_dir = MOTOR_STOP;
		pending_speed =0;
	}
	if (target_speed == 0 && current_speed == 0 && pending_dir == MOTOR_STOP)
	{
		current_dir = MOTOR_STOP;
	}
	
	if(pwm_pulse > PWM_PERIOD_ARR)
	{
			pwm_pulse = PWM_PERIOD_ARR;
	}
	switch (current_dir)
	{
			case MOTOR_UP:
				__HAL_TIM_SET_COMPARE(&MOTOR_TIM,MOTOR_CH_UP,pwm_pulse);
				__HAL_TIM_SET_COMPARE(&MOTOR_TIM,MOTOR_CH_DOWN,0);
			break;
			
			case MOTOR_DOWN:
				__HAL_TIM_SET_COMPARE(&MOTOR_TIM,MOTOR_CH_UP,0);
				__HAL_TIM_SET_COMPARE(&MOTOR_TIM,MOTOR_CH_DOWN,pwm_pulse);		
				break;
			
			case MOTOR_STOP:
			default:
				__HAL_TIM_SET_COMPARE(&MOTOR_TIM,MOTOR_CH_UP,0);
				__HAL_TIM_SET_COMPARE(&MOTOR_TIM,MOTOR_CH_DOWN,0);
			break;
	}
}
void Motor_ESop(void)
{
	target_speed =0;
	current_speed =0;
	pending_dir = MOTOR_STOP;
	pending_dir =0;
	current_dir = MOTOR_STOP;
	
	__HAL_TIM_SET_COMPARE (&MOTOR_TIM, MOTOR_CH_UP, 0);
	__HAL_TIM_SET_COMPARE (&MOTOR_TIM, MOTOR_CH_DOWN,0);
}
