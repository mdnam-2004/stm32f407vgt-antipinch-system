#include "encoder.h"
#include "motor.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdint.h>

#define SIGNAL_CHECK_THRESHOLD 360
#define MIN_TOGGLE_REQUIRED 4
#define INVALID_MOTION     2
#define ERR_THRESHOLD 10
#define ENC_PHASE_A GPIOA, GPIO_PIN_8
#define ENC_PHASE_B GPIOC, GPIO_PIN_8

static const int8_t QUAD_TABLE[16] ={
    0, -1, 1, 2, // 00=0 01=1 10=-1 11=error
    1, 0, 2, -1,
    -1, 2, 0, 1,
    2, 1, -1, 0
};

void encoder_update(encoder_handle_t *penc)
{
uint8_t A = (HAL_GPIO_ReadPin(ENC_PHASE_A) == GPIO_PIN_SET) ?1 :0;
uint8_t B = (HAL_GPIO_ReadPin(ENC_PHASE_B) == GPIO_PIN_SET)?1:0;
uint8_t new_state = (A <<1) | B;
uint8_t old_state = penc->old_state;

if ( old_state != new_state) {
    uint8_t diff = new_state ^ penc->old_state;
    if (diff & 0x02u /* 0x10*/) penc->toggle_cnt_A ++;
    if (diff & 0x01u /* 0x01*/) penc->toggle_cnt_B ++;
}

uint8_t index = (uint8_t)(old_state << 2 | new_state);
int8_t motion = QUAD_TABLE[index];

if (motion == INVALID_MOTION) 
{
    penc->error_count++;
    if(penc->error_count >= ERR_THRESHOLD)
    {
      penc-> is_fault = 1u;
      penc->fault_code = ENC_ERR_INVALID;
    }
}
else if(motion != 0 )
{
        penc->pulse_count += motion;
        penc->direction = motion;
        penc->fault_code = ENC_OK;
        penc->is_fault = 0;
        penc->error_count =0;

       if (penc->fault_code != ENC_ERR_INVALID) 
       {
            penc->fault_code = ENC_OK;
            penc->is_fault = 0;  
       }
}

if ( penc ->toggle_cnt_A >= SIGNAL_CHECK_THRESHOLD || 
         penc ->toggle_cnt_B >= SIGNAL_CHECK_THRESHOLD) 
{
    if (penc -> toggle_cnt_A < MIN_TOGGLE_REQUIRED)
        {
            penc -> fault_code = ENC_ERR_A;
            penc->is_fault =1;
        }

    else if (penc -> toggle_cnt_B < MIN_TOGGLE_REQUIRED)
        {
            penc -> fault_code = ENC_ERR_B;
            penc ->is_fault =1;
        }
    else 
    {
        if(penc->fault_code != ENC_ERR_INVALID)
            {
               penc->fault_code = ENC_OK;
               penc ->is_fault = 0;
               penc->toggle_cnt_A = 0;
               penc->toggle_cnt_B = 0;
            }
    }
}

penc->old_state = new_state;
}

void encoder_process_periodic(encoder_handle_t *penc)
{
    encoder_update(penc); 
}