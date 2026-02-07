#include "encoder.h"
#include "motor.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdint.h>
#include <stdlib.h>

#define SIGNAL_CHECK_THRESHOLD 520
#define MIN_TOGGLE_REQUIRED 10
#define INVALID_MOTION     2
#define ERR_THRESHOLD 10
#define ENC_PHASE_A GPIOA, GPIO_PIN_8
#define ENC_PHASE_B GPIOC, GPIO_PIN_8
#define ENC_MAX_INDEX        15U
#define ENC_ERR_LIMIT        10U
#define ENC_PHASE_A_MASK     0x02U
#define ENC_PHASE_B_MASK     0x01U

static const int8_t QUAD_TABLE[16] ={
    0, -1, 1, 2, // 00=0 01=1 10=-1 11=error
    1, 0, 2, -1,
    -1, 2, 0, 1,
    2, 1, -1, 0
};

void encoder_update(encoder_handle_t *penc)
{
    
    if (penc == NULL) 
    {
        return;
    }

    uint8_t pinA = (HAL_GPIO_ReadPin(ENC_PHASE_A) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t pinB = (HAL_GPIO_ReadPin(ENC_PHASE_B) == GPIO_PIN_SET)? 1U : 0U;
    uint8_t new_state = (pinA <<1) | pinB;
    uint8_t old_state = penc->old_state;

   if ( old_state != new_state) 
   {
      uint8_t index = (uint8_t)(old_state << 2 | new_state);
      if( index <= ENC_MAX_INDEX )
      {
        int8_t motion = QUAD_TABLE[index];
        if(motion == INVALID_MOTION)
        {
            penc->error_count++;
        }
        else if (motion != 0) 
        {
          penc->pulse_count += (int32_t)motion;
          penc->direction =motion;
          uint8_t diff = new_state ^ penc->old_state;
          if (diff & 0x02u /* 0x10*/) penc->toggle_cnt_A ++;
          if (diff & 0x01u /* 0x01*/) penc->toggle_cnt_B ++;
          if (penc->error_count > 0u) penc->error_count --;
        }
        else{}
      }
   }

    if(penc->error_count >= ERR_THRESHOLD)
    {
      penc-> is_fault = 1u;
      penc->fault_code = ENC_ERR_INVALID;
    }

    uint32_t total_toggles =(penc->toggle_cnt_A + penc->toggle_cnt_B);
    if(total_toggles >= SIGNAL_CHECK_THRESHOLD)
    {
        if(penc->toggle_cnt_A < MIN_TOGGLE_REQUIRED)
        {
            penc->is_fault=1u;
            penc->fault_code= ENC_ERR_A;
        }
        else if(penc->toggle_cnt_B < MIN_TOGGLE_REQUIRED)
        {
            penc->is_fault =1u;
            penc->fault_code=ENC_ERR_B;
        }
        else
        {
            penc->toggle_cnt_A = 0u;
            penc->toggle_cnt_B = 0u;
            if(penc->fault_code != ENC_ERR_INVALID)
            {
                penc->is_fault=0;
                penc->fault_code = ENC_OK;
            }
        } 
    }
    penc->old_state = new_state;
}

void encoder_process_periodic(encoder_handle_t *penc)
{
    encoder_update(penc); 
}