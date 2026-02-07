#include "encoder.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

void encoder_update(encoder_handle_t *penc, TIM_HandleTypeDef *htim)
{
  if (penc == NULL || htim == NULL)
    {
        return;
    }

    uint32_t current_cnt = __HAL_TIM_GET_COUNTER(htim);
    int32_t delta = (int32_t)((int16_t)current_cnt - (int16_t)penc->last_hw_cnt);
    penc->debug_delta = delta;

if (delta != 0) 
    {
        penc->stop_timeout_cnt = ENC_STOP_TIMEOUT;

        int8_t new_dir = 0;
        if (delta > 0) new_dir = 1;
        else new_dir = -1;
        #if ENC_INVERT_DIR
            new_dir = -new_dir; 
        #endif
        penc->direction = new_dir;
        #if ENC_INVERT_DIR
            penc->pulse_count -= delta; 
        #else
            penc->pulse_count += delta;
        #endif
        uint32_t activity = (uint32_t)((delta >= 0) ? delta : -delta);
        penc->toggle_cnt_A += activity;
        penc->toggle_cnt_B += activity;
        if (penc->error_count > 0u) 
        {
            penc->error_count--;
        }
    }
else 
    {
        if (penc->stop_timeout_cnt > 0)
        {
            penc->stop_timeout_cnt--;
        }
        else
        {
            penc->direction = 0;
        }
    }

    uint32_t total_toggles = (penc->toggle_cnt_A + penc->toggle_cnt_B);
    if(total_toggles >= SIGNAL_CHECK_THRESHOLD)
    {
        if(penc->toggle_cnt_A < MIN_TOGGLE_REQUIRED)
        {
            penc->is_fault = 1u;
            penc->fault_code = ENC_ERR_A;
        }
        else if(penc->toggle_cnt_B < MIN_TOGGLE_REQUIRED)
        {
            penc->is_fault = 1u;
            penc->fault_code = ENC_ERR_B;
        }
        else
        {
            penc->toggle_cnt_A = 0u;
            penc->toggle_cnt_B = 0u;
            if(penc->fault_code != ENC_ERR_INVALID)
            {
                penc->is_fault = 0;
                penc->fault_code = ENC_OK;
            }
        } 
    }
    penc->last_hw_cnt = (uint32_t)current_cnt;
}

void ENC_angle(encoder_handle_t *penc)
{
    if (penc == NULL)
    {
        return;
    }
    penc->total_revolutions = (float)penc->pulse_count / (float)PULSES_PER_REV_OUTPUT;
    float raw_angle = penc->total_revolutions *360.0f;
    penc->angle_deg = fmodf(penc->total_revolutions * 360.0f, 360.0f);
    if (penc->angle_deg < 0) penc->angle_deg += 360.0f;
}

void positon_mm (encoder_handle_t *penc)
{
    if (penc == NULL) return;
    float raw_mm = (float)penc->pulse_count/ (float)ENC_PULSES_PER_MM;
    penc->position_mm =raw_mm; 
}

void speed_mm_s (encoder_handle_t *penc)
{
    if (penc == NULL) return;
    uint32_t current_tick = HAL_GetTick();
    uint32_t dt_ms = current_tick - penc->last_tick;

    if (dt_ms == 0) return;

    float dt_sec = (float)dt_ms / 1000.0f;

    int32_t current_pulse = penc->pulse_count;
    int32_t delta_pulse = current_pulse - penc->last_pulse_count;
    
    float distance_mm = (float)delta_pulse / ENC_PULSES_PER_MM;
    float current_speed = distance_mm / dt_sec;
    penc->speed_mm_s = current_speed;

    float delta_v = current_speed - penc->last_speed_mm_s;
    float raw_accel = delta_v / dt_sec;

   // penc->accel_mm_s2 = raw_accel;
    penc->accel_mm_s2 = delta_v / dt_sec;
    penc->accel_filtered = (FILTER_ALPHA * raw_accel) + ((1.0f - FILTER_ALPHA) * penc->accel_filtered);

    penc->last_pulse_count = current_pulse;
    penc->last_tick = current_tick;
    penc->last_speed_mm_s = current_speed;
}