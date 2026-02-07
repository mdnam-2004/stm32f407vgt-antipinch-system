#ifndef ENCODER_H
#define ENCODER_H
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include <stdlib.h>

#define SIGNAL_CHECK_THRESHOLD 520
#define MIN_TOGGLE_REQUIRED 10
#define INVALID_MOTION     2
#define ERR_THRESHOLD 10
#define ENC_MAX_INDEX        15U
#define ENC_ERR_LIMIT        10U
#define ENC_PHASE_A_MASK     0x02U
#define ENC_PHASE_B_MASK     0x01U

#define PPR_BASE 11.0f
#define GEAR_RATIO 45.0f
#define MODE_MULTIPLIER 4.0f
#define PULSES_PER_REV_OUTPUT (PPR_BASE * GEAR_RATIO * MODE_MULTIPLIER)
#define WINDOW_HEIGHT_MAX_MM 400.0f
#define ENC_PULSES_PER_MM    99.0f
#define FILTER_ALPHA  0.05f
#define ENC_STOP_TIMEOUT  20   // Số chu kỳ chờ trước khi về 0 
#define ENC_INVERT_DIR    1
typedef struct{
    uint32_t last_hw_cnt;
    volatile int64_t pulse_count;
    
    volatile uint16_t error_count;
    volatile uint8_t is_fault;
    volatile uint32_t toggle_cnt_A;
    volatile uint32_t toggle_cnt_B;
    volatile uint8_t fault_code;

    float angle_deg; // goc quay truc ra 0-360
    float total_revolutions; // tong so vong quay
    float position_mm;
    int32_t last_pulse_count;
    float speed_mm_s;
    uint32_t last_tick;
    float last_speed_mm_s;
    float accel_mm_s2;  // gia toc (mm/s^2)
    float accel_filtered;
    int32_t debug_delta;
    volatile int8_t  direction;      // 1: UP, -1: DOWN, 0: STOP
    uint32_t stop_timeout_cnt;

}encoder_handle_t;

typedef enum{
    ENC_OK =0,
    ENC_ERR_A, // loi phase A 
    ENC_ERR_B, // Loi phase B
    ENC_NOT_ROTA,
    ENC_ERR_INVALID // Loi bi mat tin hieu ( 00 -> 11)
} encoder_error_t;

void encoder_update(encoder_handle_t *penc, TIM_HandleTypeDef *htim);
void encoder_process_periodic(encoder_handle_t *pEnc);
void ENC_angle(encoder_handle_t *penc);
void positon_mm (encoder_handle_t *penc);
void speed_mm_s (encoder_handle_t *penc);
#endif