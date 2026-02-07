#ifndef ENCODER_H
#define ENCODER_H
#include "stm32f4xx_hal.h"
#include "stdint.h"

typedef struct{
    volatile uint8_t old_state; // trang thai phase A va B
    volatile int64_t pulse_count;
    volatile int8_t direction; // 1 tien -1lui 0dung yen
    volatile uint16_t error_count;
    volatile uint8_t is_fault;
    volatile uint32_t toggle_cnt_A;
    volatile uint32_t toggle_cnt_B;
    volatile uint8_t fault_code;
    volatile uint32_t timer_ticks;
    volatile uint32_t period_raw;
}encoder_handle_t;

typedef enum{
    ENC_OK =0,
    ENC_ERR_A, // loi phase A 
    ENC_ERR_B, // Loi phase B
    ENC_NOT_ROTA,
    ENC_ERR_INVALID // Loi bi mat tin hieu ( 00 -> 11)
} encoder_error_t;

void encoder_update(encoder_handle_t *penc);
void encoder_process_periodic(encoder_handle_t *pEnc);
#endif