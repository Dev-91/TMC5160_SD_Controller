#ifndef PHOTO_INTERRUPT_H
#define PHOTO_INTERRUPT_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "esp_err.h"
#include <string.h>

#include "tmc5160_SD_driver.h"

#include "esp_log.h"

#define PHOTO_INTERRUPT_PIN GPIO_NUM_7

// 인터럽트 이벤트 구조체
typedef struct {
    int pin_number;
    bool new_state;  // 인터럽트 발생 후의 새로운 상태
} photo_interrupt_event_t;

void photo_interrupt_init();
uint32_t get_photo_interrupt_count(void); // 포토 인터럽트 카운트를 외부에서 읽을 수 있는 함수
void reset_photo_interrupt_count(void); // 포토 인터럽트 카운트를 리셋하는 함수

#endif // PHOTO_INTERRUPT_H