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

#define PHOTO_INTERRUPT_PIN GPIO_NUM_4

// 디바운싱 설정 (더 강화된 버전)
#define DEBOUNCE_DELAY_MS 500  // 500ms 디바운싱 (릴레이 노이즈 대응)
#define NOISE_CHECK_COUNT 20   // 노이즈 체크 횟수 증가
#define MIN_INTERRUPT_INTERVAL_MS 1000  // 최소 인터럽트 간격 1초
#define RELAY_NOISE_FILTER_MS 200  // 릴레이 노이즈 필터링 시간

// 인터럽트 이벤트 구조체
typedef struct {
    int pin_number;
    bool new_state;  // 인터럽트 발생 후의 새로운 상태
    TickType_t timestamp;  // 인터럽트 발생 시간
} photo_interrupt_event_t;

extern volatile int top_photo_state;
extern volatile int bottom_photo_state;

void photo_interrupt_init();
uint32_t get_photo_interrupt_count(void); // 포토 인터럽트 카운트를 외부에서 읽을 수 있는 함수
void reset_photo_interrupt_count(void); // 포토 인터럽트 카운트를 리셋하는 함수

void photo_task_init();

#endif // PHOTO_INTERRUPT_H