#include "photo_interrupt.h"

static QueueHandle_t interrupt_queue = NULL;

static volatile bool last_photo_state = false;
static volatile uint32_t photo_interrupt_count = 0;

static const char *TAG = "PHOTO_INTERRUPT";

// 포토 인터럽트 카운트를 외부에서 읽을 수 있는 함수
uint32_t get_photo_interrupt_count(void) {
    return photo_interrupt_count;
}

// 포토 인터럽트 카운트를 리셋하는 함수
void reset_photo_interrupt_count(void) {
    photo_interrupt_count = 0;
    printf("Photo interrupt count reset to 0\n");
}

static void IRAM_ATTR photo_interrupt_isr(void *arg) {
    int pin_number = (int)arg;
    
    photo_interrupt_event_t event = {
        .pin_number = pin_number,
        .new_state = false  // NEGEDGE = LOW 상태
    };
    
    xQueueSendFromISR(interrupt_queue, &event, NULL);
}

static void photo_interrupt_task(void *arg) {
    photo_interrupt_event_t event;
    while (1)
    {
        if (xQueueReceive(interrupt_queue, &event, portMAX_DELAY))
        {
            // 인터럽트가 발생할 때마다 카운트 증가 (상태 변화 감지 제거)
            photo_interrupt_count++;
            
            stepper_stop_continuous();
            ESP_LOGI(TAG, "PHOTO INTERRUPT STOP");
            reset_photo_interrupt_count();
            
            // 디버깅: 현재 GPIO 상태도 함께 출력
            bool current_gpio_state = gpio_get_level(PHOTO_INTERRUPT_PIN);
            printf("Photo Interrupt: Object detected (count: %lu, GPIO state: %s)\n", 
                   photo_interrupt_count, current_gpio_state ? "HIGH" : "LOW");
        }
    }
}

void photo_interrupt_init() {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << PHOTO_INTERRUPT_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        printf("GPIO config failed: %s\n", esp_err_to_name(ret));
        return;
    }

    // 초기 상태 설정
    last_photo_state = gpio_get_level(PHOTO_INTERRUPT_PIN);

    interrupt_queue = xQueueCreate(10, sizeof(photo_interrupt_event_t));
    xTaskCreate(photo_interrupt_task, "photo_interrupt_task", 2048*2, NULL, 1, NULL);

    gpio_install_isr_service(0);
    ret = gpio_isr_handler_add(PHOTO_INTERRUPT_PIN, photo_interrupt_isr, (void *)PHOTO_INTERRUPT_PIN);
    
    printf("Photo interrupt initialized. Initial state: %s\n", 
           last_photo_state ? "HIGH" : "LOW");
    printf("GPIO %d configured for NEGEDGE interrupt\n", PHOTO_INTERRUPT_PIN);
}