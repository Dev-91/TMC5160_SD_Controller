#include "photo_interrupt.h"

static QueueHandle_t interrupt_queue = NULL;

static volatile bool last_photo_state = false;
static volatile uint32_t photo_interrupt_count = 0;
static volatile TickType_t last_interrupt_time = 0;  // 마지막 인터럽트 시간

volatile int top_photo_state = 0;
volatile int bottom_photo_state = 0;

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
        .new_state = false,  // NEGEDGE = LOW 상태
        .timestamp = xTaskGetTickCountFromISR()
    };
    
    xQueueSendFromISR(interrupt_queue, &event, NULL);
}

static void photo_interrupt_task(void *arg) {
    photo_interrupt_event_t event;
    while (1)
    {
        if (xQueueReceive(interrupt_queue, &event, portMAX_DELAY))
        {
            // 디바운싱: 이전 인터럽트와의 시간 간격 확인
            TickType_t current_time = event.timestamp;
            TickType_t time_diff = current_time - last_interrupt_time;
            
            // 디바운싱 시간보다 짧으면 무시
            if (time_diff < pdMS_TO_TICKS(DEBOUNCE_DELAY_MS)) {
                ESP_LOGI(TAG, "Debounced interrupt ignored (time diff: %lu ms)", 
                         (unsigned long)(time_diff * portTICK_PERIOD_MS));
                continue;
            }
            
            // 포토 인터럽트 신호 검증 (매우 강화된 버전)
            // 1. 즉시 GPIO 상태 확인
            bool immediate_state = gpio_get_level(PHOTO_INTERRUPT_PIN);
            
            // 2. 연속적으로 여러 번 상태 확인 (노이즈와 실제 신호 구분)
            int low_count = 0;
            for (int i = 0; i < NOISE_CHECK_COUNT; i++) {
                vTaskDelay(pdMS_TO_TICKS(10));  // 10ms 간격으로 체크
                if (gpio_get_level(PHOTO_INTERRUPT_PIN) == false) {
                    low_count++;
                }
            }
            
            // 3. 80% 이상이 LOW이면 실제 물체 감지로 판단
            if (immediate_state == false && low_count >= (NOISE_CHECK_COUNT * 8 / 10)) {
                // 유효한 인터럽트로 인식
                last_interrupt_time = current_time;
                photo_interrupt_count++;
                
                stepper_stop_continuous(MOTOR_1);
                ESP_LOGI(TAG, "PHOTO INTERRUPT STOP (verified)");
                reset_photo_interrupt_count();
                
                printf("Photo Interrupt: Object detected (count: %lu, low_count: %d/%d, time: %lu ms)\n", 
                       photo_interrupt_count, low_count, NOISE_CHECK_COUNT,
                       (unsigned long)(current_time * portTICK_PERIOD_MS));
            } else {
                // 노이즈로 판단하고 무시
                ESP_LOGI(TAG, "Noise detected - ignored (low_count: %d/%d)", 
                         low_count, NOISE_CHECK_COUNT);
            }
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

static void photo_task(void *arg) {
    while (1)
    {
        top_photo_state = gpio_get_level(PHOTO_INTERRUPT_PIN);  
        // bottom_photo_state = gpio_get_level(PHOTO_INTERRUPT_PIN);
        
        ESP_LOGI(TAG, "photo state: Top %d, Bottom %d", top_photo_state, bottom_photo_state);
        // if (gpio_get_level(PHOTO_INTERRUPT_PIN) == 0) {
        //     stepper_stop_continuous();
        // }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void photo_task_init() {
    gpio_set_direction(PHOTO_INTERRUPT_PIN, GPIO_MODE_INPUT);
    xTaskCreate(photo_task, "photo_task", 2048*2, NULL, 1, NULL);
}