#include "main.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    button_init();
    // photo_interrupt_init();
    // photo_task_init();

    tmc5160_sd_driver_init();
    
    // 초기화 후 TMC5160 상태 확인
    vTaskDelay(pdMS_TO_TICKS(1000));
    check_tmc5160_status();
    
    // 모터 동작 중에도 주기적으로 상태 확인
    int check_count = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        // check_count++;
        
        // // 10초마다 TMC5160 상태 확인
        // if (check_count % 10 == 0) {
        //     ESP_LOGI("MAIN", "=== Periodic TMC5160 Status Check ===");
        //     check_tmc5160_status();
        // }
    }
}