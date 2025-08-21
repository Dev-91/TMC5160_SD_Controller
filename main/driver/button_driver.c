#include "button_driver.h"

static const char *TAG = "BUTTON";

// button_event_cb 함수 수정
static void button_event_cb(void *arg, void *usr_data) {
    button_handle_t button = (button_handle_t)arg;
    button_event_t event = iot_button_get_event(button);
    uint8_t gpio_num = (uint32_t)usr_data;  // GPIO 번호를 usr_data로 전달받음

    ESP_LOGI(TAG, "button_event_cb: gpio_num=%d, event=%d", gpio_num, event);

    switch (gpio_num) {
        case BOOT_BUTTON_PIN:
        if (event == BUTTON_SINGLE_CLICK) {
            // +60 RPM (wrap), 즉시 반영
            float next = g_target_rpm + RPM_STEP;
            if (next > RPM_MAX) next = RPM_MAX;
            set_target_rpm(next);
            ESP_LOGI(TAG, "RPM -> %.1f (Step Period: %lu µs)", (double)g_target_rpm, get_current_step_period());
        } else if (event == BUTTON_DOUBLE_CLICK) {
            // TOFF 레벨 토글 (12 -> 16 -> 20 -> 12)
            toggle_toff_level();
        }
        break;
        case SUB_BUTTON_PIN:
        if (event == BUTTON_SINGLE_CLICK) {
            // -60 RPM (wrap), 즉시 반영
            float next = g_target_rpm - RPM_STEP;
            if (next <= RPM_MIN) next = RPM_MIN;
            set_target_rpm(next);
            ESP_LOGI(TAG, "RPM -> %.1f (Step Period: %lu µs)", (double)g_target_rpm, get_current_step_period());
        } else if (event == BUTTON_DOUBLE_CLICK) {
            // 히스테리시스 레벨 토글 (HSTRT/HEND 조정)
            // toggle_hysteresis_level();
            set_current_level(1.0f, 0.5f);
            // toggle_bbmtime_level();
        }
        break;
        default:
        break;
    }
}

static void button_long_press_event_cb(void *arg, void *usr_data) {
    button_handle_t button = (button_handle_t)arg;
    button_event_t event = iot_button_get_event(button);
    uint8_t gpio_num = (uint32_t)usr_data;

    switch (gpio_num) {
        case BOOT_BUTTON_PIN:
        // 길게 누름 시작에서만 토글 (한 번만 실행)
        if (event == BUTTON_LONG_PRESS_START) {
            if (g_run_enabled) {
                stepper_stop_continuous();
                ESP_LOGI(TAG, "STOP");
            } else {
                gpio_set_level(RELAY_BREAK_A_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                stepper_start_continuous(1);
                ESP_LOGI(TAG, "START");
            }
        }
        break;
        case SUB_BUTTON_PIN:
        if (event == BUTTON_LONG_PRESS_START) {
            // if (g_run_enabled) {
            //     stepper_stop_continuous();
            //     vTaskDelay(ms_to_ticks(100));
            //     gpio_set_level(RELAY_BREAK_A_PIN, 0);
            //     ESP_LOGI(TAG, "STOP");
            // } else {
            //     gpio_set_level(RELAY_BREAK_A_PIN, 1);
            //     vTaskDelay(ms_to_ticks(100));
            //     stepper_start_continuous(1);
            //     ESP_LOGI(TAG, "START");
            // }
        }
        break;
        default:
            break;
    }

}

static void iot_button_init(uint32_t button_num) {
    button_config_t btn_cfg = {0};
    button_gpio_config_t gpio_cfg = {
        .gpio_num = button_num,
        .active_level = BUTTON_ACTIVE_LEVEL,
        .enable_power_save = false,
    };

    button_handle_t btn;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn);
    assert(ret == ESP_OK);

    ret = iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, NULL, button_event_cb, (void *)button_num);
    ret |= iot_button_register_cb(btn, BUTTON_DOUBLE_CLICK, NULL, button_event_cb, (void *)button_num);
    ret |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, NULL, button_long_press_event_cb, (void *)button_num);

    ESP_ERROR_CHECK(ret);
}

void button_init() {
    gpio_set_direction(SUB_BUTTON_VCC_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SUB_BUTTON_VCC_PIN, 1);

    iot_button_init(BOOT_BUTTON_PIN);
    iot_button_init(SUB_BUTTON_PIN);
}