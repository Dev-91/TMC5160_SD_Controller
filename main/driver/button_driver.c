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
            toggle_direction();
        } else if (event == BUTTON_DOUBLE_CLICK) {
        }
        break;
        case SUB_BUTTON_PIN_1:
        if (event == BUTTON_SINGLE_CLICK) {
            // +60 RPM (wrap), 즉시 반영
            float next = g_target_rpm_1 + RPM_STEP;
            if (next > RPM_MAX) next = RPM_MAX;
            set_target_rpm(MOTOR_1, next);
            ESP_LOGI(TAG, "RPM -> %.1f (Step Period: %lu µs)", (double)g_target_rpm_1, get_current_step_period(MOTOR_1));
        } else if (event == BUTTON_DOUBLE_CLICK) {
            for (int i = 0; i < 5; i++) {
                motor_precise_test_sequence();
            }
            set_target_rpm(MOTOR_1, 0);
        }
        break;
        case SUB_BUTTON_PIN_2:
        if (event == BUTTON_SINGLE_CLICK) {
            // -60 RPM (wrap), 즉시 반영
            float next = g_target_rpm_1 - RPM_STEP;
            if (next <= RPM_MIN) next = RPM_MIN;
            set_target_rpm(MOTOR_1, next);
            ESP_LOGI(TAG, "RPM -> %.1f (Step Period: %lu µs)", (double)g_target_rpm_1, get_current_step_period(MOTOR_1));
        } else if (event == BUTTON_DOUBLE_CLICK) {
            motor_step_acceleration_test_sequence();
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
            if (g_run_enabled_1) {
                stepper_stop_continuous(MOTOR_1);
                break_control(0);
                ESP_LOGI(TAG, "STOP");
            } else {
                break_control(1);
                stepper_start_continuous(MOTOR_1, CW_DIR);
                ESP_LOGI(TAG, "START");
            }
        }
        break;
        case SUB_BUTTON_PIN_1:
        if (event == BUTTON_LONG_PRESS_START) {
        }
        break;
        case SUB_BUTTON_PIN_2:
        if (event == BUTTON_LONG_PRESS_START) {
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

    gpio_set_direction(SUB_BUTTON_GND_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SUB_BUTTON_GND_PIN, 0);

    iot_button_init(BOOT_BUTTON_PIN);
    iot_button_init(SUB_BUTTON_PIN_1);
    iot_button_init(SUB_BUTTON_PIN_2);
}