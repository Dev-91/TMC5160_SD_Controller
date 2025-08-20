// Minimal STEP/DIR control for TMC5160 SD mode using ESP-IDF
// Adjust pin numbers to match your wiring

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include <string.h>
#include "driver/gptimer.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "iot_button.h"
#include "button_gpio.h"

#define SPI_SDO_CFG0 GPIO_NUM_42
#define SPI_SDI_CFG1 GPIO_NUM_41
#define SPI_SCK_CFG2 GPIO_NUM_40
#define SPI_CS_CFG3  GPIO_NUM_39

#define REFL_STEP GPIO_NUM_2   // STEP pin -> TMC5160 REFL_STEP
#define REFR_DIR  GPIO_NUM_1   // DIR  pin -> TMC5160 REFR_DIR

#define BUTTON_ACTIVE_LEVEL     0

#define RELAY_BREAK_A_PIN GPIO_NUM_47 // GPIO_NUM_17
#define RELAY_BREAK_B_PIN GPIO_NUM_48 // GPIO_NUM_18

#define BOOT_BUTTON_PIN GPIO_NUM_0 // GPIO_NUM_0
#define SUB_BUTTON_PIN GPIO_NUM_19 // GPIO_NUM_35

#define PHOTO_INTERRUPT_PIN GPIO_NUM_7

#define SUB_BUTTON_VCC_PIN GPIO_NUM_21 // 임시 버튼 전원

// Optional: define DRV_ENN if you wired it to MCU (active-low enable)
// Set to -1 if not connected
#define DRV_ENN_PIN (-1)

// --- SPI (TMC5160) ---
#if CONFIG_IDF_TARGET_ESP32
#define TMC_SPI_HOST HSPI_HOST
#else
#define TMC_SPI_HOST SPI2_HOST
#endif

#define TMC_SPI_MOSI  SPI_SDI_CFG1  // MCU MOSI -> TMC SDI
#define TMC_SPI_MISO  SPI_SDO_CFG0  // MCU MISO <- TMC SDO
#define TMC_SPI_SCLK  SPI_SCK_CFG2
#define TMC_SPI_CS    SPI_CS_CFG3

#define CW_DIR 0
#define CCW_DIR 1

static spi_device_handle_t tmc_spi = NULL;
static QueueHandle_t interrupt_queue = NULL;
static volatile uint32_t photo_interrupt_count = 0;
static volatile bool last_photo_state = false;

// 인터럽트 이벤트 구조체
typedef struct {
    int pin_number;
    bool new_state;  // 인터럽트 발생 후의 새로운 상태
} photo_interrupt_event_t;

// --- Hardware step generation (GPTimer) ---
static gptimer_handle_t step_timer = NULL;
static SemaphoreHandle_t step_done_sem = NULL; // given on each completed step (falling edge)
static volatile int32_t hw_steps_remaining = 0;
static volatile uint32_t hw_current_period_us = 100; // full period per step (µs)
static volatile bool hw_step_state_high = false;      // current level of STEP pin
static volatile bool g_stop_requested = false;        // request to stop mid-motion
static volatile bool g_run_enabled = false;           // continuous run flag
static volatile float g_target_rpm = 0.0f;           // current target RPM (continuous)
static const float g_rpm_step = 60.0f;                // short press increment (button push)
static const float g_rpm_max  = 900.0f;               // max RPM before wrap
static const float g_rpm_min  = 0.0f;              // min RPM before wrap
static const int   g_microsteps = 16;                 // external microsteps (16µsteps with 10µs limit)
static volatile int dir_level = CW_DIR;

typedef struct {
    int level;             // 0 pressed (active-low), 1 released
    TickType_t tick;       // timestamp
} button_evt_t;

static const char *TAG = "MAIN";

// Forward declaration for ISR usage
void motion_request_stop(void);

static inline TickType_t ms_to_ticks(uint32_t ms)
{
    // Convert milliseconds to FreeRTOS ticks safely
    if (ms == 0) return 0;
    uint32_t tick_ms = portTICK_PERIOD_MS;
    if (tick_ms == 0) return ms; // fallback
    return (ms + tick_ms - 1) / tick_ms;
}

// Convert RPM to period in microseconds per microstep
static inline uint32_t rpm_to_period_us(float rpm)
{
    if (rpm <= 0.5f) return 1000000; // very slow fallback
    const int fullsteps_per_rev = 200;
    int steps_per_rev = fullsteps_per_rev * g_microsteps;
    if (steps_per_rev <= 0) steps_per_rev = 3200;
    float seconds_per_step = 60.0f / (rpm * (float)steps_per_rev);
    uint32_t us = (uint32_t)(seconds_per_step * 1e6f + 0.5f);
    // Minimum period to prevent ISR overload (GPTimer can handle down to ~10µs)
    if (us < 10) us = 10; // Reduced to 10µs minimum for higher speeds
    return us;
}

static bool IRAM_ATTR step_timer_isr(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_ctx)
{
    // Toggle STEP pin every alarm (half-period)
    hw_step_state_high = !hw_step_state_high;
    gpio_set_level(REFL_STEP, hw_step_state_high ? 1 : 0);

    bool request_yield = false;

    // On falling edge, one full step completed
    if (!hw_step_state_high) {
        // If continuous run was disabled, stop immediately at this safe edge
        if (!g_run_enabled) {
            gptimer_stop(timer);
            BaseType_t hpw = pdFALSE;
            if (step_done_sem) xSemaphoreGiveFromISR(step_done_sem, &hpw);
            request_yield = (hpw == pdTRUE);
            return request_yield;
        }
        if (hw_steps_remaining > 0) {
            hw_steps_remaining--;
        }
        if (hw_steps_remaining == 0) {
            // Stop timer; no more alarms
            gptimer_stop(timer);
            // Notify waiter one last time so the task can proceed
            BaseType_t hpw = pdFALSE;
            if (step_done_sem) xSemaphoreGiveFromISR(step_done_sem, &hpw);
            request_yield = (hpw == pdTRUE);
            return request_yield;
        }
        // Notify task that a step has completed
        BaseType_t hpw = pdFALSE;
        if (step_done_sem) xSemaphoreGiveFromISR(step_done_sem, &hpw);
        request_yield = (hpw == pdTRUE);
    }

    // Schedule next half-period alarm (simplified)
    uint64_t next_delta_us = (uint64_t)(hw_current_period_us >> 1);
    if (next_delta_us < 10) next_delta_us = 10; // Minimum 10µs for safety
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = edata->alarm_value + next_delta_us,
        .reload_count = 0,
        .flags = { .auto_reload_on_alarm = false },
    };
    gptimer_set_alarm_action(timer, &alarm_cfg);
    return request_yield;
}

static void stepper_timer_init_once(void)
{
    if (step_timer) return;

    // Create semaphore
    if (step_done_sem == NULL) {
        step_done_sem = xSemaphoreCreateBinary();
    }

    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz -> 1 tick = 1 µs (safe resolution)
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &step_timer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = step_timer_isr,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(step_timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(step_timer));
}

static inline void set_target_rpm(float rpm)
{
    if (rpm < g_rpm_step) rpm = g_rpm_min;
    if (rpm > g_rpm_max) rpm = g_rpm_max;
    g_target_rpm = rpm;
    hw_current_period_us = rpm_to_period_us(rpm);
}
 
static void stepper_start_continuous(int dir)
{
    stepper_timer_init_once();
    gpio_set_level(REFR_DIR, dir ? 1 : 0);
    gpio_set_level(REFL_STEP, 0);
    hw_step_state_high = false;
    hw_steps_remaining = -1; // continuous
    g_run_enabled = true;

    uint64_t now = 0;
    ESP_ERROR_CHECK(gptimer_get_raw_count(step_timer, &now));
    uint64_t half = (uint64_t)(hw_current_period_us >> 1);
    if (half == 0) half = 1;
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = now + half,
        .reload_count = 0,
        .flags = { .auto_reload_on_alarm = false },
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(step_timer, &alarm_cfg));
    ESP_ERROR_CHECK(gptimer_start(step_timer));
}

static inline void stepper_stop_continuous(void)
{
    g_run_enabled = false; // ISR will stop at next falling edge
    vTaskDelay(ms_to_ticks(100));
    gpio_set_level(RELAY_BREAK_A_PIN, 0);
}


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
            float next = g_target_rpm + g_rpm_step;
            if (next > g_rpm_max) next = g_rpm_max;
            set_target_rpm(next);
            ESP_LOGI(TAG, "RPM -> %.1f", (double)g_target_rpm);
        } else if (event == BUTTON_DOUBLE_CLICK) {
            if (dir_level == CW_DIR) {
                gpio_set_level(REFR_DIR, 1);
                dir_level = CCW_DIR;
            } else if (dir_level == CCW_DIR) {
                gpio_set_level(REFR_DIR, 0);
                dir_level = CW_DIR;
            }
            ESP_LOGI(TAG, "DIR -> %d", dir_level);
        }
        break;
        case SUB_BUTTON_PIN:
        if (event == BUTTON_SINGLE_CLICK) {
            // -60 RPM (wrap), 즉시 반영
            float next = g_target_rpm - g_rpm_step;
            if (next <= g_rpm_min) next = g_rpm_min;
            set_target_rpm(next);
            ESP_LOGI(TAG, "RPM -> %.1f", (double)g_target_rpm);
        } else if (event == BUTTON_DOUBLE_CLICK) {
            
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
                vTaskDelay(ms_to_ticks(100));
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

void iot_button_init(uint32_t button_num) {
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

void break_init() {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << RELAY_BREAK_A_PIN | 1ULL << RELAY_BREAK_B_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

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

// Public helpers to control mid-motion stop
void motion_request_stop(void) { g_stop_requested = true; }
void motion_clear_stop(void)   { g_stop_requested = false; }

static esp_err_t tmc5160_spi_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = TMC_SPI_MOSI,
        .miso_io_num = TMC_SPI_MISO,
        .sclk_io_num = TMC_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8,
        .flags = 0,
        .intr_flags = 0,
    };

    esp_err_t err = spi_bus_initialize(TMC_SPI_HOST, &buscfg, SPI_DMA_DISABLED);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 3,                         // SPI MODE 3
        .spics_io_num = TMC_SPI_CS,
        .queue_size = 1,
        .flags = 0,
    };

    err = spi_bus_add_device(TMC_SPI_HOST, &devcfg, &tmc_spi);
    return err;
}

// Write 32-bit data to register (address without write bit)
static esp_err_t tmc5160_write_reg(uint8_t addr, uint32_t data)
{
    if (tmc_spi == NULL) return ESP_ERR_INVALID_STATE;
    uint8_t tx[5];
    uint8_t rx[5];
    tx[0] = addr | 0x80; // write
    tx[1] = (data >> 24) & 0xFF;
    tx[2] = (data >> 16) & 0xFF;
    tx[3] = (data >> 8) & 0xFF;
    tx[4] = (data) & 0xFF;
    spi_transaction_t t = {
        .length = 40,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(tmc_spi, &t);
}

static esp_err_t tmc5160_basic_startup(void)
{
    // CHOPCONF (0x6C): TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0
    ESP_ERROR_CHECK(tmc5160_write_reg(0x6C, 0x000100C3));

    // Compute IHOLD/IRUN from motor spec and Rsense
    // IRMS = ((CS+1)/32) * VFS/Rsense * 1/sqrt(2)
    // -> (CS+1) = IRMS * 32 * Rsense / (VFS/√2)
    const float VFS = 0.325f;        // typ. sense threshold (datasheet VSRT)
    const float RSENSE_OHMS = 0.15f; // set to your board's actual Rsense value (Ohms)
    const float TARGET_IRMS_A = 2.00f; // reduce run RMS current target to lower heating
    const float HOLD_RATIO = 0.50f;   // reduce hold current to ~30% of run
    const float denom = (VFS / 1.41421356f);
    float cs_plus_1 = (denom > 0.0f) ? (TARGET_IRMS_A * 32.0f * RSENSE_OHMS / denom) : 1.0f;
    int irun = (int)(cs_plus_1 + 0.5f) - 1; // round to nearest CS, then minus 1
    if (irun < 8) irun = 8;           // ensure >=8 for StealthChop tuning
    if (irun > 31) irun = 31;
    int ihold = (int)(irun * HOLD_RATIO + 0.5f);
    if (ihold < 1) ihold = 1;
    if (ihold > 31) ihold = 31;
    int iholddelay = 6; // smooth power-down
    uint32_t ihold_irun = ((uint32_t)iholddelay << 16) | ((uint32_t)irun << 8) | (uint32_t)ihold;
    ESP_ERROR_CHECK(tmc5160_write_reg(0x10, ihold_irun));

    // TPOWERDOWN (0x11): 10
    ESP_ERROR_CHECK(tmc5160_write_reg(0x11, 0x0000000A));

    // GCONF (0x00): en_pwm_mode=1 (optional)
    ESP_ERROR_CHECK(tmc5160_write_reg(0x00, 0x00000004));

    // TPWMTHRS (0x13): 500
    ESP_ERROR_CHECK(tmc5160_write_reg(0x13, 0x000001F4));

    return ESP_OK;
}

// Apply combined settings: StealthChop at low speed (TPWMTHRS),
// SpreadCycle at high, MicroPlyer interpolation enabled, MRES=16, TPFD=4
static esp_err_t tmc5160_apply_quiet_smooth_defaults(void)
{
    if (tmc_spi == NULL) return ESP_ERR_INVALID_STATE;
    // Enable StealthChop (low speed)
    ESP_ERROR_CHECK(tmc5160_write_reg(0x00, 0x00000004)); // GCONF: en_pwm_mode=1
    // CHOPCONF: intpol=1, MRES=4 (16µsteps), TPFD=8, TBL=2, HSTRT=4, HEND=1, TOFF=3
    ESP_ERROR_CHECK(tmc5160_write_reg(0x6C, 0x148100C3));
    // TPWMTHRS (switch to StealthChop below this speed). Keep reasonable for practical use
    ESP_ERROR_CHECK(tmc5160_write_reg(0x13, 0x000000C8)); // 200 (실용적인 값)
    return ESP_OK;
}

// Thermal-oriented tweaks: reduce overall current, enable CoolStep, reduce switching losses
static esp_err_t tmc5160_apply_thermal_optimizations(void)
{
    if (tmc_spi == NULL) return ESP_ERR_INVALID_STATE;
    // Reduce overall current scale (keep CS resolution high): GLOBAL_SCALER=180 (~70%)
    ESP_ERROR_CHECK(tmc5160_write_reg(0x0B, 180));

    // DRV_CONF: FILT_ISENSE=1(200ns), DRVSTRENGTH=0(weak), BBMTIME=4 (~100ns), others default
    // value = (FILT_ISENSE<<20) | (DRVSTRENGTH<<18) | (BBMCLKS<<8) | (BBMTIME)
    ESP_ERROR_CHECK(tmc5160_write_reg(0x0A, 0x00100004));

    // CHOPCONF: keep intpol/MRES=4/TPFD=8 etc. but raise TOFF further to 8 for smoother switching
    // previous 0x148100C3 -> 0x148100C8 (TOFF=8, MRES=4, TPFD=8)
    ESP_ERROR_CHECK(tmc5160_write_reg(0x6C, 0x148100C8));

    // COOLCONF: enable CoolStep with wider down-scaling window, min current 1/4
    // SEMIN=2, SEMAX=10, SEUP=1, SEDN=2, SEIMIN=1 -> 0x0000CA22
    ESP_ERROR_CHECK(tmc5160_write_reg(0x6D, 0x0000CA22));

    // TCOOLTHRS: CoolStep/StallGuard lower velocity threshold (tune later). Start with 2000
    ESP_ERROR_CHECK(tmc5160_write_reg(0x14, 0x000007D0));

    return ESP_OK;
}

static void gpio_init_step_dir(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << REFL_STEP) | (1ULL << REFR_DIR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(REFL_STEP, 0);
    gpio_set_level(REFR_DIR, 0);

    if (DRV_ENN_PIN >= 0) {
        gpio_config_t en_conf = {
            .pin_bit_mask = (1ULL << DRV_ENN_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&en_conf);
        // Active-low: drive low to enable outputs
        gpio_set_level(DRV_ENN_PIN, 0);
    }
}

void app_main(void)
{
    // When SPI_MODE=1 and SD_MODE=1 (SD+SPI), initialize SPI and bring driver out of reset defaults (TOFF>0)
    if (tmc5160_spi_init() == ESP_OK) {
        tmc5160_basic_startup();
    }

    gpio_init_step_dir();
    // 사용자가 추가한 iot_button 기반 초기화 사용
    button_init();
    break_init();
    photo_interrupt_init();

    // Apply quiet & smooth defaults (StealthChop + SpreadCycle, MicroPlyer, MRES=16, TPFD=4)
    tmc5160_apply_quiet_smooth_defaults();
    // Apply thermal optimizations (current scale, DRV strength, TOFF, CoolStep)
    tmc5160_apply_thermal_optimizations();

    // 연속 모드 초기화 (정지 상태에서 시작)
    set_target_rpm(g_target_rpm);
    vTaskDelay(ms_to_ticks(100));
    // 길게 누르면 START/STOP 토글, 단일/더블 클릭으로 속도 변경, 로터리로 ±10RPM

    while (1) {
        vTaskDelay(ms_to_ticks(200));
    }
}