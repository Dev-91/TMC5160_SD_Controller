#include "tmc5160_SD_driver.h"

static spi_device_handle_t tmc_spi = NULL;

// --- Hardware step generation (GPTimer) ---
static gptimer_handle_t step_timer = NULL;
static SemaphoreHandle_t step_done_sem = NULL; // given on each completed step (falling edge)
static volatile int32_t hw_steps_remaining = 0;
static volatile uint32_t hw_current_period_us = 100; // full period per step (µs)
static volatile bool hw_step_state_high = false;      // current level of STEP pin
static volatile bool g_stop_requested = false;        // request to stop mid-motion

volatile bool g_run_enabled = false;           // continuous run flag
volatile float g_target_rpm = 0.0f;           // current target RPM (continuous)

static volatile int dir_level = CW_DIR;
static volatile int bbmtime_level = 2; // 현재 BBMTIME 값 (더 극적인 시작값)
static volatile int hstrt_level = 2; // 현재 HSTRT 값 (최적값으로 변경)
static volatile int hend_level = 2; // 현재 HEND 값 (최적값으로 변경)
static volatile int toff_level = 8; // 현재 TOFF 값 (안전한 시작값)
static volatile int g_microsteps = MICROSTEPS; // 헤더에서 정의된 마이크로스텝 값 사용 (런타임 동기화용)

static const char *TAG = "TMC5160";

static inline TickType_t ms_to_ticks(uint32_t ms) {
    // Convert milliseconds to FreeRTOS ticks safely
    if (ms == 0) return 0;
    uint32_t tick_ms = portTICK_PERIOD_MS;
    if (tick_ms == 0) return ms; // fallback
    return (ms + tick_ms - 1) / tick_ms;
}

static esp_err_t tmc5160_spi_init(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_SDI_CFG1,
        .miso_io_num = SPI_SDO_CFG0,
        .sclk_io_num = SPI_SCK_CFG2,
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
        .spics_io_num = SPI_CS_CFG3,
        .queue_size = 1,
        .flags = 0,
    };

    err = spi_bus_add_device(TMC_SPI_HOST, &devcfg, &tmc_spi);
    return err;
}

// Write 32-bit data to register (address without write bit)
static esp_err_t tmc5160_write_reg(uint8_t addr, uint32_t data) {
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

// Read 32-bit data from register (address without read bit)
static esp_err_t tmc5160_read_reg(uint8_t addr, uint32_t *data) {
    if (tmc_spi == NULL || data == NULL) return ESP_ERR_INVALID_STATE;
    uint8_t tx[5] = {0};
    uint8_t rx[5];
    tx[0] = addr & 0x7F; // read (no write bit)
    spi_transaction_t t = {
        .length = 40,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_transmit(tmc_spi, &t);
    if (err == ESP_OK) {
        *data = ((uint32_t)rx[1] << 24) | ((uint32_t)rx[2] << 16) | 
                ((uint32_t)rx[3] << 8) | rx[4];
    }
    return err;
}

// 전류 설정만 담당하는 함수 (CHOPCONF 제외)
static esp_err_t tmc5160_basic_startup_current_only(void) {
    // Compute IHOLD/IRUN from motor spec and Rsense
    // IRMS = ((CS+1)/32) * VFS/Rsense * 1/sqrt(2)
    // -> (CS+1) = IRMS * 32 * Rsense / (VFS/√2)
    const float VFS = 0.325f;        // typ. sense threshold (datasheet VSRT)
    const float RSENSE_OHMS = 0.15f; // set to your board's actual Rsense value (Ohms)
    const float TARGET_IRMS_A = 1.60f; // reduce run RMS current target to lower heating
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

// 완전한 기본 설정 함수 (CHOPCONF 포함) - 필요시 사용
static esp_err_t tmc5160_basic_startup(void) {
    // CHOPCONF: 헤더의 MICROSTEPS 값을 반영한 기본 설정
    // MRES 값을 MICROSTEPS에 맞게 계산
    int mres;
    switch (MICROSTEPS) {
        case 256: mres = 0; break;
        case 128: mres = 1; break;
        case 64:  mres = 2; break;
        case 32:  mres = 3; break;
        case 16:  mres = 4; break;
        case 8:   mres = 5; break;
        case 4:   mres = 6; break;
        case 2:   mres = 7; break;
        case 1:   mres = 8; break;
        default:  mres = 4; break; // 기본값: 16 µsteps
    }
    
    // CHOPCONF 구성: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0, MRES=계산된값
    uint32_t chopconf = 0x000100C0 | ((uint32_t)mres << 24);
    ESP_ERROR_CHECK(tmc5160_write_reg(0x6C, chopconf));
    
    ESP_LOGI(TAG, "Basic startup: MICROSTEPS=%d, MRES=%d, CHOPCONF=0x%08lX", MICROSTEPS, mres, chopconf);

    // 전류 설정 호출
    return tmc5160_basic_startup_current_only();
}

// Convert RPM to period in microseconds per microstep
static inline uint32_t rpm_to_period_us(float rpm) {
    if (rpm <= 0.5f) return 1000000; // very slow fallback
    const int fullsteps_per_rev = 200;
    int steps_per_rev = fullsteps_per_rev * MICROSTEPS; // 직접 매크로 사용
    if (steps_per_rev <= 0) steps_per_rev = 3200;
    float seconds_per_step = 60.0f / (rpm * (float)steps_per_rev);
    uint32_t us = (uint32_t)(seconds_per_step * 1e6f + 0.5f);
    
    // 디버깅: 계산 과정 출력
    ESP_LOGI(TAG, "Period Calc -> RPM:%.1f, MICROSTEPS=%d, g_microsteps=%d, Steps/Rev:%d, Seconds/Step:%.6f, Period:%lu µs", 
             (double)rpm, MICROSTEPS, g_microsteps, steps_per_rev, (double)seconds_per_step, us);
    
    // Minimum period to prevent ISR overload (GPTimer can handle down to ~10µs)
    if (us < 10) us = 10; // Reduced to 10µs minimum for higher speeds
    return us;
}

static bool IRAM_ATTR step_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
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

void set_hysteresis_level(int hstrt, int hend) {
    // CHOPCONF 레지스터 업데이트
    uint32_t chopconf = 0x140100C0 | (toff_level & 0x0F); // TOFF 유지
    chopconf &= ~(0x7F); // HSTRT(4-6) + HEND(0-3) 비트 클리어
    chopconf |= ((hstrt & 0x07) << 4) | (hend & 0x0F); // 새로운 값 설정
    
    ESP_ERROR_CHECK(tmc5160_write_reg(0x6C, chopconf));
    ESP_LOGI(TAG, "Hysteresis -> HSTRT:%d, HEND:%d", (int)hstrt, (int)hend);
}

// =====================================================
// 마이크로스텝 조정 함수
// =====================================================
// 마이크로스텝 설정 원리:
// - 높은 값: 부드러움, 조용함, 정밀함 (MCU 부하 증가)
// - 낮은 값: 빠른 응답, 안정성, 효율성 (약간의 진동)
// - 속도에 따라 자동 조정 권장
// =====================================================

void set_microstep_resolution(int microsteps) {
    ESP_LOGI(TAG, "set_microstep_resolution() called with microsteps=%d", microsteps);
    // 마이크로스텝 값을 MRES 레지스터 값으로 변환
    int mres;
    
    switch (microsteps) {
        case 256: mres = 0; break;   // 256 µsteps
        case 128: mres = 1; break;   // 128 µsteps
        case 64:  mres = 2; break;   // 64 µsteps
        case 32:  mres = 3; break;   // 32 µsteps
        case 16:  mres = 4; break;   // 16 µsteps
        case 8:   mres = 5; break;   // 8 µsteps
        case 4:   mres = 6; break;   // 4 µsteps
        case 2:   mres = 7; break;   // 2 µsteps
        case 1:   mres = 8; break;   // 1 µstep (full step)
        default:
            ESP_LOGW(TAG, "Invalid microsteps: %d, using 16", microsteps);
            mres = 4;  // 기본값: 16 µsteps
            microsteps = 16;
            break;
    }
    
    // 현재 설정값으로 CHOPCONF 구성 (TOFF, HSTRT, HEND 등 유지)
    uint32_t current_chopconf = 0x140100C0;  // 기본값 (TOFF=8, HSTRT=2, HEND=2)
    
    // 전역 변수에서 현재 HSTRT, HEND 값 적용
    current_chopconf &= ~(0x7F);  // HSTRT(4-6) + HEND(0-3) 비트 클리어
    current_chopconf |= ((hstrt_level & 0x07) << 4) | (hend_level & 0x0F);  // 현재 값 적용
    
    // TOFF 값도 적용
    current_chopconf &= ~(0x0F);  // TOFF 비트 클리어
    current_chopconf |= (toff_level & 0x0F);  // 현재 TOFF 값 적용
    
    // MRES 비트만 변경 (비트 24-27)
    current_chopconf &= ~(0x0F << 24);  // MRES 비트 클리어
    current_chopconf |= ((uint32_t)mres << 24);  // 새로운 MRES 값 설정
    
    // CHOPCONF 레지스터 업데이트
    ESP_ERROR_CHECK(tmc5160_write_reg(0x6C, current_chopconf));
    
    // 전역 변수 업데이트 (다른 함수에서 사용)
    g_microsteps = microsteps;
    
    ESP_LOGI(TAG, "Microstep Resolution -> %d µsteps (MRES:%d)", microsteps, mres);
}

// 현재 마이크로스텝 값 반환
int get_current_microsteps(void) {
    return g_microsteps;
}

// 현재 스텝 주기 값 반환 (µs)
uint32_t get_current_step_period(void) {
    return hw_current_period_us;
}

void set_current_level(float irun_current, float ihold_current) {
    // =====================================================
    // TMC5160A 전류 설정 함수
    // =====================================================
    // 목적: 모터의 구동 전류(IRUN)와 홀드 전류(IHOLD)를 설정
    // 
    // TMC5160A의 전류 제어 원리:
    // 1. 전류 감지: 외부 Rsense 저항으로 모터 전류를 측정
    // 2. 전류 제한: VFS 임계값과 비교하여 PWM 듀티 사이클 조정
    // 3. CS 값 계산: 실제 전류를 원하는 전류로 맞추기 위한 레지스터 값
    //
    // 수식: IRMS = ((CS+1)/32) * VFS/Rsense * 1/sqrt(2)
    // 역산: (CS+1) = IRMS * 32 * Rsense / (VFS/√2)
    // =====================================================
    
    // VFS (Voltage Sense Reference): 전류 감지 임계 전압 (V)
    // - TMC5160A 데이터시트 표준값: 0.325V
    // - 이 값보다 높은 전압이 Rsense에 걸리면 전류 제한 동작
    const float VFS = 0.325f;
    
    // RSENSE_OHMS: 외부 전류 감지 저항값 (Ω)
    // - TMC5160 데이터시트 기준: 0.12Ω = 2.0A RMS (모터 정격 전류와 정확히 일치)
    // - 0.15Ω = 1.6A RMS (부족), 0.10Ω = 2.3A RMS (과다)
    // - 보드에 실제 연결된 Rsense 저항값을 0.12Ω으로 설정 권장
    const float RSENSE_OHMS = 0.12f;  // 모터 정격 전류 2.0A에 최적화
    
    // HOLD_RATIO: 홀드 전류 비율 (0.0 ~ 1.0)
    // - IRUN 대비 IHOLD의 비율
    // - 예: 0.5 = IRUN의 50%를 홀드 전류로 설정
    // - 낮을수록 전력 절약, 높을수록 토크 유지
    const float HOLD_RATIO = ihold_current;
    
    // denom: 전류 계산식의 분모 부분 (VFS/√2)
    // - √2 = 1.41421356 (RMS to Peak 변환 계수)
    // - VFS는 피크값이므로 RMS로 변환하여 계산
    const float denom = (VFS / 1.41421356f);
    
    // CS+1 값 계산: 원하는 전류를 위한 레지스터 값
    // - irun_current: 목표 구동 전류 (A)
    // - 32.0f: TMC5160A의 CS 레지스터 스케일 팩터
    // - 0.5f: 반올림을 위한 오프셋
    float cs_plus_1 = (denom > 0.0f) ? (irun_current * 32.0f * RSENSE_OHMS / denom) : 1.0f;
    
    // IRUN (Run Current) 레지스터 값 계산
    // - CS+1에서 1을 빼서 실제 CS 값으로 변환
    // - 반올림 후 범위 제한 (8~31)
    int irun = (int)(cs_plus_1 + 0.5f) - 1;
    if (irun < 8) irun = 8;    // 최소값: StealthChop 튜닝을 위해 8 이상 필요
    if (irun > 31) irun = 31;  // 최대값: 5비트 레지스터 제한
    
    // IHOLD (Hold Current) 레지스터 값 계산
    // - IRUN 대비 HOLD_RATIO 비율로 설정
    // - 범위 제한 (1~31): 0은 사용 불가
    int ihold = (int)(irun * HOLD_RATIO + 0.5f);
    if (ihold < 1) ihold = 1;   // 최소값: 1 (0은 무효)
    if (ihold > 31) ihold = 31; // 최대값: 5비트 레지스터 제한
    
    // IHOLDDELAY: 홀드 전류로 전환되는 지연 시간
    // - 값 6: 약 6ms 후 홀드 전류로 전환
    // - 긴 지연: 부드러운 정지, 짧은 지연: 빠른 전력 절약
    int iholddelay = 6;
    uint32_t ihold_irun = ((uint32_t)iholddelay << 16) | ((uint32_t)irun << 8) | (uint32_t)ihold;
    
    ESP_ERROR_CHECK(tmc5160_write_reg(0x10, ihold_irun));
    ESP_LOGI(TAG, "Current -> IRUN:%.1fA, IHOLD:%.1fA", (double)irun_current, (double)ihold_current);
}

// =====================================================
// 모터 스펙 기반 전류 계산 함수
// =====================================================
// 모터 스펙:
// - 정격 전류: 2.0A
// - 정격 전압: 2.2V  
// - 저항: 1.1Ω ±10%
// - 인덕턴스: 1.65mH ±20%
// - 홀딩 토크: 0.27 N.m
// =====================================================
void set_current_for_motor_spec(float current_ratio) {
    // current_ratio: 정격 전류 대비 비율 (0.1 ~ 1.0)
    // 예: 0.8 = 정격 전류의 80% (1.6A)
    
    if (current_ratio < 0.1f) current_ratio = 0.1f;  // 최소 10%
    if (current_ratio > 1.0f) current_ratio = 1.0f;  // 최대 100%
    
    const float MOTOR_RATED_CURRENT = 2.0f;  // 모터 정격 전류 (A)
    const float MOTOR_RATED_VOLTAGE = 2.2f;  // 모터 정격 전압 (V)
    const float MOTOR_RESISTANCE = 1.1f;     // 모터 저항 (Ω)
    
    // 계산된 전류값
    float irun_current = MOTOR_RATED_CURRENT * current_ratio;
    float ihold_current = irun_current * 0.3f;  // 홀드 전류는 구동 전류의 30%
    
    // 전류 설정 적용
    set_current_level(irun_current, ihold_current);
    
    ESP_LOGI(TAG, "Motor Spec Current -> Ratio:%.1f, IRUN:%.1fA (%.1f%% of rated)", 
             (double)current_ratio, (double)irun_current, (double)(current_ratio * 100.0f));
}

static void set_adjust(float rpm) {
    // =====================================================
    // 속도별 전류 및 히스테리시스 자동 조정
    // =====================================================
    // 전류 설정 원리:
    // - 저속: 높은 토크 필요 (가속, 정지, 방향 전환)
    // - 고속: 낮은 토크 필요 (관성으로 회전, 안정적 구동)
    // - 실제 모터에서 측정되는 전류는 속도가 높을수록 낮아짐
    // =====================================================
    
    int hstrt, hend;
    float irun_current, ihold_current;
    // int micro_steps;

    if (rpm <= 60.0f) {
        // 낮은 속도: 높은 토크 필요 (가속, 정지 시)
        hstrt = 2; hend = 2;
        irun_current = 1.8f;  // 높은 전류: 토크 부족 방지
        ihold_current = 0.5f; // 홀드 토크도 높게
    } else if (rpm >= 60.0f && rpm <= 120.0f) {
        // 중간 속도: 균형 설정
        hstrt = 3; hend = 1;
        irun_current = 1.6f;  // 중간 전류: 안정적 구동
        ihold_current = 0.4f;
    } else if (rpm >= 120.0f && rpm <= 240.0f) {
        // 높은 속도: 관성으로 회전, 낮은 토크
        hstrt = 4; hend = 1;
        irun_current = 1.4f;  // 낮은 전류: 전력 절약
        ihold_current = 0.3f; // 홀드 전류도 낮게
    } else if (rpm >= 240.0f && rpm <= 360.0f) {
        hstrt = 5; hend = 1;
        irun_current = 1.2f;  // 낮은 전류: 전력 절약
        ihold_current = 0.2f; // 홀드 전류도 낮게
    } else if (rpm >= 360.0f && rpm <= 480.0f) {
        hstrt = 6; hend = 1;
        irun_current = 1.2f;  // 최소 전류 보장
        ihold_current = 0.3f; // 홀드 전류도 보장
    } else {
        hstrt = 7; hend = 1;
        irun_current = 1.0f;  // 최소 전류 보장
        ihold_current = 0.2f; // 홀드 전류도 보장
    }
    set_hysteresis_level(hstrt, hend);
    set_current_level(irun_current, ihold_current);
    
}

void set_target_rpm(float rpm)
{
    if (rpm < RPM_STEP) rpm = RPM_MIN;
    if (rpm > RPM_MAX) rpm = RPM_MAX;
    g_target_rpm = rpm;
    hw_current_period_us = rpm_to_period_us(rpm);
    
    // 속도에 따른 자동 조정
    set_adjust(rpm);
}

void stepper_start_continuous(int dir)
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

void stepper_stop_continuous(void)
{
    g_run_enabled = false; // ISR will stop at next falling edge
    vTaskDelay(ms_to_ticks(100));
    gpio_set_level(RELAY_BREAK_A_PIN, 0);
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

static void break_init() {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << RELAY_BREAK_A_PIN | 1ULL << RELAY_BREAK_B_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

// Apply optimized settings: StealthChop + SpreadCycle + Thermal optimizations
static esp_err_t tmc5160_apply_optimized_settings(void) {
    if (tmc_spi == NULL) return ESP_ERR_INVALID_STATE;
    
    // StealthChop + SpreadCycle 설정
    ESP_ERROR_CHECK(tmc5160_write_reg(0x00, 0x00000004)); // GCONF: en_pwm_mode=1
    
    // CHOPCONF: 헤더의 MICROSTEPS 값을 반영한 설정
    // MRES 값을 MICROSTEPS에 맞게 계산
    int mres;
    switch (MICROSTEPS) {
        case 256: mres = 0; break;
        case 128: mres = 1; break;
        case 64:  mres = 2; break;
        case 32:  mres = 3; break;
        case 16:  mres = 4; break;
        case 8:   mres = 5; break;
        case 4:   mres = 6; break;
        case 2:   mres = 7; break;
        case 1:   mres = 8; break;
        default:  mres = 4; break; // 기본값: 16 µsteps
    }
    
    // CHOPCONF 구성: TOFF=8, HSTRT=2, HEND=2, TPFD=0, MRES=계산된값
    uint32_t chopconf = 0x140100C0 | ((uint32_t)mres << 24);
    ESP_ERROR_CHECK(tmc5160_write_reg(0x6C, chopconf));
    
    ESP_LOGI(TAG, "Optimized settings: MICROSTEPS=%d, MRES=%d, CHOPCONF=0x%08lX", MICROSTEPS, mres, chopconf);
    
    // 실제 CHOPCONF 레지스터 값을 읽어서 확인
    uint32_t actual_chopconf = 0;
    tmc5160_read_reg(0x6C, &actual_chopconf);
    ESP_LOGI(TAG, "Actual CHOPCONF register: 0x%08lX", actual_chopconf);
    
    // TPWMTHRS: StealthChop 전환 임계점
    ESP_ERROR_CHECK(tmc5160_write_reg(0x13, 0x000001F4)); // 500
    
    // 전류 스케일
    ESP_ERROR_CHECK(tmc5160_write_reg(0x0B, 150)); // GLOBAL_SCALER
    
    // 드라이버 설정
    ESP_ERROR_CHECK(tmc5160_write_reg(0x0A, 0x00100004)); // DRV_CONF
    
    // CoolStep 설정
    ESP_ERROR_CHECK(tmc5160_write_reg(0x6D, 0x0000CA22)); // COOLCONF
    ESP_ERROR_CHECK(tmc5160_write_reg(0x14, 0x000007D0)); // TCOOLTHRS
    
    return ESP_OK;
}

void toggle_bbmtime_level(void) {
    // BBMTIME 값 순환: 2 -> 4 -> 8 -> 12 -> 2 (더 극적인 변화)
    if (bbmtime_level == 2) bbmtime_level = 4;
    else if (bbmtime_level == 4) bbmtime_level = 8;
    else if (bbmtime_level == 8) bbmtime_level = 12;
    else bbmtime_level = 2;
    
    // DRV_CONF 레지스터 업데이트 (BBMTIME만 변경)
    // 기존: 0x00100004 (BBMTIME=4)
    // 새로운 BBMTIME 값으로 계산
    uint32_t drv_conf = 0x00100000 | (bbmtime_level & 0x0F); // BBMTIME 비트만 변경
    
    ESP_ERROR_CHECK(tmc5160_write_reg(0x0A, drv_conf));
    
    ESP_LOGI(TAG, "BBMTIME -> %d", bbmtime_level);
}

void toggle_hysteresis_level(void) {
    // HSTRT/HEND 값 순환: (4,1) -> (2,2) -> (0,4) -> (6,0) -> (4,1)
    if (hstrt_level == 4 && hend_level == 1) {
        hstrt_level = 2; hend_level = 2;
    } else if (hstrt_level == 2 && hend_level == 2) {
        hstrt_level = 0; hend_level = 4;
    } else if (hstrt_level == 0 && hend_level == 4) {
        hstrt_level = 6; hend_level = 0;
    } else {
        hstrt_level = 4; hend_level = 1;
    }
    
    // CHOPCONF 레지스터 업데이트 (HSTRT/HEND만 변경)
    // 기존 CHOPCONF에서 HSTRT/HEND 비트만 변경
    // HSTRT: 비트 4-6, HEND: 비트 0-3
    uint32_t chopconf = 0x140100C3; // 기본값 (TOFF=3, HSTRT=4, HEND=1, MRES=16)
    
    // HSTRT/HEND 비트만 변경
    chopconf &= ~(0x7F); // HSTRT(4-6) + HEND(0-3) 비트 클리어
    chopconf |= ((hstrt_level & 0x07) << 4) | (hend_level & 0x0F); // 새로운 값 설정
    
    ESP_ERROR_CHECK(tmc5160_write_reg(0x6C, chopconf));
    
    ESP_LOGI(TAG, "Hysteresis -> HSTRT:%d, HEND:%d", hstrt_level, hend_level);
}

void toggle_toff_level(void) {
    // TOFF 값 순환: 8 -> 10 -> 12 -> 8 (더 안전한 범위)
    if (toff_level == 8) toff_level = 10;
    else if (toff_level == 10) toff_level = 12;
    else toff_level = 8;
    
    // CHOPCONF 레지스터 업데이트 (TOFF만 변경)
    // 기존: 0x140100CC (TOFF=12)
    // 새로운 TOFF 값으로 계산
    uint32_t chopconf = 0x140100C0 | (toff_level & 0x0F); // TOFF 비트만 변경
    
    ESP_ERROR_CHECK(tmc5160_write_reg(0x6C, chopconf));
    
    ESP_LOGI(TAG, "TOFF -> %d", toff_level);
}

void tmc5160_sd_driver_init() {

    gpio_init_step_dir();
    break_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    // When SPI_MODE=1 and SD_MODE=1 (SD+SPI), initialize SPI and bring driver out of reset defaults (TOFF>0)
    if (tmc5160_spi_init() == ESP_OK) {
        // 기본 전류 설정만 수행 (CHOPCONF는 최적화 함수에서 설정)
        tmc5160_basic_startup_current_only();
    }

    // Apply optimized settings (StealthChop + SpreadCycle + Thermal optimizations)
    tmc5160_apply_optimized_settings();
    
    // SD 모드에서 마이크로스텝 설정을 강제로 적용
    // CHOPCONF를 다시 설정하여 SD 모드에서도 올바른 마이크로스텝이 적용되도록 함
    int mres;
    switch (MICROSTEPS) {
        case 256: mres = 0; break;
        case 128: mres = 1; break;
        case 64:  mres = 2; break;
        case 32:  mres = 3; break;
        case 16:  mres = 4; break;
        case 8:   mres = 5; break;
        case 4:   mres = 6; break;
        case 2:   mres = 7; break;
        case 1:   mres = 8; break;
        default:  mres = 4; break; // 기본값: 16 µsteps
    }
    
    // SD 모드에서 마이크로스텝 설정 강제 적용
    uint32_t chopconf = 0x140100C0 | ((uint32_t)mres << 24);
    ESP_ERROR_CHECK(tmc5160_write_reg(0x6C, chopconf));
    ESP_LOGI(TAG, "SD mode microstep force set: MICROSTEPS=%d, MRES=%d, CHOPCONF=0x%08lX", MICROSTEPS, mres, chopconf);
    
    // 설정 후 실제 레지스터 값 확인
    uint32_t actual_chopconf = 0;
    tmc5160_read_reg(0x6C, &actual_chopconf);
    uint8_t actual_mres = (actual_chopconf >> 24) & 0x0F;
    ESP_LOGI(TAG, "After force set - Actual CHOPCONF: 0x%08lX, MRES: %d", actual_chopconf, actual_mres);
    
    // 다른 중요한 레지스터들도 확인
    uint32_t gconf = 0, tpowerdown = 0, tstep = 0, drv_status = 0;
    tmc5160_read_reg(0x00, &gconf);      // GCONF
    tmc5160_read_reg(0x11, &tpowerdown); // TPOWERDOWN
    tmc5160_read_reg(0x12, &tstep);      // TSTEP
    tmc5160_read_reg(0x6F, &drv_status); // DRV_STATUS
    
    ESP_LOGI(TAG, "Other registers - GCONF: 0x%08lX, TPOWERDOWN: 0x%08lX, TSTEP: 0x%08lX, DRV_STATUS: 0x%08lX", 
             gconf, tpowerdown, tstep, drv_status);

    ESP_LOGI(TAG, "TMC5160 SD driver initialized");
}

// TMC5160 레지스터 상태 확인 함수
void check_tmc5160_status(void) {
    uint32_t chopconf = 0, gconf = 0, tstep = 0, drv_status = 0;
    
    tmc5160_read_reg(0x6C, &chopconf); // CHOPCONF
    tmc5160_read_reg(0x00, &gconf);    // GCONF
    tmc5160_read_reg(0x12, &tstep);    // TSTEP
    tmc5160_read_reg(0x6F, &drv_status); // DRV_STATUS
    
    uint8_t mres = (chopconf >> 24) & 0x0F;
    uint8_t toff = chopconf & 0x0F;
    uint8_t hstrt = (chopconf >> 4) & 0x07;
    uint8_t hend = (chopconf >> 7) & 0x0F;
    
    ESP_LOGI(TAG, "TMC5160 Status Check:");
    ESP_LOGI(TAG, "  CHOPCONF: 0x%08lX (MRES:%d, TOFF:%d, HSTRT:%d, HEND:%d)", 
             chopconf, mres, toff, hstrt, hend);
    ESP_LOGI(TAG, "  GCONF: 0x%08lX", gconf);
    ESP_LOGI(TAG, "  TSTEP: 0x%08lX", tstep);
    ESP_LOGI(TAG, "  DRV_STATUS: 0x%08lX", drv_status);
    
    // 현재 마이크로스텝 설정과 비교
    int expected_mres;
    switch (MICROSTEPS) {
        case 256: expected_mres = 0; break;
        case 128: expected_mres = 1; break;
        case 64:  expected_mres = 2; break;
        case 32:  expected_mres = 3; break;
        case 16:  expected_mres = 4; break;
        case 8:   expected_mres = 5; break;
        case 4:   expected_mres = 6; break;
        case 2:   expected_mres = 7; break;
        case 1:   expected_mres = 8; break;
        default:  expected_mres = 4; break;
    }
    ESP_LOGI(TAG, "  Expected MRES: %d (for %d microsteps)", expected_mres, MICROSTEPS);
    
    if (mres == expected_mres) {
        ESP_LOGI(TAG, "  ✓ MRES setting is correct!");
    } else {
        ESP_LOGI(TAG, "  ✗ MRES setting is wrong! Expected: %d, Actual: %d", expected_mres, mres);
    }
}