#ifndef TMC5160_SD_DRIVER_H
#define TMC5160_SD_DRIVER_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"

#include "driver/spi_master.h"
#include "esp_err.h"
#include <string.h>

#include "esp_log.h"

#define SPI_SDO_CFG0 GPIO_NUM_42
#define SPI_SDI_CFG1 GPIO_NUM_41
#define SPI_SCK_CFG2 GPIO_NUM_40
#define SPI_CS_CFG3  GPIO_NUM_39

#define MOTOR_1 1
#define MOTOR_2 2

#define REFL_STEP_1 GPIO_NUM_2   // STEP pin -> TMC5160 REFL_STEP
#define REFR_DIR_1  GPIO_NUM_1   // DIR  pin -> TMC5160 REFR_DIR

#define REFL_STEP_2 GPIO_NUM_36   // STEP pin -> TMC5160 REFL_STEP
#define REFR_DIR_2  GPIO_NUM_37   // DIR  pin -> TMC5160 REFR_DIR

#define RELAY_BREAK_A_PIN GPIO_NUM_35 // GPIO_NUM_17
#define RELAY_BREAK_B_PIN GPIO_NUM_48 // GPIO_NUM_18

#define PHOTO_INTERRUPT_PIN GPIO_NUM_7

// Optional: define DRV_ENN if you wired it to MCU (active-low enable)
// Set to -1 if not connected
#define DRV_ENN_PIN (-1)

#define TMC_SPI_HOST SPI2_HOST

#define CW_DIR 0
#define CCW_DIR 1

#define MICROSTEPS 16

#define RPM_MAX 900.0f               // max RPM before wrap
#define RPM_MIN 0.0f              // min RPM before wrap
#define RPM_STEP 60.0f                // short press increment (button push)

extern volatile bool g_run_enabled_1;           // continuous run flag
extern volatile float g_target_rpm_1;
extern volatile bool g_run_enabled_2;           // continuous run flag
extern volatile float g_target_rpm_2;

typedef struct {
    int level;             // 0 pressed (active-low), 1 released
    TickType_t tick;       // timestamp
} button_evt_t;

void tmc5160_sd_driver_init();

void set_target_rpm(int motor_num, float rpm);
void stepper_start_continuous(int motor_num, int dir);
void stepper_stop_continuous(int motor_num);
void break_control(int break_on_off);

// 정확한 위치 제어 함수들
void stepper_move_steps(int motor_num, int steps, int dir, float rpm);
void stepper_move_rotations(int motor_num, float rotations, int dir, float rpm);
void stepper_move_to_position(int motor_num, int target_position, float rpm);



uint32_t get_current_step_period(int motor_num);

// TMC5160 레지스터 상태 확인 함수
void check_tmc5160_status(void);

// 모터 동작 테스트 함수
void motor_test_sequence(void);
void motor_precise_test_sequence(void);
void motor_step_acceleration_test_sequence(void);

void set_current_level(float irun_current, float ihold_current);

void set_direction(int motor_num, int dir);
void toggle_direction(void);

void toggle_bbmtime_level(void);
void toggle_hysteresis_level(void);
void toggle_toff_level(void);

uint32_t get_photo_interrupt_count(void); // 포토 인터럽트 카운트를 외부에서 읽을 수 있는 함수
void reset_photo_interrupt_count(void); // 포토 인터럽트 카운트를 리셋하는 함수

#endif // TMC5160_SD_DRIVER_H