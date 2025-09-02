#ifndef BUTTON_DRIVER_H
#define BUTTON_DRIVER_H

#include "iot_button.h"
#include "button_gpio.h"

#include "driver/gpio.h"

#include "tmc5160_SD_driver.h"

#include "esp_log.h"

#define BUTTON_ACTIVE_LEVEL     0

#define BOOT_BUTTON_PIN GPIO_NUM_0 // GPIO_NUM_0
#define SUB_BUTTON_PIN_1 GPIO_NUM_19 // GPIO_NUM_35
#define SUB_BUTTON_PIN_2 GPIO_NUM_47 // GPIO_NUM_35

#define SUB_BUTTON_VCC_PIN GPIO_NUM_20 // 임시 버튼 전원
#define SUB_BUTTON_GND_PIN GPIO_NUM_45 // 임시 버튼 전원

void button_init();

#endif // BUTTON_DRIVER_H