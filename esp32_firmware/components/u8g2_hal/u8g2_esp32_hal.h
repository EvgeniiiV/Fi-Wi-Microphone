#ifndef U8G2_ESP32_HAL_H_
#define U8G2_ESP32_HAL_H_

#include "u8g2.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h" // <-- Важно: подключаем новый API

// Эта структура больше не нужна для I2C, но оставляем для совместимости
typedef struct {
	gpio_num_t sda;
	gpio_num_t scl;
} u8g2_esp32_hal_t;
#define U8G2_ESP32_HAL_DEFAULT { GPIO_NUM_NC, GPIO_NUM_NC }

// ФИНАЛЬНАЯ ПРАВИЛЬНАЯ СИГНАТУРА: init принимает дескриптор устройства
void u8g2_esp32_hal_init(i2c_master_dev_handle_t dev_handle);

uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

#endif /* U8G2_ESP32_HAL_H_ */