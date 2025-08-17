#include "u8g2_esp32_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "rom/ets_sys.h"

static const char* TAG = "u8g2_hal";

// Внутренний указатель для хранения дескриптора OLED-устройства
static i2c_master_dev_handle_t dev_handle_oled = NULL;

// ФИНАЛЬНАЯ ПРАВИЛЬНАЯ РЕАЛИЗАЦИЯ: init сохраняет переданный дескриптор
void u8g2_esp32_hal_init(i2c_master_dev_handle_t dev_handle) {
    dev_handle_oled = dev_handle;
}

uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    static uint8_t buffer[32];
    static uint8_t buf_len = 0;
    switch(msg) {
        case U8X8_MSG_BYTE_INIT:
        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_len = 0;
            break;
        case U8X8_MSG_BYTE_SEND: {
            uint8_t* data = (uint8_t*)arg_ptr;
            if (buf_len + arg_int <= sizeof(buffer)) {
                memcpy(&buffer[buf_len], data, arg_int);
                buf_len += arg_int;
            }
            break;
        }
        case U8X8_MSG_BYTE_END_TRANSFER: {
            if (dev_handle_oled != NULL && buf_len > 0) {
                esp_err_t err = i2c_master_transmit(dev_handle_oled, buffer, buf_len, pdMS_TO_TICKS(100));
                vTaskDelay(1); 
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "I2C transmit failed: %s", esp_err_to_name(err));
                    return 0;
                }
            }
            buf_len = 0;
            break;
        }
        case U8X8_MSG_BYTE_SET_DC: break;
        default: return 0;
    }
    return 1;
}

uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_DELAY_MILLI: vTaskDelay(pdMS_TO_TICKS(arg_int)); break;
        case U8X8_MSG_DELAY_10MICRO: ets_delay_us(10); break;
        case U8X8_MSG_DELAY_100NANO: ets_delay_us(1); break;
        case U8X8_MSG_GPIO_AND_DELAY_INIT: break;
        default: u8x8_SetGPIOResult(u8x8, 1); break;
    }
    return 1;
}