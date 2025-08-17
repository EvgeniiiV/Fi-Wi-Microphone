#ifndef HARDWARE_CONTROL_H
#define HARDWARE_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "u8g2.h"
#include "project_config.h"

extern u8g2_t u8g2;
extern QueueHandle_t encoder_evt_queue;

void hardware_init(void);
void apply_initial_settings(const audio_settings_t* settings, uint8_t initial_volume);
void set_volume(uint8_t volume);
void set_gate(uint8_t value);
void set_compression(uint8_t value);
void set_mute_led(bool is_on);
int get_battery_voltage_mv(void);
bool is_usb_plugged_in(void);
esp_err_t read_i2s_data(uint8_t* buffer, size_t size, size_t* bytes_read);
esp_err_t save_settings_to_nvs(const audio_settings_t* settings);
esp_err_t load_settings_from_nvs(audio_settings_t* settings);

#endif // HARDWARE_CONTROL_H