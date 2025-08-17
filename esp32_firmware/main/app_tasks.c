#include "app_tasks.h"
#include "hardware_control.h"
#include "network_stream.h" // Нужен для флага client_connected
#include "esp_log.h"
#include <string.h> // для sprintf
#include <stdio.h>  // для sprintf

static const char *TAG = "APP_TASKS";

/* ========================================================================== */
/*                 ОПРЕДЕЛЕНИЕ ГЛОБАЛЬНЫХ ПЕРЕМЕННЫХ СОСТОЯНИЯ                */
/* ========================================================================== */

volatile system_mode_t current_mode = MODE_OPERATING;
volatile settings_param_t current_setting_param = SETTING_COMPRESSION;
audio_settings_t current_audio_settings = { .compression = 5, .noise_gate = 8 };
volatile int current_volume = 64;
volatile bool is_muted = false;
volatile int battery_percentage = 0;
volatile bool is_usb_plugged = false;

/* ========================================================================== */
/*                      РЕАЛИЗАЦИЯ ЗАДАЧИ ЭНКОДЕРА                            */
/* ========================================================================== */

void encoder_handler_task(void *pvParameters) {
    encoder_event_t evt;
    TickType_t last_click_time = 0;
    const TickType_t double_click_timeout = pdMS_TO_TICKS(400);

    while (1) {
        // Ждем событие. Если его нет, проверяем таймаут одиночного клика.
        if (xQueueReceive(encoder_evt_queue, &evt, pdMS_TO_TICKS(50))) {
            switch (evt) {
                case ENCODER_EVENT_CLICK: {
                    TickType_t now = xTaskGetTickCount();
                    // Это двойной клик?
                    if (last_click_time != 0 && (now - last_click_time < double_click_timeout)) {
                        last_click_time = 0; // Сбрасываем таймер
                        // Переключаем режим
                        current_mode = (current_mode == MODE_OPERATING) ? MODE_SETTINGS : MODE_OPERATING;
                        // Если вышли из режима настроек - сохраняем их
                        if (current_mode == MODE_OPERATING) {
                            save_settings_to_nvs(&current_audio_settings);
                        }
                        ESP_LOGI(TAG, "Double-click: Mode switched to %s", current_mode == MODE_OPERATING ? "OPERATING" : "SETTINGS");
                    } else {
                        // Это первый клик, запускаем таймер
                        last_click_time = now;
                    }
                    break;
                }
                case ENCODER_EVENT_ROTATE_CW: // По часовой стрелке
                    if (current_mode == MODE_OPERATING) {
                        if (!is_muted && current_volume < VOLUME_MAX) {
                            current_volume += VOLUME_STEP;
                            if (current_volume > VOLUME_MAX) current_volume = VOLUME_MAX;
                            set_volume(current_volume);
                        }
                    } else { // Режим настроек
                        if (current_setting_param == SETTING_COMPRESSION) {
                            if (current_audio_settings.compression < COMPRESSION_STEPS - 1) current_audio_settings.compression++;
                            set_compression(current_audio_settings.compression);
                        } else { // SETTING_GATE
                            if (current_audio_settings.noise_gate < GATE_STEPS - 1) current_audio_settings.noise_gate++;
                            set_gate(current_audio_settings.noise_gate);
                        }
                    }
                    break;
                case ENCODER_EVENT_ROTATE_CCW: // Против часовой стрелки
                     if (current_mode == MODE_OPERATING) {
                        if (!is_muted && current_volume > VOLUME_MIN) {
                            current_volume -= VOLUME_STEP;
                            if (current_volume < VOLUME_MIN) current_volume = VOLUME_MIN;
                            set_volume(current_volume);
                        }
                    } else { // Режим настроек
                        if (current_setting_param == SETTING_COMPRESSION) {
                            if (current_audio_settings.compression > 0) current_audio_settings.compression--;
                            set_compression(current_audio_settings.compression);
                        } else { // SETTING_GATE
                            if (current_audio_settings.noise_gate > 0) current_audio_settings.noise_gate--;
                            set_gate(current_audio_settings.noise_gate);
                        }
                    }
                    break;
            }
        }
        
        // Проверка таймаута одиночного клика
        if (last_click_time != 0 && (xTaskGetTickCount() - last_click_time >= double_click_timeout)) {
            if (current_mode == MODE_OPERATING) {
                is_muted = !is_muted;
                set_mute_led(is_muted);
                ESP_LOGI(TAG, "Single-click: Mute toggled to %s", is_muted ? "ON" : "OFF");
            } else { // Режим настроек
                current_setting_param = (current_setting_param == SETTING_COMPRESSION) ? SETTING_GATE : SETTING_COMPRESSION;
                ESP_LOGI(TAG, "Single-click: Setting toggled to %s", current_setting_param == SETTING_COMPRESSION ? "COMPRESSION" : "GATE");
            }
            last_click_time = 0; // Сбрасываем таймер
        }
    }
}

/* ========================================================================== */
/*                       РЕАЛИЗАЦИЯ ЗАДАЧИ ДИСПЛЕЯ                            */
/* ========================================================================== */

void display_task(void *pvParameters) {
    // Графика для иконок
    static const unsigned char wifi_icon_bits[] = {0x00,0x00,0x18,0x00,0x66,0x01,0xC3,0x03,0x87,0x07,0x0F,0x0E,0x1E,0x1C,0x3C,0x38,0x78,0x70,0xF0,0xE0,0xE0,0xC0,0xC0,0x81,0x80,0x03,0x00,0x06,0x00,0x0C,0x00,0x00};
    static const unsigned char charge_icon_bits[] = {0x00,0x00,0xfe,0x03,0xfe,0x07,0xfe,0x07,0x7e,0x06,0x3c,0x04,0x3c,0x04,0x78,0x04,0xe0,0x03,0xc0,0x03,0x80,0x03,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x00,0x00,0x00};

    char buffer[32];

    while(1) {
        u8g2_FirstPage(&u8g2);
        do {
            /* ----- ЖЕЛТАЯ ЗОНА (Y < 16) - СТАТУС-БАР ----- */
            u8g2_SetFont(&u8g2, u8g2_font_7x13B_tr);
            u8g2_DrawStr(&u8g2, 2, 12, "Wi-Fi Mic");

            // Иконка Wi-Fi (статус берем из сетевого модуля)
            if (is_client_connected()) {
                u8g2_DrawXBM(&u8g2, 128 - 18, 0, 16, 16, wifi_icon_bits);
            }
            
            // Иконка батареи/зарядки
            if (is_usb_plugged) {
                 u8g2_DrawXBM(&u8g2, 128 - 38, 0, 16, 16, charge_icon_bits);
            } else {
                u8g2_DrawFrame(&u8g2, 128 - 38, 2, 18, 10);
                u8g2_DrawBox(&u8g2, 128 - 36, 4, (14 * battery_percentage) / 100, 6);
            }

            /* ----- СИНЯЯ ЗОНА (Y >= 16) - РАБОЧАЯ ОБЛАСТЬ ----- */
            if (current_mode == MODE_OPERATING) {
                if (is_muted) {
                    u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tr);
                    u8g2_DrawStr(&u8g2, 24, 45, "-- MUTED --");
                } else {
                    u8g2_SetFont(&u8g2, u8g2_font_7x13B_tr);
                    u8g2_DrawStr(&u8g2, 2, 28, "Volume");
                    u8g2_DrawFrame(&u8g2, 2, 32, 100, 18);
                    int bar_width = (current_volume * (100-4)) / VOLUME_MAX;
                    u8g2_DrawBox(&u8g2, 4, 34, bar_width, 14);
                    sprintf(buffer, "%d", current_volume);
                    u8g2_DrawStr(&u8g2, 105, 46, buffer);
                }
            } else { // MODE_SETTINGS
                u8g2_SetFont(&u8g2, u8g2_font_profont12_tr);
                // Компрессия
                sprintf(buffer, "Comp: %d:1", current_audio_settings.compression + 1);
                if (current_setting_param == SETTING_COMPRESSION) u8g2_DrawStr(&u8g2, 0, 32, ">");
                u8g2_DrawStr(&u8g2, 10, 32, buffer);
                // Гейт
                sprintf(buffer, "Gate: -%d dBV", 55 - current_audio_settings.noise_gate);
                if (current_setting_param == SETTING_GATE) u8g2_DrawStr(&u8g2, 0, 52, ">");
                u8g2_DrawStr(&u8g2, 10, 52, buffer);
            }
        } while (u8g2_NextPage(&u8g2));

        vTaskDelay(pdMS_TO_TICKS(100)); // Обновляем экран 10 раз в секунду
    }
}


/* ========================================================================== */
/*                    РЕАЛИЗАЦИЯ ЗАДАЧИ МОНИТОРИНГА                           */
/* ========================================================================== */

void system_monitor_task(void *pvParameters) {
    while(1) {
        // Обновляем статус подключения USB
        is_usb_plugged = is_usb_plugged_in();
        
        // Читаем напряжение батареи
        int voltage_mv = get_battery_voltage_mv();
        
        // Линейная интерполяция процентов заряда (3.3V * 2 = 6.6V -> 0%; 4.2V * 2 = 8.4V -> 100%)
        const int min_volt = 6600; // 0%
        const int max_volt = 8400; // 100%
        if (voltage_mv < min_volt) {
            battery_percentage = 0;
        } else if (voltage_mv > max_volt) {
            battery_percentage = 100;
        } else {
            battery_percentage = ((voltage_mv - min_volt) * 100) / (max_volt - min_volt);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Проверяем раз в секунду
    }
}