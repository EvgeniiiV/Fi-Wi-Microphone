#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"

// Подключаем заголовочные файлы всех наших модулей
#include "project_config.h"
#include "hardware_control.h"
#include "app_tasks.h"
#include "network_stream.h"

// Статический тэг для логирования
static const char *TAG = "MAIN";

static TaskHandle_t i2s_task_handle = NULL;
static TaskHandle_t udp_task_handle = NULL;
static TaskHandle_t encoder_task_handle = NULL;
static TaskHandle_t display_task_handle = NULL;
static TaskHandle_t monitor_task_handle = NULL;


void app_main(void) {
    ESP_LOGI(TAG, "ESP32 Wi-Fi Microphone v3.0 - Application Start");

    // 1. Инициализация NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Пауза для стабилизации питания
    vTaskDelay(pdMS_TO_TICKS(500));

    // 3. Создание объектов синхронизации
    encoder_evt_queue = xQueueCreate(10, sizeof(encoder_event_t));
    if (encoder_evt_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create encoder queue");
        return;
    }

    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }

    // 4. Инициализация hardware и загрузка настроек
    hardware_init();
    load_settings_from_nvs(&current_audio_settings);
    
    // 5. Инициализация WiFi до создания задач
    wifi_init_softap();
    
    // 6. Применение настроек
    apply_initial_settings(&current_audio_settings, current_volume);
    
    // 7. Создание задач
    ESP_LOGI(TAG, "Creating application tasks...");
    BaseType_t xReturned;

    // Ядро 0: Аудио и сеть (реального времени)
    xReturned = xTaskCreatePinnedToCore(i2s_to_udp_task, "i2s_to_udp", 
        8192,  // Увеличенный стек для I2S задачи
        NULL, 
        configMAX_PRIORITIES-1, 
        &i2s_task_handle, 
        0);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create I2S task");
        return;
    }

    xReturned = xTaskCreatePinnedToCore(udp_server_task, "udp_server", 
        6144,  // Увеличенный стек для UDP
        NULL, 
        configMAX_PRIORITIES-2, 
        &udp_task_handle, 
        0);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UDP task");
        return;
    }

    // Ядро 1: Интерфейс
    xReturned = xTaskCreatePinnedToCore(encoder_handler_task, "encoder", 
        3072, 
        NULL, 
        5, 
        &encoder_task_handle, 
        1);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create encoder task");
        return;
    }

    xReturned = xTaskCreatePinnedToCore(display_task, "display", 
        4096, 
        NULL, 
        4, 
        &display_task_handle, 
        1);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display task");
        return;
    }

    xReturned = xTaskCreatePinnedToCore(system_monitor_task, "sys_monitor", 
        2048, 
        NULL, 
        3, 
        &monitor_task_handle, 
        1);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
        return;
    }

    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "System is ready");
}