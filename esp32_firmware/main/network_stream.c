#include "network_stream.h"
#include "hardware_control.h" // Для read_i2s_data()
#include "app_tasks.h"        // Для current_volume, is_muted

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include <string.h>
#include "esp_mac.h"
#include "project_config.h"

static const char *TAG = "NETWORK";

// Биты для группы событий
const int WIFI_AP_STA_CONNECTED_BIT = BIT0;
const int UDP_CLIENT_READY_BIT = BIT1;

/* ========================================================================== */
/*                 ОПРЕДЕЛЕНИЕ ГЛОБАЛЬНЫХ ПЕРЕМЕННЫХ                          */
/* ========================================================================== */
EventGroupHandle_t wifi_event_group;

// Статические переменные, видимые только внутри этого модуля
static int sock = -1;
static struct sockaddr_in dest_addr;
static volatile bool client_is_connected_flag = false;


/* ========================================================================== */
/*                      СТАТИЧЕСКИЕ ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ                   */
/* ========================================================================== */

// Обработчик событий Wi-Fi
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station " MACSTR " joined, AID=%d", MAC2STR(event->mac), event->aid);
        client_is_connected_flag = true;
        xEventGroupSetBits(wifi_event_group, WIFI_AP_STA_CONNECTED_BIT);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Station " MACSTR " left, AID=%d", MAC2STR(event->mac), event->aid);
        client_is_connected_flag = false;
        // Сбрасываем все флаги, чтобы остановить и UDP-поток
        xEventGroupClearBits(wifi_event_group, WIFI_AP_STA_CONNECTED_BIT | UDP_CLIENT_READY_BIT);
    }
}

// Упаковка 24-битного звука из 32-битного I2S-слота
static void pack_24bit_sample(const uint8_t *i2s_sample_ptr, uint8_t *packed_buffer) {
    // I2S данные приходят в формате MSB first (старший байт первый).
    // Для 24-битного звука в 32-битном слоте значащими являются 3 старших байта.
    // Мы пропускаем младший, незначащий байт.
    packed_buffer[0] = i2s_sample_ptr[1];
    packed_buffer[1] = i2s_sample_ptr[2];
    packed_buffer[2] = i2s_sample_ptr[3];
}

// Расчет чек-суммы
static uint32_t calculate_checksum(const uint8_t *data, size_t length) {
    uint32_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

/* ========================================================================== */
/*                      РЕАЛИЗАЦИЯ ПУБЛИЧНЫХ ФУНКЦИЙ                          */
/* ========================================================================== */

bool is_client_connected(void) {
    return client_is_connected_flag;
}

void wifi_init_softap(void) {
    ESP_LOGI(TAG, "Initializing WiFi in SoftAP mode with optimized settings...");

    // Базовая инициализация сети
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Создаем netif для AP и сохраняем результат для использования
    esp_netif_create_default_wifi_ap();

    // Инициализация WiFi с оптимизированными параметрами
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.ampdu_tx_enable = 1;
    cfg.dynamic_tx_buf_num = 32;
    cfg.cache_tx_buf_num = 32;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Регистрируем обработчик событий
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL));

    // Конфигурация точки доступа
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,
            .channel = WIFI_CHANNEL,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .beacon_interval = 100,
            .pmf_cfg = {
                .required = true
            },
        }
    };

    // Настройка WiFi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Оптимизация после запуска WiFi
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP, 
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    #if CONFIG_IDF_TARGET_ESP32
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(78));
    #endif
    
    ESP_LOGI(TAG, "WiFi SoftAP initialized successfully");
    ESP_LOGI(TAG, "SSID: %s, Channel: %d", WIFI_SSID, WIFI_CHANNEL);
    ESP_LOGI(TAG, "IP Address: 192.168.4.1");
}

void udp_server_task(void *pvParameters) {
    char rx_buffer[128];
    char addr_str[128];
    struct sockaddr_storage source_addr;
    socklen_t socklen = sizeof(source_addr);

    while (1) {
        // Ждем, пока клиент не подключится к нашей точке доступа
        xEventGroupWaitBits(wifi_event_group, WIFI_AP_STA_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        
        // Создаем и биндим сокет
        struct sockaddr_in local_bind_addr = {
            .sin_family = AF_INET,
            .sin_port = htons(UDP_PORT),
            .sin_addr.s_addr = htonl(INADDR_ANY),
        };
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock < 0) {
            ESP_LOGE(TAG, "Failed to create UDP socket: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (bind(sock, (struct sockaddr *)&local_bind_addr, sizeof(local_bind_addr)) < 0) {
            ESP_LOGE(TAG, "Failed to bind UDP socket: errno %d", errno);
            close(sock);
            sock = -1;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ESP_LOGI(TAG, "UDP socket bound, waiting for START command...");

        // Цикл ожидания команды, пока клиент подключен
        while (client_is_connected_flag) {
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            if (len > 0) {
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                rx_buffer[len] = 0;
                ESP_LOGI(TAG, "UDP: Received %d bytes from %s: %s", len, addr_str, rx_buffer);
                
                if (strncmp(rx_buffer, START_COMMAND, strlen(START_COMMAND)) == 0) {
                    ESP_LOGI(TAG, "START_COMMAND received from %s.", addr_str);
                    memcpy(&dest_addr, &source_addr, sizeof(struct sockaddr_in));
                    xEventGroupSetBits(wifi_event_group, UDP_CLIENT_READY_BIT);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100)); // Небольшая пауза
        }
        
        // Если мы здесь, значит клиент отключился
        ESP_LOGW(TAG, "Client disconnected, closing UDP socket.");
        if (sock != -1) {
            shutdown(sock, 0);
            close(sock);
            sock = -1;
        }
    }
}


void i2s_to_udp_task(void *pvParameters) {
    // Определяем размеры буферов
    const size_t AUDIO_DATA_SIZE = SAMPLES_PER_PACKET * BYTES_PER_SAMPLE_OUTPUT;
    const size_t TOTAL_PACKET_SIZE = sizeof(audio_packet_header_t) + AUDIO_DATA_SIZE;
    
    // Выделяем память под буферы
    uint8_t *i2s_read_buf = heap_caps_malloc(I2S_READ_BUFFER_SIZE, MALLOC_CAP_DMA);
    uint8_t *udp_packet_buffer = heap_caps_malloc(TOTAL_PACKET_SIZE, MALLOC_CAP_8BIT);

    // Проверяем выделение памяти
    if (!i2s_read_buf || !udp_packet_buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for audio buffers!");
        if (i2s_read_buf) heap_caps_free(i2s_read_buf);
        if (udp_packet_buffer) heap_caps_free(udp_packet_buffer);
        vTaskDelete(NULL);
        return;
    }

    // Настраиваем указатели на структуры в буфере
    audio_packet_header_t *packet_header = (audio_packet_header_t*)udp_packet_buffer;
    uint8_t *audio_data = udp_packet_buffer + sizeof(audio_packet_header_t);
    uint32_t packet_num = 0;
    
    while (1) {
        // Ждем готовности клиента
        ESP_LOGI(TAG, "Waiting for UDP client to be ready...");
        xEventGroupWaitBits(wifi_event_group, UDP_CLIENT_READY_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "Client ready. Starting audio stream.");

        size_t bytes_read;
        
        // Главный цикл отправки данных
        while ((xEventGroupGetBits(wifi_event_group) & UDP_CLIENT_READY_BIT) != 0) {
            if (is_muted) {
                // Режим Mute - отправляем пустые пакеты
                packet_header->sample_count = 0;
                memset(audio_data, 0, AUDIO_DATA_SIZE);
                vTaskDelay(pdMS_TO_TICKS(10));
            } else {
                // Читаем данные с I2S
                esp_err_t ret = read_i2s_data(i2s_read_buf, I2S_READ_BUFFER_SIZE, &bytes_read);
                if (ret != ESP_OK || bytes_read == 0) {
                    ESP_LOGW(TAG, "I2S read error or timeout: %s", esp_err_to_name(ret));
                    continue;
                }
                
                // Конвертируем 32-бит в 24-бит
                for (int i = 0; i < SAMPLES_PER_PACKET; i++) {
                    pack_24bit_sample(
                        &i2s_read_buf[i * BYTES_PER_SAMPLE],
                        &audio_data[i * BYTES_PER_SAMPLE_OUTPUT]
                    );
                }
                packet_header->sample_count = SAMPLES_PER_PACKET;
            }

            // Формируем заголовок пакета
            packet_header->magic = PACKET_HEADER_MAGIC;
            packet_header->packet_num = packet_num++;
            
            // Формируем слово статуса
            uint32_t status_word = (current_volume & 0xFF) | 
                                 ((is_muted ? 1 : 0) << 8);
            packet_header->status = status_word;
            
            // Считаем контрольную сумму
            packet_header->checksum = calculate_checksum(audio_data, AUDIO_DATA_SIZE);
            
            // Отправляем пакет если сокет открыт
            if (sock != -1) {
                sendto(sock, 
                       udp_packet_buffer, 
                       TOTAL_PACKET_SIZE, 
                       0, 
                       (struct sockaddr *)&dest_addr, 
                       sizeof(dest_addr));
            }
        }
        ESP_LOGW(TAG, "Client disconnected. Stopping audio stream.");
    }
    
    // Освобождаем ресурсы
    heap_caps_free(i2s_read_buf);
    heap_caps_free(udp_packet_buffer);
    vTaskDelete(NULL);
}