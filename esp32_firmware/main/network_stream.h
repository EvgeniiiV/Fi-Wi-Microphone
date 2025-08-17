#ifndef NETWORK_STREAM_H
#define NETWORK_STREAM_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/* ========================================================================== */
/*                          ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ                             */
/* ========================================================================== */

// Группа событий для синхронизации Wi-Fi и UDP
extern EventGroupHandle_t wifi_event_group;

/* ========================================================================== */
/*                          ПУБЛИЧНЫЕ ФУНКЦИИ                                 */
/* ========================================================================== */

/**
 * @brief Инициализирует Wi-Fi в режиме точки доступа (SoftAP).
 */
void wifi_init_softap(void);

/**
 * @brief Задача, прослушивающая UDP-порт в ожидании команды START.
 */
void udp_server_task(void *pvParameters);

/**
 * @brief Задача, читающая аудио из I2S и отправляющая его по UDP.
 */
void i2s_to_udp_task(void *pvParameters);

/**
 * @brief Возвращает статус подключения клиента.
 * Используется в display_task для отрисовки иконки Wi-Fi.
 * @return true если клиент подключен, иначе false.
 */
bool is_client_connected(void);


#endif // NETWORK_STREAM_H