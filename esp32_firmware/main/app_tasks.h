#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "project_config.h"

/* ========================================================================== */
/*                   ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ СОСТОЯНИЯ                          */
/* ========================================================================== */
// Эти переменные описывают текущее состояние всей системы.
// Ключевое слово 'volatile' сообщает компилятору, что переменная может
// быть изменена в любой момент из разных задач, и ее нельзя кэшировать.

// Режим работы: MODE_OPERATING или MODE_SETTINGS
extern volatile system_mode_t current_mode;

// Выбранный параметр в режиме настроек: SETTING_COMPRESSION или SETTING_GATE
extern volatile settings_param_t current_setting_param;

// Текущие настройки звука (для NVS и применения)
extern audio_settings_t current_audio_settings;

// Текущая громкость
extern volatile int current_volume;

// Состояние Mute
extern volatile bool is_muted;

// Состояние батареи и USB
extern volatile int battery_percentage;
extern volatile bool is_usb_plugged;

/* ========================================================================== */
/*                             ПРОТОТИПЫ ЗАДАЧ                                */
/* ========================================================================== */

/**
 * @brief Задача, обрабатывающая события от энкодера.
 * Управляет громкостью, Mute, режимами и настройками.
 */
void encoder_handler_task(void *pvParameters);

/**
 * @brief Задача, обновляющая OLED-дисплей.
 * Отображает всю актуальную информацию о состоянии системы.
 */
void display_task(void *pvParameters);

/**
 * @brief Задача, отслеживающая состояние батареи и подключение USB.
 */
void system_monitor_task(void *pvParameters);


#endif // APP_TASKS_H