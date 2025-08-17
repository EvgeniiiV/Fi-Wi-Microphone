#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

/* ========================================================================== */
/*                         СИСТЕМНЫЕ ЗАГОЛОВКИ                                 */
/* ========================================================================== */
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include "esp_adc/adc_oneshot.h"

/* ========================================================================== */
/*                            СЕТЕВЫЕ ПАРАМЕТРЫ                                */
/* ========================================================================== */
// -- Wi-Fi --
#define WIFI_SSID              "ESP32_Audio_AP"
#define WIFI_PASS              "password"
#define WIFI_CHANNEL           1
#define MAX_STA_CONN           1

// -- UDP --
#define PACKET_HEADER_MAGIC   0xAABBCCDD  // Магическое число для синхронизации пакетов
#define UDP_PORT               3333
#define START_COMMAND          "START_AUDIO_STREAM"

/* ========================================================================== */
/*                             ПАРАМЕТРЫ АУДИО                                 */
/* ========================================================================== */
// -- I2S Configuration --
#define SAMPLE_RATE            48000
#define I2S_SLOT_BITS         32          // Размер слота I2S
#define I2S_DMA_DESC_NUM      3           // Количество DMA дескрипторов
#define I2S_DMA_FRAME_NUM     48          // Размер DMA буфера (2мс при 48kHz)

// -- I2S GPIO --
#define I2S_MCK_GPIO          GPIO_NUM_0    // MCLK
#define I2S_BCK_GPIO          GPIO_NUM_27   // BCLK
#define I2S_WS_GPIO           GPIO_NUM_26   // WS/LRCK
#define I2S_DATA_IN_GPIO      GPIO_NUM_35   // Data In

// -- Audio Buffer Configuration --
#define I2S_SAMPLE_BITS       32          // Размер входного сэмпла в битах
#define BYTES_PER_SAMPLE      (I2S_SAMPLE_BITS/8)  // 4 байта на сэмпл
#define I2S_BYTES_PER_FRAME   BYTES_PER_SAMPLE     // То же, что и BYTES_PER_SAMPLE
#define OUTPUT_SAMPLE_BITS    24          // Размер выходного сэмпла в битах
#define BYTES_PER_SAMPLE_OUTPUT (OUTPUT_SAMPLE_BITS/8) // 3 байта на сэмпл
#define SAMPLES_PER_PACKET    I2S_DMA_FRAME_NUM    // Совпадает с размером DMA буфера
#define I2S_READ_BUFFER_SIZE  (SAMPLES_PER_PACKET * BYTES_PER_SAMPLE)
#define UDP_PACKET_DATA_SIZE  (SAMPLES_PER_PACKET * BYTES_PER_SAMPLE_OUTPUT)

// Добавляем параметры оптимизации сети
#define UDP_TX_BUFFER_SIZE    2048        // Оптимальный размер сетевого буфера
#define UDP_PRIORITY_HIGH     1           // Высокий приоритет для UDP пакетов
#define WIFI_TX_POWER        78          // Максимальная мощность передачи (19.5dBm)
#define WIFI_AMPDU_ENABLED    1           // Включаем агрегацию пакетов
#define WIFI_AMSDU_ENABLED    1           // Включаем объединение пакетов

/* ========================================================================== */
/*                         АППАРАТНЫЕ ПАРАМЕТРЫ (I2C)                          */
/* ========================================================================== */
// -- I2C Configuration --
#define I2C_PORT_NUM          I2C_NUM_0
#define I2C_SCL_IO            GPIO_NUM_22
#define I2C_SDA_IO            GPIO_NUM_21
#define I2C_FREQ_HZ           100000      // Безопасная скорость для всех компонентов

// -- I2C Device Addresses --
#define I2C_ADDR_OLED         0x3C
#define I2C_ADDR_VOL_POT      0x2E        // MCP4531 для громкости (A0=GND)
#define I2C_ADDR_GATE_POT     0x2F        // MCP4531 для гейта (A0=VDD)
#define I2C_ADDR_COMP_POT     0x28        // DS1803 для компрессии (A0,A1,A2=GND)
#define DS1803_CMD_WRITE_BOTH 0xAF

/* ========================================================================== */
/*                         GPIO УПРАВЛЕНИЯ                                      */
/* ========================================================================== */
// -- Control GPIOs --
#define ENCODER_CLK_GPIO      GPIO_NUM_17
#define ENCODER_DT_GPIO       GPIO_NUM_19
#define ENCODER_SW_GPIO       GPIO_NUM_5
#define MUTE_LED_GPIO         GPIO_NUM_23

// -- Monitoring GPIOs --
#define USB_PLUGGED_GPIO      GPIO_NUM_34
#define VOLT_CONTROL_PIN      ADC_CHANNEL_0  // IO36 -> ADC1_CHANNEL_0

/* ========================================================================== */
/*                            ПАРАМЕТРЫ ПРИЛОЖЕНИЯ                             */
/* ========================================================================== */
// -- NVS Configuration --
#define NVS_NAMESPACE         "mic_settings"
#define NVS_KEY              "audio_params"

// -- Control Parameters --
#define VOLUME_MIN            0
#define VOLUME_MAX            128
#define VOLUME_STEP           1
#define COMPRESSION_STEPS     10           // от 1:1 до 10:1
#define GATE_STEPS           16           // от -55 до -40 dBV

// -- Timing Parameters --
#define ENCODER_DEBOUNCE_MS   20          // Антидребезг для вращения
#define BUTTON_DEBOUNCE_MS    250         // Антидребезг для кнопки

/* ========================================================================== */
/*                          СТРУКТУРЫ И ПЕРЕЧИСЛЕНИЯ                           */
/* ========================================================================== */
// UDP Packet Structure
typedef struct {
    uint32_t magic;
    uint32_t packet_num;
    uint32_t sample_count;
    uint32_t status;                      // Биты 0-7: громкость, Бит 8: флаг Mute
    uint32_t checksum;
} audio_packet_header_t;

// NVS Settings Structure
typedef struct {
    uint8_t compression;                  // 0..9
    uint8_t noise_gate;                   // 0..15
} audio_settings_t;

// Encoder Events
typedef enum {
    ENCODER_EVENT_ROTATE_CW,
    ENCODER_EVENT_ROTATE_CCW,
    ENCODER_EVENT_CLICK,
} encoder_event_t;

// System Modes
typedef enum {
    MODE_OPERATING,
    MODE_SETTINGS
} system_mode_t;

// Settings Parameters
typedef enum {
    SETTING_COMPRESSION,
    SETTING_GATE
} settings_param_t;

#endif // PROJECT_CONFIG_H