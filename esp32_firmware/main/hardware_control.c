#include "hardware_control.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "u8g2_esp32_hal.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h" 


static const char *TAG = "HW_CTRL";

u8g2_t u8g2;
QueueHandle_t encoder_evt_queue;


// --- ИЗМЕНЕНИЕ №2: Переменные для НОВОГО I2C API ---
static i2s_chan_handle_t i2s_rx_handle = NULL;
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t i2c_dev_oled = NULL;
static i2c_master_dev_handle_t i2c_dev_vol_pot = NULL;
static i2c_master_dev_handle_t i2c_dev_gate_pot = NULL;
static i2c_master_dev_handle_t i2c_dev_comp_pot = NULL;

static const uint8_t calibrated_compression_map[COMPRESSION_STEPS] = { 255, 225, 194, 164, 133, 103, 72, 42, 20, 10 };
static const uint8_t gate_map[GATE_STEPS] = {0, 1, 3, 4, 6, 7, 9, 11, 13, 14, 17, 19, 22, 26, 36, 64};

// // Add debug function to test I2C device presence
// static bool test_i2c_device(i2c_master_dev_handle_t dev_handle, uint8_t device_addr) {
//     uint8_t dummy_data = 0;
//     esp_err_t err = i2c_master_transmit(dev_handle, &dummy_data, 1, pdMS_TO_TICKS(100));
//     if (err != ESP_OK) {
//         ESP_LOGW(TAG, "Device 0x%02X not responding: %s", device_addr, esp_err_to_name(err));
//         return false;
//     }
//     ESP_LOGI(TAG, "Device 0x%02X responding OK", device_addr);
//     return true;
// }

static void i2c_init(void) {
    ESP_LOGI(TAG, "Initializing I2C Master with NEW Driver...");
    
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_NUM,
        .scl_io_num = I2C_SCL_IO,
        .sda_io_num = I2C_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));

    // Конфигурация для всех I2C устройств
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = I2C_FREQ_HZ
    };

    // Добавляем все устройства на шину
    dev_cfg.device_address = I2C_ADDR_OLED;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &i2c_dev_oled));
    
    dev_cfg.device_address = I2C_ADDR_VOL_POT;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &i2c_dev_vol_pot));
    
    dev_cfg.device_address = I2C_ADDR_GATE_POT;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &i2c_dev_gate_pot));
    
    dev_cfg.device_address = I2C_ADDR_COMP_POT;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &i2c_dev_comp_pot));

    ESP_LOGI(TAG, "I2C Master initialized and all devices added.");
}



static void i2s_init(void) {
    ESP_LOGI(TAG, "Initializing optimized I2S...");
    
    // Базовая конфигурация канала I2S
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = I2S_DMA_DESC_NUM,
        .dma_frame_num = I2S_DMA_FRAME_NUM,
        .auto_clear = true
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &i2s_rx_handle));

    // Стандартная конфигурация I2S для приема данных
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_MONO
        ),
        .gpio_cfg = {
            .mclk = I2S_MCK_GPIO,
            .bclk = I2S_BCK_GPIO,
            .ws = I2S_WS_GPIO,
            .dout = GPIO_NUM_NC,
            .din = I2S_DATA_IN_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    // Устанавливаем APLL как источник тактирования
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_rx_handle));
    
    ESP_LOGI(TAG, "I2S initialized with optimized parameters");
    ESP_LOGI(TAG, "Sample rate: %d Hz", SAMPLE_RATE);
    ESP_LOGI(TAG, "DMA: %d frames, %d descriptors", I2S_DMA_FRAME_NUM, I2S_DMA_DESC_NUM);
}

// ЭНКОДЕР


static volatile int64_t last_encoder_time_us = 0;
static volatile int64_t last_button_time_us = 0;

// Static variables for encoder
static volatile uint8_t encoder_last_state = 0;


// State table for clockwise rotation: 0,1,3,2,0,1,3,2...
static const int8_t encoder_states[] = {
    0,  // 00 -> 00
   -1,  // 00 -> 01
    1,  // 00 -> 10
    0,  // 00 -> 11
    1,  // 01 -> 00
    0,  // 01 -> 01
    0,  // 01 -> 10
   -1,  // 01 -> 11
   -1,  // 10 -> 00
    0,  // 10 -> 01
    0,  // 10 -> 10
    1,  // 10 -> 11
    0,  // 11 -> 00
    1,  // 11 -> 01
   -1,  // 11 -> 10
    0   // 11 -> 11
};

// Add debug queue for encoder states
// #define DEBUG_QUEUE_SIZE 10
// static QueueHandle_t encoder_debug_queue;

// Structure for debug data
// typedef struct {
//     uint8_t clk;
//     uint8_t dt;
//     uint8_t last_state;
//     uint8_t current_state;
//     int8_t direction;
// } encoder_debug_t;

static void IRAM_ATTR encoder_isr_handler(void* arg) {
    static uint8_t prev_state = 0;
    static int8_t steps = 0;
    static uint8_t valid_transitions = 0;
    
    int64_t current_time = esp_timer_get_time();
    if ((current_time - last_encoder_time_us) < (ENCODER_DEBOUNCE_MS * 1000)) {
        return;
    }

    uint8_t clk = gpio_get_level(ENCODER_CLK_GPIO);
    uint8_t dt = gpio_get_level(ENCODER_DT_GPIO);
    uint8_t current_state = (clk << 1) | dt;
    
    uint8_t index = (prev_state << 2) | current_state;
    int8_t direction = encoder_states[index & 0x0F];   

    if (direction != 0 && current_state != prev_state) {
        steps += direction; // Remove inversion, use direction directly
        valid_transitions++;
        
        if (valid_transitions >= 4) {
            // Fix direction: CCW when steps negative, CW when steps positive
            encoder_event_t evt = (steps < 0) ? ENCODER_EVENT_ROTATE_CCW : ENCODER_EVENT_ROTATE_CW;
            steps = 0;
            valid_transitions = 0;
            last_encoder_time_us = current_time;
            
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(encoder_evt_queue, &evt, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken) {
                portYIELD_FROM_ISR();
            }
        }
    }
    
    prev_state = current_state;
}

static void IRAM_ATTR encoder_button_isr_handler(void* arg) {
    int64_t current_time_us = esp_timer_get_time();
    if ((current_time_us - last_button_time_us) < (BUTTON_DEBOUNCE_MS * 1000)) {
        return;
    }
    last_button_time_us = current_time_us;
    
    if (gpio_get_level(ENCODER_SW_GPIO) == 0) {
        encoder_event_t evt = ENCODER_EVENT_CLICK;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(encoder_evt_queue, &evt, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}


static void gpio_init(void) {
    ESP_LOGI(TAG, "Initializing GPIO...");
    gpio_config_t io_conf = {};
    
    // Энкодер
    io_conf.pin_bit_mask = (1ULL << ENCODER_CLK_GPIO) | (1ULL << ENCODER_DT_GPIO) | (1ULL << ENCODER_SW_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    
    // ... (остальная инициализация GPIO остается без изменений) ...
    io_conf.pin_bit_mask = (1ULL << MUTE_LED_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    set_mute_led(false);
    
    io_conf.pin_bit_mask = (1ULL << USB_PLUGGED_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);
    
    // Установка службы ISR
    gpio_install_isr_service(0);
    
    // --- ИЗМЕНЕНИЕ ЛОГИКИ ПРЕРЫВАНИЙ ---
    // Прерывание ТОЛЬКО по спадающему фронту на CLK
    //gpio_set_intr_type(ENCODER_CLK_GPIO, GPIO_INTR_NEGEDGE); 
    //gpio_isr_handler_add(ENCODER_CLK_GPIO, encoder_isr_handler, NULL);

    
    // --- ИЗМЕНЕНИЕ ЛОГИКИ ПРЕРЫВАНИЙ ---
    // Прерывания по ОБОИМ фронтам на CLK
    gpio_set_intr_type(ENCODER_CLK_GPIO, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(ENCODER_CLK_GPIO, encoder_isr_handler, NULL);
    
    
    // Оставляем обработчик для кнопки
    gpio_set_intr_type(ENCODER_SW_GPIO, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(ENCODER_SW_GPIO, encoder_button_isr_handler, NULL);

    ESP_LOGI(TAG, "GPIO Initialized.");
}

static void adc_init(void) {
    ESP_LOGI(TAG, "Initializing ADC...");
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, VOLT_CONTROL_PIN, &config));
    ESP_LOGI(TAG, "ADC Initialized.");
}


static void u8g2_display_init(void) {
    if (i2c_dev_oled == NULL) {
        ESP_LOGE(TAG, "OLED device handle is NULL!");
        return;
    }
    
    u8g2_esp32_hal_init(i2c_dev_oled);

    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb);
    
    u8x8_SetI2CAddress(u8g2_GetU8x8(&u8g2), I2C_ADDR_OLED * 2);
    
    ESP_LOGI(TAG, "Initializing U8g2 display...");
    
    // Исправленная проверка инициализации
    u8x8_t *u8x8 = u8g2_GetU8x8(&u8g2);
    if (u8x8 == NULL) {
        ESP_LOGE(TAG, "Failed to get U8x8 instance!");
        return;
    }
    
    u8x8_InitDisplay(u8x8);
    vTaskDelay(pdMS_TO_TICKS(100));  // Задержка после инициализации
    
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);
    
    ESP_LOGI(TAG, "U8g2 display initialized successfully");
}

// Change debug task to only log meaningful state changes
// static void encoder_debug_task(void *arg) {
//     encoder_debug_t debug_data;
//     uint8_t last_logged_state = 0;
    
//     while(1) {
//         if(xQueueReceive(encoder_debug_queue, &debug_data, portMAX_DELAY)) {
//             // Only log if state actually changed and direction is non-zero
//             if(debug_data.current_state != last_logged_state && debug_data.direction != 0) {
//                 ESP_LOGI("ENC_DBG", "CLK:%d DT:%d Last:0x%02X Curr:0x%02X Dir:%d", 
//                          debug_data.clk, debug_data.dt, 
//                          debug_data.last_state, debug_data.current_state,
//                          debug_data.direction);
//                 last_logged_state = debug_data.current_state;
//             }
//         }
//     }
// }

void hardware_init(void) {
    // Create debug queue before GPIO init
    // encoder_debug_queue = xQueueCreate(DEBUG_QUEUE_SIZE, sizeof(encoder_debug_t));
    // xTaskCreate(encoder_debug_task, "encoder_debug", 2048, NULL, 1, NULL);
    
    gpio_init();
    adc_init();
    i2c_init(); 
    u8g2_display_init();
    i2s_init();
}

void apply_initial_settings(const audio_settings_t* settings, uint8_t initial_volume) {
    ESP_LOGI(TAG, "Applying initial settings: Vol=%d, Comp=%d, Gate=%d", 
             initial_volume, settings->compression, settings->noise_gate);
    set_volume(initial_volume);
    set_compression(settings->compression);
    set_gate(settings->noise_gate);
}



// --- Полностью новые, простые и потокобезопасные функции set_... ---
void set_volume(uint8_t volume) {
    uint8_t write_buf[2] = {0x00, volume};
    esp_err_t err = i2c_master_transmit(i2c_dev_vol_pot, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set volume: %s", esp_err_to_name(err));
    }
}

void set_gate(uint8_t value) {
    if (value >= GATE_STEPS) value = GATE_STEPS - 1;
    uint8_t pot_value = gate_map[value];
    uint8_t write_buf[2] = {0x00, pot_value};
    esp_err_t err = i2c_master_transmit(i2c_dev_gate_pot, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gate: %s", esp_err_to_name(err));
    }
}

void set_compression(uint8_t value) {
    if (value >= COMPRESSION_STEPS) value = COMPRESSION_STEPS - 1;
    uint8_t pot_value = calibrated_compression_map[value];
    uint8_t write_buf[2] = {0xA9, pot_value};
    esp_err_t err = i2c_master_transmit(i2c_dev_comp_pot, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set compression: %s", esp_err_to_name(err));
    }
}


void set_mute_led(bool is_on) {
    gpio_set_level(MUTE_LED_GPIO, is_on ? 0 : 1);
}

int get_battery_voltage_mv(void) {
    int adc_raw;
    if (adc_oneshot_read(adc1_handle, VOLT_CONTROL_PIN, &adc_raw) == ESP_OK) {
        int voltage_mv = (int)((3300.0f * (adc_raw / 4095.0f)) / 0.385f);
        return voltage_mv;
    }
    return 0;
}

bool is_usb_plugged_in(void) {
    return (gpio_get_level(USB_PLUGGED_GPIO) == 1);
}

esp_err_t read_i2s_data(uint8_t* buffer, size_t size, size_t* bytes_read) {
    return i2s_channel_read(i2s_rx_handle, buffer, size, bytes_read, pdMS_TO_TICKS(1000));
}

esp_err_t save_settings_to_nvs(const audio_settings_t* settings) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        err = nvs_set_blob(nvs_handle, NVS_KEY, settings, sizeof(audio_settings_t));
        if (err == ESP_OK) nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Settings saved to NVS");
    } else { 
        ESP_LOGE(TAG, "Failed to open NVS for writing"); 
    }
    return err;
}

esp_err_t load_settings_from_nvs(audio_settings_t* settings) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        size_t required_size = sizeof(audio_settings_t);
        err = nvs_get_blob(nvs_handle, NVS_KEY, settings, &required_size);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Settings loaded from NVS: Comp=%d, Gate=%d", settings->compression, settings->noise_gate);
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "Settings not found in NVS, using defaults.");
        }
        nvs_close(nvs_handle);
    } else { 
        ESP_LOGE(TAG, "Failed to open NVS for reading"); 
    }
    return err;
}

