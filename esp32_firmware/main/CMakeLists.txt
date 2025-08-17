# Указываем, что наш компонент 'main' состоит из следующих исходных файлов:
idf_component_register(SRCS "main.c"
                           "hardware_control.c"
                           "app_tasks.c"
                           "network_stream.c"
                       # Каталог с заголовочными файлами - текущий
                       INCLUDE_DIRS "."
                       # Список всех системных компонентов, от которых мы зависим
                       PRIV_REQUIRES 
                           nvs_flash      # Для хранения настроек
                           esp_wifi       # Для Wi-Fi
                           esp_event      # Для системы событий
                           lwip           # TCP/IP стек для UDP
                           driver         # Основные драйверы (I2C, I2S, GPIO)
                           freertos       # Ядро операционной системы (задачи, очереди)
                           esp_adc        # Для АЦП (монитор батареи)
                           u8g2_hal       # Наш HAL-адаптер для дисплея
                           esp_timer      # <-- ВОЗВРАЩАЕМ НЕДОСТАЮЩИЙ КОМПОНЕНТ
)