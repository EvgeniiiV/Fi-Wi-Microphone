idf_component_register(SRCS "u8g2_esp32_hal.c"
                    INCLUDE_DIRS "."
                    # ОБЪЕДИНЯЕМ ВСЕ ЗАВИСИМОСТИ В ОДИН СПИСОК REQUIRES
                    REQUIRES u8g2 driver esp_rom
)

