idf_component_register(SRCS "main.cpp"
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_adc "ESP_CRSF" "telometer" "HD107S" "H3LIS331DL" "MMC5983MA" "orientator" "DonutPhysics" "DShot")
