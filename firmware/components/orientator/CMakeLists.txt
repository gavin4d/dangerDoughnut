idf_component_register(SRCS "kalmanFilter.cpp" "orientator.cpp"
                       INCLUDE_DIRS .
                       REQUIRES nvs_flash driver esp_timer "H3LIS331DL" "MMC5983MA" "mathUtils" "DonutPhysics")
