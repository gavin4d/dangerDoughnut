idf_component_register(SRCS "kalmanFilter.cpp" "orientator.cpp"
                       INCLUDE_DIRS .
                       REQUIRES driver esp_timer "H3LIS331DL" "MMC5983MA" "mathUtils")
