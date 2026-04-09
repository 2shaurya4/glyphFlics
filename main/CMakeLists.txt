idf_component_register(
    SRCS "main.cpp" "VL53L0X.cpp" "i2c.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer
)