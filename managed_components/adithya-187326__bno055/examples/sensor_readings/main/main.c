#include <stdio.h>
#include "bno055.h"

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(BNO055_TAG, ESP_LOG_VERBOSE);

    bno055_t bno055 = {0};

    i2c_master_bus_config_t i2c_master_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_NUM_0,
        .intr_priority = 0,
        .scl_io_num = CONFIG_BNO055_SCL_PIN,
        .sda_io_num = CONFIG_BNO055_SDA_PIN,
        .trans_queue_depth = 0,
    };
    i2c_master_bus_handle_t i2c_master_bus = NULL;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_conf, &i2c_master_bus));

    i2c_device_config_t bno055_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_BNO055_I2C_ADDR,
        .flags.disable_ack_check = 0,
        .scl_speed_hz = CONFIG_BNO055_I2C_FREQUENCY * 1000,
        .scl_wait_us = 0xffff,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_master_bus, &bno055_conf, &bno055.config.slave_handle));

    ESP_ERROR_CHECK(bno055_initialize(&bno055));

    ESP_ERROR_CHECK(bno055_configure(&bno055, NDOF_MODE, (ACC_MG | GY_RPS | EUL_DEG)));

    /* Calibrate the sensor */
    ESP_LOGI("BNO055", "Calibrating the sensor, please move the sensor");
    while (1)
    {
        ESP_ERROR_CHECK(bno055_get_calibration_status(&bno055));
        if (bno055.config.is_calibrated)
            break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI("BNO055", "Calibration done");
    while (1)
    {
        bno055_get_readings(&bno055, GRAVITY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}