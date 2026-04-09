#include "i2c.h"
#include "freertos/FreeRTOS.h"
_i2c i2c;

void _i2c::init(i2c_master_bus_handle_t bus, uint8_t address) {
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = address;
    dev_cfg.scl_speed_hz = 400000;
    dev_cfg.scl_wait_us = 0xFFFF;  // clock stretching tolerance
    i2c_master_bus_add_device(bus, &dev_cfg, &dev_handle);
}

void _i2c::write(uint8_t address, uint8_t *data, size_t length) {
    i2c_master_transmit(dev_handle, data, length, 1000 / portTICK_PERIOD_MS);
}

void _i2c::write(uint8_t address, uint8_t data) {
    uint8_t buf = data;
    i2c_master_transmit(dev_handle, &buf, 1, 1000 / portTICK_PERIOD_MS);
}

void _i2c::read(uint8_t address, uint8_t *data, size_t length) {
    i2c_master_receive(dev_handle, data, length, 1000 / portTICK_PERIOD_MS);
}