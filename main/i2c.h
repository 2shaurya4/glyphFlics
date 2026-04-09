#pragma once
#include "driver/i2c_master.h"

class _i2c {
public:
    i2c_master_dev_handle_t dev_handle;
    void init(i2c_master_bus_handle_t bus, uint8_t address);
    void write(uint8_t address, uint8_t *data, size_t length);
    void write(uint8_t address, uint8_t data);
    void read(uint8_t address, uint8_t *data, size_t length);
};

extern _i2c i2c;