#ifndef _BNO055_H_
#define _BNO055_H_

#pragma once

#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#endif
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "esp_err.h"

extern const char *BNO055_TAG;

typedef enum bno055_operation_mode_t
{
    CONFIG_MODE = 0x00,
    ACC_ONLY_MODE = 0x01,
    MAG_ONLY_MODE = 0x02,
    GYRO_ONLY_MODE = 0x03,
    ACC_MAG_MODE = 0x04,
    ACC_GYRO_MODE = 0x05,
    MAG_GYRO_MODE = 0x06,
    AMG_MODE = 0x07,
    IMU_MODE = 0x08,
    COMPASS_MODE = 0x09,
    M4G_MODE = 0x0A,
    NDOF_FMC_OFF_MODE = 0x0B,
    NDOF_MODE = 0x0C
} bno055_operation_mode_t;

typedef enum bno055_units_t
{
    ACC_M_S2 = 0x00,    // Acceleration in m/s^2
    ACC_MG = 0x01,      // Acceleration in mg
    GY_DPS = 0x00,      // Gyroscope in degrees per second
    GY_RPS = 0x02,      // Gyroscope in radians per second
    EUL_DEG = 0x00,     // Euler angles in degrees
    EUL_RAD = 0x04,     // Euler angles in radians
    TEMP_C = 0x00,      // Temperature in degrees Celsius
    TEMP_F = 0x10,      // Temperature in degrees Fahrenheit
    ORI_ANDRIOD = 0x00, // Android orientation mode
    ORI_WINDOWS = 0x80  // Windows orientation mode
} bno055_units_t;

typedef enum bno055_sensor_t
{
    ACCELEROMETER = 0x08,
    MAGNETOMETER = 0x0e,
    GYROSCOPE = 0x14,
    EULER_ANGLE = 0x1a,
    QUATERNION = 0x20,
    LINEAR_ACCELERATION = 0x28,
    GRAVITY = 0x2e,
    TEMPERATURE = 0x34
} bno055_sensor_t;

typedef enum bno055_reset_method_t
{
    RESET_SW = 0x00,
    RESET_HW = 0x01
} bno055_reset_method_t;

typedef enum axis_t
{
    POSITIVE_X = 0x00,
    NEGATIVE_X = 0x04,
    POSITIVE_Y = 0x01,
    NEGATIVE_Y = 0x05,
    POSITIVE_Z = 0x02,
    NEGATIVE_Z = 0x06
} axis_t;

typedef struct sensor_offset_t
{
    struct
    {
        float x;
        float y;
        float z;
    } accelerometer;
    struct
    {
        float x;
        float y;
        float z;
    } gyroscope;
    struct
    {
        float x;
        float y;
        float z;
    } magnetometer;
    int16_t accelerometer_radius;
    int16_t magnetometer_radius;
} sensor_offset_t;

typedef struct scale_t
{
    float accelerometer;
    float gyroscope;
    float euler_angle;
    float magnetometer;
    float temperature;
    float quaternion;
} scale_t;

typedef struct sensor_config_t
{
    sensor_offset_t offsets;
    scale_t sensor_scale;
    struct
    {
        bno055_reset_method_t method;
        int pin;
    } reset;
    struct
    {
        bno055_operation_mode_t mode;
        uint8_t units;
        uint8_t page;
        bool external_crystal;
    } state;
    gpio_num_t interrupt_pin;
    i2c_master_dev_handle_t slave_handle;
    struct
    {
        uint8_t xl;
        uint8_t gyro;
        uint8_t mag;
        uint8_t sys;
    } calibration;
    bool is_calibrated;
} sensor_config_t;

typedef struct imu_t
{
    struct
    {
        float x;
        float y;
        float z;
    } raw_acceleration;
    struct
    {
        float x;
        float y;
        float z;
    } linear_acceleration;
    struct
    {
        float x;
        float y;
        float z;
    } gravity;
    struct
    {
        float x;
        float y;
        float z;
    } gyroscope;
    struct
    {
        float x;
        float y;
        float z;
    } magnetometer;
    struct
    {
        float roll;
        float pitch;
        float yaw;
    } euler_angle;
    struct
    {
        float x;
        float y;
        float z;
        float w;
    } quaternion;
    float temperature;
    sensor_config_t config;
} bno055_t;

typedef struct bno055_axes_t
{
    axis_t x;
    axis_t y;
    axis_t z;
} bno055_axes_t;

#ifdef __cplusplus
extern "C"
{
#endif

    esp_err_t bno055_initialize(bno055_t *imu);

    esp_err_t bno055_configure(bno055_t *imu, bno055_operation_mode_t operation_mode, uint8_t units_selected);

    esp_err_t bno055_set_operation_mode(bno055_t *imu, bno055_operation_mode_t operation_mode);

    esp_err_t bno055_reset_chip(bno055_t *imu);

    esp_err_t bno055_start_self_test(bno055_t *imu);

    esp_err_t bno055_get_calibration_status(bno055_t *imu);

    esp_err_t bno055_get_readings(bno055_t *imu, bno055_sensor_t sensor);

    esp_err_t bno055_get_offsets(bno055_t *imu);

    esp_err_t bno055_set_offsets(bno055_t *imu);

    esp_err_t bno055_remap_axis(bno055_t *imu, bno055_axes_t *axes_config);

#ifdef __cplusplus
}
#endif

#endif