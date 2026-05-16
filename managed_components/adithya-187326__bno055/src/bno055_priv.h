#ifndef _BNO055_PRIV_H_
#define _BNO055_PRIV_H_

#pragma once

#include <stdint.h>
#include "math.h"

typedef enum bno055_reg_t
{
    // Page 0 Registers
    CHIP_ID = 0x00,       // Chip ID
    ACC_ID = 0x01,        // Accelerometer ID
    MAG_ID = 0x02,        // Magnetometer ID
    GYR_ID = 0x03,        // Gyroscope ID
    SW_REV_ID_LSB = 0x04, // Software Revision ID (LSB)
    SW_REV_ID_MSB = 0x05, // Software Revision ID (MSB)
    BL_REV_ID = 0x06,     // Bootloader Revision ID

    PAGE_ID = 0x07, // Page ID

    ACC_DATA_X_LSB = 0x08, // Accelerometer Data X LSB
    ACC_DATA_X_MSB = 0x09, // Accelerometer Data X MSB
    ACC_DATA_Y_LSB = 0x0A, // Accelerometer Data Y LSB
    ACC_DATA_Y_MSB = 0x0B, // Accelerometer Data Y MSB
    ACC_DATA_Z_LSB = 0x0C, // Accelerometer Data Z LSB
    ACC_DATA_Z_MSB = 0x0D, // Accelerometer Data Z MSB

    MAG_DATA_X_LSB = 0x0E, // Magnetometer Data X LSB
    MAG_DATA_X_MSB = 0x0F, // Magnetometer Data X MSB
    MAG_DATA_Y_LSB = 0x10, // Magnetometer Data Y LSB
    MAG_DATA_Y_MSB = 0x11, // Magnetometer Data Y MSB
    MAG_DATA_Z_LSB = 0x12, // Magnetometer Data Z LSB
    MAG_DATA_Z_MSB = 0x13, // Magnetometer Data Z MSB

    GYR_DATA_X_LSB = 0x14, // Gyroscope Data X LSB
    GYR_DATA_X_MSB = 0x15, // Gyroscope Data X MSB
    GYR_DATA_Y_LSB = 0x16, // Gyroscope Data Y LSB
    GYR_DATA_Y_MSB = 0x17, // Gyroscope Data Y MSB
    GYR_DATA_Z_LSB = 0x18, // Gyroscope Data Z LSB
    GYR_DATA_Z_MSB = 0x19, // Gyroscope Data Z MSB

    EUL_DATA_X_LSB = 0x1A, // Euler Angle X LSB
    EUL_DATA_X_MSB = 0x1B, // Euler Angle X MSB
    EUL_DATA_Y_LSB = 0x1C, // Euler Angle Y LSB
    EUL_DATA_Y_MSB = 0x1D, // Euler Angle Y MSB
    EUL_DATA_Z_LSB = 0x1E, // Euler Angle Z LSB
    EUL_DATA_Z_MSB = 0x1F, // Euler Angle Z MSB

    QUA_DATA_W_LSB = 0x20, // Quaternion W LSB
    QUA_DATA_W_MSB = 0x21, // Quaternion W MSB
    QUA_DATA_X_LSB = 0x22, // Quaternion X LSB
    QUA_DATA_X_MSB = 0x23, // Quaternion X MSB
    QUA_DATA_Y_LSB = 0x24, // Quaternion Y LSB
    QUA_DATA_Y_MSB = 0x25, // Quaternion Y MSB
    QUA_DATA_Z_LSB = 0x26, // Quaternion Z LSB
    QUA_DATA_Z_MSB = 0x27, // Quaternion Z MSB

    LIA_DATA_X_LSB = 0x28, // Linear Acceleration X LSB
    LIA_DATA_X_MSB = 0x29, // Linear Acceleration X MSB
    LIA_DATA_Y_LSB = 0x2A, // Linear Acceleration Y LSB
    LIA_DATA_Y_MSB = 0x2B, // Linear Acceleration Y MSB
    LIA_DATA_Z_LSB = 0x2C, // Linear Acceleration Z LSB
    LIA_DATA_Z_MSB = 0x2D, // Linear Acceleration Z MSB

    GRV_DATA_X_LSB = 0x2E, // Gravity Vector X LSB
    GRV_DATA_X_MSB = 0x2F, // Gravity Vector X MSB
    GRV_DATA_Y_LSB = 0x30, // Gravity Vector Y LSB
    GRV_DATA_Y_MSB = 0x31, // Gravity Vector Y MSB
    GRV_DATA_Z_LSB = 0x32, // Gravity Vector Z LSB
    GRV_DATA_Z_MSB = 0x33, // Gravity Vector Z MSB

    TEMP = 0x34,           // Temperature
    CALIB_STAT = 0x35,     // Calibration Status
    ST_RESULT = 0x36,      // Self-Test Result
    INT_STA = 0x37,        // Interrupt Status
    SYS_CLK_STATUS = 0x38, // System Clock Status
    SYS_STATUS = 0x39,     // System Status
    SYS_ERR = 0x3A,        // System Error

    UNIT_SEL = 0x3B,    // Unit Selection
    OPR_MODE = 0x3D,    // Operation Mode
    PWR_MODE = 0x3E,    // Power Mode
    SYS_TRIGGER = 0x3F, // System Trigger
    TEMP_SOURCE = 0x40, // Temperature Source

    AXIS_MAP_CONFIG = 0x41, // Axis Mapping Configuration
    AXIS_MAP_SIGN = 0x42,   // Axis Sign Configuration

    ACC_OFFSET_X_LSB = 0x55, // Accelerometer Offset X LSB
    ACC_OFFSET_X_MSB = 0x56, // Accelerometer Offset X MSB
    ACC_OFFSET_Y_LSB = 0x57, // Accelerometer Offset Y LSB
    ACC_OFFSET_Y_MSB = 0x58, // Accelerometer Offset Y MSB
    ACC_OFFSET_Z_LSB = 0x59, // Accelerometer Offset Z LSB
    ACC_OFFSET_Z_MSB = 0x5A, // Accelerometer Offset Z MSB

    MAG_OFFSET_X_LSB = 0x5B, // Magnetometer Offset X LSB
    MAG_OFFSET_X_MSB = 0x5C, // Magnetometer Offset X MSB
    MAG_OFFSET_Y_LSB = 0x5D, // Magnetometer Offset Y LSB
    MAG_OFFSET_Y_MSB = 0x5E, // Magnetometer Offset Y MSB
    MAG_OFFSET_Z_LSB = 0x5F, // Magnetometer Offset Z LSB
    MAG_OFFSET_Z_MSB = 0x60, // Magnetometer Offset Z MSB

    GYR_OFFSET_X_LSB = 0x61, // Gyroscope Offset X LSB
    GYR_OFFSET_X_MSB = 0x62, // Gyroscope Offset X MSB
    GYR_OFFSET_Y_LSB = 0x63, // Gyroscope Offset Y LSB
    GYR_OFFSET_Y_MSB = 0x64, // Gyroscope Offset Y MSB
    GYR_OFFSET_Z_LSB = 0x65, // Gyroscope Offset Z LSB
    GYR_OFFSET_Z_MSB = 0x66, // Gyroscope Offset Z MSB

    ACC_RADIUS_LSB = 0x67, // Accelerometer Radius LSB
    ACC_RADIUS_MSB = 0x68, // Accelerometer Radius MSB
    MAG_RADIUS_LSB = 0x69, // Magnetometer Radius LSB
    MAG_RADIUS_MSB = 0x6A, // Magnetometer Radius MSB

    // Page 1 Registers
    ACC_CONFIG = 0x08,       // Accelerometer Configuration
    MAG_CONFIG = 0x09,       // Magnetometer Configuration
    GYR_CONFIG_0 = 0x0A,     // Gyroscope Configuration 0
    GYR_CONFIG_1 = 0x0B,     // Gyroscope Configuration 1
    ACC_SLEEP_CONFIG = 0x0C, // Accelerometer Sleep Configuration
    GYR_SLEEP_CONFIG = 0x0D, // Gyroscope Sleep Configuration
    INT_MSK = 0x0F,          // Interrupt Mask
    INT_EN = 0x10,           // Interrupt Enable
    ACC_AM_THRES = 0x11,     // Accelerometer Any-Motion Threshold
    ACC_INT_SETTINGS = 0x12, // Accelerometer Interrupt Settings
    ACC_HG_DURATION = 0x13,  // Accelerometer High-G Duration
    ACC_HG_THRES = 0x14,     // Accelerometer High-G Threshold
    ACC_NM_THRES = 0x15,     // Accelerometer No-Motion Threshold
    ACC_NM_SET = 0x16,       // Accelerometer No-Motion Settings
    GYR_INT_SETTING = 0x17,  // Gyroscope Interrupt Settings
    GYR_HR_X_SET = 0x18,     // Gyroscope High Rate X-Axis Settings
    GYR_DUR_X = 0x19,        // Gyroscope Duration X
    GYR_HR_Y_SET = 0x1A,     // Gyroscope High Rate Y-Axis Settings
    GYR_DUR_Y = 0x1B,        // Gyroscope Duration Y
    GYR_HR_Z_SET = 0x1C,     // Gyroscope High Rate Z-Axis Settings
    GYR_DUR_Z = 0x1D,        // Gyroscope Duration Z
    GYR_AM_THRES = 0x1E,     // Gyroscope Any-Motion Threshold
    GYR_AM_SET = 0x1F,       // Gyroscope Any-Motion Settings
} bno055_reg_t;

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Get the sign of a given double value.
     *
     * @param[in] value The double value to get the sign of.
     *
     * @return -1 if the value is negative, 1 if the value is positive, and 0 if the value is zero.
     */
    static inline int signum(double value)
    {
        return (value > 0) - (value < 0);
    }

    /**
     * @brief Calculate the absolute value of a given double.
     *
     * @param[in] value The double value to calculate the absolute value of.
     *
     * @return The absolute value of the given double.
     */
    static inline double absolute(double value)
    {
        return value < 0 ? -value : value;
    }

    /**
     * @brief Calculate the quotient of two double values.
     *
     * @param[in] dividend The double value to be divided.
     * @param[in] divisor The double value to divide by.
     *
     * @return The quotient of the two given double values.
     */
    static inline int quotient(double dividend, double divisor)
    {
        return (int)(dividend / divisor);
    }

    char *operation_mode_to_string(uint8_t operation_mode);

    /**
     * @brief Convert two uint8_t values to a single int16_t value.
     *
     * The function takes two uint8_t values, msb and lsb, and returns a single int16_t value where msb is the most significant byte and lsb is the least significant byte.
     *
     * @param[in] msb The most significant byte to be converted.
     * @param[in] lsb The least significant byte to be converted.
     *
     * @return The converted int16_t value.
     */
    static inline int16_t u8_to_i16(uint8_t msb, uint8_t lsb)
    {
        return (int16_t)((uint16_t)msb << 8 | (uint16_t)lsb);
    }

    /**
     * @brief Convert two uint8_t values to a single float value.
     *
     * The function takes two uint8_t values, msb and lsb, and a float scale, and returns a single float value where msb is the most significant byte and lsb is the least significant byte.
     *
     * @param[in] msb The most significant byte to be converted.
     * @param[in] lsb The least significant byte to be converted.
     * @param[in] scale The scale to divide the converted value by.
     *
     * @return The converted float value.
     */
    static inline float u8_to_f(uint8_t msb, uint8_t lsb, float scale)
    {
        return (float)u8_to_i16(msb, lsb) / scale;
    }

    /**
     * @brief Convert a single int16_t value to two uint8_t values.
     *
     * The function takes a single int16_t value and two uint8_t pointers, lsb and msb, and sets lsb to the least significant byte and msb to the most significant byte of the int16_t value.
     *
     * @param[in] v The int16_t value to be converted.
     * @param[out] lsb Pointer to the least significant byte to be set.
     * @param[out] msb Pointer to the most significant byte to be set.
     */
    static inline void i16_to_u8(int16_t v, uint8_t *lsb, uint8_t *msb)
    {
        *lsb = (uint8_t)(v & 0xFF);
        *msb = (uint8_t)((uint16_t)v >> 8);
    }

    /**
     * @brief Convert a float value to an int16_t value.
     *
     * The function takes a float value and a float scale, and returns an int16_t value where the float value is multiplied by the scale and then converted to an int16_t value, with out-of-range values being clamped to the limits of int16_t.
     *
     * @param[in] val The float value to be converted.
     * @param[in] scale The scale to multiply the float value by.
     *
     * @return The converted int16_t value.
     */
    static inline int16_t f_to_i16(float val, float scale)
    {
        long r = lrintf(val * scale);
        if (r > INT16_MAX)
            r = INT16_MAX;
        if (r < INT16_MIN)
            r = INT16_MIN;
        return (int16_t)r;
    }

#ifdef __cplusplus
}
#endif

#endif