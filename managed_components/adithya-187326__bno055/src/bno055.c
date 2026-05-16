#include "bno055.h"
#include "bno055_priv.h"

const char *BNO055_TAG = "BNO055";

/**
 * @brief Set the page of the BNO055 sensor.
 *
 * @param[in] imu Pointer to the imu structure.
 * @param[in] page The page to set. 0 or 1.
 *
 * @return ESP_OK if the operation is successful, otherwise an error code.
 */
esp_err_t bno055_set_page(bno055_t *imu, uint8_t page)
{
    /* Check inputs */
    if (imu == NULL)
    {
        ESP_LOGE(BNO055_TAG, "NULL pointer passed to function");
        return ESP_ERR_INVALID_ARG;
    }
    if (page > 1)
    {
        ESP_LOGE(BNO055_TAG, "Invalid page passed: %d", page);
        return ESP_ERR_INVALID_ARG;
    }

    /* Initialize variables */
    uint8_t write_buffer[2] = {PAGE_ID, page};
    /* Write to register */
    esp_err_t ret = i2c_master_transmit(imu->config.slave_handle, write_buffer, sizeof(write_buffer), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }

    imu->config.state.page = page;
    return ESP_OK;
}

/**
 * @brief Set the external crystal usage of the BNO055 sensor.
 *
 * @param[in] imu Pointer to the imu structure.
 * @param[in] state The state of the external crystal usage. True to use the external crystal, false not to.
 *
 * @return ESP_OK if the operation is successful, otherwise an error code.
 * @note The BNO055 sensor must be in configuration mode to set the external crystal usage.
 */
static esp_err_t bno055_set_external_crystal_use(bno055_t *imu, bool state)
{
    /* Check inputs */
    if (imu == NULL)
    {
        ESP_LOGE(BNO055_TAG, "NULL pointer passed to function");
        return ESP_ERR_INVALID_ARG;
    }

    /* Read operation mode register */
    if (imu->config.state.mode != CONFIG_MODE)
    {
        ESP_LOGE(BNO055_TAG, "Cannot set external crystal usage in non configuration modes. Current mode: %d", imu->config.state.mode);
        return ESP_ERR_INVALID_STATE;
    }

    /* Initialize variables */
    uint8_t register_address, register_content = 0x00;
    esp_err_t ret;

    /* Set page to 0 */
    if (imu->config.state.page != 0)
    {
        ret = bno055_set_page(imu, 0);
        if (ret != ESP_OK)
            return ret;
    }

    /* Read system trigger register */
    register_address = SYS_TRIGGER;
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &register_address, sizeof(register_address), &register_content, sizeof(register_content), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to read from register. Error: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set external crystal usage */
    register_content = (register_content & ~0x80) | (state ? 0x80 : 0x00);
    /* Write to register */
    uint8_t write_buffer[2] = {register_address, register_content};
    ret = i2c_master_transmit(imu->config.slave_handle, write_buffer, sizeof(write_buffer), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Mandatory 650ms for external crystal usage change - see datasheet */
    vTaskDelay(pdMS_TO_TICKS(650));
    imu->config.state.external_crystal = state;
    return ESP_OK;
}

/**
 * @brief Set the operation mode of the BNO055 sensor.
 *
 * @param[in] imu Pointer to the imu structure.
 * @param[in] operation_mode The operation mode to be set.
 *
 * @return ESP_OK if the operation is successful, otherwise an error code.
 */
esp_err_t bno055_set_operation_mode(bno055_t *imu, bno055_operation_mode_t operation_mode)
{
    /* Check inputs */
    if (imu == NULL)
    {
        ESP_LOGE(BNO055_TAG, "NULL pointer passed to function");
        return ESP_ERR_INVALID_ARG;
    }
    if ((operation_mode < 0x00) || (operation_mode > 0x0c))
    {
        ESP_LOGE(BNO055_TAG, "Invalid operation mode passed: %d", operation_mode);
        return ESP_ERR_INVALID_ARG;
    }

    if (imu->config.state.mode == operation_mode)
        return ESP_OK;

    /* Set page to 0 */
    esp_err_t ret;
    if (imu->config.state.page != 0)
    {
        ret = bno055_set_page(imu, 0);
        if (ret != ESP_OK)
        {
            ESP_LOGE(BNO055_TAG, "Failed to set page 0. Error: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    /* Initialize variables */
    uint8_t register_address = OPR_MODE, register_content = 0x00;

    /* Read operation mode register */
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &register_address, sizeof(register_address), &register_content, sizeof(register_content), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to read from register. Error: %s", esp_err_to_name(ret));
        return ret;
    }
    /* Set operation mode */
    register_content = (register_content & ~0x0f) | operation_mode;
    /* Write to register */
    uint8_t write_buffer[2] = {register_address, register_content};
    ret = i2c_master_transmit(imu->config.slave_handle, write_buffer, sizeof(write_buffer), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Buffer 30ms for operation mode change - maximum recommended 19ms */
    vTaskDelay(pdMS_TO_TICKS(30));
    imu->config.state.mode = operation_mode;
    return ESP_OK;
}

/**
 * @brief Set the units of the BNO055 sensor.
 *
 * @param[in] imu Pointer to the imu structure.
 * @param[in] units_selected The units to be set.
 *
 * @return ESP_OK if the operation is successful, otherwise an error code.
 *
 * @note The function sets the BNO055 into config mode. It has to be set to appropriate mode post function call.
 */
static esp_err_t bno055_set_units(bno055_t *imu, uint8_t units_selected)
{
    /* Check inputs */
    if (imu == NULL)
    {
        ESP_LOGE(BNO055_TAG, "NULL pointer passed to function");
        return ESP_ERR_INVALID_ARG;
    }
    if (units_selected & ~0x97)
    {
        ESP_LOGE(BNO055_TAG, "Invalid units selected: 0x%2x", units_selected);
        return ESP_ERR_INVALID_ARG;
    }

    /* Initialize variables */
    uint8_t register_address = OPR_MODE, register_content = 0x00;
    esp_err_t ret;
    ESP_LOGV(BNO055_TAG, "Setting units to: '0x%2x'", units_selected);

    /* Check if BNO is in config mode */
    bno055_operation_mode_t temp_mode = imu->config.state.mode;
    if (temp_mode != CONFIG_MODE)
    {
        ESP_LOGW(BNO055_TAG, "BNO is not in config mode. Setting to config mode.");
        ret = bno055_set_operation_mode(imu, CONFIG_MODE);
        if (ret != ESP_OK)
            return ret;
    }
    ESP_LOGV(BNO055_TAG, " BNO is in config mode, will set page 0");

    /* Set page to 0 */
    if (imu->config.state.page != 0)
    {
        ret = bno055_set_page(imu, 0);
        if (ret != ESP_OK)
        {
            ESP_LOGE(BNO055_TAG, "Failed to set page 0. Error: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    ESP_LOGV(BNO055_TAG, " BNO is page 0, will check units");

    if (imu->config.state.units == units_selected)
    {
        ESP_LOGV(BNO055_TAG, "Already in units: '%d'", units_selected);
    }
    else
    {
        /* Read units register */
        register_address = UNIT_SEL;
        ret = i2c_master_transmit_receive(imu->config.slave_handle, &register_address, sizeof(register_address), &register_content, sizeof(register_content), 100);
        if (ret != ESP_OK)
        {
            ESP_LOGE("I2C", "Failed to read from register. Error: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGV(BNO055_TAG, " BNO has different units, will set units");
        /* Set units */
        register_content = (register_content & ~0x9f) | units_selected;
        /* Write to register */
        uint8_t write_buffer[2] = {register_address, register_content};
        ret = i2c_master_transmit(imu->config.slave_handle, write_buffer, sizeof(write_buffer), 100);
        if (ret != ESP_OK)
        {
            ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    /* Set scaling factors appropriately */
    switch (units_selected & 0x01)
    {
    case ACC_MG:
        imu->config.sensor_scale.accelerometer = 1.0f;
        break;

    case ACC_M_S2:
        imu->config.sensor_scale.accelerometer = 100.0f;
        break;
    }
    switch (units_selected & 0x02)
    {
    case GY_RPS:
        imu->config.sensor_scale.gyroscope = 900.0f;
        break;

    case GY_DPS:
        imu->config.sensor_scale.gyroscope = 16.0f;
        break;
    }
    switch (units_selected & 0x04)
    {
    case EUL_RAD:
        imu->config.sensor_scale.euler_angle = 900.0f;
        break;

    case EUL_DEG:
        imu->config.sensor_scale.euler_angle = 16.0f;
        break;
    }
    switch (units_selected & 0x10)
    {
    case TEMP_F:
        imu->config.sensor_scale.temperature = 0.5f;
        break;

    case TEMP_C:
        imu->config.sensor_scale.temperature = 1.0f;
        break;
    }
    imu->config.sensor_scale.magnetometer = 16.0f;
    imu->config.sensor_scale.quaternion = 16384.0f;

    imu->config.state.units = units_selected;

    ret = bno055_set_operation_mode(imu, temp_mode);
    if (ret != ESP_OK)
        return ret;

    return ESP_OK;
}

/**
 * @brief Initialize the BNO055 sensor.
 *
 * @param[in] imu Pointer to the imu structure.
 *
 * @return ESP_OK if the operation is successful, otherwise an error code.
 *
 * @note The function checks the chip ID of the BNO055 sensor, configures the reset pin and sets it to high, sets the page to 0 and sets the operation mode to config mode.
 */
esp_err_t bno055_initialize(bno055_t *imu)
{
    /* Check inputs */
    if (imu == NULL)
    {
        ESP_LOGE(BNO055_TAG, "NULL pointer passed to function");
        return ESP_ERR_INVALID_ARG;
    }

    if (imu->config.reset.method != RESET_SW && imu->config.reset.method != RESET_HW)
    {
        ESP_LOGE(BNO055_TAG, "Invalid reset method");
        return ESP_ERR_INVALID_ARG;
    }

    /* Set page to 0 */
    esp_err_t ret = bno055_set_page(imu, 0);
    if (ret != ESP_OK)
        return ret;

    /* Initialize variables */
    uint8_t register_address = CHIP_ID, register_content = 0x00;
    /* Read chip ID register */
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &register_address, sizeof(register_address), &register_content, sizeof(register_content), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to read from register. Error: %s", esp_err_to_name(ret));
        return ret;
    }
    /* Verify chip ID */
    switch (register_content)
    {
    case 0xA0:
        ESP_LOGD(BNO055_TAG, "Verified BNO055 chip ID '0x%x'", register_content);
        break;

    default:
        ESP_LOGE(BNO055_TAG, "Invalid chip ID: '0x%x'", register_content);
        return ESP_ERR_INVALID_MAC;
    }

    /* Configure reset pin and set to high */
    switch (imu->config.reset.method)
    {
    case RESET_SW:
        imu->config.reset.pin = -1;
        break;

    case RESET_HW:
        ret = gpio_set_direction(imu->config.reset.pin, GPIO_MODE_OUTPUT);
        if (ret != ESP_OK)
            return ret;
        ret = gpio_set_pull_mode(imu->config.reset.pin, GPIO_PULLUP_ENABLE);
        if (ret != ESP_OK)
            return ret;
        ret = gpio_set_level(imu->config.reset.pin, 1);
        if (ret != ESP_OK)
            return ret;
        break;

    default:
        ESP_LOGE(BNO055_TAG, "Invalid reset method");
        return ESP_ERR_INVALID_ARG;
    }

    /* Set operation mode to config */
    if (imu->config.state.mode != CONFIG_MODE)
    {
        ret = bno055_set_operation_mode(imu, CONFIG_MODE);
        if (ret != ESP_OK)
            return ret;
    }

    ESP_LOGI(BNO055_TAG, "BNO055 initialized with operation mode set to config.");
    imu->config.state.mode = CONFIG_MODE;

    /* Set units */
    ret = bno055_set_units(imu, imu->config.state.units);
    if (ret != ESP_OK)
        return ret;

    return ESP_OK;
}

/**
 * @brief Configure the BNO055 sensor.
 *
 * @param[in] imu Pointer to the imu structure.
 * @param[in] operation_mode The operation mode to be set.
 * @param[in] units_selected The units to be set.
 *
 * @return ESP_OK if the operation is successful, otherwise an error code.
 */
esp_err_t bno055_configure(bno055_t *imu, bno055_operation_mode_t operation_mode, uint8_t units_selected)
{
    /* Check inputs */
    if (imu == NULL)
    {
        ESP_LOGE(BNO055_TAG, "NULL pointer passed to function");
        return ESP_ERR_INVALID_ARG;
    }
    if ((operation_mode < 0) || (operation_mode > 0x0c))
    {
        ESP_LOGE(BNO055_TAG, "Invalid operation mode passed: %d", operation_mode);
        return ESP_ERR_INVALID_ARG;
    }
    if (units_selected & ~0x97)
    {
        ESP_LOGE(BNO055_TAG, "Invalid units selected: 0x%2x", units_selected);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    /* Set operation mode to config */
    if (imu->config.state.mode != CONFIG_MODE)
    {
        ret = bno055_set_operation_mode(imu, CONFIG_MODE);
        if (ret != ESP_OK)
            return ret;
    }
    ESP_LOGV(BNO055_TAG, "Operation mode set to config.");

    /* Set external crystal */
    if (imu->config.state.external_crystal != 1)
    {
        ret = bno055_set_external_crystal_use(imu, 1);
        if (ret != ESP_OK)
            return ret;
    }
    ESP_LOGV(BNO055_TAG, "External crystal set.");

    /* Set units */
    if (imu->config.state.units != units_selected)
    {
        ret = bno055_set_units(imu, units_selected);
        if (ret != ESP_OK)
            return ret;
    }
    ESP_LOGV(BNO055_TAG, "Desired units set.");

    /* Set operation mode */
    ret = bno055_set_operation_mode(imu, operation_mode);
    if (ret != ESP_OK)
        return ret;

    ESP_LOGI(BNO055_TAG, "Configured BNO055 for %s operation.", operation_mode_to_string(operation_mode));
    return ESP_OK;
}

/**
 * @brief Check the calibration status of the BNO055 sensor.
 *
 * @param[in] imu Pointer to the imu structure.
 *
 * @return ESP_OK if the operation is successful, otherwise an error code.
 *
 * @note The function reads the calibration status register and prints the calibration status of the accelerometer, gyroscope, magnetometer and system.
 */
esp_err_t bno055_get_calibration_status(bno055_t *imu)
{
    /* Check inputs */
    if (imu == NULL)
    {
        ESP_LOGE(BNO055_TAG, "NULL pointer passed to function");
        return ESP_ERR_INVALID_ARG;
    }

    /* Set page to 0 */
    esp_err_t ret;
    if (imu->config.state.page != 0)
    {
        ret = bno055_set_page(imu, 0);
        if (ret != ESP_OK)
        {
            ESP_LOGE(BNO055_TAG, "Failed to set page 0. Error: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    /* Initialize variables */
    uint8_t register_address = CALIB_STAT, register_content = 0x00;
    /* Read calibration status register */
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &register_address, sizeof(register_address), &register_content, sizeof(register_content), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to read from register. Error: %s", esp_err_to_name(ret));
        return ret;
    }

    imu->config.calibration.xl = (register_content & 0x0c) >> 2;
    imu->config.calibration.gyro = (register_content & 0x30) >> 4;
    imu->config.calibration.mag = (register_content & 0x03);
    imu->config.calibration.sys = (register_content & 0xc0) >> 6;

    ESP_LOGD(BNO055_TAG, "Calibration status - Acc: %d, Gyro: %d, Mag: %d, Sys: %d", imu->config.calibration.xl, imu->config.calibration.gyro, imu->config.calibration.mag, imu->config.calibration.sys);

    imu->config.is_calibrated = (imu->config.calibration.xl == 3) && (imu->config.calibration.gyro == 3) && (imu->config.calibration.mag == 3) && (imu->config.calibration.sys == 3);

    return ESP_OK;
}

/**
 * @brief Get the readings from the BNO055 sensor.
 *
 * @param[in] imu Pointer to the imu structure.
 * @param[in] sensor The type of sensor to read from.
 *
 * @return ESP_OK if the operation is successful, otherwise an error code.
 *
 * @note - The function reads the sensor registers, processes the data and stores it in the imu struct.
 *
 * @note - The sensor readings can be seen on logs directly by setting the `BNO055` tag to `ESP_LOG_VERBOSE`. The function logs the readings.
 */
esp_err_t bno055_get_readings(bno055_t *imu, bno055_sensor_t sensor)
{
    /* Check inputs */
    if (imu == NULL)
    {
        ESP_LOGE(BNO055_TAG, "NULL pointer passed to function");
        return ESP_ERR_INVALID_ARG;
    }

    /* Initialize variables */
    uint8_t register_address = sensor, register_content[8] = {0}, reg_count;

    /* Read sensor type to set count */
    switch (sensor)
    {
    case QUATERNION:
        reg_count = 8;
        break;

    case TEMPERATURE:
        reg_count = 1;
        break;

    case ACCELEROMETER:
    case MAGNETOMETER:
    case GYROSCOPE:
    case EULER_ANGLE:
    case LINEAR_ACCELERATION:
    case GRAVITY:
        reg_count = 6;
        break;

    default:
        ESP_LOGE(BNO055_TAG, "Invalid sensor type passed: %d", sensor);
        return ESP_ERR_INVALID_ARG;
    }

    /* Set page to 0 */
    esp_err_t ret;
    if (imu->config.state.page != 0)
    {
        ret = bno055_set_page(imu, 0);
        if (ret != ESP_OK)
        {
            ESP_LOGE(BNO055_TAG, "Failed to set page 0. Error: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    /* Read sensor registers (multi read operation) */
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &register_address, sizeof(register_address), register_content, reg_count, 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to read from register. Error: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Process sensor data */
    switch (sensor)
    {
    case ACCELEROMETER:
        imu->raw_acceleration.x = u8_to_f(register_content[1], register_content[0], imu->config.sensor_scale.accelerometer);
        imu->raw_acceleration.y = u8_to_f(register_content[3], register_content[2], imu->config.sensor_scale.accelerometer);
        imu->raw_acceleration.z = u8_to_f(register_content[5], register_content[4], imu->config.sensor_scale.accelerometer);
        ESP_LOGV(BNO055_TAG, "Acceleration - X: %.3f, Y: %.3f, Z: %.3f", imu->raw_acceleration.x, imu->raw_acceleration.y, imu->raw_acceleration.z);
        return ESP_OK;

    case MAGNETOMETER:
        imu->magnetometer.x = u8_to_f(register_content[1], register_content[0], imu->config.sensor_scale.magnetometer);
        imu->magnetometer.y = u8_to_f(register_content[3], register_content[2], imu->config.sensor_scale.magnetometer);
        imu->magnetometer.z = u8_to_f(register_content[5], register_content[4], imu->config.sensor_scale.magnetometer);
        ESP_LOGV(BNO055_TAG, "Magnetometer - X: %.3f, Y: %.3f, Z: %.3f", imu->magnetometer.x, imu->magnetometer.y, imu->magnetometer.z);
        return ESP_OK;

    case GYROSCOPE:
        imu->gyroscope.x = u8_to_f(register_content[1], register_content[0], imu->config.sensor_scale.gyroscope);
        imu->gyroscope.y = u8_to_f(register_content[3], register_content[2], imu->config.sensor_scale.gyroscope);
        imu->gyroscope.z = u8_to_f(register_content[5], register_content[4], imu->config.sensor_scale.gyroscope);
        ESP_LOGV(BNO055_TAG, "Gyroscope - X: %.3f, Y: %.3f, Z: %.3f", imu->gyroscope.x, imu->gyroscope.y, imu->gyroscope.z);
        return ESP_OK;

    case EULER_ANGLE:
        imu->euler_angle.yaw = u8_to_f(register_content[1], register_content[0], imu->config.sensor_scale.euler_angle);
        imu->euler_angle.roll = u8_to_f(register_content[3], register_content[2], imu->config.sensor_scale.euler_angle);
        imu->euler_angle.pitch = u8_to_f(register_content[5], register_content[4], imu->config.sensor_scale.euler_angle);
        ESP_LOGV(BNO055_TAG, "Euler - Yaw: %.3f, Pitch: %.3f, Roll: %.3f", imu->euler_angle.yaw, imu->euler_angle.pitch, imu->euler_angle.roll);
        return ESP_OK;

    case QUATERNION:
        imu->quaternion.w = u8_to_f(register_content[1], register_content[0], imu->config.sensor_scale.quaternion);
        imu->quaternion.x = u8_to_f(register_content[3], register_content[2], imu->config.sensor_scale.quaternion);
        imu->quaternion.y = u8_to_f(register_content[5], register_content[4], imu->config.sensor_scale.quaternion);
        imu->quaternion.z = u8_to_f(register_content[7], register_content[6], imu->config.sensor_scale.quaternion);
        ESP_LOGV(BNO055_TAG, "Quaternion - W: %.3f, X: %.3f, Y: %.3f, Z: %.3f", imu->quaternion.w, imu->quaternion.x, imu->quaternion.y, imu->quaternion.z);
        return ESP_OK;

    case LINEAR_ACCELERATION:
        imu->linear_acceleration.x = u8_to_f(register_content[1], register_content[0], imu->config.sensor_scale.accelerometer);
        imu->linear_acceleration.y = u8_to_f(register_content[3], register_content[2], imu->config.sensor_scale.accelerometer);
        imu->linear_acceleration.z = u8_to_f(register_content[5], register_content[4], imu->config.sensor_scale.accelerometer);
        ESP_LOGV(BNO055_TAG, "Linear acceleration - X: %.3f, Y: %.3f, Z: %.3f", imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
        return ESP_OK;

    case GRAVITY:
        imu->gravity.x = u8_to_f(register_content[1], register_content[0], imu->config.sensor_scale.accelerometer);
        imu->gravity.y = u8_to_f(register_content[3], register_content[2], imu->config.sensor_scale.accelerometer);
        imu->gravity.z = u8_to_f(register_content[5], register_content[4], imu->config.sensor_scale.accelerometer);
        ESP_LOGV(BNO055_TAG, "Gravity - X: %.3f, Y: %.3f, Z: %.3f", imu->gravity.x, imu->gravity.y, imu->gravity.z);
        return ESP_OK;

    case TEMPERATURE:
        imu->temperature = (float)((int8_t)register_content[0]) / imu->config.sensor_scale.temperature;
        ESP_LOGV(BNO055_TAG, "Temperature - %.3f", imu->temperature);
        return ESP_OK;
    }

    return ESP_OK;
}

/**
 * @brief Get the sensor offsets.
 *
 * @param[in] imu Pointer to the imu structure.
 *
 * @return ESP_OK if the operation is successful, otherwise an error code.
 *
 * @note - The function reads the sensor offsets from the BNO055 and assigns them to the imu structure.
 *
 * @note - The operation mode of the BNO055 should be set to whatever is needed, post function call. This function internally changes it to `CONFIG_MODE`.
 *
 * @note - The sensor offsets can be seen on logs by setting the `BNO055` tag to `ESP_LOG_VERBOSE`. The function logs the offsets.
 */
esp_err_t bno055_get_offsets(bno055_t *imu)
{
    /* Check inputs */
    if (imu == NULL)
    {
        ESP_LOGE(BNO055_TAG, "NULL pointer passed to function");
        return ESP_ERR_INVALID_ARG;
    }

    /* Set page to 0 */
    esp_err_t ret;
    if (imu->config.state.page != 0)
    {
        ret = bno055_set_page(imu, 0);
        if (ret != ESP_OK)
        {
            ESP_LOGE(BNO055_TAG, "Failed to set page 0. Error: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    bno055_operation_mode_t temp_mode = imu->config.state.mode;

    /* Check for CONFIG_MODE */
    if (temp_mode != CONFIG_MODE)
    {
        ESP_LOGW(BNO055_TAG, "Not in CONFIG_MODE. Setting operation mode to CONFIG_MODE");
        ret = bno055_set_operation_mode(imu, CONFIG_MODE);
        if (ret != ESP_OK)
            return ret;
        vTaskDelay(pdMS_TO_TICKS(25));
    }

    /* Initialize variables */
    uint8_t register_address = ACC_OFFSET_X_LSB, register_content[22];
    /* Read all sensor offsets (multi read operation) */
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &register_address, sizeof(register_address), register_content, sizeof(register_content), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to read from register. Error: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Assign offsets */
    imu->config.offsets.accelerometer.x = u8_to_f(register_content[1], register_content[0], imu->config.sensor_scale.accelerometer);
    imu->config.offsets.accelerometer.y = u8_to_f(register_content[3], register_content[2], imu->config.sensor_scale.accelerometer);
    imu->config.offsets.accelerometer.z = u8_to_f(register_content[5], register_content[4], imu->config.sensor_scale.accelerometer);
    ESP_LOGV(BNO055_TAG, "Accel offset - X: %.3f, Y: %.3f, Z: %.3f", imu->config.offsets.accelerometer.x, imu->config.offsets.accelerometer.y, imu->config.offsets.accelerometer.z);

    imu->config.offsets.magnetometer.x = u8_to_f(register_content[7], register_content[6], imu->config.sensor_scale.magnetometer);
    imu->config.offsets.magnetometer.y = u8_to_f(register_content[9], register_content[8], imu->config.sensor_scale.magnetometer);
    imu->config.offsets.magnetometer.z = u8_to_f(register_content[11], register_content[10], imu->config.sensor_scale.magnetometer);
    ESP_LOGV(BNO055_TAG, "Mag offset - X: %.3f, Y: %.3f, Z: %.3f", imu->config.offsets.magnetometer.x, imu->config.offsets.magnetometer.y, imu->config.offsets.magnetometer.z);

    imu->config.offsets.gyroscope.x = u8_to_f(register_content[13], register_content[12], imu->config.sensor_scale.gyroscope);
    imu->config.offsets.gyroscope.y = u8_to_f(register_content[15], register_content[14], imu->config.sensor_scale.gyroscope);
    imu->config.offsets.gyroscope.z = u8_to_f(register_content[17], register_content[16], imu->config.sensor_scale.gyroscope);
    ESP_LOGV(BNO055_TAG, "Gyro offset - X: %.3f, Y: %.3f, Z: %.3f", imu->config.offsets.gyroscope.x, imu->config.offsets.gyroscope.y, imu->config.offsets.gyroscope.z);

    imu->config.offsets.accelerometer_radius = u8_to_i16(register_content[19], register_content[18]);
    imu->config.offsets.magnetometer_radius = u8_to_i16(register_content[21], register_content[20]);
    ESP_LOGV(BNO055_TAG, "Radius offset - Accel: %d, Mag: %d", imu->config.offsets.accelerometer_radius, imu->config.offsets.magnetometer_radius);

    ret = bno055_set_operation_mode(imu, temp_mode);
    if (ret != ESP_OK)
        return ret;

    return ESP_OK;
}

/**
 * @brief Write the sensor offsets to the BNO055.
 *
 * @param[in] imu Pointer to the imu structure.
 * @param[in] imu Pointer to the imu structure.
 *
 * @return ESP_OK if the operation is successful, otherwise an error code.
 *
 * @note - The function writes the sensor offsets to the BNO055, from the imu struct.
 */
esp_err_t bno055_set_offsets(bno055_t *imu)
{
    /* Check inputs */
    if (imu == NULL)
    {
        ESP_LOGE(BNO055_TAG, "NULL pointer passed to function");
        return ESP_ERR_INVALID_ARG;
    }

    /* Set page to 0 */
    esp_err_t ret;
    if (imu->config.state.page != 0)
    {
        ret = bno055_set_page(imu, 0);
        if (ret != ESP_OK)
        {
            ESP_LOGE(BNO055_TAG, "Failed to set page 0. Error: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    bno055_operation_mode_t temp_mode = imu->config.state.mode;

    /* Check for CONFIG_MODE */
    if (temp_mode != CONFIG_MODE)
    {
        ESP_LOGW(BNO055_TAG, "Not in CONFIG_MODE. Setting operation mode to CONFIG_MODE");
        ret = bno055_set_operation_mode(imu, CONFIG_MODE);
        if (ret != ESP_OK)
            return ret;
        vTaskDelay(pdMS_TO_TICKS(25));
    }

    /* Initialize variables */
    uint8_t write_buffer[2], register_content[18];

    /* Convert all sensor offsets */
    int16_t temp_offset_int = f_to_i16(imu->config.offsets.accelerometer.x, imu->config.sensor_scale.accelerometer);
    i16_to_u8(temp_offset_int, &register_content[0], &register_content[1]);
    temp_offset_int = f_to_i16(imu->config.offsets.accelerometer.y, imu->config.sensor_scale.accelerometer);
    i16_to_u8(temp_offset_int, &register_content[2], &register_content[3]);
    temp_offset_int = f_to_i16(imu->config.offsets.accelerometer.z, imu->config.sensor_scale.accelerometer);
    i16_to_u8(temp_offset_int, &register_content[4], &register_content[5]);

    temp_offset_int = f_to_i16(imu->config.offsets.magnetometer.x, imu->config.sensor_scale.magnetometer);
    i16_to_u8(temp_offset_int, &register_content[6], &register_content[7]);
    temp_offset_int = f_to_i16(imu->config.offsets.magnetometer.y, imu->config.sensor_scale.magnetometer);
    i16_to_u8(temp_offset_int, &register_content[8], &register_content[9]);
    temp_offset_int = f_to_i16(imu->config.offsets.magnetometer.z, imu->config.sensor_scale.magnetometer);
    i16_to_u8(temp_offset_int, &register_content[10], &register_content[11]);

    temp_offset_int = f_to_i16(imu->config.offsets.gyroscope.x, imu->config.sensor_scale.gyroscope);
    i16_to_u8(temp_offset_int, &register_content[12], &register_content[13]);
    temp_offset_int = f_to_i16(imu->config.offsets.gyroscope.y, imu->config.sensor_scale.gyroscope);
    i16_to_u8(temp_offset_int, &register_content[14], &register_content[15]);
    temp_offset_int = f_to_i16(imu->config.offsets.gyroscope.z, imu->config.sensor_scale.gyroscope);
    i16_to_u8(temp_offset_int, &register_content[16], &register_content[17]);

    /* Write offsets to BNO055 */
    write_buffer[0] = ACC_OFFSET_X_LSB - 1;
    for (uint8_t i = 0; i < 18; i++)
    {
        write_buffer[0]++;
        write_buffer[1] = register_content[i];
        ret = i2c_master_transmit(imu->config.slave_handle, write_buffer, sizeof(write_buffer), 100);
        if (ret != ESP_OK)
        {
            ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    ESP_LOGI(BNO055_TAG, "Offsets have been written to BNO055!");

    /* Set operation mode back to normal */
    ret = bno055_set_operation_mode(imu, temp_mode);
    if (ret != ESP_OK)
        return ret;

    return ESP_OK;
}

/**
 * @brief Resets the BNO055 chip.
 *
 * @param[in] imu Pointer to the imu structure.
 *
 * @return ESP_OK if the operation is successful, otherwise an error code.
 */
esp_err_t bno055_reset_chip(bno055_t *imu)
{
    /* Check inputs */
    if (imu == NULL)
    {
        ESP_LOGE(BNO055_TAG, "NULL pointer passed to function");
        return ESP_ERR_INVALID_ARG;
    }
    /* Initialize variables */
    esp_err_t ret;
    /* Check reset method and execute reset */
    switch (imu->config.reset.method)
    {
    case RESET_SW:
        /* Set page to 0 */
        if (imu->config.state.page != 0)
        {
            ret = bno055_set_page(imu, 0);
            if (ret != ESP_OK)
            {
                ESP_LOGE(BNO055_TAG, "Failed to set page 0. Error: %s", esp_err_to_name(ret));
                return ret;
            }
        }

        uint8_t write_buffer[2] = {SYS_TRIGGER, 0x20};
        ret = i2c_master_transmit(imu->config.slave_handle, write_buffer, sizeof(write_buffer), 100);
        if (ret != ESP_OK)
        {
            ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
            return ret;
        }
        break;

    case RESET_HW:
        ret = gpio_set_level(imu->config.reset.pin, 0);
        if (ret != ESP_OK)
            return ret;
        /* Datasheet says atleast 10ns delay -> Accounting for standard RTOS tickrate 100Hz, 10ms delay is given */
        vTaskDelay(pdMS_TO_TICKS(10));
        ret = gpio_set_level(imu->config.reset.pin, 1);
        if (ret != ESP_OK)
            return ret;
        break;

    default:
        ESP_LOGE(BNO055_TAG, "Invalid reset method");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(BNO055_TAG, "BNO055 has been reset!");
    vTaskDelay(pdMS_TO_TICKS(650));

    /* Get values on reset */
    uint8_t buffer[2] = {SYS_TRIGGER, 0x00};
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &buffer[0], sizeof(buffer[0]), &buffer[1], sizeof(buffer[1]), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }
    imu->config.state.external_crystal = buffer[1] >> 7;

    buffer[0] = OPR_MODE;
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &buffer[0], sizeof(buffer[0]), &buffer[1], sizeof(buffer[1]), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }
    imu->config.state.mode = buffer[1] & 0x0c;

    buffer[0] = PAGE_ID;
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &buffer[0], sizeof(buffer[0]), &buffer[1], sizeof(buffer[1]), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }
    imu->config.state.page = buffer[1];

    buffer[0] = UNIT_SEL;
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &buffer[0], sizeof(buffer[0]), &buffer[1], sizeof(buffer[1]), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }
    imu->config.state.units = buffer[1];

    return ESP_OK;
}

/**
 * @brief Start the self-test sequence of the BNO055 sensor.
 *
 * @param[in] imu Pointer to the imu structure.
 *
 * @return ESP_OK if the operation is successful, otherwise an error code.
 *
 * @note The function sets the self-test bit, waits for 200ms, fetches the result and checks if all four components of the BNO055 sensor (MCU, Gyro, Mag, Accel) have passed the self-test. If any component has failed the self-test, the function returns ESP_ERR_INVALID_STATE. The function also updates the imu structure with the current state of the BNO055 sensor.
 */
esp_err_t bno055_start_self_test(bno055_t *imu)
{
    /* Check inputs */
    if (imu == NULL)
    {
        ESP_LOGE(BNO055_TAG, "NULL pointer passed to function");
        return ESP_ERR_INVALID_ARG;
    }

    /* Set page to 0 */
    esp_err_t ret;
    if (imu->config.state.page != 0)
    {
        ret = bno055_set_page(imu, 0);
        if (ret != ESP_OK)
            return ret;
    }

    /* Initialize variables */
    uint8_t buffer[2] = {SYS_TRIGGER, 0x00};
    /* Read from register */
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &buffer[0], sizeof(buffer[0]), &buffer[1], sizeof(buffer[1]), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }
    /* Set self test bit */
    buffer[1] = buffer[1] | 0x01;
    /* Write to register */
    ret = i2c_master_transmit(imu->config.slave_handle, buffer, sizeof(buffer), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
    /* Fetch result */
    buffer[0] = ST_RESULT;
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &buffer[0], sizeof(buffer[0]), &buffer[1], sizeof(buffer[1]), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }

    if ((buffer[1] & 0x0f) != 0x0f)
    {
        ESP_LOGE(BNO055_TAG, "Self test failed. MCU: %d, Gyro: %d, Mag: %d, Accel: %d", (buffer[1] & 0x08) >> 3, (buffer[1] & 0x04) >> 2, (buffer[1] & 0x02) >> 1, (buffer[1] & 0x01));
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(BNO055_TAG, "Self test passed. MCU: %d, Gyro: %d, Mag: %d, Accel: %d", (buffer[1] & 0x08) >> 3, (buffer[1] & 0x04) >> 2, (buffer[1] & 0x02) >> 1, (buffer[1] & 0x01));

    /* Get values on self-test completion */
    buffer[0] = SYS_TRIGGER;
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &buffer[0], sizeof(buffer[0]), &buffer[1], sizeof(buffer[1]), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }
    imu->config.state.external_crystal = buffer[1] >> 7;

    buffer[0] = OPR_MODE;
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &buffer[0], sizeof(buffer[0]), &buffer[1], sizeof(buffer[1]), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }
    imu->config.state.mode = buffer[1] & 0x0c;

    buffer[0] = PAGE_ID;
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &buffer[0], sizeof(buffer[0]), &buffer[1], sizeof(buffer[1]), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }
    imu->config.state.page = buffer[1];

    buffer[0] = UNIT_SEL;
    ret = i2c_master_transmit_receive(imu->config.slave_handle, &buffer[0], sizeof(buffer[0]), &buffer[1], sizeof(buffer[1]), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }
    imu->config.state.units = buffer[1];

    return ESP_OK;
}

/**
 * @brief Set the axis remap of the BNO055 sensor.
 *
 * @param[in] imu Pointer to the imu structure.
 * @param[in] axes_config The axis configuration to be set.
 *
 * @return ESP_OK if the operation is successful, otherwise an error code.
 */
esp_err_t bno055_remap_axis(bno055_t *imu, bno055_axes_t *axes_config)
{
    /* Check inputs */
    if ((imu == NULL) || (axes_config == NULL))
    {
        ESP_LOGE(BNO055_TAG, "NULL pointer passed to function");
        return ESP_ERR_INVALID_ARG;
    }
    if (((axes_config->x < 0x00) || (axes_config->x > 0x06) || (axes_config->x == 0x03)) || ((axes_config->y < 0x00) || (axes_config->y > 0x06) || (axes_config->y == 0x03)) || ((axes_config->z < 0x00) || (axes_config->z > 0x06) || (axes_config->z == 0x03)))
    {
        ESP_LOGE(BNO055_TAG, "Invalid axis configuration - Argument out of bounds");
        return ESP_ERR_INVALID_ARG;
    }
    if ((axes_config->x == axes_config->y) || (axes_config->x == axes_config->z) || (axes_config->y == axes_config->z))
    {
        ESP_LOGE(BNO055_TAG, "Invalid axis configuration - Repeated argument");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    /* Set page to 0 */
    if (imu->config.state.page != 0)
    {
        ret = bno055_set_page(imu, 0);
        if (ret != ESP_OK)
            return ret;
    }

    bno055_operation_mode_t temp_mode = imu->config.state.mode;

    /* Set to config mode */
    if (temp_mode != CONFIG_MODE)
    {
        ret = bno055_set_operation_mode(imu, CONFIG_MODE);
        if (ret != ESP_OK)
            return ret;
        vTaskDelay(pdMS_TO_TICKS(25));
    }

    /* Initialize buffers and assign values */
    uint8_t write_buffer[2] = {AXIS_MAP_CONFIG, (((axes_config->z & 0x03) << 4) | ((axes_config->y & 0x03) << 2) | (axes_config->x & 0x03))};
    /* Transmit data axis remap data */
    ret = i2c_master_transmit(imu->config.slave_handle, write_buffer, sizeof(write_buffer), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    /* Assign axis sign values */
    write_buffer[0] = AXIS_MAP_SIGN;
    write_buffer[1] = (axes_config->x >> 2) | (axes_config->y >> 2) | (axes_config->z >> 2);
    /* Transmit data axis sign data */
    ret = i2c_master_transmit(imu->config.slave_handle, write_buffer, sizeof(write_buffer), 100);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "Failed to write to register. Error: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Set back to previous mode */
    ret = bno055_set_operation_mode(imu, temp_mode);
    if (ret != ESP_OK)
        return ret;
    vTaskDelay(pdMS_TO_TICKS(20));

    return ESP_OK;
}
